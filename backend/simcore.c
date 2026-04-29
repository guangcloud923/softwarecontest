#include "simcore.h"
#include "map.h"
#include "conveyor.h"
#include "agv.h"
#include "robot.h"
#include "scheduler.h"
#include "defrag.h"
#include "constraint.h"
#include "stats.h"
#include "server.h"
#include "pathfinding.h"
#ifdef _WIN32
#include <windows.h>
#endif

/* 简单的LCG随机数 */
static unsigned int lcg_rand(unsigned int *state) {
    *state = *state * 1103515245 + 12345;
    return (*state >> 16) & 0x7FFF;
}

void sim_init(SimState *s, int item_count, int seed) {
    memset(s, 0, sizeof(SimState));
    s->time = 0;
    s->running = 1;
    s->paused = 0;
    s->rng_state = (unsigned int)seed;
    s->frame_id = 0;

    /* 初始化地图 */
    map_init(s);

    /* 初始化实体 */
    conveyor_init(s);

    /* 缓冲区: B1在(3,2), B2在(12,2) */
    s->buffer_cnt = MAX_BUFFERS;
    Pos2D buf_pos[] = {{3, 2}, {12, 2}};
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        s->buffers[i].id = i;
        s->buffers[i].pos = buf_pos[i];
        s->buffers[i].item_cnt = 0;
        s->buffers[i].capacity = 6;
        s->buffers[i].conveyor_id = i;
        s->buffers[i].agv_assigned = -1;
    }

    /* 交接区: T1(1,8), T2(5,8), T3(9,8), T4(13,8) */
    s->tzone_cnt = MAX_TRANSFER_ZONES;
    Pos2D tz_pos[] = {{1, 8}, {5, 8}, {10, 8}, {14, 8}};
    for (i = 0; i < s->tzone_cnt; i++) {
        s->tzones[i].id = i;
        s->tzones[i].pos = tz_pos[i];
        s->tzones[i].item_cnt = 0;
        s->tzones[i].total_vol = 0.0;
        s->tzones[i].capacity_vol = 1.0;
        s->tzones[i].robot_id = (i < 2) ? 0 : 1;
        s->tzones[i].shelf_ids[0] = i;
        s->tzones[i].shelf_ids[1] = -1;
    }

    /* 货架: S1(1,10), S2(5,10), S3(9,10), S4(13,10) */
    s->shelf_cnt = MAX_SHELVES;
    Pos2D sh_pos[] = {{1, 10}, {5, 10}, {9, 10}, {13, 10}};
    for (i = 0; i < s->shelf_cnt; i++) {
        s->shelves[i].id = i;
        s->shelves[i].pos = sh_pos[i];
        s->shelves[i].robot_id = (i % 2 == 0) ? 0 : 1;
        s->shelves[i].tp_id = i;
        s->shelves[i].total_vol = 0.0;
        memset(s->shelves[i].slots, 0, sizeof(s->shelves[i].slots));
    }

    /* 刷新地图实体绑定 */
    map_recalc_special(s);

    agv_init(s);
    robot_init(s);

    /* 生成货物 */
    int to_create = item_count;
    if (to_create > MAX_ITEMS) to_create = MAX_ITEMS;
    if (to_create < 1) to_create = 30;
    s->items_total = to_create;

    for (i = 0; i < to_create; i++) {
        Item *it = &s->items[i];
        it->id = i;
        int vol = lcg_rand(&s->rng_state) % 3;
        it->volume = (ItemVolume)vol;
        it->target_shelf = lcg_rand(&s->rng_state) % MAX_SHELVES;
        it->state = ITEM_INIT;
        it->location_id = -1;
        it->t_spawn = 0;
        it->t_shelved = 0;
    }
    s->item_cnt = to_create;
    s->next_item_id = to_create;

    /* 标记多余槽无效 */
    for (i = to_create; i < MAX_ITEMS; i++) {
        s->items[i].id = -1;
    }

    /* 货物由conveyor_step按目标货架自动生成到对应传送带 */
    stats_init(s);

    printf("仿真初始化完成: %d 货物, %d AGV, %d 传送带, %d 货架, %d 交接区\n\n",
           s->items_total, s->agv_cnt, s->conveyor_cnt, s->shelf_cnt, s->tzone_cnt);
}

void sim_run(SimState *s) {
    printf("========== 仿真开始 (终端模式) ==========\n\n");

    while (s->running && s->time < MAX_TIME) {
        s->time += 1.0;

        /* 1. 传送带 */
        int i;
        for (i = 0; i < s->conveyor_cnt; i++)
            conveyor_step(s, i);

        /* 2. AGV调度 */
        scheduler_dispatch_all(s);

        /* 2.5 初始化全局预约表 */
        pathfinding_resv_clear();
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *agv = &s->agvs[i];
            int has_active = (agv->traj_len > 0 && agv->traj_idx < agv->traj_len);
            if (has_active) {
                /* 有未完成轨迹: 加入预约表 */
                pathfinding_resv_add_trajectory(
                    &agv->trajectory[agv->traj_idx],
                    agv->traj_len - agv->traj_idx);
            } else {
                /* 无轨迹(静止/等待): 生成停留预约 */
                TrajectoryPoint stay[200];
                int t;
                for (t = 0; t < 200; t++) {
                    stay[t].x = agv->pos.x;
                    stay[t].y = agv->pos.y;
                    stay[t].t = agv->cur_t + 1 + t;
                }
                pathfinding_resv_add_trajectory(stay, 200);
            }
        }

        /* 2.6 检查活跃轨迹的终点是否与静止AGV冲突 */
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *agv = &s->agvs[i];
            if (agv->traj_len == 0 || agv->traj_idx >= agv->traj_len) continue;
            TrajectoryPoint endpt = agv->trajectory[agv->traj_len - 1];
            int j;
            for (j = 0; j < s->agv_cnt; j++) {
                if (i == j) continue;
                AGV *other = &s->agvs[j];
                int other_static = !(other->traj_len > 0 && other->traj_idx < other->traj_len);
                if (other_static && other->pos.x == endpt.x && other->pos.y == endpt.y) {
                    agv->traj_len = 0;
                    agv->traj_idx = 0;
                    break;
                }
            }
        }

        /* 2.7 检查两两活跃轨迹终点是否冲突 */
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *a = &s->agvs[i];
            if (a->traj_len == 0 || a->traj_idx >= a->traj_len) continue;
            TrajectoryPoint ea = a->trajectory[a->traj_len - 1];
            int j;
            for (j = i + 1; j < s->agv_cnt; j++) {
                AGV *b = &s->agvs[j];
                if (b->traj_len == 0 || b->traj_idx >= b->traj_len) continue;
                TrajectoryPoint eb = b->trajectory[b->traj_len - 1];
                if (ea.x == eb.x && ea.y == eb.y &&
                    abs(ea.t - eb.t) <= 15) {
                    /* 两个AGV目标格相同且到达时间接近, 让后到的重规划 */
                    if (ea.t >= eb.t)
                        a->traj_len = 0, a->traj_idx = 0;
                    else
                        b->traj_len = 0, b->traj_idx = 0;
                }
            }
        }

        /* 3. AGV移动 */
        for (i = 0; i < s->agv_cnt; i++)
            agv_step(s, i);

        /* 4. 机器人 */
        for (i = 0; i < s->robot_cnt; i++)
            robot_step(s, i);

        /* 5. 碎片整理 */
        defrag_tick(s);

        /* 6. 约束校验 */
        if ((int)s->time % 10 == 0) {
            constraint_check_all(s);
            if (!s->running) break;
        }

        /* 7. 得分更新 */
        stats_update(s);

        /* 调试输出 */
        if ((int)s->time <= 100 && (int)s->time % 10 == 0) {
            printf("  T%.0f: 已上架=%d/%d | AGV: ", s->time, s->items_shelved, s->items_total);
            for (i = 0; i < s->agv_cnt; i++) {
                AGV *a = &s->agvs[i];
                const char *st = "IDL";
                if (a->status == AGV_MOVING_TO_BUFFER) st = "->B";
                else if (a->status == AGV_MOVING_TO_TRANSFER) st = "->T";
                else if (a->status == AGV_LOADING) st = "LD";
                else if (a->status == AGV_UNLOADING) st = "UD";
                else if (a->status == AGV_WAITING) st = "WT";
                else if (a->status == AGV_RETURNING) st = "RT";
                printf("AGV%d(%d,%d,%s,c%d) ", i, a->pos.x, a->pos.y, st, a->item_cnt);
            }
            printf("\n");
        }

        /* 终止条件 */
        if (s->items_shelved >= s->items_total) {
            s->running = 0;
        }

        /* 死锁检测: 所有AGV空闲 + 全系统无在途货物 */
        if (s->items_shelved < s->items_total && s->time > 200) {
            int all_idle = 1, all_empty = 1;
            for (i = 0; i < s->agv_cnt; i++)
                if (s->agvs[i].busy) all_idle = 0;
            for (i = 0; i < s->buffer_cnt; i++)
                if (s->buffers[i].item_cnt > 0) all_empty = 0;
            for (i = 0; i < s->tzone_cnt; i++)
                if (s->tzones[i].item_cnt > 0) all_empty = 0;
            for (i = 0; i < s->conveyor_cnt; i++)
                if (s->conveyors[i].item_cnt > 0) all_empty = 0;
            for (i = 0; i < s->agv_cnt; i++)
                if (s->agvs[i].item_cnt > 0) all_empty = 0;
            for (i = 0; i < s->robot_cnt; i++)
                if (s->robots[i].item_cnt > 0) all_empty = 0;
            /* 还有未生成的货物(仍在传送带上排队) */
            int unspawned = 0;
            for (i = 0; i < s->item_cnt; i++)
                if (s->items[i].state == ITEM_INIT) unspawned = 1;
            if (all_idle && all_empty && !unspawned) break;
        }
    }

    constraint_check_all(s);
    stats_print(s);
}

void sim_run_with_server(SimState *s, int ws_port) {
    server_start(s, ws_port);
    printf("========== 仿真开始 (服务器模式: ws://localhost:%d/ws) ==========\n\n", ws_port);

    while (s->running && s->time < MAX_TIME) {
        s->time += 1.0;

        int i;
        for (i = 0; i < s->conveyor_cnt; i++)
            conveyor_step(s, i);
        scheduler_dispatch_all(s);

        /* 初始化全局预约表 */
        pathfinding_resv_clear();
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *agv = &s->agvs[i];
            int has_active = (agv->traj_len > 0 && agv->traj_idx < agv->traj_len);
            if (has_active) {
                pathfinding_resv_add_trajectory(
                    &agv->trajectory[agv->traj_idx],
                    agv->traj_len - agv->traj_idx);
            } else {
                TrajectoryPoint stay[30];
                int t;
                for (t = 0; t < 30; t++) {
                    stay[t].x = agv->pos.x;
                    stay[t].y = agv->pos.y;
                    stay[t].t = agv->cur_t + 1 + t;
                }
                pathfinding_resv_add_trajectory(stay, 200);
            }
        }

        /* 检查活跃轨迹终点是否与静止AGV冲突 */
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *agv = &s->agvs[i];
            if (agv->traj_len == 0 || agv->traj_idx >= agv->traj_len) continue;
            TrajectoryPoint endpt = agv->trajectory[agv->traj_len - 1];
            int j;
            for (j = 0; j < s->agv_cnt; j++) {
                if (i == j) continue;
                AGV *other = &s->agvs[j];
                int other_static = !(other->traj_len > 0 && other->traj_idx < other->traj_len);
                if (other_static && other->pos.x == endpt.x && other->pos.y == endpt.y) {
                    agv->traj_len = 0;
                    agv->traj_idx = 0;
                    break;
                }
            }
        }

        /* 检查两两活跃轨迹终点是否冲突 */
        for (i = 0; i < s->agv_cnt; i++) {
            AGV *a = &s->agvs[i];
            if (a->traj_len == 0 || a->traj_idx >= a->traj_len) continue;
            TrajectoryPoint ea = a->trajectory[a->traj_len - 1];
            int j;
            for (j = i + 1; j < s->agv_cnt; j++) {
                AGV *b = &s->agvs[j];
                if (b->traj_len == 0 || b->traj_idx >= b->traj_len) continue;
                TrajectoryPoint eb = b->trajectory[b->traj_len - 1];
                if (ea.x == eb.x && ea.y == eb.y &&
                    abs(ea.t - eb.t) <= 15) {
                    if (ea.t >= eb.t)
                        a->traj_len = 0, a->traj_idx = 0;
                    else
                        b->traj_len = 0, b->traj_idx = 0;
                }
            }
        }

        for (i = 0; i < s->agv_cnt; i++)
            agv_step(s, i);
        for (i = 0; i < s->robot_cnt; i++)
            robot_step(s, i);
        defrag_tick(s);

        if ((int)s->time % 10 == 0) {
            constraint_check_all(s);
            if (!s->running) break;
        }

        stats_update(s);

        /* 每2 tick推送一帧(约30fps有效帧) */
        if ((int)s->time % 2 == 0) {
            server_push_frame(s);
        }

        if ((int)s->time <= 100 && (int)s->time % 20 == 0) {
            printf("  T%.0f: 已上架=%d/%d 得分=%.1f\n",
                   s->time, s->items_shelved, s->items_total, s->score);
        }

        if (s->items_shelved >= s->items_total) {
            s->running = 0;
        }
    }

    /* 推送最终帧 */
    server_push_frame(s);
    stats_print(s);

#ifdef _WIN32
    Sleep(2000); /* 让客户端收到最后帧 */
#else
    sleep(2);
#endif
    server_stop();
}
