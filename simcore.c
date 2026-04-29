#include "simcore.h"
#include "map.h"
#include "conveyor.h"
#include "agv.h"
#include "robot.h"
#include "constraint.h"
#include "stats.h"

/*
 * 初始化仿真环境
 */
void sim_init(SimState *s, int cargo_count, int seed) {
    memset(s, 0, sizeof(SimState));
    s->time = 0;
    s->running = 1;
    s->max_time = MAX_TIME;
    s->cargo_shelved = 0;
    s->violations = 0;
    s->collision_flag = 0;
    s->violation_msg[0] = '\0';

    srand(seed);

    /* 初始化地图拓扑 */
    map_init(s);

    /* 初始化传送带 */
    conveyor_init(s);

    /* 初始化缓冲区 */
    s->buffer_cnt = MAX_BUFFERS;
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        s->buffers[i].id = i;
        s->buffers[i].cargo_cnt = 0;
        s->buffers[i].capacity = 6;
        s->buffers[i].conveyor_id = i;
        s->buffers[i].node_id = (i == 0) ? 2 : 3;  /* B1->node2, B2->node3 */
        s->buffers[i].agv_assigned = -1;
    }

    /* 初始化交接区 */
    s->tp_cnt = MAX_TRANSFER_POINTS;
    for (i = 0; i < s->tp_cnt; i++) {
        s->tps[i].id = i;
        s->tps[i].node_id = (i == 0) ? 5 : 6;  /* TP1->node5, TP2->node6 */
        s->tps[i].cargo_cnt = 0;
        s->tps[i].capacity = 4;
        s->tps[i].shelf_id = i * 2;
        s->tps[i].robot_id = i;
    }

    /* 初始化货架 */
    s->shelf_cnt = MAX_SHELVES;
    for (i = 0; i < s->shelf_cnt; i++) {
        s->shelves[i].id = i;
        s->shelves[i].slot_cnt = 10;
        s->shelves[i].robot_id = (i % 2 == 0) ? 0 : 1;  /* S0,S2->R0; S1,S3->R1 */
        s->shelves[i].tp_id = (i % 2 == 0) ? 0 : 1;     /* S0,S2->TP0; S1,S3->TP1 */
        s->shelves[i].node_id = 7 + i;                    /* S1->7, S2->8, S3->10, S4->11 */
        int j;
        for (j = 0; j < s->shelves[i].slot_cnt; j++) {
            s->shelves[i].slots[j].id = j;
            s->shelves[i].slots[j].occupied = 0;
            s->shelves[i].slots[j].cargo_id = -1;
        }
    }

    /* 修正货架node_id: 地图中 node7=S1, node8=S2, node10=S3, node11=S4 */
    int shelf_nodes[] = {7, 8, 10, 11};
    for (i = 0; i < s->shelf_cnt; i++) {
        s->shelves[i].node_id = shelf_nodes[i];
    }

    /* 初始化AGV */
    agv_init(s);

    /* 初始化机器人 */
    robot_init(s);

    /* 生成货物 */
    int cargo_to_create = (cargo_count > 0 && cargo_count <= MAX_CARGOS) ?
                           cargo_count : 20;
    s->cargo_total = cargo_to_create;

    for (i = 0; i < cargo_to_create; i++) {
        s->cargos[i].id = i;
        /* 随机体积 */
        int vol = rand() % 3;  /* 0,1,2 */
        s->cargos[i].volume = (CargoVolume)vol;
        /* 随机目标货架 */
        s->cargos[i].target_shelf_id = rand() % MAX_SHELVES;
        s->cargos[i].state = CS_INIT;
        s->cargos[i].location_id = -1;
        s->cargos[i].t_create = 0;
        s->cargos[i].t_shelved = 0;
    }
    s->cargo_cnt = cargo_to_create;

    /* 标记未使用槽位无效, 防止spawn误取 */
    for (i = cargo_to_create; i < MAX_CARGOS; i++) {
        s->cargos[i].id = -1;
    }

    /* 根据目标货架分配到对应传送带:
       Shelf0,2 -> Conv0 (左侧分支), Shelf1,3 -> Conv1 (右侧分支) */
    for (i = 0; i < cargo_to_create; i++) {
        int target = s->cargos[i].target_shelf_id;
        int belt = (target == 0 || target == 2) ? 0 : 1;
        Conveyor *c = &s->conveyors[belt];
        if (c->cargo_cnt < c->capacity) {
            s->cargos[i].state = CS_ON_CONVEYOR;
            s->cargos[i].location_id = belt;
            s->cargos[i].t_create = 0;
            c->cargo_ids[c->cargo_cnt++] = i;
        }
    }

    printf("仿真初始化完成: %d 货物, %d AGV, %d 传送带, %d 货架\n\n",
           s->cargo_cnt, s->agv_cnt, s->conveyor_cnt, s->shelf_cnt);
}

/*
 * 主仿真循环
 */
void sim_run(SimState *s) {
    printf("========== 仿真开始 ==========\n\n");

    while (s->running && s->time < s->max_time) {
        s->time += 1.0;

        /* 1. 传送带步骤 */
        int i;
        for (i = 0; i < s->conveyor_cnt; i++) {
            conveyor_step(s, i);
            if (!s->running) break;
        }
        if (!s->running) break;

        /* 2. AGV调度 */
        agv_dispatch(s);

        /* 3. AGV移动 */
        for (i = 0; i < s->agv_cnt; i++) {
            agv_step(s, i);
            if (!s->running) break;
        }
        if (!s->running) break;

        /* 4. 机器人步骤 */
        for (i = 0; i < s->robot_cnt; i++) {
            robot_step(s, i);
            if (!s->running) break;
        }
        if (!s->running) break;

        /* 5. 约束校验(每10 tick做一次,节省开销) */
        if ((int)s->time % 10 == 0) {
            constraint_check_all(s);
            if (!s->running) break;
        }

        /* 调试: 前100 tick每10 tick输出详细状态 */
        if ((int)s->time <= 100 && (int)s->time % 10 == 0) {
            int n_init=0,n_conv=0,n_buf=0,n_agv=0,n_xfer=0,n_shelf=0;
            int ci;
            for (ci=0;ci<s->cargo_cnt;ci++) {
                switch(s->cargos[ci].state) {
                    case CS_INIT: n_init++; break;
                    case CS_ON_CONVEYOR: n_conv++; break;
                    case CS_IN_BUFFER: n_buf++; break;
                    case CS_ON_AGV: n_agv++; break;
                    case CS_AT_TRANSFER: n_xfer++; break;
                    case CS_ON_SHELF: n_shelf++; break;
                }
            }
            printf("  T%.0f: sh=%d | i=%d c=%d b=%d a=%d x=%d s=%d | ",
                   s->time, s->cargo_shelved, n_init,n_conv,n_buf,n_agv,n_xfer,n_shelf);
            for (ci=0;ci<s->agv_cnt;ci++) {
                AGV *a = &s->agvs[ci];
                const char *st = "IDL";
                if (a->state==AGV_TO_BUFFER) st="->B";
                else if (a->state==AGV_WAIT_BUFFER) st="wB";
                else if (a->state==AGV_LOADING) st="LOD";
                else if (a->state==AGV_TO_TRANSFER) st="->T";
                else if (a->state==AGV_WAIT_TRANSFER) st="wT";
                else if (a->state==AGV_UNLOADING) st="ULD";
                else if (a->state==AGV_BLOCKED) st="BLK";
                printf("AGV%d(%d,%s,c%d) ", ci, a->cur_node, st, a->cargo_cnt);
            }
            int bf_free=0;
            for (ci=0;ci<s->buffer_cnt;ci++) bf_free += s->buffers[ci].capacity - s->buffers[ci].cargo_cnt;
            int tp_free=0;
            for (ci=0;ci<s->tp_cnt;ci++) tp_free += s->tps[ci].capacity - s->tps[ci].cargo_cnt;
            printf("bf=%d tp=%d | r0[%d]=", bf_free, tp_free, s->robots[0].state);
            for (ci=0;ci<s->robots[0].cargo_cnt;ci++) printf("%d,", s->robots[0].cargo_ids[ci]);
            printf(" r1[%d]=", s->robots[1].state);
            for (ci=0;ci<s->robots[1].cargo_cnt;ci++) printf("%d,", s->robots[1].cargo_ids[ci]);
            printf(" | TP0:");
            for (ci=0;ci<s->tps[0].cargo_cnt;ci++) printf("%d(s%d,v%d),", s->tps[0].cargo_ids[ci],
                s->cargos[s->tps[0].cargo_ids[ci]].target_shelf_id,
                s->cargos[s->tps[0].cargo_ids[ci]].volume);
            printf(" TP1:");
            for (ci=0;ci<s->tps[1].cargo_cnt;ci++) printf("%d(s%d,v%d),", s->tps[1].cargo_ids[ci],
                s->cargos[s->tps[1].cargo_ids[ci]].target_shelf_id,
                s->cargos[s->tps[1].cargo_ids[ci]].volume);
            printf("\n");
        }

        /* 终止条件: 所有货物已上架 */
        if (s->cargo_shelved >= s->cargo_total) {
            s->running = 0;
        }

        /* 如果所有AGV空闲, 所有传送带空且缓冲区空, 仍有未上架货物 -> 死锁 */
        int all_idle = 1;
        for (i = 0; i < s->agv_cnt; i++) {
            if (s->agvs[i].busy) { all_idle = 0; break; }
        }
        if (all_idle && s->cargo_shelved < s->cargo_total) {
            /* 检查是否还有货物在流程中 */
            int in_flow = 0;
            int ci;
            for (ci = 0; ci < s->cargo_cnt; ci++) {
                if (s->cargos[ci].state != CS_ON_SHELF &&
                    s->cargos[ci].state != CS_INIT) {
                    in_flow++;
                }
            }
            if (in_flow == 0 && s->cargo_shelved < s->cargo_total) {
                /* 还有未进入流程的货物 -> 可能是生成不足, 继续 */
                /* 但检查传送带, 缓冲区等地方 */
                int buf_empty = 1;
                for (i = 0; i < s->buffer_cnt; i++) {
                    if (s->buffers[i].cargo_cnt > 0) buf_empty = 0;
                }
                int tp_empty = 1;
                for (i = 0; i < s->tp_cnt; i++) {
                    if (s->tps[i].cargo_cnt > 0) tp_empty = 0;
                }
                if (buf_empty && tp_empty) {
                    int conv_empty = 1;
                    for (i = 0; i < s->conveyor_cnt; i++) {
                        if (s->conveyors[i].cargo_cnt > 0) conv_empty = 0;
                    }
                    if (conv_empty) {
                        /* 系统死锁或所有在途货物无法继续 */
                        if (s->time > 50) break; /* 给足够时间 */
                    }
                }
            }
        }
    }

    /* 最终约束校验 */
    constraint_check_all(s);

    printf("\n========== 仿真结束 ==========\n");
    printf("结束时间: %.0f ticks\n", s->time);
    printf("上架: %d / %d\n", s->cargo_shelved, s->cargo_total);
    if (!s->running && s->violations > 0) {
        printf("终止原因: %s\n", s->violation_msg[0] ?
               s->violation_msg : "约束违规");
    } else if (s->time >= s->max_time) {
        printf("终止原因: 达到最大仿真时间\n");
    } else if (s->cargo_shelved >= s->cargo_total) {
        printf("终止原因: 全部货物已上架\n");
    }
    printf("\n");
}
