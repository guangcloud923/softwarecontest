#include "agv.h"
#include "map.h"
#include "scheduler.h"
#include "pathfinding.h"

/* 停车位坐标 */
static const Pos2D PARK_POS[MAX_AGVS] = {{1,0}, {14,0}, {1,6}, {14,6}};

void agv_init(SimState *s) {
    s->agv_cnt = MAX_AGVS;
    int i;

    for (i = 0; i < s->agv_cnt; i++) {
        AGV *a = &s->agvs[i];
        a->id = i;
        a->pos = PARK_POS[i];
        a->cur_t = 0;
        a->status = AGV_IDLE;
        a->load_vol = 0.0;
        a->item_cnt = 0;
        a->buffer_target = -1;
        a->transfer_target = -1;
        a->busy = 0;
        a->traj_len = 0;
        a->traj_idx = 0;
        a->constraint_cnt = 0;
        a->wait_ticks = 0;
        a->home_park = i;
    }
}

/* 找最近空闲停车位 */
static int find_nearest_park(SimState *s, int agv_id) {
    AGV *a = &s->agvs[agv_id];
    int best = a->home_park, best_dist = 9999;
    int i;
    for (i = 0; i < MAX_AGVS; i++) {
        int occupied = 0;
        int j;
        for (j = 0; j < s->agv_cnt; j++) {
            if (j == agv_id) continue;
            AGV *oa = &s->agvs[j];
            /* 有其他AGV正停在车位上(无论状态) */
            if (oa->pos.x == PARK_POS[i].x &&
                oa->pos.y == PARK_POS[i].y) {
                occupied = 1;
                break;
            }
            /* 有其他AGV的活跃轨迹终点是此车位 */
            if (oa->traj_len > 0 && oa->traj_idx < oa->traj_len) {
                TrajectoryPoint ep = oa->trajectory[oa->traj_len - 1];
                if (ep.x == PARK_POS[i].x && ep.y == PARK_POS[i].y) {
                    occupied = 1;
                    break;
                }
            }
        }
        if (occupied) continue;
        int dist = map_manhattan(a->pos.x, a->pos.y, PARK_POS[i].x, PARK_POS[i].y);
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }
    return best;
}

/* 沿轨迹移动一步 */
static void agv_move_along_traj(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    if (a->traj_len == 0) return;

    /* 找到当前时间对应的轨迹点 */
    int next_t = a->cur_t + 1;
    while (a->traj_idx < a->traj_len && a->trajectory[a->traj_idx].t < next_t) {
        a->traj_idx++;
    }

    if (a->traj_idx >= a->traj_len) return;

    TrajectoryPoint *tp = &a->trajectory[a->traj_idx];
    if (tp->t == next_t) {
        /* 运行时安全校验: 目标格已被其他AGV占据则原地等待 */
        int j;
        for (j = 0; j < s->agv_cnt; j++) {
            if (j == id) continue;
            if (s->agvs[j].pos.x == tp->x && s->agvs[j].pos.y == tp->y) {
                return; /* 等待一 tick */
            }
        }
        a->pos.x = tp->x;
        a->pos.y = tp->y;
        a->cur_t = next_t;
    }
    /* 如果tp->t > next_t, 说明需要等待(原地不动) */
}

/* 从缓冲区装载货物(使用背包拼单) */
static void agv_load_from_buffer(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    Buffer *b = &s->buffers[a->buffer_target];

    int selected[32], sel_cnt;
    knapsack_batch(s, a->buffer_target, selected, &sel_cnt);

    if (sel_cnt == 0) {
        /* 缓冲区空了, 释放AGV */
        b->agv_assigned = -1;
        a->busy = 0;
        a->status = AGV_RETURNING;
        a->buffer_target = -1;
        a->traj_len = 0;
        a->traj_idx = 0;
        return;
    }

    int loaded = 0;
    int si;
    for (si = 0; si < sel_cnt; si++) {
        int item_id = selected[si];
        Item *it = &s->items[item_id];
        double vol = VOL_VAL[it->volume];

        if (a->load_vol + vol > 1.001) continue;
        if (a->item_cnt >= MAX_ITEMS_PER_AGV) break;

        /* 从缓冲区移除该货物 */
        int pos = -1;
        int pi;
        for (pi = 0; pi < b->item_cnt; pi++) {
            if (b->items[pi] == item_id) { pos = pi; break; }
        }
        if (pos < 0) continue;

        memmove(&b->items[pos], &b->items[pos + 1],
                (b->item_cnt - pos - 1) * sizeof(int));
        b->item_cnt--;

        a->items[a->item_cnt++] = item_id;
        a->load_vol += vol;
        it->state = ITEM_ON_AGV;
        it->location_id = a->id;
        loaded = 1;
    }

    if (loaded) {
        b->agv_assigned = -1;
        a->transfer_target = select_best_transfer(s, id);
        a->status = AGV_MOVING_TO_TRANSFER;
        a->traj_len = 0;
        a->traj_idx = 0;
    } else {
        /* 没装上货, 返回停车位 */
        a->busy = 0;
        a->buffer_target = -1;
        a->traj_len = 0;
        a->traj_idx = 0;
        a->status = AGV_RETURNING;
    }
}

/* 检查货物是否匹配交接区的机器人管辖范围 */
static int item_matches_tz(Item *it, int tz_id) {
    int shelf = it->target_shelf;
    if (tz_id < 2) return (shelf == 0 || shelf == 1);  /* Robot 0: shelf 0,1 */
    return (shelf == 2 || shelf == 3);                  /* Robot 1: shelf 2,3 */
}

/* 向交接区卸载货物(仅卸匹配的货物) */
static void agv_unload_to_transfer(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    TransferZone *tz = &s->tzones[a->transfer_target];
    int unloaded = 0;

    int i = 0;
    while (i < a->item_cnt) {
        int item_id = a->items[i];
        Item *it = &s->items[item_id];

        /* 跳过不匹配本交接区的货物 */
        if (!item_matches_tz(it, a->transfer_target)) {
            i++;
            continue;
        }

        double vol = VOL_VAL[it->volume];
        if (tz->total_vol + vol > 1.001) break;

        /* 从AGV移除货物i */
        memmove(&a->items[i], &a->items[i + 1],
                (a->item_cnt - i - 1) * sizeof(int));
        a->item_cnt--;
        a->load_vol -= vol;

        tz->items[tz->item_cnt++] = item_id;
        tz->total_vol += vol;
        it->state = ITEM_AT_TRANSFER;
        it->location_id = tz->id;
        unloaded = 1;
        /* 不递增i, 因为下一元素移到了当前位置 */
    }

    if (a->item_cnt > 0) {
        /* 还有货物: 检查是否属于另一个交接区 */
        int first_item = a->items[0];
        Item *it = &s->items[first_item];
        if (!item_matches_tz(it, a->transfer_target)) {
            /* 剩余货物不匹配当前区, 换目标交接区 */
            a->transfer_target = select_best_transfer(s, id);
            a->status = AGV_MOVING_TO_TRANSFER;
            a->traj_len = 0; a->traj_idx = 0;
            a->busy = 1;
        } else if (!unloaded) {
            /* 匹配但交接区满, 等待 */
            a->status = AGV_WAITING;
            a->wait_ticks = 0;
            a->busy = 0;
        } else {
            /* 卸了一部分, 继续等待卸货 */
            a->status = AGV_WAITING;
            a->wait_ticks = 0;
            a->busy = 0;
        }
    } else {
        a->busy = 0;
        a->status = AGV_RETURNING;
        a->buffer_target = -1;
        a->transfer_target = -1;
        a->traj_len = 0;
        a->traj_idx = 0;
    }
}

void agv_step(SimState *s, int id) {
    AGV *a = &s->agvs[id];

    a->cur_t = (int)s->time;

    switch (a->status) {
    case AGV_IDLE:
        /* 空闲时留在停车位, 如果不在停车位则返回 */
        {
            int at_park = 0;
            int i;
            for (i = 0; i < MAX_AGVS; i++) {
                if (a->pos.x == PARK_POS[i].x && a->pos.y == PARK_POS[i].y) {
                    at_park = 1;
                    break;
                }
            }
            if (!at_park) {
                a->status = AGV_RETURNING;
                a->busy = 0;
            }
        }
        if (a->traj_len > 0 && a->traj_idx < a->traj_len) {
            agv_move_along_traj(s, id);
            if (a->traj_idx >= a->traj_len) {
                a->traj_len = 0;
                a->traj_idx = 0;
                a->busy = 0;
            }
        }
        break;

    case AGV_MOVING_TO_BUFFER:
        if (a->traj_len == 0 || a->traj_idx >= a->traj_len) {
            Buffer *b = &s->buffers[a->buffer_target];
            plan_agv_to_target(s, id, b->pos.x, b->pos.y);
        }
        if (a->traj_len > 0)
            agv_move_along_traj(s, id);
        {
            Buffer *b = &s->buffers[a->buffer_target];
            if (a->pos.x == b->pos.x && a->pos.y == b->pos.y) {
                a->status = AGV_LOADING;
                a->traj_len = 0; a->traj_idx = 0;
            }
        }
        break;

    case AGV_LOADING:
        agv_load_from_buffer(s, id);
        break;

    case AGV_MOVING_TO_TRANSFER:
        if (a->traj_len == 0 || a->traj_idx >= a->traj_len) {
            TransferZone *tz = &s->tzones[a->transfer_target];
            plan_agv_to_target(s, id, tz->pos.x, tz->pos.y);
        }
        if (a->traj_len > 0)
            agv_move_along_traj(s, id);
        {
            TransferZone *tz = &s->tzones[a->transfer_target];
            if (a->pos.x == tz->pos.x && a->pos.y == tz->pos.y) {
                if (tz->total_vol < 0.999) {
                    a->status = AGV_UNLOADING;
                } else {
                    a->status = AGV_WAITING;
                    a->wait_ticks = 0;
                }
                a->traj_len = 0; a->traj_idx = 0;
            }
        }
        break;

    case AGV_WAITING:
        /* 等待交接区空闲 */
        {
            TransferZone *tz = &s->tzones[a->transfer_target];
            if (tz->total_vol < 0.999) {
                a->status = AGV_UNLOADING;
            } else {
                a->wait_ticks++;
            }
        }
        break;

    case AGV_UNLOADING:
        agv_unload_to_transfer(s, id);
        break;

    case AGV_RETURNING:
        if (a->traj_len == 0 || a->traj_idx >= a->traj_len) {
            int park_idx = find_nearest_park(s, id);
            Pos2D park = PARK_POS[park_idx];
            plan_agv_to_target(s, id, park.x, park.y);
        }
        if (a->traj_len > 0)
            agv_move_along_traj(s, id);
        {
            int i;
            for (i = 0; i < MAX_AGVS; i++) {
                if (a->pos.x == PARK_POS[i].x && a->pos.y == PARK_POS[i].y) {
                    a->status = AGV_IDLE;
                    a->busy = 0;
                    a->traj_len = 0;
                    a->traj_idx = 0;
                    break;
                }
            }
        }
        break;

    default:
        break;
    }
}
