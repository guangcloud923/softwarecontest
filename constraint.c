#include "constraint.h"

/*
 * 一票否决项检查 (PRD 1.1)
 * 任何违规立即终止仿真, 直接0分
 */

/* AGV负载 ≤ 1.0 m3 */
static void check_agv_load(SimState *s) {
    int i;
    for (i = 0; i < s->agv_cnt; i++) {
        AGV *a = &s->agvs[i];
        if (a->load_vol > 1.001) {
            s->violations++;
            s->running = 0;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "FATAL: AGV%d overload %.3f m3 > 1.0", i, a->load_vol);
            return;
        }
    }
}

/* 缓冲区容量 ≤ 6 */
static void check_buffer_capacity(SimState *s) {
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        if (s->buffers[i].capacity != 6) {
            fprintf(stderr, "  [BUG] T%.0f Buffer%d capacity=%d (corrupted!), resetting to 6\n",
                    s->time, i, s->buffers[i].capacity);
            s->buffers[i].capacity = 6;
        }
        if (s->buffers[i].item_cnt > 6) {
            s->violations++;
            s->running = 0;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "FATAL: Buffer%d overflow %d > 6",
                     i, s->buffers[i].item_cnt);
            return;
        }
    }
}

/* 交接区容积 ≤ capacity_vol */
static void check_transfer_volume(SimState *s) {
    int i;
    for (i = 0; i < s->tzone_cnt; i++) {
        if (s->tzones[i].total_vol > s->tzones[i].capacity_vol + 0.001) {
            s->violations++;
            s->running = 0;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "FATAL: TransferZone%d overflow %.3f m3 > %.1f",
                     i, s->tzones[i].total_vol, s->tzones[i].capacity_vol);
            return;
        }
    }
}

/* 机器人: 同体积约束 + 容量 ≤ 1.0 m3 */
static void check_robot_constraints(SimState *s) {
    int i, j;
    for (i = 0; i < s->robot_cnt; i++) {
        ShelfRobot *r = &s->robots[i];
        if (r->item_cnt <= 1) continue;

        if (r->load_vol > 1.001) {
            s->violations++;
            s->running = 0;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "FATAL: Robot%d overload %.3f m3", i, r->load_vol);
            return;
        }

        ItemVolume v0 = s->items[r->items[0]].volume;
        for (j = 1; j < r->item_cnt; j++) {
            if (s->items[r->items[j]].volume != v0) {
                s->violations++;
                s->running = 0;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "FATAL: Robot%d mixed volumes (must be same)", i);
                return;
            }
        }
    }
}

/* 物理碰撞检测: 两AGV占据同一格子 */
static void check_collision(SimState *s) {
    int i, j;
    for (i = 0; i < s->agv_cnt; i++) {
        for (j = i + 1; j < s->agv_cnt; j++) {
            if (s->agvs[i].pos.x == s->agvs[j].pos.x &&
                s->agvs[i].pos.y == s->agvs[j].pos.y) {
                s->collision_flag = 1;
                s->violations++;
                s->running = 0;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "FATAL: AGV%d and AGV%d collided at (%d,%d)",
                         i, j, s->agvs[i].pos.x, s->agvs[i].pos.y);
                return;
            }
        }
    }
}

/* 状态一致性: 每个货物只在一个位置 */
static void check_state_consistency(SimState *s) {
    int i, j, found;
    for (i = 0; i < s->item_cnt; i++) {
        Item *it = &s->items[i];
        if (it->id < 0) continue;

        switch (it->state) {
        case ITEM_ON_CONVEYOR: {
            int cid = it->location_id;
            if (cid < 0 || cid >= MAX_CONVEYORS) goto bad;
            found = 0;
            for (j = 0; j < s->conveyors[cid].item_cnt; j++)
                if (s->conveyors[cid].items[j] == i) found = 1;
            if (!found) goto bad;
            break;
        }
        case ITEM_IN_BUFFER: {
            int bid = it->location_id;
            if (bid < 0 || bid >= MAX_BUFFERS) goto bad;
            found = 0;
            for (j = 0; j < s->buffers[bid].item_cnt; j++)
                if (s->buffers[bid].items[j] == i) found = 1;
            if (!found) goto bad;
            break;
        }
        case ITEM_ON_AGV: {
            int aid = it->location_id;
            if (aid < 0 || aid >= MAX_AGVS) goto bad;
            found = 0;
            for (j = 0; j < s->agvs[aid].item_cnt; j++)
                if (s->agvs[aid].items[j] == i) found = 1;
            if (!found) goto bad;
            break;
        }
        case ITEM_AT_TRANSFER: {
            int lid = it->location_id;
            if (lid >= 100) { /* 在机器人手上 */
                int rid = lid - 100;
                if (rid < 0 || rid >= MAX_ROBOTS) goto bad;
                found = 0;
                for (j = 0; j < s->robots[rid].item_cnt; j++)
                    if (s->robots[rid].items[j] == i) found = 1;
                if (!found) goto bad;
            } else {
                if (lid < 0 || lid >= MAX_TRANSFER_ZONES) goto bad;
                found = 0;
                for (j = 0; j < s->tzones[lid].item_cnt; j++)
                    if (s->tzones[lid].items[j] == i) found = 1;
                if (!found) goto bad;
            }
            break;
        }
        case ITEM_ON_SHELF: {
            if (it->target_shelf != it->location_id) goto bad;
            break;
        }
        default: break;
        }
        continue;
    bad:
        s->violations++;
        s->running = 0;
        snprintf(s->violation_msg, sizeof(s->violation_msg),
                 "FATAL: Item%d state=%d loc=%d inconsistent",
                 i, it->state, it->location_id);
        return;
    }
}

void constraint_check_all(SimState *s) {
    if (!s->running) return;

    check_agv_load(s);          if (!s->running) return;
    check_buffer_capacity(s);   if (!s->running) return;
    check_transfer_volume(s);   if (!s->running) return;
    check_robot_constraints(s); if (!s->running) return;
    check_collision(s);         if (!s->running) return;
    check_state_consistency(s);
}
