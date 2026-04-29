#include "robot.h"

void robot_init(SimState *s) {
    s->robot_cnt = MAX_ROBOTS;
    int i;

    for (i = 0; i < s->robot_cnt; i++) {
        ShelfRobot *r = &s->robots[i];
        r->id = i;
        r->status = ROBOT_IDLE;
        /* 机器人0 管货架 0,2 (交接区 0,2) */
        /* 机器人1 管货架 1,3 (交接区 1,3) */
        if (i == 0) {
            r->shelf_ids[0] = 0; r->shelf_ids[1] = 1;
            r->tp_id = 0;  /* 优先服务交接区0 */
        } else {
            r->shelf_ids[0] = 2; r->shelf_ids[1] = 3;
            r->tp_id = 2;
        }
        r->item_cnt = 0;
        r->load_vol = 0.0;
        r->wait_ticks = 0;
        r->busy = 0;
    }
}

/* 在交接区中找合适的货物: 目标货架归本机器人管 + 同体积约束 */
static int find_suitable_item(SimState *s, ShelfRobot *r, TransferZone *tz) {
    int i, j;
    for (i = 0; i < tz->item_cnt; i++) {
        int item_id = tz->items[i];
        Item *it = &s->items[item_id];

        /* 检查目标货架是否归本机器人 */
        int ok = 0;
        for (j = 0; j < 2; j++) {
            if (r->shelf_ids[j] == it->target_shelf) { ok = 1; break; }
        }
        if (!ok) continue;

        /* 同体积约束: 如已有货物, 必须同体积 */
        if (r->item_cnt > 0) {
            Item *first = &s->items[r->items[0]];
            if (first->volume != it->volume) continue;
        }

        /* 不超载 (1.0 m3) */
        double vol = VOL_VAL[it->volume];
        if (r->load_vol + vol > 1.001) continue;

        return i;
    }
    return -1;
}

/* 服务本机器人管辖的另一个交接区 */
static void try_other_tzone(SimState *s, ShelfRobot *r) {
    /* 机器人0: 交接区0和2; 机器人1: 交接区1和3 */
    int tz_ids[2][2] = {{0, 1}, {2, 3}};
    int current = r->tp_id;
    int other = (current == tz_ids[r->id][0]) ? tz_ids[r->id][1] : tz_ids[r->id][0];

    if (s->tzones[other].item_cnt > 0) {
        if (find_suitable_item(s, r, &s->tzones[other]) >= 0) {
            r->tp_id = other;
        }
    }
}

void robot_step(SimState *s, int id) {
    ShelfRobot *r = &s->robots[id];
    TransferZone *tz = &s->tzones[r->tp_id];

    switch (r->status) {
    case ROBOT_IDLE: {
        /* 检查管辖的交接区是否有货 */
        int tz_ids[2][2] = {{0, 1}, {2, 3}};
        int found = 0;
        int ti;

        for (ti = 0; ti < 2; ti++) {
            int tid = tz_ids[r->id][ti];
            if (s->tzones[tid].item_cnt > 0 &&
                find_suitable_item(s, r, &s->tzones[tid]) >= 0) {
                r->tp_id = tid;
                tz = &s->tzones[tid];
                found = 1;
                break;
            }
        }

        if (!found) break;

        r->status = ROBOT_FETCHING;
        r->wait_ticks = 2;  /* 取货耗时 */
        r->busy = 1;
        break;
    }

    case ROBOT_FETCHING:
        r->wait_ticks--;
        if (r->wait_ticks > 0) break;

        {
            int pos = find_suitable_item(s, r, tz);
            if (pos < 0) {
                /* 当前交接区无合适货物, 试试另一个 */
                try_other_tzone(s, r);
                tz = &s->tzones[r->tp_id];
                pos = find_suitable_item(s, r, tz);
                if (pos < 0) {
                    /* 两个交接区都没有 → 手上有货就去上架, 否则空闲 */
                    if (r->item_cnt > 0) {
                        r->status = ROBOT_SHELVING;
                        r->wait_ticks = 2;
                    } else {
                        r->status = ROBOT_IDLE;
                        r->busy = 0;
                    }
                    break;
                }
            }

            int item_id = tz->items[pos];
            Item *it = &s->items[item_id];
            double vol = VOL_VAL[it->volume];

            /* 从交接区取出 */
            memmove(&tz->items[pos], &tz->items[pos + 1],
                    (tz->item_cnt - pos - 1) * sizeof(int));
            tz->item_cnt--;
            tz->total_vol -= vol;

            r->items[r->item_cnt++] = item_id;
            r->load_vol += vol;
            /* 货物在机器人手上 (仍标记为 AT_TRANSFER, location=100+robot_id) */
            it->state = ITEM_AT_TRANSFER;
            it->location_id = 100 + r->id;

            /* 继续取或去上架 */
            if (r->load_vol < 0.99 &&
                find_suitable_item(s, r, tz) >= 0) {
                r->wait_ticks = 1;
            } else {
                r->status = ROBOT_SHELVING;
                r->wait_ticks = 2;
            }
        }
        break;

    case ROBOT_SHELVING:
        r->wait_ticks--;
        if (r->wait_ticks > 0) break;

        /* 将手中所有货物上架 */
        while (r->item_cnt > 0) {
            int item_id = r->items[0];
            Item *it = &s->items[item_id];
            int shelf_id = it->target_shelf;
            Shelf *sh = &s->shelves[shelf_id];

            /* 在货架中找空位 */
            int row, col, dep;
            int placed = 0;
            for (row = 0; row < MAX_SHELF_ROWS && !placed; row++) {
                for (col = 0; col < MAX_SHELF_COLS && !placed; col++) {
                    for (dep = 0; dep < MAX_SHELF_DEPTH && !placed; dep++) {
                        if (!sh->slots[row][col][dep].occupied) {
                            sh->slots[row][col][dep].occupied = 1;
                            sh->slots[row][col][dep].item_id = item_id;
                            sh->slots[row][col][dep].vol = it->volume;
                            sh->slots[row][col][dep].t_occupied = s->time;
                            placed = 1;
                        }
                    }
                }
            }

            if (!placed) {
                /* 货架满: 违规记录但标记已上架 */
                s->violations++;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "Shelf%d full! Cannot shelve item %d", shelf_id, item_id);
            }

            it->state = ITEM_ON_SHELF;
            it->location_id = shelf_id;
            it->t_shelved = s->time;
            s->items_shelved++;
            sh->total_vol += VOL_VAL[it->volume];

            memmove(&r->items[0], &r->items[1],
                    (r->item_cnt - 1) * sizeof(int));
            r->item_cnt--;
            r->load_vol -= VOL_VAL[it->volume];
        }

        r->status = ROBOT_IDLE;
        r->busy = 0;
        break;
    }
}
