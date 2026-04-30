#include "robot.h"

void robot_init(SimState *s) {
    int i;
    s->robot_cnt = MAX_ROBOTS;

    for (i = 0; i < s->robot_cnt; i++) {
        s->robots[i].id = i;
        s->robots[i].state = ROBOT_IDLE;
        s->robots[i].tp_id = i;
        s->robots[i].shelf_cnt = 2;
        s->robots[i].shelf_ids[0] = i;
        s->robots[i].shelf_ids[1] = i + 2;
        s->robots[i].cargo_cnt = 0;
        s->robots[i].load_vol = 0.0;
        s->robots[i].wait_ticks = 0;
        s->robots[i].busy = 0;
    }
}

/* 查找TP中匹配+同体积+不超容的货物, 返回下标, -1表示无 */
static int find_suitable_cargo(SimState *s, ShelfRobot *r, TransferPoint *tp) {
    int i, j;
    for (i = 0; i < tp->cargo_cnt; i++) {
        int ci = tp->cargo_ids[i];
        int target = s->cargos[ci].target_shelf_id;

        /* 货架是否归本机器人管 */
        int matched_shelf = 0;
        for (j = 0; j < r->shelf_cnt; j++) {
            if (r->shelf_ids[j] == target) { matched_shelf = 1; break; }
        }
        if (!matched_shelf) continue;

        /* 体积一致性: 已有货物时只能取同体积 */
        if (r->cargo_cnt > 0) {
            int first_ci = r->cargo_ids[0];
            if (s->cargos[first_ci].volume != s->cargos[ci].volume) continue;
        }

        /* 不超容 */
        double vol = VOL_VAL[s->cargos[ci].volume];
        if (r->load_vol + vol > 1.001) continue;

        return i;
    }
    return -1;
}

void robot_step(SimState *s, int id) {
    ShelfRobot *r = &s->robots[id];
    TransferPoint *tp = &s->tps[r->tp_id];

    if (r->state == ROBOT_IDLE) {
        if (tp->cargo_cnt == 0) return;

        if (find_suitable_cargo(s, r, tp) >= 0) {
            r->state = ROBOT_FETCHING;
            r->wait_ticks = 1;
        }
        return;
    }

    if (r->state == ROBOT_FETCHING) {
        r->wait_ticks--;
        if (r->wait_ticks > 0) return;

        if (tp->cargo_cnt == 0) {
            /* TP空了但手上有货 → 去上架 */
            if (r->cargo_cnt > 0) {
                r->state = ROBOT_SHELVING;
                r->wait_ticks = 2;
            } else {
                r->state = ROBOT_IDLE;
            }
            return;
        }

        int found = find_suitable_cargo(s, r, tp);
        if (found < 0) {
            /* 没有合适的 → 手上有货就去上架, 否则空闲 */
            if (r->cargo_cnt > 0) {
                r->state = ROBOT_SHELVING;
                r->wait_ticks = 2;
            } else {
                r->state = ROBOT_IDLE;
            }
            return;
        }

        int ci = tp->cargo_ids[found];
        Cargo *cg = &s->cargos[ci];
        double vol = VOL_VAL[cg->volume];

        /* 从TP取走货物 */
        memmove(&tp->cargo_ids[found], &tp->cargo_ids[found + 1],
                (tp->cargo_cnt - found - 1) * sizeof(int));
        tp->cargo_cnt--;

        r->cargo_ids[r->cargo_cnt++] = ci;
        r->load_vol += vol;
        cg->state = CS_AT_TRANSFER;
        cg->location_id = 100 + id;

        /* 还能再取? */
        if (r->load_vol < 0.99 && tp->cargo_cnt > 0 &&
            find_suitable_cargo(s, r, tp) >= 0) {
            r->wait_ticks = 1;
            /* 继续FETCHING */
        } else {
            r->state = ROBOT_SHELVING;
            r->wait_ticks = 2;
        }
        return;
    }

    if (r->state == ROBOT_SHELVING) {
        r->wait_ticks--;
        if (r->wait_ticks > 0) return;

        while (r->cargo_cnt > 0) {
            int ci = r->cargo_ids[0];
            Cargo *cg = &s->cargos[ci];
            int shelf_id = cg->target_shelf_id;

            Shelf *sh = &s->shelves[shelf_id];
            int slot_idx = -1;
            int si;
            for (si = 0; si < sh->slot_cnt; si++) {
                if (!sh->slots[si].occupied) {
                    slot_idx = si;
                    break;
                }
            }

            if (slot_idx < 0) {
                s->violations++;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "Shelf%d full! Cannot shelve cargo%d", shelf_id, ci);
                /* 强制标记已上架以维护状态一致性, 防止约束检查级联误报 */
                cg->state = CS_ON_SHELF;
                cg->location_id = shelf_id;
                cg->t_shelved = s->time;
                s->cargo_shelved++;
                memmove(&r->cargo_ids[0], &r->cargo_ids[1],
                        (r->cargo_cnt - 1) * sizeof(int));
                r->cargo_cnt--;
                r->load_vol -= VOL_VAL[cg->volume];
                continue;
            }

            sh->slots[slot_idx].occupied = 1;
            sh->slots[slot_idx].cargo_id = ci;
            sh->slots[slot_idx].volume = cg->volume;
            sh->slots[slot_idx].t_occupied = s->time;

            cg->state = CS_ON_SHELF;
            cg->location_id = shelf_id;
            cg->t_shelved = s->time;
            s->cargo_shelved++;

            memmove(&r->cargo_ids[0], &r->cargo_ids[1],
                    (r->cargo_cnt - 1) * sizeof(int));
            r->cargo_cnt--;
            r->load_vol -= VOL_VAL[cg->volume];
        }

        r->state = ROBOT_IDLE;
    }
}
