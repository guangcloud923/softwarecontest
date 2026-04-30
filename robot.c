#include "robot.h"

void robot_init(SimState *s) {
    s->robot_cnt = MAX_ROBOTS;
    int i;

    for (i = 0; i < s->robot_cnt; i++) {
        ShelfRobot *r = &s->robots[i];
        r->id = i;
        r->status = ROBOT_IDLE;
        if (i == 0) {
            r->shelf_ids[0] = 0; r->shelf_ids[1] = 1;
            r->tp_id = 0;
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

static int find_suitable_item(SimState *s, ShelfRobot *r, TransferZone *tz) {
    int i, j;
    for (i = 0; i < tz->item_cnt; i++) {
        int item_id = tz->items[i];
        Item *it = &s->items[item_id];

        int ok = 0;
        for (j = 0; j < 2; j++) {
            if (r->shelf_ids[j] == it->target_shelf) { ok = 1; break; }
        }
        if (!ok) continue;

        if (r->item_cnt > 0) {
            Item *first = &s->items[r->items[0]];
            if (first->volume != it->volume) continue;
        }

        double vol = VOL_VAL[it->volume];
        if (r->load_vol + vol > 1.001) continue;

        return i;
    }
    return -1;
}

static void try_other_tzone(SimState *s, ShelfRobot *r) {
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
        r->wait_ticks = 1;
        r->busy = 1;
        break;
    }

    case ROBOT_FETCHING:
        r->wait_ticks--;
        if (r->wait_ticks > 0) break;

        {
            int pos = find_suitable_item(s, r, tz);
            if (pos < 0) {
                try_other_tzone(s, r);
                tz = &s->tzones[r->tp_id];
                pos = find_suitable_item(s, r, tz);
                if (pos < 0) {
                    if (r->item_cnt > 0) {
                        r->status = ROBOT_SHELVING;
                        r->wait_ticks = 1;
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

            memmove(&tz->items[pos], &tz->items[pos + 1],
                    (tz->item_cnt - pos - 1) * sizeof(int));
            tz->item_cnt--;
            tz->total_vol -= vol;

            r->items[r->item_cnt++] = item_id;
            r->load_vol += vol;
            it->state = ITEM_AT_TRANSFER;
            it->location_id = 100 + r->id;

            if (r->load_vol < 0.99 &&
                find_suitable_item(s, r, tz) >= 0) {
                r->wait_ticks = 1;
            } else {
                r->status = ROBOT_SHELVING;
                r->wait_ticks = 1;
            }
        }
        break;

    case ROBOT_SHELVING:
        r->wait_ticks--;
        if (r->wait_ticks > 0) break;

        while (r->item_cnt > 0) {
            int item_id = r->items[0];
            Item *it = &s->items[item_id];
            int shelf_id = it->target_shelf;
            Shelf *sh = &s->shelves[shelf_id];

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
