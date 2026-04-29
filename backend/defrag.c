#include "defrag.h"

/*
 * 货架碎片整理 (启发式盘位整理)
 *
 * 触发条件: 任一机器人空闲 且 对应交接区为空 且 货架存在碎片
 *
 * 策略:
 * 1. 扫描货架三维矩阵
 * 2. 找到浅层(行0-1)的 VOL_SMALL (0.25m3) 货物
 * 3. 检查深层(行2+)是否有空位
 * 4. 将浅层小货搬至深层, 释放浅层连续空间给大货(1.0m3)
 *
 * 注意: 整理操作不改变货物的target_shelf (物理位置不变, 只是盘位重组)
 */

static int can_defrag(SimState *s, int shelf_id) {
    Shelf *sh = &s->shelves[shelf_id];

    /* 检查浅层是否有小货物 */
    int has_small_shallow = 0;
    int row, col, dep;
    for (row = 0; row < 2; row++) {
        for (col = 0; col < MAX_SHELF_COLS; col++) {
            for (dep = 0; dep < MAX_SHELF_DEPTH; dep++) {
                if (sh->slots[row][col][dep].occupied &&
                    sh->slots[row][col][dep].vol == VOL_SMALL) {
                    has_small_shallow = 1;
                    break;
                }
            }
        }
    }

    if (!has_small_shallow) return 0;

    /* 检查深层是否有空位 */
    for (row = 2; row < MAX_SHELF_ROWS; row++) {
        for (col = 0; col < MAX_SHELF_COLS; col++) {
            for (dep = 0; dep < MAX_SHELF_DEPTH; dep++) {
                if (!sh->slots[row][col][dep].occupied) return 1;
            }
        }
    }

    return 0;
}

static void defrag_shelf(SimState *s, int shelf_id) {
    Shelf *sh = &s->shelves[shelf_id];
    int row, col, dep;

    for (row = 0; row < 2; row++) {
        for (col = 0; col < MAX_SHELF_COLS; col++) {
            for (dep = 0; dep < MAX_SHELF_DEPTH; dep++) {
                if (!sh->slots[row][col][dep].occupied) continue;
                if (sh->slots[row][col][dep].vol != VOL_SMALL) continue;

                /* 找深层空位 */
                int dr, dc, dd, found = 0;
                for (dr = 2; dr < MAX_SHELF_ROWS && !found; dr++) {
                    for (dc = 0; dc < MAX_SHELF_COLS && !found; dc++) {
                        for (dd = 0; dd < MAX_SHELF_DEPTH && !found; dd++) {
                            if (!sh->slots[dr][dc][dd].occupied) {
                                /* 搬运 */
                                sh->slots[dr][dc][dd] = sh->slots[row][col][dep];
                                memset(&sh->slots[row][col][dep], 0, sizeof(ShelfSlot));
                                found = 1;

                                /* 记录事件 */
                                if (s->event_cnt < MAX_EVENTS) {
                                    snprintf(s->events[s->event_cnt].msg,
                                             sizeof(s->events[s->event_cnt].msg),
                                             "[整理] 货架%d: 盘位(%d,%d,%d)→(%d,%d,%d) 释放浅层空间",
                                             shelf_id, row, col, dep, dr, dc, dd);
                                    s->events[s->event_cnt].time = s->time;
                                    s->event_cnt++;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void defrag_tick(SimState *s) {
    /* 每20 tick检查一次, 减少开销 */
    if ((int)s->time % 20 != 0) return;

    int r;
    for (r = 0; r < s->robot_cnt; r++) {
        ShelfRobot *rb = &s->robots[r];
        if (rb->status != ROBOT_IDLE) continue;

        /* 检查管辖交接区是否为空 */
        int si;
        for (si = 0; si < 2; si++) {
            int tp_id = (r == 0) ? (si * 2) : (si * 2 + 1); /* R0:T0,T2; R1:T1,T3 */
            if (s->tzones[tp_id].item_cnt > 0) continue;

            int sh_id = rb->shelf_ids[si];
            if (can_defrag(s, sh_id)) {
                defrag_shelf(s, sh_id);
            }
        }
    }
}
