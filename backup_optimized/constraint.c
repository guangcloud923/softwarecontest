#include "constraint.h"

/*
 * 在关键节点执行约束校验:
 * 1. AGV负载不超过1m3
 * 2. 缓冲区不超过容量
 * 3. 交接区不超过容量
 * 4. 货架盘位不冲突
 * 5. 状态一致性: 每个货物只能在一个地方
 * 6. 机器人相同体积约束
 */

static void check_agv_load(SimState *s) {
    int i, j;
    for (i = 0; i < s->agv_cnt; i++) {
        AGV *a = &s->agvs[i];
        double calc = 0.0;
        for (j = 0; j < a->cargo_cnt; j++) {
            int ci = a->cargo_ids[j];
            calc += VOL_VAL[s->cargos[ci].volume];
        }
        if (calc > 1.001) {
            s->violations++;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "VIOLATION: AGV%d load %.2f > 1.0", i, calc);
            s->running = 0;
        }
        if (a->load_vol != calc) {
            s->violations++;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "VIOLATION: AGV%d load_vol %.2f != calculated %.2f",
                     i, a->load_vol, calc);
            s->running = 0;
        }
    }
}

static void check_buffer_capacity(SimState *s) {
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        Buffer *b = &s->buffers[i];
        if (b->cargo_cnt > b->capacity) {
            s->violations++;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "VIOLATION: Buffer%d overflow %d/%d",
                     i, b->cargo_cnt, b->capacity);
            s->running = 0;
        }
    }
}

static void check_transfer_capacity(SimState *s) {
    int i;
    for (i = 0; i < s->tp_cnt; i++) {
        TransferPoint *tp = &s->tps[i];
        if (tp->cargo_cnt > tp->capacity) {
            s->violations++;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "VIOLATION: TP%d overflow %d/%d",
                     i, tp->cargo_cnt, tp->capacity);
            s->running = 0;
        }
    }
}

static void check_shelf_consistency(SimState *s) {
    int i, j;
    for (i = 0; i < s->shelf_cnt; i++) {
        Shelf *sh = &s->shelves[i];
        for (j = 0; j < sh->slot_cnt; j++) {
            if (!sh->slots[j].occupied) continue;
            int ci = sh->slots[j].cargo_id;
            if (ci < 0 || ci >= s->cargo_cnt) continue;
            if (s->cargos[ci].state != CS_ON_SHELF) {
                s->violations++;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "VIOLATION: Shelf%d slot%d cargo%d state mismatch",
                         i, j, ci);
                s->running = 0;
            }
            if (s->cargos[ci].target_shelf_id != i) {
                s->violations++;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "VIOLATION: cargo%d on wrong shelf %d (target %d)",
                         ci, i, s->cargos[ci].target_shelf_id);
                s->running = 0;
            }
        }
    }
}

/*
 * 状态一致性: 检查每个货物出现在且只出现在一个地方
 * 这里做轻量级检查: 每个state对应的location_id合理
 */
static void check_state_consistency(SimState *s) {
    int i;
    for (i = 0; i < s->cargo_cnt; i++) {
        Cargo *cg = &s->cargos[i];
        if (cg->id < 0) continue;

        switch (cg->state) {
        case CS_INIT:
            break;
        case CS_ON_CONVEYOR: {
            int cv_id = cg->location_id;
            if (cv_id < 0 || cv_id >= s->conveyor_cnt) goto bad;
            int found = 0, j;
            for (j = 0; j < s->conveyors[cv_id].cargo_cnt; j++) {
                if (s->conveyors[cv_id].cargo_ids[j] == i) { found = 1; break; }
            }
            if (!found) goto bad;
            break;
        }
        case CS_IN_BUFFER: {
            int b_id = cg->location_id;
            if (b_id < 0 || b_id >= s->buffer_cnt) goto bad;
            int found = 0, j;
            for (j = 0; j < s->buffers[b_id].cargo_cnt; j++) {
                if (s->buffers[b_id].cargo_ids[j] == i) { found = 1; break; }
            }
            if (!found) goto bad;
            break;
        }
        case CS_ON_AGV: {
            int a_id = cg->location_id;
            if (a_id < 0 || a_id >= s->agv_cnt) goto bad;
            int found = 0, j;
            for (j = 0; j < s->agvs[a_id].cargo_cnt; j++) {
                if (s->agvs[a_id].cargo_ids[j] == i) { found = 1; break; }
            }
            if (!found) goto bad;
            break;
        }
        case CS_AT_TRANSFER: {
            int t_id = cg->location_id;
            if (t_id >= 100) {
                /* 被机器人持有 */
                int r_id = t_id - 100;
                if (r_id < 0 || r_id >= s->robot_cnt) goto bad;
                int found = 0, j;
                for (j = 0; j < s->robots[r_id].cargo_cnt; j++) {
                    if (s->robots[r_id].cargo_ids[j] == i) { found = 1; break; }
                }
                if (!found) goto bad;
            } else {
                if (t_id < 0 || t_id >= s->tp_cnt) goto bad;
                int found = 0, j;
                for (j = 0; j < s->tps[t_id].cargo_cnt; j++) {
                    if (s->tps[t_id].cargo_ids[j] == i) { found = 1; break; }
                }
                if (!found) goto bad;
            }
            break;
        }
        case CS_ON_SHELF:
            break;
        default:
            goto bad;
        }
        continue;
    bad:
        s->violations++;
        snprintf(s->violation_msg, sizeof(s->violation_msg),
                 "VIOLATION: cargo%d state=0x%x loc=%d inconsistent",
                 i, cg->state, cg->location_id);
        s->running = 0;
        return;
    }
}

static void check_robot_constraint(SimState *s) {
    int i;
    for (i = 0; i < s->robot_cnt; i++) {
        ShelfRobot *r = &s->robots[i];
        if (r->cargo_cnt <= 1) continue;

        /* 检查体积一致性 */
        CargoVolume v0 = s->cargos[r->cargo_ids[0]].volume;
        int j;
        for (j = 1; j < r->cargo_cnt; j++) {
            if (s->cargos[r->cargo_ids[j]].volume != v0) {
                s->violations++;
                snprintf(s->violation_msg, sizeof(s->violation_msg),
                         "VIOLATION: Robot%d mixed volumes", i);
                s->running = 0;
                return;
            }
        }

        if (r->load_vol > 1.001) {
            s->violations++;
            snprintf(s->violation_msg, sizeof(s->violation_msg),
                     "VIOLATION: Robot%d overload %.2f", i, r->load_vol);
            s->running = 0;
        }
    }
}

void constraint_check_all(SimState *s) {
    if (!s->running) return;

    check_agv_load(s);
    if (!s->running) return;

    check_buffer_capacity(s);
    if (!s->running) return;

    check_transfer_capacity(s);
    if (!s->running) return;

    check_shelf_consistency(s);
    if (!s->running) return;

    check_state_consistency(s);
    if (!s->running) return;

    check_robot_constraint(s);
}
