#include "conveyor.h"

void conveyor_init(SimState *s) {
    int i;
    s->conveyor_cnt = 2;

    for (i = 0; i < s->conveyor_cnt; i++) {
        s->conveyors[i].id = i;
        s->conveyors[i].cargo_cnt = 0;
        s->conveyors[i].capacity = 10;
        s->conveyors[i].paused = 0;
        s->conveyors[i].output_buffer_id = i;       /* C1->B1, C2->B2 */
        s->conveyors[i].node_id = (i == 0) ? 0 : 1; /* C1->node0, C2->node1 */
        s->conveyors[i].spawn_timer = 0;
        s->conveyors[i].spawn_interval = 3;          /* 每3 tick生成一个货物 */
    }
}

/* 传送带步骤: 生成新货物 + 推向缓冲区 */
void conveyor_step(SimState *s, int id) {
    Conveyor *c = &s->conveyors[id];

    if (c->paused) return;

    c->spawn_timer++;
    if (c->spawn_timer >= c->spawn_interval) {
        c->spawn_timer = 0;
        /* 找空闲cargo槽位(只挑属于本传送带分支的货物) */
        int ci;
        for (ci = 0; ci < MAX_CARGOS; ci++) {
            if (s->cargos[ci].state == CS_INIT && s->cargos[ci].id >= 0) {
                int tgt = s->cargos[ci].target_shelf_id;
                int correct = (tgt == 0 || tgt == 2) ? 0 : 1;
                if (correct == id) break;
            }
        }
        if (ci >= MAX_CARGOS) return; /* 无空闲货物 */

        /* 限制传送带货物数量 */
        if (c->cargo_cnt >= c->capacity) {
            c->paused = 1;
            return;
        }

        /* 将货物放到传送带 */
        s->cargos[ci].state = CS_ON_CONVEYOR;
        s->cargos[ci].location_id = id;
        c->cargo_ids[c->cargo_cnt++] = ci;
    }

    /* 尝试推向缓冲区 */
    Buffer *b = &s->buffers[c->output_buffer_id];
    while (c->cargo_cnt > 0 && b->cargo_cnt < b->capacity) {
        int ci = c->cargo_ids[0];
        /* 从传送带移除 */
        memmove(&c->cargo_ids[0], &c->cargo_ids[1],
                (c->cargo_cnt - 1) * sizeof(int));
        c->cargo_cnt--;

        /* 放入缓冲区 */
        s->cargos[ci].state = CS_IN_BUFFER;
        s->cargos[ci].location_id = b->id;
        b->cargo_ids[b->cargo_cnt++] = ci;

        if (c->paused && c->cargo_cnt < c->capacity) {
            c->paused = 0;
        }
    }
}
