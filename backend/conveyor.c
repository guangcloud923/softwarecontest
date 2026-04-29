#include "conveyor.h"

void conveyor_init(SimState *s) {
    s->conveyor_cnt = MAX_CONVEYORS;

    /* C1 在 (3,0), C2 在 (12,0) */
    Pos2D ends[] = {{3, 0}, {12, 0}};
    int i;

    for (i = 0; i < s->conveyor_cnt; i++) {
        Conveyor *c = &s->conveyors[i];
        c->id = i;
        c->end_pos = ends[i];
        c->output_buffer_id = i;
        c->item_cnt = 0;
        c->paused = 0;
        c->spawn_timer = 0;
        c->spawn_interval = 4;  /* 每4 tick出一个货 */
    }
}

void conveyor_step(SimState *s, int id) {
    Conveyor *c = &s->conveyors[id];
    Buffer *b = &s->buffers[c->output_buffer_id];

    if (c->paused) return;

    /* 生成新货物 */
    c->spawn_timer++;
    if (c->spawn_timer >= c->spawn_interval) {
        c->spawn_timer = 0;

        /* 查找待生成的货物 (状态=INIT, 属于本传送带分支) */
        int i, found = -1;
        for (i = 0; i < s->item_cnt; i++) {
            if (s->items[i].state != ITEM_INIT) continue;
            int tgt = s->items[i].target_shelf;
            /* 传送带0处理货架0,2; 传送带1处理货架1,3 */
            int belt = (tgt == 0 || tgt == 2) ? 0 : 1;
            if (belt == id) { found = i; break; }
        }

        if (found >= 0) {
            /* 传送带满则不下发, 等下一轮 */
            if (c->item_cnt >= 10) {
                c->spawn_timer = c->spawn_interval - 1;
            } else {
                s->items[found].state = ITEM_ON_CONVEYOR;
                s->items[found].location_id = id;
                s->items[found].t_spawn = s->time;
                c->items[c->item_cnt++] = found;
            }
        }
    }

    /* 推向缓冲区 (硬限制: 6, 防止capacity被意外改写) */
    while (c->item_cnt > 0 && b->item_cnt < 6) {
        int item_id = c->items[0];
        memmove(&c->items[0], &c->items[1],
                (c->item_cnt - 1) * sizeof(int));
        c->item_cnt--;

        s->items[item_id].state = ITEM_IN_BUFFER;
        s->items[item_id].location_id = b->id;
        b->items[b->item_cnt++] = item_id;
    }
}
