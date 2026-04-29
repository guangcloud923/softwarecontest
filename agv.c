#include "agv.h"
#include "map.h"

void agv_init(SimState *s) {
    int i;
    int park_nodes[MAX_AGVS] = {12, 13, 14, 15};
    s->agv_cnt = MAX_AGVS;

    for (i = 0; i < s->agv_cnt; i++) {
        s->agvs[i].id = i;
        s->agvs[i].cur_node = park_nodes[i];
        s->agvs[i].prev_node = park_nodes[i];
        s->agvs[i].target_node = park_nodes[i];
        s->agvs[i].state = AGV_IDLE;
        s->agvs[i].load_vol = 0.0;
        s->agvs[i].cargo_cnt = 0;
        s->agvs[i].path_len = 0;
        s->agvs[i].path_idx = 0;
        s->agvs[i].wait_ticks = 0;
        s->agvs[i].buffer_assigned = -1;
        s->agvs[i].transfer_assigned = -1;
        s->agvs[i].busy = 0;
    }
}

/* 判断AGV是否可以直接取货(路径上没有其他AGV阻挡) */
/* 不检查目标节点(最后节点), 因为多个AGV需要依次到达同一个目标 */
static int can_agv_move_to(SimState *s, int agv_id, int target_node) {
    AGV *a = &s->agvs[agv_id];
    int path[MAX_NODES];
    int len = map_find_path(s, a->cur_node, target_node, path);
    if (len < 2) return 0;

    /* 检查路径上每个中间节点是否被其他AGV占用(不检查起点和终点) */
    int i, j;
    for (i = 1; i < len - 1; i++) {
        int node = path[i];
        for (j = 0; j < s->agv_cnt; j++) {
            if (j == agv_id) continue;
            if (s->agvs[j].cur_node == node) return 0;
        }
    }
    return 1;
}

/* 找最优AGV分配任务 */
static void try_dispatch_agv(SimState *s, int buf_id) {
    Buffer *b = &s->buffers[buf_id];
    if (b->cargo_cnt == 0) return;
    if (b->agv_assigned >= 0) return; /* 已有AGV前往 */

    /* 找缓冲区对应的交接区: buf0->TP1, buf1->TP2 */
    int tp_id = buf_id;

    /* 查找最优AGV: 空闲且路径通畅 */
    int best_agv = -1, best_dist = 999999;
    int i;
    for (i = 0; i < s->agv_cnt; i++) {
        if (s->agvs[i].busy) continue;
        if (s->agvs[i].state != AGV_IDLE) continue;

        /* 检查此AGV是否能到达缓冲区 */
        if (can_agv_move_to(s, i, b->node_id)) {
            int path[MAX_NODES];
            int len = map_find_path(s, s->agvs[i].cur_node, b->node_id, path);
            if (len > 0 && len < best_dist) {
                best_dist = len;
                best_agv = i;
            }
        }
    }

    if (best_agv < 0) return; /* 无可用AGV */

    /* TP容量门控: 当交接区已满且有AGV正在前往时, 不再派发新AGV */
    {
        TransferPoint *tp = &s->tps[tp_id];
        if (tp->cargo_cnt >= tp->capacity) {
            int incoming = 0;
            int k;
            for (k = 0; k < s->agv_cnt; k++) {
                if (s->agvs[k].transfer_assigned == tp_id && s->agvs[k].busy)
                    incoming++;
            }
            if (incoming > 0) return;
        }
    }

    /* 分配任务 */
    AGV *a = &s->agvs[best_agv];
    a->buffer_assigned = buf_id;
    a->transfer_assigned = tp_id;
    a->busy = 1;
    b->agv_assigned = best_agv;

    /* 规划路径到缓冲区 */
    a->path_len = map_find_path(s, a->cur_node, b->node_id, a->path);
    a->path_idx = 0;
    a->state = AGV_TO_BUFFER;

    /* printf("  AGV%d dispatched: buf%d -> node%d\n", best_agv, buf_id, b->node_id); */
}

/* 更新AGV沿路径移动一步 */
static void agv_move(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    if (a->path_len == 0) return;

    int next_idx = a->path_idx + 1;
    if (next_idx >= a->path_len) return;

    int next_node = a->path[next_idx];

    /* 避让检测: 下一节点是否被其他AGV占用 */
    int j;
    for (j = 0; j < s->agv_cnt; j++) {
        if (j == id) continue;
        if (s->agvs[j].cur_node != next_node) continue;

        /* 发现阻塞 */
        AGV *ba = &s->agvs[j];

        /* 情况1: 对面AGV也在往我这里走 → 正常交换 */
        if (ba->path_len > ba->path_idx + 1) {
            int their_next = ba->path[ba->path_idx + 1];
            if (their_next == a->cur_node) {
                ba->prev_node = ba->cur_node;
                ba->cur_node = their_next;
                ba->path_idx++;
                a->prev_node = a->cur_node;
                a->cur_node = next_node;
                a->path_idx = next_idx;
                return;
            }
        }

        /* 情况2: 对面AGV空闲(IDLE) → 无论在哪都让路, 回停车位一步 */
        if (ba->state == AGV_IDLE && ba->cargo_cnt == 0) {
            int park_nodes[] = {12, 13, 14, 15};
            int ba_park = park_nodes[ba->id];
            int ba_path[MAX_NODES];
            int ba_plen = map_find_path(s, ba->cur_node, ba_park, ba_path);
            if (ba_plen >= 2) {
                int ba_next = ba_path[1];
                int safe = 1;
                int kj;
                for (kj = 0; kj < s->agv_cnt && safe; kj++) {
                    if (kj == ba->id) continue;
                    if (kj == id) continue; /* 主动AGV正要离开 */
                    if (s->agvs[kj].cur_node == ba_next) safe = 0;
                }
                if (safe) {
                    ba->prev_node = ba->cur_node;
                    ba->cur_node = ba_next;
                }
            }
            /* 不管idle AGV有没有移动, 主动AGV都前进 */
            a->prev_node = a->cur_node;
            a->cur_node = next_node;
            a->path_idx = next_idx;
            return;
        }
        /* 无法让路, 原地等待 */
        return;
    }

    /* 移动一步 */
    a->prev_node = a->cur_node;
    a->cur_node = next_node;
    a->path_idx = next_idx;
}

/* 向缓冲区装载货物 */
static void agv_load(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    Buffer *b = &s->buffers[a->buffer_assigned];
    if (!b) return;

    /* 尽可能多地装载 */
    int loaded = 0;
    while (b->cargo_cnt > 0 && a->load_vol < 0.99) {
        /* 找第一个适合的货物 */
        int ci = b->cargo_ids[0];
        Cargo *cg = &s->cargos[ci];

        double vol = VOL_VAL[cg->volume];
        if (a->load_vol + vol > 1.001) break; /* 超体积 */

        /* 装载 */
        memmove(&b->cargo_ids[0], &b->cargo_ids[1],
                (b->cargo_cnt - 1) * sizeof(int));
        b->cargo_cnt--;
        a->cargo_ids[a->cargo_cnt++] = ci;
        a->load_vol += vol;
        cg->state = CS_ON_AGV;
        cg->location_id = id;
        loaded = 1;
    }

    if (loaded) {
        a->state = AGV_TO_TRANSFER;
        /* 规划到交接区路径 */
        TransferPoint *tp = &s->tps[a->transfer_assigned];
        a->path_len = map_find_path(s, a->cur_node, tp->node_id, a->path);
        a->path_idx = 0;
        b->agv_assigned = -1;
    }
}

/* 向交接区卸载货物 */
static void agv_unload(SimState *s, int id) {
    AGV *a = &s->agvs[id];
    TransferPoint *tp = &s->tps[a->transfer_assigned];

    while (a->cargo_cnt > 0 && tp->cargo_cnt < tp->capacity) {
        int ci = a->cargo_ids[0];
        Cargo *cg = &s->cargos[ci];

        memmove(&a->cargo_ids[0], &a->cargo_ids[1],
                (a->cargo_cnt - 1) * sizeof(int));
        a->cargo_cnt--;
        a->load_vol -= VOL_VAL[cg->volume];

        tp->cargo_ids[tp->cargo_cnt++] = ci;
        cg->state = CS_AT_TRANSFER;
        cg->location_id = tp->id;
    }

    if (a->cargo_cnt > 0) {
        /* 未卸完(TP满), 继续等待 */
        a->state = AGV_WAIT_TRANSFER;
        a->wait_ticks = 0;
    } else {
        /* AGV回到空闲 */
        a->busy = 0;
        a->state = AGV_IDLE;
        a->buffer_assigned = -1;
        a->transfer_assigned = -1;
    }
}

void agv_dispatch(SimState *s) {
    /* 检查缓冲区是否有待处理货物, 尝试分配AGV */
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        try_dispatch_agv(s, i);
    }
}

void agv_step(SimState *s, int id) {
    AGV *a = &s->agvs[id];

    switch (a->state) {
    case AGV_IDLE: {
        /* 空闲时如果不在停车位, 自动回park (但只在有明确路径时执行) */
        static const int park_home[] = {12, 13, 14, 15};
        int home = park_home[a->id];
        if (a->cur_node != home && a->cargo_cnt == 0) {
            int path_to_home[MAX_NODES];
            int plen = map_find_path(s, a->cur_node, home, path_to_home);
            if (plen >= 2) {
                /* 检查第一步是否安全 */
                int next = path_to_home[1];
                int blocked = 0;
                int j;
                for (j = 0; j < s->agv_cnt; j++) {
                    if (j == id) continue;
                    if (s->agvs[j].cur_node == next) { blocked = 1; break; }
                }
                if (!blocked) {
                    /* 走一步回家 */
                    a->prev_node = a->cur_node;
                    a->cur_node = next;
                }
            }
        }
        break;
    }

    case AGV_TO_BUFFER:
        /* 向缓冲区移动(或回家) */
        agv_move(s, id);
        if (!s->running) return;

        if (a->buffer_assigned >= 0) {
            /* 目标: 缓冲区 */
            if (a->cur_node == s->nodes[s->buffers[a->buffer_assigned].node_id].id) {
                a->state = AGV_LOADING;
                agv_load(s, id);
                if (!s->running) return;
            }
        } else {
            /* 目标: 回家 (buffer_assigned==-1 表示回家模式) */
            if (a->cur_node == a->target_node) {
                a->state = AGV_IDLE;
                a->path_len = 0;
            }
        }
        break;

    case AGV_LOADING:
        agv_load(s, id);
        break;

    case AGV_TO_TRANSFER:
        agv_move(s, id);
        if (!s->running) return;

        /* 到达交接区? */
        if (a->cur_node == s->nodes[s->tps[a->transfer_assigned].node_id].id) {
            if (s->tps[a->transfer_assigned].cargo_cnt <
                s->tps[a->transfer_assigned].capacity) {
                a->state = AGV_UNLOADING;
                agv_unload(s, id);
            } else {
                a->state = AGV_WAIT_TRANSFER;
                a->wait_ticks++;
            }
        }
        break;

    case AGV_WAIT_TRANSFER:
        /* 等待交接区空闲 */
        if (s->tps[a->transfer_assigned].cargo_cnt <
            s->tps[a->transfer_assigned].capacity) {
            a->state = AGV_UNLOADING;
            agv_unload(s, id);
        } else {
            a->wait_ticks++;
        }
        break;

    case AGV_UNLOADING:
        agv_unload(s, id);
        break;

    case AGV_BLOCKED:
        /* 不会进入此状态(碰撞直接原地等待), 保留为安全兜底 */
        a->wait_ticks++;
        if (a->wait_ticks > 50) {
            a->wait_ticks = 0;
            a->state = AGV_IDLE;
            a->busy = 0;
        }
        break;

    default:
        break;
    }
}
