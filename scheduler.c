#include "scheduler.h"
#include "map.h"
#include "pathfinding.h"

/*
 * 0-1 背包拼单
 * 权重: 体积 * 4 → 1, 2, 4 (对应 0.25, 0.5, 1.0)
 * 容量: 4 (对应 1.0 m3)
 * 价值: 体积权重 + 目的地邻近加成
 *   - 同货架: +3
 *   - 同侧(同一机器人管): +1
 *   - 同交接区: +2
 */

#define KNAP_CAP 4  /* 1.0 m3 * 4 */

int knapsack_batch(SimState *s, int buffer_id,
                   int *selected, int *sel_cnt) {
    Buffer *b = &s->buffers[buffer_id];
    int n = b->item_cnt;
    if (n == 0) { *sel_cnt = 0; return 0; }

    int weight[32]; /* 最多6个货物 */
    int value[32];
    int idx[32];
    int i, w;

    for (i = 0; i < n; i++) {
        Item *it = &s->items[b->items[i]];
        idx[i] = b->items[i];
        weight[i] = (int)(VOL_VAL[it->volume] * 4.0 + 0.01);
        value[i] = weight[i]; /* 基础价值 = 体积 */
    }

    /* 目的地邻近加成: 对每个货物, 如果目标货架在同一机器人管区则加分 */
    /* 货架分组: {0,2} → 机器人0, {1,3} → 机器人1 */
    /* 交接区: 0→S0, 1→S1, 2→S2, 3→S3 */
    for (i = 0; i < n; i++) {
        Item *it = &s->items[idx[i]];
        int shelf = it->target_shelf;
        /* 热门货架(浅层)优先级更高 */
        if (shelf == 0 || shelf == 2) value[i] += 1;
    }

    /* DP表: dp[i][w] = max value */
    int dp[32][KNAP_CAP + 1];
    int keep[32][KNAP_CAP + 1]; /* 是否选中第i个 */
    memset(dp, 0, sizeof(dp));
    memset(keep, 0, sizeof(keep));

    for (i = 0; i < n; i++) {
        for (w = 0; w <= KNAP_CAP; w++) {
            dp[i + 1][w] = dp[i][w];
            keep[i][w] = 0;
            if (w >= weight[i]) {
                int cand = dp[i][w - weight[i]] + value[i];
                if (cand > dp[i + 1][w]) {
                    dp[i + 1][w] = cand;
                    keep[i][w] = 1;
                }
            }
        }
    }

    /* 回溯 */
    int best_w = KNAP_CAP;
    *sel_cnt = 0;
    for (i = n - 1; i >= 0; i--) {
        if (keep[i][best_w]) {
            selected[*sel_cnt] = idx[i];
            (*sel_cnt)++;
            best_w -= weight[i];
        }
    }

    /* 反转顺序(保持原始顺序) */
    for (i = 0; i < *sel_cnt / 2; i++) {
        int tmp = selected[i];
        selected[i] = selected[*sel_cnt - 1 - i];
        selected[*sel_cnt - 1 - i] = tmp;
    }

    /* 如果背包没完全满载且有更多货, 尝试继续填充 */
    return dp[n][KNAP_CAP];
}

/* 找最优AGV派往缓冲区 */
int dispatch_best_agv(SimState *s, int buffer_id) {
    Buffer *b = &s->buffers[buffer_id];
    if (b->item_cnt == 0) return -1;
    if (b->agv_assigned >= 0) return b->agv_assigned;

    int best = -1, best_dist = 9999;
    int i;

    for (i = 0; i < s->agv_cnt; i++) {
        AGV *a = &s->agvs[i];
        if (a->busy) continue;
        if (a->status != AGV_IDLE && a->status != AGV_RETURNING) continue;

        int dist = map_manhattan(a->pos.x, a->pos.y, b->pos.x, b->pos.y);
        if (dist < best_dist) {
            best_dist = dist;
            best = i;
        }
    }

    if (best >= 0) {
        b->agv_assigned = best;
        s->agvs[best].busy = 1;
        s->agvs[best].buffer_target = buffer_id;
        s->agvs[best].status = AGV_MOVING_TO_BUFFER;
    }

    return best;
}

/* 为AGV选择最优交接区 (考虑货物目的地) */
int select_best_transfer(SimState *s, int agv_id) {
    AGV *a = &s->agvs[agv_id];
    if (a->item_cnt == 0) return a->transfer_target;

    /* 统计货物目标货架分布 */
    int shelf_votes[4] = {0, 0, 0, 0};
    int i;
    for (i = 0; i < a->item_cnt; i++) {
        Item *it = &s->items[a->items[i]];
        int sh = it->target_shelf;
        if (sh >= 0 && sh < 4) shelf_votes[sh]++;
    }

    /* 选择票数最多的货架对应的交接区 */
    int best_shelf = 0, best_votes = 0;
    for (i = 0; i < 4; i++) {
        if (shelf_votes[i] > best_votes) {
            best_votes = shelf_votes[i];
            best_shelf = i;
        }
    }

    /* 交接区ID = 货架ID (1:1映射) */
    return best_shelf;
}

void scheduler_dispatch_all(SimState *s) {
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        if (s->buffers[i].item_cnt > 0 && s->buffers[i].agv_assigned < 0) {
            dispatch_best_agv(s, i);
        }
    }
}
