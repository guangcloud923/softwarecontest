#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "datatypes.h"

/* 0-1背包拼单: 从缓冲区选最优货物组合, 最大化体积利用率+目的地邻近权重 */
int  knapsack_batch(SimState *s, int buffer_id, int *selected, int *sel_cnt);

/* 分配最优AGV到缓冲区 */
int  dispatch_best_agv(SimState *s, int buffer_id);

/* 全局调度: 检查所有缓冲区并分派AGV */
void scheduler_dispatch_all(SimState *s);

/* 为AGV选择最优交接区 */
int  select_best_transfer(SimState *s, int agv_id);

#endif
