#include "stats.h"
#include <math.h>

/*
 * 评分公式 (PRD 1.3):
 *   Sn = D * 3000 / (T + T0)
 *
 * D  = 难度系数 (基于货物数量)
 * T  = 完成时间 (秒)
 * T0 = 基准时间偏移 (60秒)
 *
 * 碰撞/违规 → 直接0分
 */

void stats_init(SimState *s) {
    s->score = 0.0;
    s->difficulty = 1.0;
    s->collision_flag = 0;
    s->violations = 0;
    s->violation_msg[0] = '\0';
}

void stats_update(SimState *s) {
    /* 难度系数: 货物数 / 10 (最小1.0) */
    s->difficulty = (double)s->items_total / 10.0;
    if (s->difficulty < 1.0) s->difficulty = 1.0;

    /* 违规或碰撞 → 0分 */
    if (s->violations > 0 || s->collision_flag) {
        s->score = 0.0;
        return;
    }

    /* 计算得分 */
    double D = s->difficulty;
    double T = s->time;  /* ticks → 秒 (1 tick = 1 秒) */
    double T0 = 60.0;    /* 基准偏移 */

    s->score = D * 3000.0 / (T + T0);
}

void stats_print(SimState *s) {
    stats_update(s);

    printf("\n========================================\n");
    printf("         仿真统计与评分\n");
    printf("========================================\n");
    printf(" 难度系数 (D):     %.1f\n", s->difficulty);
    printf(" 总仿真时间 (T):   %.0f 秒\n", s->time);
    printf(" 总货物:           %d\n", s->items_total);
    printf(" 已上架:           %d\n", s->items_shelved);
    printf(" 上架率:           %.1f%%\n",
           s->items_total > 0 ? 100.0 * s->items_shelved / s->items_total : 0.0);
    printf(" 约束违规:         %d\n", s->violations);
    printf(" 碰撞:             %s\n", s->collision_flag ? "发生碰撞!" : "无");
    printf(" 最终得分 (Sn):    %.1f\n", s->score);

    if (s->violation_msg[0]) {
        printf(" 终止原因:         %s\n", s->violation_msg);
    }
    printf("========================================\n");
}
