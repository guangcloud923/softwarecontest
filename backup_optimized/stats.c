#include "stats.h"

SimScore sim_compute_score(SimState *s) {
    SimScore sc;
    memset(&sc, 0, sizeof(sc));

    sc.total_time   = s->time;
    sc.total_cargo  = s->cargo_total;
    sc.shelved_cargo = s->cargo_shelved;
    sc.violations   = s->violations;
    sc.collision    = s->collision_flag;

    /* 吞吐率 */
    if (sc.total_time > 0) {
        sc.throughput = (double)sc.shelved_cargo / sc.total_time;
    }

    /*
     * 评分公式:
     *   base = (shelved / total) * 100    -- 上架率
     *   time_bonus = max(0, 20 - total_time / 100)  -- 时间奖励(快则加分)
     *   violation_penalty = violations * 50          -- 每次违规扣50分
     *   collision_penalty = collision ? 100 : 0     -- 碰撞直接扣100
     *   score = max(0, base + time_bonus - violation_penalty - collision_penalty)
     */
    double base = sc.total_cargo > 0 ?
                  ((double)sc.shelved_cargo / sc.total_cargo) * 100.0 : 0.0;

    double time_bonus = 200.0 - sc.total_time * 0.5;
    if (time_bonus < 0) time_bonus = 0;

    double violation_penalty = sc.violations * 50.0;
    double collision_penalty = sc.collision ? 100.0 : 0.0;

    sc.score = base + time_bonus - violation_penalty - collision_penalty;
    if (sc.score < 0) sc.score = 0;

    return sc;
}

void sim_print_score(SimState *s) {
    SimScore sc = sim_compute_score(s);

    printf("\n========================================\n");
    printf("       仿真统计与评分\n");
    printf("========================================\n");
    printf(" 总仿真时间:      %.0f ticks\n", sc.total_time);
    printf(" 总货物:          %d\n", sc.total_cargo);
    printf(" 已上架:          %d\n", sc.shelved_cargo);
    printf(" 上架率:          %.1f%%\n",
           sc.total_cargo > 0 ? 100.0 * sc.shelved_cargo / sc.total_cargo : 0.0);
    printf(" 吞吐率:          %.3f cargo/tick\n", sc.throughput);
    printf(" 约束违规:        %d\n", sc.violations);
    printf(" 碰撞:            %s\n", sc.collision ? "发生碰撞!" : "无");
    printf(" 最终得分:        %.1f\n", sc.score);

    if (s->violation_msg[0]) {
        printf(" 最后错误信息:    %s\n", s->violation_msg);
    }
    printf("========================================\n");
}
