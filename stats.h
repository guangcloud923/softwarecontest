#ifndef STATS_H
#define STATS_H

#include "datatypes.h"

typedef struct {
    double total_time;
    int    total_cargo;
    int    shelved_cargo;
    double throughput;
    int    violations;
    int    collision;
    double score;
} SimScore;

SimScore sim_compute_score(SimState *s);
void     sim_print_score(SimState *s);

#endif
