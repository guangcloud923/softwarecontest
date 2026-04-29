#ifndef SIMCORE_H
#define SIMCORE_H

#include "datatypes.h"

void sim_init(SimState *s, int cargo_count, int random_seed);
void sim_run(SimState *s);
void sim_print_stats(SimState *s);

#endif
