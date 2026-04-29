#ifndef SIMCORE_H
#define SIMCORE_H

#include "datatypes.h"

void sim_init(SimState *s, int item_count, int random_seed);
void sim_run(SimState *s);
void sim_run_with_server(SimState *s, int ws_port);

#endif
