#ifndef STATS_H
#define STATS_H

#include "datatypes.h"

void stats_init(SimState *s);
void stats_update(SimState *s);
void stats_print(SimState *s);

#endif
