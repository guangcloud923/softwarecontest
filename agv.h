#ifndef AGV_H
#define AGV_H

#include "datatypes.h"

void agv_init(SimState *s);
void agv_dispatch(SimState *s);
void agv_step(SimState *s, int id);

#endif
