#ifndef ROBOT_H
#define ROBOT_H

#include "datatypes.h"

void robot_init(SimState *s);
void robot_step(SimState *s, int id);

#endif
