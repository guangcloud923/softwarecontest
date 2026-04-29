#ifndef MAP_H
#define MAP_H

#include "datatypes.h"

void map_init(SimState *s);
int  map_is_walkable(SimState *s, int x, int y);
int  map_is_crossroad(SimState *s, int x, int y);
int  map_is_narrow(SimState *s, int x, int y);
void map_get_neighbors(SimState *s, int x, int y, Pos2D *neighbors, int *cnt);
int  map_manhattan(int x1, int y1, int x2, int y2);
void map_recalc_special(SimState *s);

#endif
