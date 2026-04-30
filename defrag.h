#ifndef DEFRAG_H
#define DEFRAG_H

#include "datatypes.h"

/* 货架碎片整理: 将浅层(行0-1)的小货物搬至深层(行2+), 释放连续大空间 */
void defrag_tick(SimState *s);

#endif
