#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "datatypes.h"

/* 在所有关键节点执行约束检查, 违规则终止仿真并记录 */
void constraint_check_all(SimState *s);

#endif
