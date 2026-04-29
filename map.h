#ifndef MAP_H
#define MAP_H

#include "datatypes.h"

/* 初始化地图拓扑 */
void map_init(SimState *s);

/* 寻路: Dijkstra, 返回路径长度, 路径写入path */
int  map_find_path(SimState *s, int start_node, int end_node, int *path_out);

/* 节点名称(调试用) */
const char *node_name(int id);

/* 两个节点是否相邻 */
int  nodes_adjacent(SimState *s, int a, int b);

#endif
