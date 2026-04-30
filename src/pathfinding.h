#ifndef PATHFINDING_H
#define PATHFINDING_H

#include "datatypes.h"

/* 为单个AGV规划时空A*路径, 返回轨迹长度(时间步) */
int  astar_find_path(SimState *s, int agv_id,
                     int sx, int sy, int st,
                     int gx, int gy,
                     TrajectoryPoint *traj_out, int max_traj);

/* CBS高层: 为所有busy AGV规划零冲突轨迹 */
int  cbs_plan_all(SimState *s);

/* 为单个AGV规划(使用reservation table避免已规划的AGV轨迹) */
int  astar_with_reservations(SimState *s, int agv_id,
                             int sx, int sy, int st,
                             int gx, int gy,
                             int resv[GRID_H][GRID_W][512],
                             TrajectoryPoint *traj_out, int max_traj);

/* 检测两条轨迹是否冲突 */
int  detect_conflict(TrajectoryPoint *a, int la,
                     TrajectoryPoint *b, int lb,
                     int *conflict_t, int *conflict_x, int *conflict_y);

/* 方便函数: 规划从当前位置到目标格子的路径 */
int  plan_agv_to_target(SimState *s, int agv_id, int gx, int gy);

/* 全局预约表: 每个tick开始前清空, AGV规划后将自己的轨迹加入 */
void pathfinding_resv_clear(void);
void pathfinding_resv_add_trajectory(TrajectoryPoint *traj, int len);
void pathfinding_resv_block_window(int x, int y, int t0, int t1);

#endif
