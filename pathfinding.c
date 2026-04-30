#include "pathfinding.h"
#include "map.h"

/* ==================== 全局预约表 ==================== */
/* 所有AGV规划路径时共享, 避免碰撞 */
#define RESV_T_MAX 512
static int g_resv[GRID_H][GRID_W][RESV_T_MAX];

void pathfinding_resv_clear(void) {
    memset(g_resv, 0, sizeof(g_resv));
}

void pathfinding_resv_add_trajectory(TrajectoryPoint *traj, int len) {
    int i;
    for (i = 0; i < len; i++) {
        int x = traj[i].x, y = traj[i].y, t = traj[i].t;
        if (x >= 0 && x < GRID_W && y >= 0 && y < GRID_H && t >= 0) {
            g_resv[y][x][t & (RESV_T_MAX - 1)] = 1;
        }
    }
}

void pathfinding_resv_block_window(int x, int y, int t0, int t1) {
    int t;
    if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return;
    if (t0 < 0) t0 = 0;
    for (t = t0; t <= t1; t++) {
        g_resv[y][x][t & (RESV_T_MAX - 1)] = 1;
    }
}

/* ==================== 二叉堆(最小堆) ==================== */
#define HEAP_CAP 65536

typedef struct {
    int x, y, t;       /* 状态 */
    int g, f;          /* 代价 */
    int parent_idx;    /* 父节点在堆中的索引 */
    int closed_idx;    /* closed列表中的索引 */
} AStarNode;

static AStarNode heap[HEAP_CAP];
static int heap_sz;

static void heap_push(AStarNode n) {
    int i = heap_sz++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].f <= n.f) break;
        heap[i] = heap[p];
        i = p;
    }
    heap[i] = n;
}

static AStarNode heap_pop(void) {
    AStarNode ret = heap[0];
    AStarNode last = heap[--heap_sz];
    int i = 0;
    while (1) {
        int l = 2 * i + 1, r = 2 * i + 2, best = i;
        if (l < heap_sz && heap[l].f < heap[best].f) best = l;
        if (r < heap_sz && heap[r].f < heap[best].f) best = r;
        if (best == i) break;
        heap[i] = heap[best];
        i = best;
    }
    heap[i] = last;
    return ret;
}

/* ==================== 方向 ==================== */
static const int DX[] = {0, 0, -1, 1, 0}; /* 0=等待 */
static const int DY[] = {-1, 1, 0, 0, 0};

/* ==================== closed set ==================== */
/* 用数组+线性扫描, 因为搜索空间不大 */
typedef struct {
    int x, y, t;
    int parent_idx;
} ClosedEntry;

static ClosedEntry closed[HEAP_CAP];
static int closed_sz;

static int in_closed(int x, int y, int t) {
    int i;
    for (i = 0; i < closed_sz; i++) {
        if (closed[i].x == x && closed[i].y == y && closed[i].t == t)
            return i;
    }
    return -1;
}

/* ==================== Spatio-temporal A* ==================== */
int astar_find_path(SimState *s, int agv_id,
                    int sx, int sy, int st,
                    int gx, int gy,
                    TrajectoryPoint *traj_out, int max_traj) {
    AGV *agv = &s->agvs[agv_id];
    int i, d, found = 0, goal_closed = -1;

    heap_sz = 0;
    closed_sz = 0;

    /* 推入起点 */
    AStarNode start;
    start.x = sx; start.y = sy; start.t = st;
    start.g = 0;
    start.f = map_manhattan(sx, sy, gx, gy);
    start.parent_idx = -1;
    start.closed_idx = -1;
    heap_push(start);

    /* 时间上限 */
    int max_t = st + 200;
    if (max_t > MAX_TIME) max_t = MAX_TIME;

    while (heap_sz > 0) {
        AStarNode cur = heap_pop();

        /* 检查是否已closed */
        if (in_closed(cur.x, cur.y, cur.t) >= 0) continue;

        /* 加入closed */
        cur.closed_idx = closed_sz;
        closed[closed_sz].x = cur.x;
        closed[closed_sz].y = cur.y;
        closed[closed_sz].t = cur.t;
        closed[closed_sz].parent_idx = cur.parent_idx;
        closed_sz++;

        /* 到达目标? */
        if (cur.x == gx && cur.y == gy) {
            goal_closed = cur.closed_idx;
            found = 1;
            break;
        }

        /* 扩展邻居: 4方向移动 + 等待 */
        for (d = 0; d < 5; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            int nt = cur.t + 1;

            if (nt > max_t) continue;

            /* 等待动作(d=4)不改变位置 */
            if (d < 4) {
                if (!(nx == gx && ny == gy) && !map_is_walkable(s, nx, ny))
                    continue;
            } else {
                nx = cur.x;
                ny = cur.y;
            }

            /* 检查closed */
            if (in_closed(nx, ny, nt) >= 0) continue;

            /* 检查CBS约束 */
            int blocked = 0;
            for (i = 0; i < agv->constraint_cnt; i++) {
                CBSConstraint *c = &agv->constraints[i];
                if ((c->t == -1 || c->t == nt) &&
                    c->x == nx && c->y == ny) {
                    blocked = 1;
                    break;
                }
            }
            if (blocked) continue;

            int ng = cur.g + 1;
            int nf = ng + map_manhattan(nx, ny, gx, gy);

            AStarNode next;
            next.x = nx; next.y = ny; next.t = nt;
            next.g = ng; next.f = nf;
            next.parent_idx = cur.closed_idx;
            next.closed_idx = -1;
            heap_push(next);
        }
    }

    if (!found) return 0;

    /* 回溯轨迹 */
    int len = 0;
    int ci = goal_closed;
    while (ci >= 0) {
        if (len >= max_traj) break;
        /* 暂存 */
        len++;
        ci = closed[ci].parent_idx;
    }

    if (len > max_traj) len = max_traj;
    ci = goal_closed;
    int idx = len - 1;
    while (ci >= 0 && idx >= 0) {
        traj_out[idx].x = closed[ci].x;
        traj_out[idx].y = closed[ci].y;
        traj_out[idx].t = closed[ci].t;
        idx--;
        ci = closed[ci].parent_idx;
    }

    return len;
}

/* ==================== A* with Reservation Table ==================== */
/* resv[y][x][t] = 1 表示该时空格已被占用 */
int astar_with_reservations(SimState *s, int agv_id,
                            int sx, int sy, int st,
                            int gx, int gy,
                            int resv[GRID_H][GRID_W][RESV_T_MAX],
                            TrajectoryPoint *traj_out, int max_traj) {
    AGV *agv = &s->agvs[agv_id];
    int d, found = 0, goal_closed = -1;

    heap_sz = 0;
    closed_sz = 0;

    AStarNode start;
    start.x = sx; start.y = sy; start.t = st;
    start.g = 0;
    start.f = map_manhattan(sx, sy, gx, gy);
    start.parent_idx = -1;
    start.closed_idx = -1;
    heap_push(start);

    int max_t = st + 200;
    if (max_t > MAX_TIME) max_t = MAX_TIME;


    while (heap_sz > 0) {
        AStarNode cur = heap_pop();
        if (in_closed(cur.x, cur.y, cur.t) >= 0) continue;

        cur.closed_idx = closed_sz;
        closed[closed_sz].x = cur.x;
        closed[closed_sz].y = cur.y;
        closed[closed_sz].t = cur.t;
        closed[closed_sz].parent_idx = cur.parent_idx;
        closed_sz++;

        if (cur.x == gx && cur.y == gy) {
            goal_closed = cur.closed_idx;
            found = 1;
            break;
        }

        for (d = 0; d < 5; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            int nt = cur.t + 1;
            if (nt > max_t) continue;

            if (d < 4) {
                if (!(nx == gx && ny == gy) && !map_is_walkable(s, nx, ny))
                    continue;
            } else {
                nx = cur.x; ny = cur.y;
            }

            if (in_closed(nx, ny, nt) >= 0) continue;

            /* 检查本AGV的约束 */
            int i, blocked = 0;
            for (i = 0; i < agv->constraint_cnt; i++) {
                CBSConstraint *c = &agv->constraints[i];
                if ((c->t == -1 || c->t == nt) &&
                    c->x == nx && c->y == ny) { blocked = 1; break; }
            }
            if (blocked) continue;

            /* 检查reservation table
             * 例外: 目标格允入(占据者会被调度离开), 但停车位不允许
             * 例外: 从起点等待(d=4,cur是根节点)允许多留一 tick */
            if (resv[ny][nx][nt & (RESV_T_MAX - 1)] &&
                !(d == 4 && cur.parent_idx == -1) &&
                !(nx == gx && ny == gy &&
                  s->grid[ny][nx].type != CELL_PARK))
                continue;

            int ng = cur.g + 1;
            int nf = ng + map_manhattan(nx, ny, gx, gy);

            AStarNode next;
            next.x = nx; next.y = ny; next.t = nt;
            next.g = ng; next.f = nf;
            next.parent_idx = cur.closed_idx;
            next.closed_idx = -1;
            heap_push(next);
        }
    }

    if (!found) return 0;

    int len = 0, ci = goal_closed;
    while (ci >= 0) { len++; ci = closed[ci].parent_idx; }
    if (len > max_traj) len = max_traj;

    ci = goal_closed;
    int idx = len - 1;
    while (ci >= 0 && idx >= 0) {
        traj_out[idx].x = closed[ci].x;
        traj_out[idx].y = closed[ci].y;
        traj_out[idx].t = closed[ci].t;
        idx--;
        ci = closed[ci].parent_idx;
    }
    return len;
}

/* ==================== 冲突检测 ==================== */
int detect_conflict(TrajectoryPoint *a, int la,
                    TrajectoryPoint *b, int lb,
                    int *conflict_t, int *conflict_x, int *conflict_y) {
    int i, j;
    for (i = 0; i < la; i++) {
        for (j = 0; j < lb; j++) {
            if (a[i].t == b[j].t && a[i].x == b[j].x && a[i].y == b[j].y) {
                *conflict_t = a[i].t;
                *conflict_x = a[i].x;
                *conflict_y = a[i].y;
                return 1;
            }
        }
    }
    return 0;
}

/* ==================== CBS 高层 ==================== */
/* CBS CT节点 */
typedef struct {
    TrajectoryPoint traj[MAX_AGVS][MAX_PATH_LEN];
    int             traj_len[MAX_AGVS];
    int             total_cost;
    int             parent;
} CTNode;

#define MAX_CTNODES 256

int cbs_plan_all(SimState *s) {
    CTNode ct_nodes[MAX_CTNODES];
    int i, j;

    /* Step 1: 独立规划每个AGV的初始路径 */
    ct_nodes[0].total_cost = 0;
    ct_nodes[0].parent = -1;

    for (i = 0; i < s->agv_cnt; i++) {
        AGV *agv = &s->agvs[i];
        if (!agv->busy) {
            ct_nodes[0].traj_len[i] = 0;
            continue;
        }

        int gx, gy;
        if (agv->status == AGV_MOVING_TO_BUFFER || agv->status == AGV_LOADING) {
            Buffer *b = &s->buffers[agv->buffer_target];
            gx = b->pos.x; gy = b->pos.y;
        } else {
            TransferZone *tz = &s->tzones[agv->transfer_target];
            gx = tz->pos.x; gy = tz->pos.y;
        }

        ct_nodes[0].traj_len[i] = astar_find_path(s, i,
            agv->pos.x, agv->pos.y, agv->cur_t,
            gx, gy,
            ct_nodes[0].traj[i], MAX_PATH_LEN);

        if (ct_nodes[0].traj_len[i] == 0) {
            /* 无路径可达: 保持原地 */
            ct_nodes[0].traj[i][0].x = agv->pos.x;
            ct_nodes[0].traj[i][0].y = agv->pos.y;
            ct_nodes[0].traj[i][0].t = agv->cur_t;
            ct_nodes[0].traj_len[i] = 1;
        }
        ct_nodes[0].total_cost += ct_nodes[0].traj_len[i];
    }

    /* Step 2: 冲突检测与解决 (迭代) */
    int iteration = 0;
    while (iteration < 50) {
        iteration++;
        int conflict_found = 0;

        for (i = 0; i < s->agv_cnt && !conflict_found; i++) {
            for (j = i + 1; j < s->agv_cnt && !conflict_found; j++) {
                if (ct_nodes[0].traj_len[i] == 0 || ct_nodes[0].traj_len[j] == 0)
                    continue;

                int ct, cx, cy;
                if (detect_conflict(
                        ct_nodes[0].traj[i], ct_nodes[0].traj_len[i],
                        ct_nodes[0].traj[j], ct_nodes[0].traj_len[j],
                        &ct, &cx, &cy)) {

                    conflict_found = 1;

                    /* 对冲突的AGV添加约束并重新规划 */

                    /* 方案A: 约束AGV i */
                    {
                        CBSConstraint nc;
                        nc.agv_id = i;
                        nc.x = cx; nc.y = cy; nc.t = ct;
                        s->agvs[i].constraints[s->agvs[i].constraint_cnt++] = nc;

                        int gx, gy;
                        AGV *agv = &s->agvs[i];
                        if (agv->status == AGV_MOVING_TO_BUFFER || agv->status == AGV_LOADING) {
                            Buffer *b = &s->buffers[agv->buffer_target];
                            gx = b->pos.x; gy = b->pos.y;
                        } else {
                            TransferZone *tz = &s->tzones[agv->transfer_target];
                            gx = tz->pos.x; gy = tz->pos.y;
                        }

                        int new_len = astar_find_path(s, i,
                            agv->pos.x, agv->pos.y, agv->cur_t,
                            gx, gy,
                            ct_nodes[0].traj[i], MAX_PATH_LEN);

                        if (new_len > 0) {
                            ct_nodes[0].traj_len[i] = new_len;
                            ct_nodes[0].total_cost += new_len;
                        } else {
                            /* 保持旧轨迹, 在冲突点等待 */
                            ct_nodes[0].traj[i][0].x = agv->pos.x;
                            ct_nodes[0].traj[i][0].y = agv->pos.y;
                            ct_nodes[0].traj[i][0].t = agv->cur_t;
                            ct_nodes[0].traj_len[i] = 1;
                        }
                    }

                    /* 方案B: 约束AGV j */
                    {
                        CBSConstraint nc;
                        nc.agv_id = j;
                        nc.x = cx; nc.y = cy; nc.t = ct;
                        s->agvs[j].constraints[s->agvs[j].constraint_cnt++] = nc;

                        int gx, gy;
                        AGV *agv = &s->agvs[j];
                        if (agv->status == AGV_MOVING_TO_BUFFER || agv->status == AGV_LOADING) {
                            Buffer *b = &s->buffers[agv->buffer_target];
                            gx = b->pos.x; gy = b->pos.y;
                        } else {
                            TransferZone *tz = &s->tzones[agv->transfer_target];
                            gx = tz->pos.x; gy = tz->pos.y;
                        }

                        int new_len = astar_find_path(s, j,
                            agv->pos.x, agv->pos.y, agv->cur_t,
                            gx, gy,
                            ct_nodes[0].traj[j], MAX_PATH_LEN);

                        if (new_len > 0) {
                            ct_nodes[0].traj_len[j] = new_len;
                            ct_nodes[0].total_cost += new_len;
                        }
                    }
                }
            }
        }

        if (!conflict_found) break;
    }

    /* 将规划结果写回AGV */
    for (i = 0; i < s->agv_cnt; i++) {
        AGV *agv = &s->agvs[i];
        if (!agv->busy) continue;

        int len = ct_nodes[0].traj_len[i];
        if (len > MAX_PATH_LEN) len = MAX_PATH_LEN;
        for (j = 0; j < len; j++) {
            agv->trajectory[j] = ct_nodes[0].traj[i][j];
        }
        agv->traj_len = len;
        agv->traj_idx = 0;
    }

    return (iteration < 50) ? 0 : -1;
}

/* 方便函数: 从AGV当前位置规划到目标 */
int plan_agv_to_target(SimState *s, int agv_id, int gx, int gy) {
    AGV *agv = &s->agvs[agv_id];
    agv->constraint_cnt = 0; /* 清空旧约束 */

    int len = astar_with_reservations(s, agv_id,
        agv->pos.x, agv->pos.y, agv->cur_t,
        gx, gy,
        g_resv,
        agv->trajectory, MAX_PATH_LEN);

    if (len == 0) {
        static int warn_cnt[4] = {0};
        if (warn_cnt[agv_id] < 10) {
            fprintf(stderr, "  [WARN] T%.0f AGV%d: astar no path (%d,%d)->(%d,%d) status=%d busy=%d\n",
                    s->time, agv_id, agv->pos.x, agv->pos.y, gx, gy, agv->status, agv->busy);
            int k;
            for (k = 0; k < s->agv_cnt; k++) {
                AGV *oa = &s->agvs[k];
                int has_traj = (oa->traj_len > 0 && oa->traj_idx < oa->traj_len);
                fprintf(stderr, "    AGV%d: pos(%d,%d) status=%d busy=%d traj=%d\n",
                        k, oa->pos.x, oa->pos.y, oa->status, oa->busy, has_traj);
            }
            warn_cnt[agv_id]++;
        }
        return 0;
    }
    agv->traj_len = len;
    agv->traj_idx = 0;

    /* 将本AGV的轨迹加入全局预约表 */
    pathfinding_resv_add_trajectory(agv->trajectory, len);
    return len;
}
