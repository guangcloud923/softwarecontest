#include "map.h"

/*
 * 仓库地图拓扑 (节点图)
 *
 * 布局示意:
 *   C1(0)    C2(1)       -- 传送带末端
 *     |         |
 *   BUF1(2)  BUF2(3)     -- 拣货缓冲区
 *      \       /
 *      JUNC1(4)           -- 十字路口
 *      /       \
 *   TP1(5)    TP2(6)     -- 交接区
 *    |   \    /   |
 *    |  S1(7) S2(8) |    -- 货架
 *   JUNC2(9)         -- 路口
 *    |   \    /   |
 *   S3(10)  S4(11)      -- 货架
 *    AGVPark(12-15)     -- AGV停车区
 */

static const char *NODE_NAMES[] = {
    "Conv1_out", "Conv2_out",
    "Buf1", "Buf2",
    "Junc1",
    "TP1", "TP2",
    "Shelf1", "Shelf2",
    "Junc2",
    "Shelf3", "Shelf4",
    "Park1", "Park2", "Park3", "Park4"
};

const char *node_name(int id) {
    if (id >= 0 && id < 16) return NODE_NAMES[id];
    return "Unknown";
}

void map_init(SimState *s) {
    int i;
    s->node_cnt = 16;

    /* 清空所有连接 */
    for (i = 0; i < s->node_cnt; i++) {
        s->nodes[i].id = i;
        s->nodes[i].conn_cnt = 0;
    }

    /* 设置节点坐标 (用于显示) */
    s->nodes[0].x = 2;  s->nodes[0].y = 0;   /* C1  */
    s->nodes[1].x = 6;  s->nodes[1].y = 0;   /* C2  */
    s->nodes[2].x = 2;  s->nodes[2].y = 2;   /* B1  */
    s->nodes[3].x = 6;  s->nodes[3].y = 2;   /* B2  */
    s->nodes[4].x = 4;  s->nodes[4].y = 4;   /* J1  */
    s->nodes[5].x = 2;  s->nodes[5].y = 6;   /* TP1 */
    s->nodes[6].x = 6;  s->nodes[6].y = 6;   /* TP2 */
    s->nodes[7].x = 0;  s->nodes[7].y = 8;   /* S1  */
    s->nodes[8].x = 8;  s->nodes[8].y = 8;   /* S2  */
    s->nodes[9].x = 4;  s->nodes[9].y = 10;  /* J2  */
    s->nodes[10].x = 2; s->nodes[10].y = 12; /* S3  */
    s->nodes[11].x = 6; s->nodes[11].y = 12; /* S4  */
    s->nodes[12].x = 0; s->nodes[12].y = 0;  /* P1  */
    s->nodes[13].x = 8; s->nodes[13].y = 0;  /* P2  */
    s->nodes[14].x = 0; s->nodes[14].y = 10; /* P3  */
    s->nodes[15].x = 8; s->nodes[15].y = 10; /* P4  */

    /* 辅助宏: 添加双向边 */
#define ADD_EDGE(a, b, d) \
    do { \
        int _n = s->nodes[a].conn_cnt++; \
        s->nodes[a].conn_ids[_n] = b; \
        s->nodes[a].conn_dist[_n] = d; \
        _n = s->nodes[b].conn_cnt++; \
        s->nodes[b].conn_ids[_n] = a; \
        s->nodes[b].conn_dist[_n] = d; \
    } while(0)

    /* 传送带 -> 缓冲区 */
    ADD_EDGE(0, 2, 2);   /* C1-B1  */
    ADD_EDGE(1, 3, 2);   /* C2-B2  */

    /* 缓冲区 -> 路口1 */
    ADD_EDGE(2, 4, 3);   /* B1-J1  */
    ADD_EDGE(3, 4, 3);   /* B2-J1  */

    /* 路口1 -> 交接区 */
    ADD_EDGE(4, 5, 3);   /* J1-TP1 */
    ADD_EDGE(4, 6, 3);   /* J1-TP2 */

    /* 交接区 -> 货架 */
    ADD_EDGE(5, 7, 3);   /* TP1-S1 */
    ADD_EDGE(6, 8, 3);   /* TP2-S2 */

    /* 交接区 -> 路口2 */
    ADD_EDGE(5, 9, 5);   /* TP1-J2 */
    ADD_EDGE(6, 9, 5);   /* TP2-J2 */

    /* 路口2 -> 货架 */
    ADD_EDGE(9, 10, 3);  /* J2-S3  */
    ADD_EDGE(9, 11, 3);  /* J2-S4  */

    /* AGV停车区 -> 传送带/缓冲区 */
    ADD_EDGE(12, 0, 2);  /* P1-C1  */
    ADD_EDGE(13, 1, 2);  /* P2-C2  */
    ADD_EDGE(14, 2, 3);  /* P3-B1  */
    ADD_EDGE(15, 3, 3);  /* P4-B2  */
    ADD_EDGE(12, 2, 4);  /* P1-B1  */
    ADD_EDGE(13, 3, 4);  /* P2-B2  */

    /* 货架之间连接 */
    ADD_EDGE(7, 10, 4);  /* S1-S3  */
    ADD_EDGE(8, 11, 4);  /* S2-S4  */
}

/* Dijkstra最短路径 */
int map_find_path(SimState *s, int start, int end, int *path_out) {
    int dist[MAX_NODES], prev[MAX_NODES], visited[MAX_NODES];
    int i, count = 0;

    for (i = 0; i < s->node_cnt; i++) {
        dist[i] = 999999;
        prev[i] = -1;
        visited[i] = 0;
    }
    dist[start] = 0;

    while (count < s->node_cnt) {
        /* 选未访问最小dist */
        int u = -1, best = 999999;
        for (i = 0; i < s->node_cnt; i++) {
            if (!visited[i] && dist[i] < best) {
                best = dist[i];
                u = i;
            }
        }
        if (u == -1 || u == end) break;
        visited[u] = 1;
        count++;

        for (i = 0; i < s->nodes[u].conn_cnt; i++) {
            int v = s->nodes[u].conn_ids[i];
            int nd = dist[u] + s->nodes[u].conn_dist[i];
            if (!visited[v] && nd < dist[v]) {
                dist[v] = nd;
                prev[v] = u;
            }
        }
    }

    if (prev[end] == -1 && start != end) return 0;

    /* 回溯路径 */
    int stack[MAX_NODES], sp = 0, cur = end;
    while (cur != -1) {
        stack[sp++] = cur;
        cur = prev[cur];
    }
    for (i = 0; i < sp; i++) {
        path_out[i] = stack[sp - 1 - i];
    }
    return sp;
}

int nodes_adjacent(SimState *s, int a, int b) {
    int i;
    for (i = 0; i < s->nodes[a].conn_cnt; i++) {
        if (s->nodes[a].conn_ids[i] == b) return 1;
    }
    return 0;
}
