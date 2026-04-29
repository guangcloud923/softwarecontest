#include "map.h"

/*
 * 仓库2D网格地图 (16x14)
 *
 * 道路采用曼哈顿网格:
 *   竖线: x ∈ {1, 3, 5, 7, 10, 12, 14}
 *   横线: y ∈ {0, 2, 4, 6, 8, 10}
 *
 *   关键位置:
 *     C1(3,0)  C2(12,0)      传送带末端
 *     B1(3,2)  B2(12,2)      缓冲区 (cap=6)
 *     J1(7,4)  J2(7,8)       路口
 *     T1(1,8)  T2(5,8)       交接区1,2 (Robot0)
 *     T3(10,8) T4(14,8)      交接区3,4 (Robot1)
 *     S1(1,10) S2(5,10)      货架1,2
 *     S3(10,10)S4(14,10)     货架3,4
 *     P1(1,0)  P2(14,0)      AGV停车区
 *     P3(1,6)  P4(14,6)
 */

/* 竖线位置 */
static int is_vert_road(int x) {
    return x == 1 || x == 3 || x == 5 || x == 7 || x == 10 || x == 12 || x == 14;
}

/* 横线位置 */
static int is_horiz_road(int y) {
    return y == 0 || y == 2 || y == 4 || y == 6 || y == 8 || y == 10;
}

/* 特殊单元格位置 */
static CellType get_special_cell(int x, int y) {
    if (x == 3 && y == 0)  return CELL_CONVEYOR_END;  /* C1 */
    if (x == 12 && y == 0) return CELL_CONVEYOR_END;  /* C2 */
    if (x == 3 && y == 2)  return CELL_BUFFER;        /* B1 */
    if (x == 12 && y == 2) return CELL_BUFFER;        /* B2 */
    if (x == 7 && y == 4)  return CELL_CROSSROAD;     /* J1 */
    if (x == 7 && y == 8)  return CELL_CROSSROAD;     /* J2 */
    if (x == 1 && y == 8)  return CELL_TRANSFER;      /* T1 */
    if (x == 5 && y == 8)  return CELL_TRANSFER;      /* T2 */
    if (x == 10 && y == 8) return CELL_TRANSFER;      /* T3 */
    if (x == 14 && y == 8) return CELL_TRANSFER;      /* T4 */
    if (x == 1 && y == 10)  return CELL_SHELF;        /* S1 */
    if (x == 5 && y == 10)  return CELL_SHELF;        /* S2 */
    if (x == 10 && y == 10) return CELL_SHELF;        /* S3 */
    if (x == 14 && y == 10) return CELL_SHELF;        /* S4 */
    if ((x == 1 && y == 0) || (x == 14 && y == 0) ||
        (x == 1 && y == 6) || (x == 14 && y == 6))
        return CELL_PARK;
    return CELL_EMPTY;
}

void map_init(SimState *s) {
    int x, y;

    for (y = 0; y < GRID_H; y++) {
        for (x = 0; x < GRID_W; x++) {
            int walkable = 0;
            CellType type = CELL_OBSTACLE;

            /* 判断是否为道路: 竖线或横线交点 */
            if (is_vert_road(x) || is_horiz_road(y)) {
                walkable = 1;
                type = CELL_EMPTY;
            }

            /* 覆盖特殊单元格 */
            CellType sp = get_special_cell(x, y);
            if (sp != CELL_EMPTY) {
                type = sp;
                walkable = 1;
            }

            /* 确保竖线×横线交叉点在竖线上也连通 */
            if (is_vert_road(x) && is_horiz_road(y)) {
                walkable = 1;
                /* 不覆盖已有的特殊类型 */
                if (type == CELL_EMPTY) type = CELL_EMPTY;
            }

            s->grid[y][x].type = type;
            s->grid[y][x].walkable = walkable;
            s->grid[y][x].entity_id = -1;
            s->grid[y][x].pos.x = x;
            s->grid[y][x].pos.y = y;
        }
    }
}

int map_is_walkable(SimState *s, int x, int y) {
    if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return 0;
    return s->grid[y][x].walkable;
}

int map_is_crossroad(SimState *s, int x, int y) {
    if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return 0;
    return s->grid[y][x].type == CELL_CROSSROAD;
}

int map_is_narrow(SimState *s, int x, int y) {
    if (x < 0 || x >= GRID_W || y < 0 || y >= GRID_H) return 0;
    return s->grid[y][x].type == CELL_NARROW;
}

static const int DX[] = {0, 0, -1, 1};
static const int DY[] = {-1, 1, 0, 0};

void map_get_neighbors(SimState *s, int x, int y, Pos2D *neighbors, int *cnt) {
    *cnt = 0;
    int d;
    for (d = 0; d < 4; d++) {
        int nx = x + DX[d];
        int ny = y + DY[d];
        if (map_is_walkable(s, nx, ny)) {
            neighbors[*cnt].x = nx;
            neighbors[*cnt].y = ny;
            (*cnt)++;
        }
    }
}

int map_manhattan(int x1, int y1, int x2, int y2) {
    return abs(x1 - x2) + abs(y1 - y2);
}

void map_recalc_special(SimState *s) {
    int i;
    for (i = 0; i < s->buffer_cnt; i++) {
        Pos2D p = s->buffers[i].pos;
        s->grid[p.y][p.x].entity_id = i;
    }
    for (i = 0; i < s->tzone_cnt; i++) {
        Pos2D p = s->tzones[i].pos;
        s->grid[p.y][p.x].entity_id = i;
    }
    for (i = 0; i < s->shelf_cnt; i++) {
        Pos2D p = s->shelves[i].pos;
        s->grid[p.y][p.x].entity_id = i;
    }
}
