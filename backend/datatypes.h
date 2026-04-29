#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>

/* ==================== 网格常量 ==================== */
#define GRID_W             16
#define GRID_H             14
#define MAX_ITEMS          100
#define MAX_AGVS           4
#define MAX_SHELVES        4
#define MAX_CONVEYORS      2
#define MAX_BUFFERS        2
#define MAX_TRANSFER_ZONES 4
#define MAX_ROBOTS         2
#define MAX_ITEMS_PER_AGV  10
#define MAX_SHELF_ROWS     3
#define MAX_SHELF_COLS     3
#define MAX_SHELF_DEPTH    3
#define MAX_SLOTS          27   /* 3x3x3 */
#define MAX_CONSTRAINTS    256
#define MAX_PATH_LEN       512
#define MAX_EVENTS         256
#define MAX_FRAME_STR      65536
#define MAX_TIME           2000

/* 货物体积 */
typedef enum {
    VOL_SMALL  = 0,  /* 0.25 m3 */
    VOL_MEDIUM = 1,  /* 0.50 m3 */
    VOL_LARGE  = 2   /* 1.00 m3 */
} ItemVolume;

static const double VOL_VAL[] = {0.25, 0.50, 1.00};

/* ==================== 网格单元格类型 ==================== */
typedef enum {
    CELL_EMPTY        = 0,  /* 空地/道路 */
    CELL_OBSTACLE     = 1,  /* 障碍物 */
    CELL_CONVEYOR_END = 2,  /* 传送带末端 */
    CELL_BUFFER       = 3,  /* 拣货缓冲区 */
    CELL_CROSSROAD    = 4,  /* 十字路口 */
    CELL_TRANSFER     = 5,  /* 交接区 */
    CELL_SHELF        = 6,  /* 货架 */
    CELL_PARK         = 7,  /* AGV停车区 */
    CELL_NARROW       = 8   /* 窄通道 */
} CellType;

/* 2D 坐标 */
typedef struct { int x, y; } Pos2D;

/* 网格单元格 */
typedef struct {
    CellType type;
    int      entity_id;   /* 关联实体ID (-1=无) */
    int      walkable;    /* AGV可通行 */
    Pos2D    pos;
} GridCell;

/* ==================== 货物状态 ==================== */
typedef enum {
    ITEM_INIT,
    ITEM_ON_CONVEYOR,
    ITEM_IN_BUFFER,
    ITEM_ON_AGV,
    ITEM_AT_TRANSFER,
    ITEM_ON_SHELF
} ItemState;

/* ==================== 货物 ==================== */
typedef struct {
    int        id;
    ItemVolume volume;
    int        target_shelf;   /* 0-3 */
    ItemState  state;
    int        location_id;    /* 所在设备ID */
    double     t_spawn;
    double     t_shelved;
} Item;

/* ==================== CBS 约束 ==================== */
typedef struct {
    int  agv_id;
    int  x, y;
    int  t;               /* -1表示永久禁止该格子 */
} CBSConstraint;

/* ==================== AGV 状态 ==================== */
typedef enum {
    AGV_IDLE,
    AGV_MOVING_TO_BUFFER,
    AGV_LOADING,
    AGV_MOVING_TO_TRANSFER,
    AGV_WAITING,
    AGV_UNLOADING,
    AGV_RETURNING
} AGVStatus;

/* 时空轨迹点 */
typedef struct {
    int x, y, t;
} TrajectoryPoint;

/* ==================== AGV 实体 ==================== */
typedef struct {
    int              id;
    Pos2D            pos;            /* 当前网格位置 */
    int              cur_t;          /* 当前时间步 */
    AGVStatus        status;
    double           load_vol;
    int              items[MAX_ITEMS_PER_AGV];
    int              item_cnt;
    int              buffer_target;   /* 目标缓冲区ID */
    int              transfer_target; /* 目标交接区ID */
    int              busy;

    /* 轨迹 */
    TrajectoryPoint  trajectory[MAX_PATH_LEN];
    int              traj_len;
    int              traj_idx;

    /* CBS约束列表 */
    CBSConstraint    constraints[MAX_CONSTRAINTS];
    int              constraint_cnt;

    /* 等待计时 */
    int              wait_ticks;
    int              home_park;     /* 归属停车位索引 */
} AGV;

/* ==================== 传送带 ==================== */
typedef struct {
    int      id;
    Pos2D    end_pos;         /* 末端网格位置 */
    int      output_buffer_id;
    int      items[10];       /* 传送带上的货物 (capacity=10) */
    int      item_cnt;
    int      paused;
    int      spawn_timer;
    int      spawn_interval;
} Conveyor;

/* ==================== 缓冲区 ==================== */
typedef struct {
    int      id;
    Pos2D    pos;
    int      items[6];
    int      item_cnt;
    int      capacity;        /* =6 */
    int      conveyor_id;
    int      agv_assigned;    /* 派往此缓冲区的AGV ID, -1=无 */
} Buffer;

/* ==================== 交接区 ==================== */
typedef struct {
    int      id;
    Pos2D    pos;
    int      items[20];
    int      item_cnt;
    double   total_vol;       /* 当前总体积, ≤1.0 */
    double   capacity_vol;    /* =1.0 */
    int      shelf_ids[2];    /* 关联的货架ID */
    int      robot_id;
} TransferZone;

/* ==================== 货架盘位 ==================== */
typedef struct {
    int        occupied;
    int        item_id;
    ItemVolume vol;
    double     t_occupied;
} ShelfSlot;

/* ==================== 货架 ==================== */
typedef struct {
    int        id;
    Pos2D      pos;
    ShelfSlot  slots[MAX_SHELF_ROWS][MAX_SHELF_COLS][MAX_SHELF_DEPTH];
    int        robot_id;
    int        tp_id;         /* 关联交接区 */
    double     total_vol;
} Shelf;

/* ==================== 货架机器人 ==================== */
typedef enum {
    ROBOT_IDLE,
    ROBOT_FETCHING,
    ROBOT_SHELVING
} RobotStatus;

typedef struct {
    int         id;
    RobotStatus status;
    int         shelf_ids[2];
    int         tp_id;
    int         items[10];
    int         item_cnt;
    double      load_vol;
    int         wait_ticks;
    int         busy;
} ShelfRobot;

/* ==================== 事件日志 ==================== */
typedef struct {
    double time;
    char   msg[256];
} SimEvent;

/* ==================== 全局仿真状态 ==================== */
typedef struct {
    /* 网格 */
    GridCell      grid[GRID_H][GRID_W];

    /* 时间 */
    double        time;           /* tick */
    double        sim_start_time; /* 实际开始时间 */
    int           running;
    int           paused;

    /* 货物 */
    Item          items[MAX_ITEMS];
    int           item_cnt;
    int           next_item_id;
    int           items_shelved;
    int           items_total;

    /* AGV */
    AGV           agvs[MAX_AGVS];
    int           agv_cnt;

    /* 传送带 */
    Conveyor      conveyors[MAX_CONVEYORS];
    int           conveyor_cnt;

    /* 缓冲区 */
    Buffer        buffers[MAX_BUFFERS];
    int           buffer_cnt;

    /* 交接区 */
    TransferZone  tzones[MAX_TRANSFER_ZONES];
    int           tzone_cnt;

    /* 货架 */
    Shelf         shelves[MAX_SHELVES];
    int           shelf_cnt;

    /* 机器人 */
    ShelfRobot    robots[MAX_ROBOTS];
    int           robot_cnt;

    /* 事件日志 */
    SimEvent      events[MAX_EVENTS];
    int           event_cnt;

    /* 统计/裁判 */
    int           violations;
    int           collision_flag;
    char          violation_msg[256];
    double        difficulty;    /* D */
    double        score;

    /* 随机数 */
    unsigned int  rng_state;

    /* 帧序列号 */
    int           frame_id;
} SimState;

#endif /* DATATYPES_H */
