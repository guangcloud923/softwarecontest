#ifndef DATATYPES_H
#define DATATYPES_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* ==================== 常量定义 ==================== */
#define MAX_CARGOS          100
#define MAX_AGVS            4
#define MAX_SHELVES         4
#define MAX_CONVEYORS       2
#define MAX_BUFFERS         2
#define MAX_ROBOTS          2
#define MAX_TRANSFER_POINTS 2
#define MAX_CARGOS_PER_AGV  10
#define MAX_SLOTS_PER_SHELF 20
#define MAX_NODES           30
#define MAX_CARGOS_PER_NODE 10
#define MAX_TIME            10000

/* ==================== 货物体积 ==================== */
typedef enum {
    VOL_SMALL  = 0, /* 0.25 m3 */
    VOL_MEDIUM = 1, /* 0.50 m3 */
    VOL_LARGE  = 2  /* 1.00 m3 */
} CargoVolume;

static const double VOL_VAL[] = {0.25, 0.5, 1.0};

/* ==================== 货物状态 ==================== */
typedef enum {
    CS_INIT,           /* 初始 */
    CS_ON_CONVEYOR,    /* 传送带上 */
    CS_IN_BUFFER,      /* 缓冲区中 */
    CS_ON_AGV,         /* AGV运输中 */
    CS_AT_TRANSFER,    /* 交接区 */
    CS_ON_SHELF        /* 已上架 */
} CargoState;


/* ==================== 货物结构体 ==================== */
typedef struct {
    int          id;
    CargoVolume  volume;
    int          target_shelf_id;    /* 0-3 */
    CargoState   state;
    int          location_id;        /* 所在设备ID */
    double       t_create;
    double       t_shelved;
} Cargo;

/* ==================== AGV ==================== */
typedef enum {
    AGV_IDLE,
    AGV_TO_BUFFER,      /* 前往缓冲区 */
    AGV_WAIT_BUFFER,    /* 等待装货 */
    AGV_LOADING,
    AGV_TO_TRANSFER,    /* 前往交接区 */
    AGV_WAIT_TRANSFER,  /* 等待卸货 */
    AGV_UNLOADING,
    AGV_BLOCKED
} AGVStateEnum;

typedef struct {
    int              id;
    int              cur_node;           /* 当前节点ID */
    int              prev_node;          /* 上一节点(用于方向判断) */
    int              target_node;
    int              path[MAX_NODES];    /* 路径节点序列 */
    int              path_len;
    int              path_idx;
    AGVStateEnum     state;
    double           load_vol;
    int              cargo_ids[MAX_CARGOS_PER_AGV];
    int              cargo_cnt;
    int              wait_ticks;         /* 等待计数器 */
    int              buffer_assigned;    /* 分配的目标缓冲区ID */
    int              transfer_assigned;  /* 分配的目标交接区ID */
    int              busy;              /* 是否有任务 */
} AGV;

/* ==================== 货架 ==================== */
typedef struct {
    int          id;
    int          occupied;
    int          cargo_id;
    CargoVolume  volume;
    double       t_occupied;
} Slot;

typedef struct {
    int      id;
    Slot     slots[MAX_SLOTS_PER_SHELF];
    int      slot_cnt;
    int      robot_id;           /* 负责此货架的机器人 */
    int      tp_id;              /* 关联的交接区 */
    int      node_id;            /* 所在节点 */
} Shelf;

/* ==================== 传送带 ==================== */
typedef struct {
    int      id;
    int      cargo_ids[MAX_CARGOS];
    int      cargo_cnt;
    int      capacity;
    int      paused;
    int      output_buffer_id;
    int      node_id;            /* 末端节点 */
    int      spawn_timer;        /* 生成计时器 */
    int      spawn_interval;     /* 生成间隔(ticks) */
} Conveyor;

/* ==================== 缓冲区 ==================== */
typedef struct {
    int      id;
    int      cargo_ids[MAX_CARGOS];
    int      cargo_cnt;
    int      capacity;
    int      conveyor_id;
    int      node_id;
    int      agv_assigned;       /* -1表示无AGV分配 */
} Buffer;

/* ==================== 交接区 ==================== */
typedef struct {
    int      id;
    int      node_id;
    int      cargo_ids[MAX_CARGOS];
    int      cargo_cnt;
    int      capacity;
    int      shelf_id;
    int      robot_id;
} TransferPoint;

/* ==================== 货架机器人 ==================== */
typedef enum {
    ROBOT_IDLE,
    ROBOT_FETCHING,       /* 从交接区取货 */
    ROBOT_SHELVING        /* 正在上架 */
} RobotStateEnum;

typedef struct {
    int              id;
    RobotStateEnum   state;
    int              shelf_ids[2];      /* 负责的2个货架 */
    int              shelf_cnt;
    int              tp_id;
    int              cargo_ids[MAX_CARGOS_PER_AGV];
    int              cargo_cnt;
    double           load_vol;
    int              wait_ticks;
    int              busy;
} ShelfRobot;

/* ==================== 地图节点 ==================== */
typedef struct {
    int  id;
    int  x, y;
    int  conn_ids[MAX_NODES];       /* 邻接节点ID */
    int  conn_dist[MAX_NODES];      /* 到邻接节点的距离 */
    int  conn_cnt;
} MapNode;

/* ==================== 全局仿真状态 ==================== */
typedef struct {
    double         time;
    int            running;
    int            max_time;

    /* 实体数组 */
    Cargo          cargos[MAX_CARGOS];
    int            cargo_cnt;
    int            next_cargo_id;

    AGV            agvs[MAX_AGVS];
    int            agv_cnt;

    Shelf          shelves[MAX_SHELVES];
    int            shelf_cnt;

    Conveyor       conveyors[MAX_CONVEYORS];
    int            conveyor_cnt;

    Buffer         buffers[MAX_BUFFERS];
    int            buffer_cnt;

    TransferPoint  tps[MAX_TRANSFER_POINTS];
    int            tp_cnt;

    ShelfRobot     robots[MAX_ROBOTS];
    int            robot_cnt;

    MapNode        nodes[MAX_NODES];
    int            node_cnt;

    /* 统计 */
    int            cargo_shelved;
    int            cargo_total;
    int            violations;
    char           violation_msg[256];
    int            collision_flag;
} SimState;

#endif /* DATATYPES_H */
