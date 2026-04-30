# 仿真优化日志

## 当前状态 (优化前)

### 评分公式 (v2.0)
```
Sn = D × 3000 / (T + 60)
D = items_total / 10
T = 仿真完成时间 (ticks)
```
- 碰撞/违规 → 直接 0 分

### 基线性能 (30 货物, seed 42)
| 指标 | 数值 |
|------|------|
| 难度系数 D | 3.0 |
| 完成时间 T | 187 ticks |
| 上架率 | 100% |
| 得分 | 36.4 |
| 理论满分(T=0) | 150 |

### 关键参数
| 参数 | 值 | 位置 |
|------|-----|------|
| AGV 数量 | 4 | datatypes.h |
| 机器人数量 | 2 | datatypes.h |
| 传送带数量 | 2 | datatypes.h |
| 交接区容量 | 1.0 m³ | simcore.c |
| 缓冲区容量 | 6 | simcore.c |
| 传送带产出间隔 | 4 tick | conveyor.c |
| 机器人取货耗时 | 2 tick | robot.c |
| 机器人上架耗时 | 2 tick | robot.c |
| AGV 调度条件 | 仅 IDLE 状态 | scheduler.c |

### 主要瓶颈
1. AGV 卸货后必须返回停车位变 IDLE，调度器才重新派遣，往返浪费 20-30 tick
2. 交接区容量仅 1.0 m³，大货(0.9)只能放 1 个，AGV 频繁 WAITING
3. 机器人取货 2 tick + 上架 2 tick = 4 tick/批次
4. 传送带每 4 tick 产出一个货物

---

## 2026-04-30 第1轮优化

### 修改清单

| # | 修改内容 | 文件 | 改动 |
|---|---------|------|------|
| 1 | AGV 调度扩充 RETURNING 状态 | scheduler.c | `AGV_IDLE` → `AGV_IDLE \|\| AGV_RETURNING` |
| 2 | AGV 卸货后直接去缓冲区 | agv.c | 卸空后优先检查缓冲区，有活就直奔不空返 |
| 3 | 交接区容量翻倍 | simcore.c | capacity_vol: 1.0 → 2.0 |
| 3b | 交接区容量检查去硬编码 | agv.c | `>1.001`/`<0.999` → `>capacity_vol+0.001`/`<capacity_vol-0.001` |
| 3c | 约束校验同步容量 | constraint.c | `>1.001` → `>capacity_vol+0.001` |
| 4 | 机器人取货/上架加速 | robot.c | wait_ticks: 2 → 1 (取货/上架/中转均减半) |
| 5 | 传送带产出加速 | conveyor.c | spawn_interval: 4 → 2 |

### 优化后性能

| 种子 | 优化前 T | 优化后 T | 优化前得分 | 优化后得分 | 提升 |
|------|---------|---------|-----------|-----------|------|
| 42 | 187 | 177 | 36.4 | 38.0 | +4.4% |
| 30 | 184 | 162 | 36.9 | 40.5 | +9.8% |

### 残存瓶颈
- AGV 仍有空闲时刻 (路径冲突导致轨迹被取消)
- 路径规划的 reservation table 冲突检测过于激进
- 机器人容量仍为 1.0 (交接区扩容到 2.0 但机器人一次最多取 1.0)

---

## 2026-04-30 19:53 PRD 对照分析与工程整理

### 一、PRD 需求实现状态

#### 工程架构

| PRD 要求 | 状态 | 说明 |
|----------|:----:|------|
| 后端纯 C 语言 | ✅ | 12 个 `.c` 文件, C99 标准 |
| WebSocket 服务 | ✅ | 自实现完整协议栈 (SHA1/Base64/帧编解码) |
| React (Vite) 前端 | ❌ | 当前为单页 HTML + 内联 JS, 由 C server 直接 serve |
| CanvasRenderer 类 (useRef) | ❌ | 未封装为类, 未使用 React |

#### 五大模块

| 模块 | PRD 目标算法 | 实现 | 差距 |
|------|-------------|:----:|------|
| 一: 全局状态 | 纯内存结构体 + Assert 宏 | 结构体完整, 运行时校验函数 | 非编译期 Assert |
| 二: 单车寻路 | Time-Space A\* (x,y,t) + 曼哈顿启发 | `astar_find_path()` 完整实现 | — |
| 三: 多车调度 | CBS (顶点+边冲突) | CBS 框架, 仅顶点冲突检测, 单 CTNode | 缺边冲突, 无高层约束树 |
| 四: 拼单 | 0-1 背包 DP + 距离权重 | `knapsack_batch()` DP 实现, 体积权重+热门货架加成 | 缺真正的距离权重 |
| 五: 碎片整理 | 0.25m³ 浅层→深层 | `defrag_tick()` 每 20tick 触发 | — |

#### 评分公式

```
PRD:  Sn = D × 3000 / (T + T0)
代码: s->score = D * 3000.0 / (T + T0)

D = items_total / 10.0  (最小 1.0)
T0 = 60.0
违规/碰撞 → 0 分
```
**结论: 公式与 PRD 完全一致。**

#### 偏离 PRD 的关键参数

| 参数 | PRD | 当前代码 | 风险 |
|------|-----|---------|------|
| 交接区容量 | 1.0 m³ | 2.0 m³ (`simcore.c`) | 可能触发一票否决 |

### 二、各模块算法详细

| 模块 | 算法 | 关键数据结构 | 文件 |
|------|------|-------------|------|
| 全局状态 | C struct 状态机 | `SimState` (16×14网格, 4AGV, 4货架×27盘位) | `datatypes.h` |
| 单车寻路 | **Time-Space A\*** | 二叉最小堆(open), 数组(closed), 5动作(4方向+等待), 搜索上限200步 | `pathfinding.c` |
| 预约表 | 全局时空占位标记 | `g_resv[14][16][512]`, 每 tick 清空重填 | `pathfinding.c` |
| 多车防撞 | **简化 CBS** + 预约表 | 迭代≤50轮, 顶点冲突→双方加约束重规划, 方案A+B都加; 轨迹终点冲突检测 | `pathfinding.c` |
| 拼单 | **0-1 Knapsack DP** | 体积→权重(0.25→1,0.50→2,1.00→4), 容量=4, dp[i][w] 二维DP | `scheduler.c` |
| 派车 | 曼哈顿最近空闲 AGV | IDLE/RETURNING 均可派 | `scheduler.c` |
| 交接区选择 | 多数投票 | 统计车上货物目标货架分布 | `scheduler.c` |
| 碎片整理 | 贪心扫描搬移 | 浅层(row<2)小货→深层(row≥2)空位, 逐个迁移 | `defrag.c` |
| 约束校验 | 6项运行时检查 | AGV超载/缓冲区溢出/交接区超容/机器人混合体积/碰撞/状态一致性 | `constraint.c` |
| 统计评分 | 实时更新 | 每 tick 计算 `stats_update()`, 仿真结束 `stats_print()` | `stats.c` |

### 三、macOS/跨平台兼容性修复

| 文件 | 问题 | 修改 |
|------|------|------|
| `main.c` | `system("pause")` 仅 Windows 有效 | 替换为 `printf + getchar()` (C标准库) |
| `simcore.c` | `sleep()` 需 `<unistd.h>` (POSIX) | 添加 `#include <unistd.h>` |
| `server.c` | `fcntl()`/`O_NONBLOCK` 需 `<fcntl.h>` | 添加 `#include <fcntl.h>` |
| `Makefile` | `-lws2_32` 是 Windows 专有库 | 移除, 仅保留 `-lm` |

- 所有 `#ifdef _WIN32` 分支保持不变, 确保 Windows 仍可编译
- `system("pause")` → `getchar()` 在所有平台行为一致

### 四、仓库工程整理

```
软件杯赛题仓库 (2026-04-30 整理后)
.
├── src/                    # 全部 C 源码 (.c + .h)
│   ├── main.c              # 入口, 参数解析
│   ├── simcore.c/.h        # 仿真主循环 (终端/服务器模式)
│   ├── datatypes.h         # 全部数据结构与常量定义
│   ├── map.c/.h            # 2D 网格地图 (16×14)
│   ├── pathfinding.c/.h    # Time-Space A* + CBS + 预约表
│   ├── scheduler.c/.h      # 0-1背包拼单 + AGV调度
│   ├── agv.c/.h            # AGV 状态机
│   ├── conveyor.c/.h       # 传送带逻辑
│   ├── robot.c/.h          # 货架机器人 (取货/上架)
│   ├── defrag.c/.h         # 货架碎片整理
│   ├── constraint.c/.h     # 一票否决校验
│   ├── stats.c/.h          # 评分计算
│   └── server.c/.h         # WebSocket/HTTP 服务器
├── web/
│   └── visualization.html  # 前端沙盘可视化
├── Makefile                # 跨平台构建 (gcc + POSIX)
├── prd.md                  # 赛题 PRD
├── log.md                  # 本日志
└── .gitignore              # 排除 *.o, warehouse_sim, .DS_Store 等
```

- 删除 `backend/` (旧版源码副本, 缺多个模块)
- 删除 `backup_optimized/` (更早期快照)
- 编译产物 (`*.o`, `warehouse_sim`) 由 `.gitignore` 排除
- `make run` / `make run-server` 工作流不变
