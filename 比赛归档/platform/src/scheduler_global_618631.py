"""
Scheduler.Global - 全局调度器
文件: src/scheduler_global_618631.py

职责: 收到货物事件后缓存, 收到AGV空闲事件后缓存,
      当 cargo 和 idle AGV 同时存在时输出 REQ_AGV_ASSIGN。

第一阶段: 最小会动版
"""

import json
import sys
import os
from collections import deque

# ============================================================
# 集中配置 - 坐标映射 (来源于 geometry.json)
# 如果仿真不接受, 根据事件流调整
# ============================================================

PICKUP_MAP = {
    "C1": {"x": 3, "y": 2},
    "C2": {"x": 12, "y": 2},
}

DROP_MAP = {
    "S1": {"x": 1, "y": 8},
    "S2": {"x": 5, "y": 8},
    "S3": {"x": 10, "y": 8},
    "S4": {"x": 14, "y": 8},
}

# conveyor_id -> pickup 坐标
CONVEYOR_TO_PICKUP = {
    "C1": PICKUP_MAP["C1"],
    "C2": PICKUP_MAP["C2"],
}

# shelf_id -> drop 坐标 (交接区)
SHELF_TO_DROP = {
    "S1": DROP_MAP["S1"],
    "S2": DROP_MAP["S2"],
    "S3": DROP_MAP["S3"],
    "S4": DROP_MAP["S4"],
}

# ============================================================
# 全局状态
# ============================================================

cargo_queue = deque()
idle_agvs = {}

# ============================================================
# 日志
# ============================================================

def log_event(event_type, payload, direction="IN"):
    """打印事件日志, 方便在平台事件流中排查"""
    try:
        payload_str = json.dumps(payload, ensure_ascii=False)
    except (TypeError, ValueError):
        payload_str = str(payload)
    print(f"[Scheduler] {direction} {event_type}: {payload_str}", file=sys.stderr, flush=True)


# ============================================================
# 兼容解析 - POINT2D
# ============================================================

def parse_point(pt):
    """
    兼容解析 POINT2D:
      {x, y} 或 [x, y] 或 {row, col}
    """
    if pt is None:
        return None
    if isinstance(pt, dict):
        x = pt.get("x")
        y = pt.get("y")
        if x is not None and y is not None:
            return {"x": int(x), "y": int(y)}
        # row/col fallback
        row = pt.get("row")
        col = pt.get("col")
        if row is not None and col is not None:
            return {"x": int(col), "y": int(row)}
    if isinstance(pt, (list, tuple)) and len(pt) >= 2:
        return {"x": int(pt[0]), "y": int(pt[1])}
    return None


# ============================================================
# 兼容解析 - CARGO_META
# ============================================================

def parse_cargo(cargo):
    """兼容解析 CARGO_META, 支持多种字段名"""
    if cargo is None:
        return None
    if isinstance(cargo, str):
        # 可能是纯 cargo_id 字符串
        return {"id": cargo}
    result = {}
    # id
    for key in ("id", "cargo_id", "item_id"):
        if key in cargo:
            result["id"] = cargo[key]
            break
    if "id" not in result:
        result["id"] = "UNKNOWN"
    # volume
    for key in ("volume", "size", "vol"):
        if key in cargo:
            result["volume"] = float(cargo[key])
            break
    if "volume" not in result:
        result["volume"] = 0.25  # fallback 小件
    # target_shelf
    for key in ("target_shelf", "shelf_id", "destination", "target"):
        if key in cargo:
            val = cargo[key]
            if isinstance(val, str) and val.startswith("S"):
                result["target_shelf"] = val
            elif isinstance(val, int):
                result["target_shelf"] = f"S{val + 1}" if val < 4 else f"S{(val % 4) + 1}"
            else:
                result["target_shelf"] = str(val)
            break
    if "target_shelf" not in result:
        result["target_shelf"] = "S1"  # fallback
    return result


def get_cargo_id(cargo):
    return cargo.get("id", "UNKNOWN")


def parse_conveyor_id(payload):
    """兼容解析 conveyor_id"""
    for key in ("conveyor_id", "conveyor", "conv_id", "source", "from_conveyor"):
        if key in payload:
            return str(payload[key])
    # conveyor_id 可能在嵌套的 cargo 子对象中
    cargo = payload.get("cargo", {})
    if isinstance(cargo, dict):
        for key in ("conveyor_id", "conveyor", "conv_id", "source"):
            if key in cargo:
                return str(cargo[key])
    return "C1"  # fallback


def map_conveyor_to_pickup(conveyor_id):
    """conveyor_id -> pickup 坐标"""
    pt = CONVEYOR_TO_PICKUP.get(str(conveyor_id))
    if pt is None:
        # fallback: 用 C1
        return PICKUP_MAP["C1"].copy()
    return pt.copy()


def map_cargo_to_drop(cargo):
    """cargo.target_shelf -> drop (交接区) 坐标"""
    shelf = cargo.get("target_shelf", "S1")
    pt = SHELF_TO_DROP.get(str(shelf))
    if pt is None:
        return DROP_MAP["S1"].copy()
    return pt.copy()


# ============================================================
# AGV IDLE 解析
# ============================================================

def parse_agv_idle(payload):
    """兼容解析 EVT_AGV_IDLE payload"""
    agv_id = None
    for key in ("agv_id", "id", "agv"):
        if key in payload:
            agv_id = str(payload[key])
            break
    if agv_id is None:
        agv_id = "AGV_UNKNOWN"

    position = None
    for key in ("position", "pos", "location"):
        if key in payload:
            position = parse_point(payload[key])
            break
    if position is None:
        # 尝试直接从 payload 中取 x, y
        position = parse_point(payload)

    return agv_id, position


# ============================================================
# 调度核心
# ============================================================

def pick_first_idle_agv():
    """选择第一个空闲 AGV"""
    if not idle_agvs:
        return None
    agv_id = next(iter(idle_agvs))
    del idle_agvs[agv_id]
    return agv_id


def try_dispatch():
    """当有 cargo 和空闲 AGV 同时存在时, 发起调度"""
    if not cargo_queue:
        return
    if not idle_agvs:
        return

    cargo, conveyor_id = cargo_queue.popleft()
    agv_id = pick_first_idle_agv()
    if agv_id is None:
        cargo_queue.appendleft((cargo, conveyor_id))
        return

    pickup = map_conveyor_to_pickup(conveyor_id)
    drop = map_cargo_to_drop(cargo)
    cargo_id = get_cargo_id(cargo)

    req = {
        "agv_id": agv_id,
        "cargo_id": cargo_id,
        "pickup": pickup,
        "drop": drop,
    }

    log_event("REQ_AGV_ASSIGN", req, "OUT")
    publish_REQ_AGV_ASSIGN(req)


# ============================================================
# 事件发布 (TODO: 平台真实 API 确认后替换)
# ============================================================

def publish_REQ_AGV_ASSIGN(req):
    """
    发布 REQ_AGV_ASSIGN 事件。
    TODO: 确认平台真实 VSOA publish 方式后替换此处实现。
    当前打印 JSON 到 stdout, 平台可能通过 stdout 捕获。
    """
    print(json.dumps({"event": "REQ_AGV_ASSIGN", "payload": req}), flush=True)


# ============================================================
# 事件处理器 (平台会调用这些函数)
# ============================================================

def on_EVT_CARGO_AVAIL(payload):
    """处理 EVT_CARGO_AVAIL 事件"""
    log_event("EVT_CARGO_AVAIL", payload, "IN")
    cargo = parse_cargo(payload.get("cargo", payload))
    conveyor_id = parse_conveyor_id(payload)
    cargo_queue.append((cargo, conveyor_id))
    try_dispatch()


def on_EVT_AGV_IDLE(payload):
    """处理 EVT_AGV_IDLE 事件"""
    log_event("EVT_AGV_IDLE", payload, "IN")
    agv_id, position = parse_agv_idle(payload)
    idle_agvs[agv_id] = position
    try_dispatch()


def on_EVT_SHELF_IDLE(payload):
    """处理 EVT_SHELF_IDLE 事件 (第一阶段仅记录)"""
    log_event("EVT_SHELF_IDLE", payload, "IN")
    # 第一阶段暂不需要处理货架机器人调度


# ============================================================
# 主入口
# ============================================================

def main():
    """
    如果直接运行, 进入事件循环读取 stdin JSON 行。
    平台可能通过 stdin 传入事件流。
    """
    print("[Scheduler] Scheduler.Global started, waiting for events...", file=sys.stderr, flush=True)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            print(f"[Scheduler] WARN: invalid JSON: {line}", file=sys.stderr, flush=True)
            continue

        event_type = msg.get("event", msg.get("type", ""))
        payload = msg.get("payload", msg.get("data", msg))

        if event_type == "EVT_CARGO_AVAIL":
            on_EVT_CARGO_AVAIL(payload)
        elif event_type == "EVT_AGV_IDLE":
            on_EVT_AGV_IDLE(payload)
        elif event_type == "EVT_SHELF_IDLE":
            on_EVT_SHELF_IDLE(payload)
        else:
            print(f"[Scheduler] WARN: unknown event type: {event_type}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()
