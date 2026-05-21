"""
本地 mock 测试脚本
用法: python run_mock_tests.py

测试目标:
  1. 模拟 EVT_CARGO_AVAIL → 检查 Scheduler 是否输出 REQ_AGV_ASSIGN
  2. 模拟 EVT_AGV_IDLE → 同上
  3. 模拟 REQ_PATH → 检查 PathPlan 是否输出 EVT_PATH
  4. 模拟 REQ_MOVE_PROPOSED → 检查 SafetyGuard 是否输出 REQ_MOVE_APPROVED
"""

import json
import sys
import os
import subprocess
import tempfile

# 确保能找到平台源文件
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLATFORM_SRC = os.path.join(SCRIPT_DIR, "..", "platform", "src")

# ============================================================
# Test 1: Scheduler 单元测试
# ============================================================

def test_scheduler():
    """测试 Scheduler 的事件处理逻辑"""
    print("=" * 60)
    print("Test 1: Scheduler.Global")
    print("=" * 60)

    # 导入 scheduler 模块
    sys.path.insert(0, PLATFORM_SRC)
    import scheduler_global_618631 as sched

    # 重置全局状态
    sched.cargo_queue.clear()
    sched.idle_agvs.clear()

    # 1.1 先发 cargo 事件 (不应触发 dispatch, 因为无空闲 AGV)
    sched.on_EVT_CARGO_AVAIL({
        "cargo": {"id": "CARGO_001", "volume": 0.25, "target_shelf": "S1"},
        "conveyor_id": "C1"
    })
    assert len(sched.cargo_queue) == 1, "cargo_queue should have 1 item"
    assert len(sched.idle_agvs) == 0, "idle_agvs should be empty"
    print("  PASS: cargo cached correctly")

    # 1.2 再发 cargo 事件
    sched.on_EVT_CARGO_AVAIL({
        "cargo": {"id": "CARGO_002", "volume": 0.5, "target_shelf": "S3"},
        "conveyor_id": "C2"
    })
    assert len(sched.cargo_queue) == 2, "cargo_queue should have 2 items"
    print("  PASS: second cargo cached")

    # 1.3 发 AGV idle 事件 → 应触发 dispatch, 消耗一个 cargo
    sched.on_EVT_AGV_IDLE({
        "agv_id": "AGV_1",
        "position": {"x": 1, "y": 0}
    })
    assert len(sched.cargo_queue) == 1, "cargo_queue should have 1 item after dispatch"
    assert len(sched.idle_agvs) == 0, "idle_agvs should be empty (AGV consumed)"
    print("  PASS: dispatch triggered on AGV idle")

    # 1.4 再发 AGV idle → 消耗最后一个 cargo
    sched.on_EVT_AGV_IDLE({
        "agv_id": "AGV_2",
        "position": {"x": 14, "y": 0}
    })
    assert len(sched.cargo_queue) == 0, "cargo_queue should be empty"
    print("  PASS: all cargos dispatched")

    # 1.5 先发 AGV idle 再发 cargo
    sched.on_EVT_AGV_IDLE({
        "agv_id": "AGV_3",
        "position": {"x": 1, "y": 6}
    })
    assert len(sched.idle_agvs) == 1, "idle_agvs should have 1 AGV"
    sched.on_EVT_CARGO_AVAIL({
        "cargo": {"id": "CARGO_003", "volume": 1.0, "target_shelf": "S2"},
        "conveyor_id": "C1"
    })
    assert len(sched.cargo_queue) == 0, "cargo should be dispatched immediately"
    assert len(sched.idle_agvs) == 0, "AGV should be consumed"
    print("  PASS: reverse order dispatch works")

    # 1.6 兼容解析测试
    cargo = sched.parse_cargo({"cargo_id": "TEST_01", "size": 0.5, "destination": "S4"})
    assert cargo["id"] == "TEST_01", f"id parse failed: {cargo}"
    assert cargo["volume"] == 0.5, f"volume parse failed: {cargo}"
    print("  PASS: compatible cargo parsing")

    pt1 = sched.parse_point({"x": 3, "y": 5})
    assert pt1 == {"x": 3, "y": 5}, f"point parse failed: {pt1}"
    pt2 = sched.parse_point([7, 9])
    assert pt2 == {"x": 7, "y": 9}, f"array point parse failed: {pt2}"
    print("  PASS: compatible POINT2D parsing")

    # 1.7 坐标映射测试
    pickup = sched.map_conveyor_to_pickup("C1")
    assert pickup == {"x": 3, "y": 2}, f"pickup map failed: {pickup}"
    drop = sched.map_cargo_to_drop({"target_shelf": "S3"})
    assert drop == {"x": 10, "y": 8}, f"drop map failed: {drop}"
    print("  PASS: coordinate mapping")

    print("  Scheduler: ALL TESTS PASSED\n")


# ============================================================
# Test 2: PathPlan 单元测试
# ============================================================

def test_pathplan():
    """测试 PathPlan 的路径生成逻辑"""
    print("=" * 60)
    print("Test 2: policy.PathPlan")
    print("=" * 60)

    sys.path.insert(0, PLATFORM_SRC)

    # 用 subprocess 跑 Node.js 测试
    test_js = """
var pp = require('./path_plan_618631.js');

// Test parsePoint
var pt1 = pp.parsePoint({x: 3, y: 5});
console.assert(pt1.x === 3 && pt1.y === 5, 'dict parse failed');

var pt2 = pp.parsePoint([7, 9]);
console.assert(pt2.x === 7 && pt2.y === 9, 'array parse failed');

var pt3 = pp.parsePoint({row: 2, col: 4});
console.assert(pt3.x === 4 && pt3.y === 2, 'row/col parse failed');

// Test manhattanPath
var path = pp.manhattanPath({x: 0, y: 0}, {x: 2, y: 1});
console.assert(path.length === 4, 'path length should be 4, got ' + path.length);
console.assert(path[0].x === 0 && path[0].y === 0, 'start point wrong');
console.assert(path[path.length-1].x === 2 && path[path.length-1].y === 1, 'end point wrong');

console.log('PathPlan: ALL TESTS PASSED');
"""
    result = subprocess.run(
        ["node", "-e", test_js],
        cwd=PLATFORM_SRC,
        capture_output=True, text=True
    )
    if result.returncode == 0:
        print("  " + result.stdout.strip())
    else:
        print("  FAIL: " + result.stderr.strip())
        return

    print()


# ============================================================
# Test 3: SafetyGuard 编译和功能测试
# ============================================================

def test_safetyguard():
    """测试 SafetyGuard 编译和基本功能"""
    print("=" * 60)
    print("Test 3: SafetyGuard.Collision")
    print("=" * 60)

    cpp_file = os.path.join(PLATFORM_SRC, "safety_guard_618631.cpp")

    # 编译
    with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as tmp:
        elf_path = tmp.name

    try:
        result = subprocess.run(
            ["g++", cpp_file, "-std=c++17", "-lm", "-o", elf_path],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"  FAIL: compilation error:\n{result.stderr}")
            return
        print("  PASS: compilation successful")

        # 测试 REQ_MOVE_PROPOSED → REQ_MOVE_APPROVED
        test_input = '{"event":"REQ_MOVE_PROPOSED","payload":{"agv_id":"AGV_1","x":1,"y":0}}\n'
        result = subprocess.run(
            [elf_path],
            input=test_input,
            capture_output=True, text=True,
            timeout=5
        )
        stdout = result.stdout.strip()
        if '"event":"REQ_MOVE_APPROVED"' in stdout:
            print(f"  PASS: REQ_MOVE_PROPOSED → APPROVED")
            print(f"    output: {stdout}")
        else:
            print(f"  FAIL: expected APPROVED, got: {stdout}")

        # 测试 EVT_POSITION
        test_input2 = '{"event":"EVT_POSITION","payload":{"agv_id":"AGV_1","x":3,"y":5}}\n'
        result2 = subprocess.run(
            [elf_path],
            input=test_input2,
            capture_output=True, text=True,
            timeout=5
        )
        stderr2 = result2.stderr.strip()
        if "EVT_POSITION" in stderr2:
            print("  PASS: EVT_POSITION logged to stderr")
        else:
            print(f"  FAIL: EVT_POSITION not logged, stderr: {stderr2}")

    finally:
        if os.path.exists(elf_path):
            os.unlink(elf_path)

    print()


# ============================================================
# Test 4: Scheduler stdin 集成测试
# ============================================================

def test_scheduler_integration():
    """测试 Scheduler 通过 stdin 接收事件的集成流程"""
    print("=" * 60)
    print("Test 4: Scheduler stdin 集成")
    print("=" * 60)

    sched_file = os.path.join(PLATFORM_SRC, "scheduler_global_618631.py")

    test_input = (
        '{"event":"EVT_CARGO_AVAIL","payload":{"cargo":{"id":"CARGO_001","volume":0.25,"target_shelf":"S1"},"conveyor_id":"C1"}}\n'
        '{"event":"EVT_AGV_IDLE","payload":{"agv_id":"AGV_1","position":{"x":1,"y":0}}}\n'
    )

    result = subprocess.run(
        [sys.executable, sched_file],
        input=test_input,
        capture_output=True, text=True,
        timeout=5
    )

    stdout = result.stdout.strip()
    if "REQ_AGV_ASSIGN" in stdout:
        print("  PASS: Scheduler produced REQ_AGV_ASSIGN via stdin")
        print(f"    output: {stdout}")
    else:
        print(f"  FAIL: no REQ_AGV_ASSIGN in output: {stdout}")

    print()


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    print()
    print("=" * 60)
    print("  平台重封装 - 本地 Mock 测试")
    print("=" * 60)
    print()

    all_passed = True

    try:
        test_scheduler()
    except Exception as e:
        print(f"  SCHEDULER TEST FAILED: {e}\n")
        all_passed = False

    try:
        test_pathplan()
    except Exception as e:
        print(f"  PATHPLAN TEST FAILED: {e}\n")
        all_passed = False

    try:
        test_safetyguard()
    except Exception as e:
        print(f"  SAFETYGUARD TEST FAILED: {e}\n")
        all_passed = False

    try:
        test_scheduler_integration()
    except Exception as e:
        print(f"  INTEGRATION TEST FAILED: {e}\n")
        all_passed = False

    print("=" * 60)
    if all_passed:
        print("  ALL TESTS PASSED")
    else:
        print("  SOME TESTS FAILED")
    print("=" * 60)

    # Windows pause
    if sys.platform == "win32":
        import os as _os
        _os.system("pause")
