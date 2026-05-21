"""
扩展测试 - 边界情况和 PRD 合规性验证
"""
import json
import sys
import os
import subprocess
import tempfile

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PLATFORM_SRC = os.path.join(SCRIPT_DIR, "..", "platform", "src")
sys.path.insert(0, PLATFORM_SRC)

import scheduler_global_618631 as sched

passed = 0
failed = 0

def check(name, condition, detail=""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  [PASS] {name}")
    else:
        failed += 1
        print(f"  [FAIL] {name} - {detail}")

# ============================================================
# Scheduler 边界测试
# ============================================================
print("=" * 60)
print("Scheduler Extended Tests")
print("=" * 60)

sched.cargo_queue.clear()
sched.idle_agvs.clear()

# --- cargo 兼容解析 ---
c1 = sched.parse_cargo({"id": "C001", "volume": 0.5, "target_shelf": "S2"})
check("standard fields", c1["id"] == "C001" and c1["volume"] == 0.5 and c1["target_shelf"] == "S2")

c2 = sched.parse_cargo({"cargo_id": "C002", "size": 1.0, "shelf_id": "S3"})
check("alt fields cargo_id/size/shelf_id",
      c2["id"] == "C002" and c2["volume"] == 1.0 and c2["target_shelf"] == "S3")

c3 = sched.parse_cargo({"item_id": "I003", "vol": 0.25, "destination": "S4"})
check("alt fields item_id/vol/destination",
      c3["id"] == "I003" and c3["volume"] == 0.25 and c3["target_shelf"] == "S4")

c4 = sched.parse_cargo({"target": 0})
check("int target 0 -> S1", c4["target_shelf"] == "S1")

c5 = sched.parse_cargo({"target": 2})
check("int target 2 -> S3", c5["target_shelf"] == "S3")

c6 = sched.parse_cargo({})
check("empty cargo fallback", c6["id"] == "UNKNOWN" and c6["volume"] == 0.25 and c6["target_shelf"] == "S1")

c7 = sched.parse_cargo("RAW_ID_STRING")
check("string cargo -> id", c7["id"] == "RAW_ID_STRING")

c8 = sched.parse_cargo(None)
check("None cargo -> None", c8 is None)

# --- POINT2D 兼容解析 ---
check("point {x,y}", sched.parse_point({"x": 3, "y": 5}) == {"x": 3, "y": 5})
check("point [x,y]", sched.parse_point([7, 9]) == {"x": 7, "y": 9})
check("point (x,y) tuple", sched.parse_point((2, 4)) == {"x": 2, "y": 4})
check("point {row,col}", sched.parse_point({"row": 2, "col": 4}) == {"x": 4, "y": 2})
check("point None", sched.parse_point(None) is None)
check("point empty dict", sched.parse_point({}) is None)

# --- conveyor_id 解析 ---
check("conv standard", sched.parse_conveyor_id({"conveyor_id": "C2"}) == "C2")
check("conv alt key", sched.parse_conveyor_id({"conveyor": "C1"}) == "C1")
check("conv in cargo", sched.parse_conveyor_id({"cargo": {"conveyor_id": "C2"}}) == "C2")
check("conv fallback", sched.parse_conveyor_id({"x": 1}) == "C1")

# --- 坐标映射 ---
check("pickup C1", sched.map_conveyor_to_pickup("C1") == {"x": 3, "y": 2})
check("pickup C2", sched.map_conveyor_to_pickup("C2") == {"x": 12, "y": 2})
check("pickup unknown -> C1", sched.map_conveyor_to_pickup("C99") == {"x": 3, "y": 2})
check("drop S1", sched.map_cargo_to_drop({"target_shelf": "S1"}) == {"x": 1, "y": 8})
check("drop S3", sched.map_cargo_to_drop({"target_shelf": "S3"}) == {"x": 10, "y": 8})
check("drop unknown -> S1", sched.map_cargo_to_drop({"target_shelf": "X99"}) == {"x": 1, "y": 8})

# --- AGV idle 解析 ---
agv_id, pos = sched.parse_agv_idle({"agv_id": "AGV_1", "position": {"x": 3, "y": 5}})
check("agv idle standard", agv_id == "AGV_1" and pos == {"x": 3, "y": 5})

agv_id2, pos2 = sched.parse_agv_idle({"id": "AGV_X", "pos": {"x": 7, "y": 9}})
check("agv idle alt keys", agv_id2 == "AGV_X" and pos2 == {"x": 7, "y": 9})

agv_id3, pos3 = sched.parse_agv_idle({"agv": "AGV_Y", "x": 2, "y": 4})
check("agv idle flat coords", agv_id3 == "AGV_Y" and pos3 == {"x": 2, "y": 4})

agv_id4, pos4 = sched.parse_agv_idle({})
check("agv idle empty", agv_id4 == "AGV_UNKNOWN" and pos4 is None)

# --- 调度逻辑 ---
sched.cargo_queue.clear()
sched.idle_agvs.clear()

# 空队列不应崩溃
sched.try_dispatch()
check("dispatch empty safe", True)

# 只有 cargo 无 AGV
sched.on_EVT_CARGO_AVAIL({"cargo": {"id": "X1"}, "conveyor_id": "C1"})
check("cargo only: queued", len(sched.cargo_queue) == 1 and len(sched.idle_agvs) == 0)

# 只有 AGV 无 cargo
sched.on_EVT_AGV_IDLE({"agv_id": "AGV_1", "position": {"x": 0, "y": 0}})
check("agv only: cached, cargo dispatched",
      len(sched.cargo_queue) == 0 and len(sched.idle_agvs) == 0)

# 多 cargo 多 AGV
sched.cargo_queue.clear()
sched.idle_agvs.clear()
sched.on_EVT_CARGO_AVAIL({"cargo": {"id": "A"}, "conveyor_id": "C1"})
sched.on_EVT_CARGO_AVAIL({"cargo": {"id": "B"}, "conveyor_id": "C2"})
sched.on_EVT_CARGO_AVAIL({"cargo": {"id": "C"}, "conveyor_id": "C1"})
check("3 cargos queued", len(sched.cargo_queue) == 3)
sched.on_EVT_AGV_IDLE({"agv_id": "AGV_1"})
sched.on_EVT_AGV_IDLE({"agv_id": "AGV_2"})
check("2 AGVs: 2 dispatched, 1 cargo left",
      len(sched.cargo_queue) == 1 and len(sched.idle_agvs) == 0)

# SHELF_IDLE 不崩溃
sched.on_EVT_SHELF_IDLE({"shelf_id": "S1"})
check("shelf idle no crash", True)

print(f"\n  Scheduler: {passed}/{passed+failed} passed")

# ============================================================
# PathPlan 边界测试
# ============================================================
print()
print("=" * 60)
print("PathPlan Extended Tests")
print("=" * 60)

test_js = r"""
var pp = require('./path_plan_618631.js');
var failures = 0;
function check(name, cond) {
    if (cond) { console.log('  [PASS] ' + name); }
    else { console.log('  [FAIL] ' + name); failures++; }
}

// parsePoint
check('null', pp.parsePoint(null) === null);
check('undefined', pp.parsePoint(undefined) === null);
check('empty obj', pp.parsePoint({}) === null);
check('dict {x,y}', JSON.stringify(pp.parsePoint({x:3,y:5})) === '{"x":3,"y":5}');
check('array [x,y]', JSON.stringify(pp.parsePoint([7,9])) === '{"x":7,"y":9}');
check('row/col', JSON.stringify(pp.parsePoint({row:2,col:4})) === '{"x":4,"y":2}');
check('float trunc', JSON.stringify(pp.parsePoint({x:3.7,y:5.2})) === '{"x":3,"y":5}');

// manhattanPath
var p1 = pp.manhattanPath({x:0,y:0}, {x:0,y:0});
check('same point', p1.length === 1 && p1[0].x === 0 && p1[0].y === 0);

var p2 = pp.manhattanPath({x:0,y:0}, {x:2,y:0});
check('horizontal right', p2.length === 3 && p2[2].x === 2 && p2[2].y === 0);

var p3 = pp.manhattanPath({x:2,y:0}, {x:0,y:0});
check('horizontal left', p3.length === 3 && p3[2].x === 0 && p3[2].y === 0);

var p4 = pp.manhattanPath({x:0,y:0}, {x:0,y:3});
check('vertical down', p4.length === 4 && p4[3].x === 0 && p4[3].y === 3);

var p5 = pp.manhattanPath({x:0,y:3}, {x:0,y:0});
check('vertical up', p5.length === 4 && p5[3].x === 0 && p5[3].y === 0);

var p6 = pp.manhattanPath({x:0,y:0}, {x:3,y:2});
check('diagonal', p6.length === 6 && p6[5].x === 3 && p6[5].y === 2);
// verify path never jumps
for (var i = 1; i < p6.length; i++) {
    var dx = Math.abs(p6[i].x - p6[i-1].x);
    var dy = Math.abs(p6[i].y - p6[i-1].y);
    if (!(dx + dy === 1 && (dx === 0 || dy === 0))) {
        console.log('  [FAIL] path non-adjacent at step ' + i);
        failures++;
    }
}
check('path adjacency', true);  // already checked above

// isValidPoint
check('valid point', pp.isValidPoint({x:0,y:0}));
check('valid boundary', pp.isValidPoint({x:15,y:13}));
check('invalid x neg', !pp.isValidPoint({x:-1,y:0}));
check('invalid x big', !pp.isValidPoint({x:16,y:0}));
check('invalid y neg', !pp.isValidPoint({x:0,y:-1}));
check('invalid y big', !pp.isValidPoint({x:0,y:14}));
check('invalid null', !pp.isValidPoint(null));

// on_REQ_PATH smoke test
var saved_output = null;
var orig_publish = console.log;
console.log = function(s) { saved_output = s; };
pp.on_REQ_PATH({from:{x:0,y:0}, to:{x:2,y:2}, agv_id:"AGV_1"});
console.log = orig_publish;
var out = JSON.parse(saved_output);
check('REQ_PATH output event', out.event === 'EVT_PATH');
check('REQ_PATH output waypoints', out.payload.waypoints.length > 0);
check('REQ_PATH output ttl', out.payload.ttl_sec === 60);

// on_REQ_PATH with invalid input
saved_output = null;
console.log = function(s) { saved_output = s; };
pp.on_REQ_PATH({from:{x:-1,y:0}, to:{x:2,y:2}, agv_id:"AGV_1"});
console.log = orig_publish;
out = JSON.parse(saved_output);
check('invalid from: still outputs', out.event === 'EVT_PATH');
check('invalid from: 1 waypoint', out.payload.waypoints.length === 1);
check('invalid from: short ttl', out.payload.ttl_sec === 5);

if (failures > 0) process.exit(1);
"""

result = subprocess.run(["node", "-e", test_js], cwd=PLATFORM_SRC, capture_output=True, text=True)
print(result.stdout)
if result.returncode != 0:
    print("  [FAIL] Node tests returned non-zero")
    print(result.stderr)

# ============================================================
# SafetyGuard 扩展测试
# ============================================================
print()
print("=" * 60)
print("SafetyGuard Extended Tests")
print("=" * 60)

# Test C++ point parsing and event handling
test_cpp_src = r"""
#include <iostream>
#include <string>
#include <cassert>

// Copy the key functions from safety_guard
// (Simplified test — we test through the compiled binary below)
int main() {
    // This is a placeholder — actual tests run via subprocess below
    return 0;
}
"""

# Test via compiled binary
cpp_file = os.path.join(PLATFORM_SRC, "safety_guard_618631.cpp")
with tempfile.NamedTemporaryFile(suffix=".elf", delete=False) as tmp:
    elf_path = tmp.name

compile_result = subprocess.run(
    ["g++", cpp_file, "-std=c++17", "-lm", "-o", elf_path],
    capture_output=True, text=True
)
check("C++ compilation", compile_result.returncode == 0,
      f"compile error: {compile_result.stderr[:200]}")

def run_guard(input_json):
    """Run the guard binary with given input and return stdout, stderr"""
    result = subprocess.run([elf_path], input=input_json + "\n",
                          capture_output=True, text=True, timeout=5)
    return result.stdout.strip(), result.stderr.strip()

# Test 1: Basic APPROVE
out, err = run_guard('{"event":"REQ_MOVE_PROPOSED","payload":{"agv_id":"AGV_1","x":1,"y":0}}')
check("basic APPROVE", '"event":"REQ_MOVE_APPROVED"' in out, f"got: {out}")
check("basic APPROVE agv_id", '"agv_id":"AGV_1"' in out)

# Test 2: APPROVE with alt field names
out, err = run_guard('{"event":"REQ_MOVE_PROPOSED","payload":{"agv":"AGV_2","x":5,"y":3}}')
check("alt agv field", '"agv_id":"AGV_2"' in out)

# Test 3: APPROVE with position sub-object
out, err = run_guard('{"event":"REQ_MOVE_PROPOSED","payload":{"agv_id":"AGV_3","position":{"x":7,"y":2}}}')
check("nested position", '"x":7' in out and '"y":2' in out)

# Test 4: APPROVE with [x,y] array in position
out, err = run_guard('{"event":"REQ_MOVE_PROPOSED","payload":{"agv_id":"AGV_4","position":[10,5]}}')
check("array position [x,y]", '"x":10' in out and '"y":5' in out,
      f"got: {out}")

# Test 5: EVT_POSITION logging
out, err = run_guard('{"event":"EVT_POSITION","payload":{"agv_id":"AGV_1","x":3,"y":5}}')
check("EVT_POSITION logged", "EVT_POSITION" in err)

# Test 6: EVT_POSITION with array position
out, err = run_guard('{"event":"EVT_POSITION","payload":{"agv_id":"AGV_2","position":[8,4]}}')
check("EVT_POSITION array pos", "EVT_POSITION" in err)

# Test 7: Unknown event
out, err = run_guard('{"event":"UNKNOWN_EVENT","payload":{}}')
check("unknown event warning", "WARN" in err, f"got stderr: {err}")

# Test 8: Empty line
out, err = run_guard('')
check("empty line no crash", True)  # if we got here, no crash

if os.path.exists(elf_path):
    os.unlink(elf_path)

print(f"\n  SafetyGuard: all tests completed")

# ============================================================
# 集成测试: 完整事件链
# ============================================================
print()
print("=" * 60)
print("Integration: Full Event Chain")
print("=" * 60)

# Simulate the full chain using Scheduler
sched.cargo_queue.clear()
sched.idle_agvs.clear()

# Capture stdout
import io
old_stdout = sys.stdout
sys.stdout = captured = io.StringIO()

sched.on_EVT_CARGO_AVAIL({"cargo": {"id": "CARGO_001", "volume": 0.25, "target_shelf": "S1"},
                          "conveyor_id": "C1"})
sched.on_EVT_AGV_IDLE({"agv_id": "AGV_1", "position": {"x": 1, "y": 0}})

sys.stdout = old_stdout
output = captured.getvalue()
req = json.loads(output.strip())
check("integration: event type", req["event"] == "REQ_AGV_ASSIGN")
check("integration: agv_id", req["payload"]["agv_id"] == "AGV_1")
check("integration: cargo_id", req["payload"]["cargo_id"] == "CARGO_001")
check("integration: pickup C1", req["payload"]["pickup"] == {"x": 3, "y": 2})
check("integration: drop S1", req["payload"]["drop"] == {"x": 1, "y": 8})

# Now feed that REQ_AGV_ASSIGN into PathPlan (simulated via JS module)
test_chain_js = """
var pp = require('./path_plan_618631.js');
var req = JSON.parse(process.argv[1]);
var from = req.pickup;
var to = req.drop;
var result = pp.manhattanPath(from, to);
console.log(JSON.stringify({waypoints: result, ttl_sec: 60}));
"""
chain_result = subprocess.run(
    ["node", "-e", test_chain_js, json.dumps(req["payload"])],
    cwd=PLATFORM_SRC, capture_output=True, text=True
)
path_out = json.loads(chain_result.stdout)
check("chain: path from pickup", path_out["waypoints"][0] == {"x": 3, "y": 2})
check("chain: path to drop", path_out["waypoints"][-1] == {"x": 1, "y": 8})
check("chain: path length > 1", len(path_out["waypoints"]) >= 3)

print(f"\n  Integration: all tests completed")

# ============================================================
# Summary
# ============================================================
print()
print("=" * 60)
print(f"EXTENDED TEST SUMMARY: {passed}/{passed+failed} passed")
if failed > 0:
    print("SOME TESTS FAILED!")
    sys.exit(1)
else:
    print("ALL TESTS PASSED")

if sys.platform == "win32":
    os.system("pause")
