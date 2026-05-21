/**
 * policy.PathPlan - 路径规划
 * 文件: src/path_plan_618631.js
 *
 * 职责: from -> to 生成 waypoints
 *
 * 第一阶段: 曼哈顿路径
 * 后续升级: A*, 时空A*, Reservation Table, 等待动作, 冲突检测, 局部重规划
 */

// ============================================================
// 兼容解析 POINT2D
// ============================================================

function parsePoint(pt) {
    if (pt === null || pt === undefined) {
        return null;
    }
    if (Array.isArray(pt) && pt.length >= 2) {
        return { x: Math.floor(pt[0]), y: Math.floor(pt[1]) };
    }
    if (typeof pt === 'object') {
        var x = pt.x;
        var y = pt.y;
        if (x !== undefined && y !== undefined) {
            return { x: Math.floor(x), y: Math.floor(y) };
        }
        // row/col fallback
        var row = pt.row;
        var col = pt.col;
        if (row !== undefined && col !== undefined) {
            return { x: Math.floor(col), y: Math.floor(row) };
        }
    }
    return null;
}

// ============================================================
// 曼哈顿路径生成
// ============================================================

function manhattanPath(from, to) {
    var path = [];
    var x = from.x;
    var y = from.y;
    path.push({ x: x, y: y });

    // 先走 x 方向, 再走 y 方向
    while (x !== to.x) {
        x += x < to.x ? 1 : -1;
        path.push({ x: x, y: y });
    }
    while (y !== to.y) {
        y += y < to.y ? 1 : -1;
        path.push({ x: x, y: y });
    }

    return path;
}

// ============================================================
// 路径合法性检查
// ============================================================

function isValidPoint(pt) {
    if (pt === null || pt === undefined) return false;
    if (typeof pt.x !== 'number' || typeof pt.y !== 'number') return false;
    if (pt.x < 0 || pt.x >= 16 || pt.y < 0 || pt.y >= 14) return false;
    return true;
}

// ============================================================
// REQ_PATH 处理
// ============================================================

function on_REQ_PATH(payload) {
    logEvent("REQ_PATH", payload, "IN");

    var from = parsePoint(payload.from);
    var to = parsePoint(payload.to);
    var agv_id = payload.agv_id || payload.agv || "UNKNOWN";

    if (!isValidPoint(from) || !isValidPoint(to)) {
        // 不合法: 返回原地等待路径
        var fallback = from || { x: 0, y: 0 };
        var evt = {
            waypoints: [fallback],
            ttl_sec: 5
        };
        logEvent("EVT_PATH", evt, "OUT(WARN:invalid)");
        publish_EVT_PATH(evt);
        return;
    }

    var waypoints = manhattanPath(from, to);
    var evt = {
        waypoints: waypoints,
        ttl_sec: 60
    };

    logEvent("EVT_PATH", evt, "OUT");
    publish_EVT_PATH(evt);
}

// ============================================================
// 日志
// ============================================================

function logEvent(eventType, payload, direction) {
    try {
        var payloadStr = JSON.stringify(payload);
        console.error("[PathPlan] " + direction + " " + eventType + ": " + payloadStr);
    } catch (e) {
        console.error("[PathPlan] " + direction + " " + eventType + ": " + payload);
    }
}

// ============================================================
// 事件发布 (TODO: 平台真实 API 确认后替换)
// ============================================================

function publish_EVT_PATH(evt) {
    /**
     * 发布 EVT_PATH 事件。
     * TODO: 确认平台真实 publish 方式后替换此处实现。
     */
    var msg = { event: "EVT_PATH", payload: evt };
    console.log(JSON.stringify(msg));
}

// ============================================================
// 主入口
// ============================================================

function main() {
    console.error("[PathPlan] policy.PathPlan started, waiting for events...");

    // 从 stdin 读取 JSON 行
    var readline = require('readline');
    var rl = readline.createInterface({
        input: process.stdin,
        output: process.stdout,
        terminal: false
    });

    rl.on('line', function (line) {
        line = line.trim();
        if (!line) return;

        try {
            var msg = JSON.parse(line);
        } catch (e) {
            console.error("[PathPlan] WARN: invalid JSON: " + line);
            return;
        }

        var eventType = msg.event || msg.type || "";
        var payload = msg.payload || msg.data || msg;

        if (eventType === "REQ_PATH") {
            on_REQ_PATH(payload);
        } else {
            console.error("[PathPlan] WARN: unknown event type: " + eventType);
        }
    });
}

// 导出给平台调用
if (typeof module !== 'undefined' && module.exports) {
    module.exports = {
        on_REQ_PATH: on_REQ_PATH,
        parsePoint: parsePoint,
        manhattanPath: manhattanPath,
        isValidPoint: isValidPoint
    };
}

// 直接运行时进入 stdin 循环
if (require.main === module) {
    main();
}
