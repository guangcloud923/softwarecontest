/**
 * SafetyGuard.Collision - 边侧安全守护
 * 文件: src/safety_guard_618631.cpp
 *
 * 职责: AGV 的 proposed move 能不能执行?
 *
 * 第一阶段 (最小会动版): 直接 APPROVE, 但保留位置缓存和检测框架。
 * 第二阶段: 同格碰撞检测 + 边冲突检测。
 * 第三阶段: 容量约束 + 库存一致性校验。
 *
 * 编译: g++ src/safety_guard_618631.cpp -std=c++17 -lm -o dist/safety_guard_618631.elf
 */

#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstring>

// ============================================================
// 简易 JSON 解析 (不引入第三方库)
// ============================================================

// 跳过空白
static const char* skip_ws(const char* p) {
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
    return p;
}

// 读取字符串 (期望 "xxxx")
static std::string json_read_str(const char*& p) {
    p = skip_ws(p);
    if (*p != '"') return "";
    p++;
    std::string s;
    while (*p && *p != '"') {
        if (*p == '\\') { p++; if (*p) s += *p++; }
        else s += *p++;
    }
    if (*p == '"') p++;
    return s;
}

// 读取数字
static double json_read_num(const char*& p) {
    p = skip_ws(p);
    char* end;
    double v = strtod(p, &end);
    p = end;
    return v;
}

// 跳过值
static void json_skip_value(const char*& p) {
    p = skip_ws(p);
    if (*p == '"') { json_read_str(p); return; }
    if (*p == '{') {
        p++; int d = 1;
        while (*p && d) { if (*p == '{') d++; if (*p == '}') d--; p++; }
        return;
    }
    if (*p == '[') {
        p++; int d = 1;
        while (*p && d) { if (*p == '[') d++; if (*p == ']') d--; p++; }
        return;
    }
    if (*p == 't' || *p == 'f' || *p == 'n') { while (*p && *p != ',' && *p != '}' && *p != ']') p++; return; }
    json_read_num(p);
}

// 在 JSON 对象中找 key 并返回其值的文本指针 (不复制)
static bool json_get_field(const std::string& json, const std::string& key,
                           std::string& value_out) {
    const char* p = json.c_str();
    p = skip_ws(p);
    if (*p != '{') return false;
    p++;

    while (*p) {
        p = skip_ws(p);
        if (*p == '}') break;
        if (*p == ',') { p++; continue; }

        std::string k = json_read_str(p);
        p = skip_ws(p);
        if (*p != ':') return false;
        p++; // skip ':'

        if (k == key) {
            // 记录值的起止位置
            p = skip_ws(p);
            const char* start = p;
            if (*p == '"') {
                value_out = json_read_str(p);
                return true;
            }
            if (*p == '{') {
                int d = 1; p++;
                while (*p && d) { if (*p == '{') d++; if (*p == '}') d--; p++; }
                value_out = std::string(start, p - start);
                return true;
            }
            if (*p == '[') {
                int d = 1; p++;
                while (*p && d) { if (*p == '[') d++; if (*p == ']') d--; p++; }
                value_out = std::string(start, p - start);
                return true;
            }
            if (*p == 't' || *p == 'f') {
                while (*p && *p != ',' && *p != '}' && *p != ']') p++;
                value_out = std::string(start, p - start);
                return true;
            }
            json_read_num(p);
            value_out = std::string(start, p - start);
            return true;
        }
        json_skip_value(p);
    }
    return false;
}

static int json_get_int(const std::string& json, const std::string& key, int default_val = 0) {
    std::string val;
    if (!json_get_field(json, key, val)) return default_val;
    return atoi(val.c_str());
}

static double json_get_double(const std::string& json, const std::string& key, double default_val = 0.0) {
    std::string val;
    if (!json_get_field(json, key, val)) return default_val;
    return atof(val.c_str());
}

// 直接从 json 中取顶层 key 的字符串值
static std::string json_get_str(const std::string& json, const std::string& key,
                                const std::string& default_val = "") {
    std::string val;
    if (!json_get_field(json, key, val)) return default_val;
    return val;
}

// ============================================================
// 数据结构
// ============================================================

struct Point2D {
    int x = 0;
    int y = 0;
};

struct AGVState {
    Point2D pos;
    double load_vol = 0.0;
    bool active = false;
};

// ============================================================
// 全局状态
// ============================================================

static std::unordered_map<std::string, AGVState> g_agv_states;

// ============================================================
// 日志
// ============================================================

static void log_event(const std::string& event_type, const std::string& detail,
                      const std::string& direction = "IN") {
    std::cerr << "[SafetyGuard] " << direction << " " << event_type
              << ": " << detail << std::endl;
}

// ============================================================
// 坐标解析 (兼容多种格式)
// ============================================================

// 解析 JSON 数组: 从 "[1, 2]" 中读取第 idx 个整数
static int json_array_get_int(const std::string& json, int idx) {
    const char* p = json.c_str();
    p = skip_ws(p);
    if (*p != '[') return -1;
    p++;
    int cur = 0;
    while (*p) {
        p = skip_ws(p);
        if (*p == ']') break;
        if (*p == ',') { p++; continue; }
        double v = json_read_num(p);
        if (cur == idx) return (int)v;
        cur++;
    }
    return -1;
}

static Point2D parse_point_from_json(const std::string& json) {
    Point2D pt;
    const char* p = skip_ws(json.c_str());
    // 数组格式: [x, y]
    if (*p == '[') {
        pt.x = json_array_get_int(json, 0);
        pt.y = json_array_get_int(json, 1);
        return pt;
    }
    // 对象格式: {x, y} 或 {row, col}
    pt.x = json_get_int(json, "x", 0);
    pt.y = json_get_int(json, "y", 0);
    if (pt.x == 0 && pt.y == 0) {
        int row = json_get_int(json, "row", -1);
        int col = json_get_int(json, "col", -1);
        if (row >= 0 && col >= 0) {
            pt.x = col;
            pt.y = row;
        }
    }
    return pt;
}

// ============================================================
// 碰撞检测 (第二阶段启用)
// ============================================================

// 安全距离
static const int SAFE_DIST = 1;

/**
 * 同格碰撞检测: 目标位置是否被其他 AGV 占据
 */
static bool check_same_cell(const std::string& agv_id, const Point2D& target) {
    for (const auto& kv : g_agv_states) {
        if (kv.first == agv_id) continue;
        if (!kv.second.active) continue;
        if (kv.second.pos.x == target.x && kv.second.pos.y == target.y) {
            return true; // 冲突
        }
    }
    return false;
}

/**
 * 安全距离检测: 目标位置是否离其他 AGV 太近
 */
static bool check_too_close(const std::string& agv_id, const Point2D& target) {
    for (const auto& kv : g_agv_states) {
        if (kv.first == agv_id) continue;
        if (!kv.second.active) continue;
        int dx = std::abs(kv.second.pos.x - target.x);
        int dy = std::abs(kv.second.pos.y - target.y);
        if (dx + dy < SAFE_DIST) {
            return true;
        }
    }
    return false;
}

// ============================================================
// 事件处理
// ============================================================

/**
 * EVT_POSITION: 更新 AGV 位置缓存
 */
static void on_EVT_POSITION(const std::string& payload_json) {
    log_event("EVT_POSITION", payload_json);

    std::string agv_id = json_get_str(payload_json, "agv_id",
                        json_get_str(payload_json, "id",
                        json_get_str(payload_json, "agv", "UNKNOWN")));

    AGVState& st = g_agv_states[agv_id];
    // 位置可能在顶层或嵌套在 position 字段中
    std::string pos_json;
    if (json_get_field(payload_json, "position", pos_json)) {
        st.pos = parse_point_from_json(pos_json);
    } else if (json_get_field(payload_json, "pos", pos_json)) {
        st.pos = parse_point_from_json(pos_json);
    } else {
        st.pos = parse_point_from_json(payload_json);
    }
    st.active = true;
}

/**
 * REQ_MOVE_PROPOSED: 判断 proposed move 能否执行
 * 第一阶段: 直接 APPROVE
 */
static void on_REQ_MOVE_PROPOSED(const std::string& payload_json) {
    log_event("REQ_MOVE_PROPOSED", payload_json);

    std::string agv_id = json_get_str(payload_json, "agv_id",
                        json_get_str(payload_json, "id",
                        json_get_str(payload_json, "agv", "UNKNOWN")));

    Point2D target;
    std::string pos_json;
    if (json_get_field(payload_json, "position", pos_json) ||
        json_get_field(payload_json, "target", pos_json)) {
        target = parse_point_from_json(pos_json);
    } else {
        target = parse_point_from_json(payload_json);
    }

    // ---- 第一阶段: 直接 APPROVE ----
    std::cout << "{\"event\":\"REQ_MOVE_APPROVED\",\"payload\":{"
              << "\"agv_id\":\"" << agv_id << "\","
              << "\"x\":" << target.x << ","
              << "\"y\":" << target.y
              << "}}" << std::endl;
    log_event("REQ_MOVE_APPROVED",
              "agv_id=" + agv_id + " x=" + std::to_string(target.x) +
              " y=" + std::to_string(target.y), "OUT");

    // ---- 第二阶段: 取消注释以启用碰撞检测 ----
    // if (check_same_cell(agv_id, target)) {
    //     std::cout << "{\"event\":\"REQ_MOVE_REJECTED\",\"payload\":{"
    //               << "\"agv_id\":\"" << agv_id << "\","
    //               << "\"reason\":\"collision: same cell occupied\""
    //               << "}}" << std::endl;
    //     log_event("REQ_MOVE_REJECTED", "agv_id=" + agv_id + " reason=collision", "OUT");
    //     return;
    // }
    //
    // if (check_too_close(agv_id, target)) {
    //     std::cout << "{\"event\":\"REQ_MOVE_REJECTED\",\"payload\":{"
    //               << "\"agv_id\":\"" << agv_id << "\","
    //               << "\"reason\":\"collision: too close to another AGV\""
    //               << "}}" << std::endl;
    //     log_event("REQ_MOVE_REJECTED", "agv_id=" + agv_id + " reason=too_close", "OUT");
    //     return;
    // }
    //
    // std::cout << "{\"event\":\"REQ_MOVE_APPROVED\",\"payload\":{"
    //           << "\"agv_id\":\"" << agv_id << "\","
    //           << "\"x\":" << target.x << ","
    //           << "\"y\":" << target.y
    //           << "}}" << std::endl;
}

// ============================================================
// 主入口
// ============================================================

int main() {
    std::cerr << "[SafetyGuard] SafetyGuard.Collision started, waiting for events..."
              << std::endl;

    std::string line;
    while (std::getline(std::cin, line)) {
        if (line.empty()) continue;

        std::string event_type = json_get_str(line, "event",
                                 json_get_str(line, "type", ""));

        std::string payload;
        if (!json_get_field(line, "payload", payload)) {
            // 整个 line 就是 payload (简化处理)
            payload = line;
        }

        if (event_type == "EVT_POSITION") {
            on_EVT_POSITION(payload);
        } else if (event_type == "REQ_MOVE_PROPOSED") {
            on_REQ_MOVE_PROPOSED(payload);
        } else {
            std::cerr << "[SafetyGuard] WARN: unknown event type: "
                      << event_type << std::endl;
        }
    }

    return 0;
}
