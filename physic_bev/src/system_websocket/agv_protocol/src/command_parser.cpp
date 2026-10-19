#include "agv_protocol/command_parser.hpp"
#include <iostream>
#include <sstream>

namespace agv_protocol {

double CommandParser::safeGetDouble(const Json::Value& node, const std::string& key, double default_val) {
    return node.isMember(key) && node[key].isNumeric() ? node[key].asDouble() : default_val;
}

std::string CommandParser::safeGetString(const Json::Value& node, const std::string& key, const std::string& default_val) {
    return node.isMember(key) && node[key].isString() ? node[key].asString() : default_val;
}

bool CommandParser::safeGetBool(const Json::Value& node, const std::string& key, bool default_val) {
    return node.isMember(key) && node[key].isBool() ? node[key].asBool() : default_val;
}

std::optional<CommandData> CommandParser::parse(const std::string& json_str) {
    // ... 原有实现保持兼容 ...
    try {
        Json::Value root;
        Json::CharReaderBuilder reader;
        std::string errors;
        std::stringstream ss(json_str);

        if (!Json::parseFromStream(reader, ss, &root, &errors)) {
            return std::nullopt;
        }

        CommandData cmd;
        if (!parseHeader(root, cmd)) return std::nullopt;

        if (root.isMember("payload") && root["payload"].isObject()) {
            parsePayload(root["payload"], cmd);
        }
        return cmd;
    } catch (...) {
        return std::nullopt;
    }
}

std::optional<RawMessage> CommandParser::parseRaw(const std::string& json_str) {
    try {
        Json::Value root;
        Json::CharReaderBuilder reader;
        std::string errors;
        std::stringstream ss(json_str);

        if (!Json::parseFromStream(reader, ss, &root, &errors)) return std::nullopt;

        RawMessage msg;
        if (!root.isMember("header") || !root["header"].isObject()) return std::nullopt;
        
        msg.header.robot_id = safeGetString(root["header"], "robot_id");
        msg.header.msg_type = safeGetString(root["header"], "msg_type");
        msg.header.schema_version = safeGetString(root["header"], "schema_version", "tank_inspection_v1");
        msg.header.request_id = safeGetString(root["header"], "request_id");
        msg.payload = root["payload"];
        return msg;
    } catch (...) {
        return std::nullopt;
    }
}

bool CommandParser::parseHeader(const Json::Value& root, CommandData& cmd) {
    if (!root.isMember("header") || !root["header"].isObject()) return false;
    const auto& header = root["header"];
    cmd.header.robot_id = safeGetString(header, "robot_id");
    cmd.header.msg_type = safeGetString(header, "msg_type");
    cmd.header.schema_version = safeGetString(header, "schema_version", "tank_inspection_v1");
    cmd.header.request_id = safeGetString(header, "request_id");
    return true;
}

void CommandParser::parsePayload(const Json::Value& payload, CommandData& cmd) {
    // 1. 机械臂 (数组形式)
    if (payload.isMember("arms")) parseArms(payload["arms"], cmd);

    // 2. 底盘 (支持 agv 或 chassis 字段)
    if (payload.isMember("agv")) parseChassis(payload["agv"], cmd);
    else if (payload.isMember("chassis")) parseChassis(payload["chassis"], cmd);

    // 3. 雷达
    if (payload.isMember("lidar")) parseLidar(payload["lidar"], cmd);

    // 4. 相机
    if (payload.isMember("camera")) parseCamera(payload["camera"], cmd);

    // 5. 涡流
    if (payload.isMember("eddy_current")) parseEddy(payload["eddy_current"], cmd);

    // 6. 机器人信息
    if (payload.isMember("robot_info")) parseRobotInfo(payload["robot_info"], cmd);

    // 7. 通用设备状态/自检 (msg_type 为 check 时)
    if (cmd.header.msg_type == "check") parseDeviceChecks(payload, cmd);
}

void CommandParser::parseArms(const Json::Value& arms_node, CommandData& cmd) {
    if (!arms_node.isArray()) return;
    for (const auto& arm : arms_node) {
        ArmCommand ac;
        ac.arm_id = safeGetString(arm, "arm_id");
        ac.command = safeGetString(arm, "command");
        
        const auto& params = arm["parameters"];
        if (params.isObject()) {
            ac.speed = safeGetDouble(params, "speed");
            if (params.isMember("val1")) {
                ac.is_joint = true;
                ac.pos_type = "joint";
                for(int i=0; i<6; ++i) ac.parameters[i] = safeGetDouble(params, "val" + std::to_string(i+1));
            } else {
                ac.is_joint = false;
                ac.pos_type = "cartesian";
                ac.parameters[0] = safeGetDouble(params, "x");
                ac.parameters[1] = safeGetDouble(params, "y");
                ac.parameters[2] = safeGetDouble(params, "z");
                ac.parameters[3] = safeGetDouble(params, "roll");
                ac.parameters[4] = safeGetDouble(params, "pitch");
                ac.parameters[5] = safeGetDouble(params, "yaw");
            }
        }
        cmd.arms.push_back(ac);
    }
    cmd.has_arms = !cmd.arms.empty();
}

void CommandParser::parseChassis(const Json::Value& chassis_node, CommandData& cmd) {
    if (!chassis_node.isObject()) return;
    cmd.chassis.command = safeGetString(chassis_node, "command");
    
    // 数据字段 (data) 或 参数字段 (parameters)
    const auto& data = chassis_node.isMember("data") ? chassis_node["data"] : chassis_node["parameters"];
    if (data.isObject()) {
        cmd.chassis.vx = data.isMember("vx") ? safeGetDouble(data, "vx") : safeGetDouble(data, "x");
        cmd.chassis.vy = data.isMember("vy") ? safeGetDouble(data, "vy") : safeGetDouble(data, "y");
        cmd.chassis.wz = data.isMember("wz") ? safeGetDouble(data, "wz") : safeGetDouble(data, "yaw");
        cmd.chassis.speed = safeGetDouble(data, "speed");
    }

    // 推杆参数 (直接在 node 下)
    cmd.chassis.ig35_start = safeGetDouble(chassis_node, "ig35_start");
    cmd.chassis.scan_speed = safeGetDouble(chassis_node, "scan_speed");
    cmd.chassis.ig35_end = safeGetDouble(chassis_node, "ig35_end");

    cmd.has_chassis = true;
}

void CommandParser::parseLidar(const Json::Value& lidar_node, CommandData& cmd) {
    if (!lidar_node.isObject()) return;
    cmd.lidar.command = safeGetString(lidar_node, "command");
    cmd.lidar.target_x = safeGetDouble(lidar_node, "target_x");
    cmd.lidar.target_y = safeGetDouble(lidar_node, "target_y");
    cmd.lidar.push_accuracy = safeGetDouble(lidar_node, "push_accuracy");
    cmd.lidar.nav_accuracy = safeGetDouble(lidar_node, "nav_accuracy");
    cmd.lidar.scan_speed = safeGetDouble(lidar_node, "scan_speed");
    cmd.lidar.ig35_start = safeGetDouble(lidar_node, "ig35_start");
    cmd.lidar.ig35_end = safeGetDouble(lidar_node, "ig35_end");
    cmd.has_lidar = true;
}

void CommandParser::parseCamera(const Json::Value& camera_node, CommandData& cmd) {
    if (!camera_node.isObject()) return;
    cmd.camera.command = safeGetString(camera_node, "command");
    cmd.camera.x = safeGetDouble(camera_node, "x");
    cmd.camera.y = safeGetDouble(camera_node, "y");
    cmd.camera.scan_mode = safeGetString(camera_node, "scan_mode");
    cmd.camera.spacing = (int)safeGetDouble(camera_node, "spacing");
    cmd.camera.shrink_factor = (int)safeGetDouble(camera_node, "shrink_factor");
    cmd.camera.default_InterPoints = (int)safeGetDouble(camera_node, "default_InterPoints");
    cmd.has_camera = true;
}

void CommandParser::parseEddy(const Json::Value& eddy_node, CommandData& cmd) {
    if (eddy_node.isArray()) {
        for (const auto& item : eddy_node) {
            cmd.eddy.channels.push_back({(int)safeGetDouble(item, "channel_index"), safeGetDouble(item, "data")});
        }
    } else if (eddy_node.isObject()) {
        cmd.eddy.command = safeGetString(eddy_node, "command");
    }
    cmd.has_eddy = true;
}

void CommandParser::parseRobotInfo(const Json::Value& info_node, CommandData& cmd) {
    if (!info_node.isObject()) return;
    cmd.info.battery = safeGetBool(info_node, "battery");
    cmd.info.lidar = safeGetBool(info_node, "lidar");
    cmd.info.camera = safeGetBool(info_node, "camera");
    cmd.info.arm1_position = safeGetBool(info_node, "arm1_position");
    cmd.info.arm2_position = safeGetBool(info_node, "arm2_position");
    cmd.info.agv_position = safeGetBool(info_node, "agv_position");
    cmd.has_info = true;
}

void CommandParser::parseDeviceChecks(const Json::Value& payload, CommandData& cmd) {
    for (const auto& key : payload.getMemberNames()) {
        const auto& dev = payload[key];
        if (dev.isObject() && dev.isMember("command")) {
            cmd.device_checks[key] = dev["command"].asString();
        }
    }
}

// --- 构建方法实现 ---

std::string CommandParser::buildRawMessage(const std::string& robot_id, const std::string& msg_type,
                                           const Json::Value& payload,
                                           const std::string& request_id) {
    Json::Value root;
    root["header"]["robot_id"] = robot_id;
    root["header"]["msg_type"] = msg_type;
    root["header"]["schema_version"] = "tank_inspection_v2";
    if (!request_id.empty()) root["header"]["request_id"] = request_id;
    root["payload"] = payload;
    Json::StreamWriterBuilder writer;
    return Json::writeString(writer, root);
}

std::string CommandParser::buildDeviceResponse(const std::string& robot_id, const std::map<std::string, std::pair<std::string, std::string>>& results) {
    Json::Value payload;
    for (const auto& [name, res] : results) {
        payload[name]["command"] = res.first;
        payload[name]["result"] = res.second;
    }
    return buildRawMessage(robot_id, "response", payload);
}

std::string CommandParser::buildChassisResponse(const std::string& robot_id, const std::string& command, const std::string& result) {
    Json::Value payload;
    payload["agv"]["command"] = command;
    payload["agv"]["result"] = result;
    return buildRawMessage(robot_id, "response", payload);
}

std::string CommandParser::buildArmResponse(const std::string& robot_id, const std::vector<std::pair<std::string, std::string>>& arm_results) {
    Json::Value payload;
    for (size_t i = 0; i < arm_results.size(); ++i) {
        payload["arms"][static_cast<int>(i)]["arm_id"] = arm_results[i].first;
        payload["arms"][static_cast<int>(i)]["result"] = arm_results[i].second;
    }
    return buildRawMessage(robot_id, "response", payload);
}

std::string CommandParser::buildArmStatusUpdate(const std::string& robot_id, const std::vector<ArmCommand>& arm_states) {
    Json::Value payload;
    for (size_t i = 0; i < arm_states.size(); ++i) {
        Json::Value arm;
        arm["arm_id"] = arm_states[i].arm_id;
        arm["pos_type"] = arm_states[i].pos_type;
        if (arm_states[i].is_joint) {
            for(int j=0; j<6; ++j) arm["joint" + std::to_string(j+1)] = arm_states[i].parameters[j];
        } else {
            arm["x"] = arm_states[i].parameters[0]; arm["y"] = arm_states[i].parameters[1]; arm["z"] = arm_states[i].parameters[2];
            arm["rx"] = arm_states[i].parameters[3]; arm["ry"] = arm_states[i].parameters[4]; arm["rz"] = arm_states[i].parameters[5];
        }
        payload["arms"].append(arm);
    }
    return buildRawMessage(robot_id, "status_update", payload);
}

std::string CommandParser::buildLidarPathResponse(const std::string& robot_id, const Json::Value& path_array) {
    Json::Value payload;
    payload["lidar"]["command"] = "nav_path";
    payload["lidar"]["path"] = path_array;
    return buildRawMessage(robot_id, "response", payload);
}

std::string CommandParser::buildEddyResponse(const std::string& robot_id, const std::string& command, const std::string& result_data) {
    Json::Value payload;
    payload["eddy_current"]["command"] = command;
    payload["eddy_current"]["result"] = result_data;
    return buildRawMessage(robot_id, "response", payload);
}

std::string CommandParser::buildSensorStatus(const std::string& robot_id, const std::string& camera_url) {
    Json::Value payload;
    payload["camera_url"] = camera_url;
    return buildRawMessage(robot_id, "status_sensor", payload);
}

} // namespace agv_protocol
