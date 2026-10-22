#pragma once

#include <jsoncpp/json/json.h>
#include <array>
#include <vector>
#include <optional>
#include <string>
#include <map>

namespace agv_protocol {

/**
 * @brief 消息头结构
 */
struct MessageHeader {
    std::string robot_id;    // vacuum_adsorption_robot, mobile_dual_arm_robot, etc.
    std::string msg_type;    // check, command, response, query, status_update, status_sensor
    std::string schema_version; // optional; absent means the legacy v1 schema
    std::string request_id;     // optional response correlation identifier
};

/**
 * @brief 机械臂命令数据
 */
struct ArmCommand {
    std::string arm_id;      // left_arm, right_arm
    std::string command;     // joint_move, cartesian_move
    std::string pos_type;    // joint, cartesian
    std::array<double, 6> parameters{}; // val1-6 or x,y,z,roll,pitch,yaw
    double speed = 0.0;
    bool is_joint = true;
};

/**
 * @brief 底盘/AGV控制数据
 */
struct ChassisCommand {
    std::string command;     // move, emergency_stop, capture, save, work_complete, single_scan
    double vx = 0, vy = 0, wz = 0;      // 速度模式
    double x = 0, y = 0, yaw = 0;       // 坐标模式
    double speed = 0;
    
    // 推杆相关 (single_scan)
    double ig35_start = 0;
    double scan_speed = 0;
    double ig35_end = 0;
};

/**
 * @brief 雷达与导航数据
 */
struct LidarData {
    std::string command;     // nav_path, check, true, false
    // 导航目标
    double target_x = 0;
    double target_y = 0;
    double push_accuracy = 0;
    double nav_accuracy = 0;
    double scan_speed = 0;
    double ig35_start = 0;
    double ig35_end = 0;
    
    // 定位反馈
    double x = 0, y = 0, angle = 0;
    int target_idx = 0;
    int loc_stat = 0;
    std::string state;
};

/**
 * @brief 涡流检测数据
 */
struct EddyCurrentData {
    std::string command;     // start, stop, cloud_point
    struct ChannelData {
        int index;
        double value;
    };
    std::vector<ChannelData> channels;
    std::string cloud_point_base64;
};

/**
 * @brief 相机控制数据
 */
struct CameraData {
    std::string command;     // image_pos, check, true, false
    double x = 0, y = 0;
    std::string scan_mode;
    int spacing = 0;
    int shrink_factor = 0;
    int default_InterPoints = 0;
    std::string rtsp_url;
};

/**
 * @brief 机器人整体状态信息 (robot_info)
 */
struct RobotInfo {
    bool battery = false;
    bool lidar = false;
    bool camera = false;
    bool arm1_position = false;
    bool arm2_position = false;
    bool agv_position = false;
};

/**
 * @brief 综合命令数据包
 */
struct CommandData {
    MessageHeader header;
    
    // 负载模块
    std::vector<ArmCommand> arms;
    ChassisCommand chassis;
    LidarData lidar;
    CameraData camera;
    EddyCurrentData eddy;
    RobotInfo info;
    
    // 设备自检请求 (Device -> command/check/true/false)
    std::map<std::string, std::string> device_checks;
    
    // 标志位
    bool has_arms = false;
    bool has_chassis = false;
    bool has_lidar = false;
    bool has_camera = false;
    bool has_eddy = false;
    bool has_info = false;
};

/**
 * @brief 原始消息报文，用于动态分发
 */
struct RawMessage {
    MessageHeader header;
    Json::Value payload;
};

/**
 * @brief 全新协议解析器
 */
class CommandParser {
public:
    CommandParser() = default;

    // --- 解析接口 ---
    std::optional<CommandData> parse(const std::string& json_str);
    
    // 新增：解析为原始报文，用于动态分发架构
    std::optional<RawMessage> parseRaw(const std::string& json_str);

    // --- 构建接口 (响应与上报) ---
    
    // 通用基础构建
    std::string buildRawMessage(const std::string& robot_id, const std::string& msg_type,
                                const Json::Value& payload,
                                const std::string& request_id = "");

    // 1. 设备使能/状态响应
    std::string buildDeviceResponse(const std::string& robot_id, const std::map<std::string, std::pair<std::string, std::string>>& results);
    
    // 2. 底盘/推杆响应
    std::string buildChassisResponse(const std::string& robot_id, const std::string& command, const std::string& result);
    
    // 3. 机械臂响应
    std::string buildArmResponse(const std::string& robot_id, const std::vector<std::pair<std::string, std::string>>& arm_results);
    
    // 4. 状态上报 (机械臂位姿)
    std::string buildArmStatusUpdate(const std::string& robot_id, const std::vector<ArmCommand>& arm_states);
    
    // 5. 雷达路径响应
    std::string buildLidarPathResponse(const std::string& robot_id, const Json::Value& path_array);
    
    // 6. 涡流数据响应 (包含点云或实时值)
    std::string buildEddyResponse(const std::string& robot_id, const std::string& command, const std::string& result_data);

    // 7. 传感器状态 (如 RTSP URL)
    std::string buildSensorStatus(const std::string& robot_id, const std::string& camera_url);

private:
    // 内部解析方法
    bool parseHeader(const Json::Value& root, CommandData& cmd);
    void parsePayload(const Json::Value& payload, CommandData& cmd);
    void parseArms(const Json::Value& arms_node, CommandData& cmd);
    void parseChassis(const Json::Value& chassis_node, CommandData& cmd);
    void parseLidar(const Json::Value& lidar_node, CommandData& cmd);
    void parseCamera(const Json::Value& camera_node, CommandData& cmd);
    void parseEddy(const Json::Value& eddy_node, CommandData& cmd);
    void parseRobotInfo(const Json::Value& info_node, CommandData& cmd);
    void parseDeviceChecks(const Json::Value& payload, CommandData& cmd);

    // 工具函数
    static double safeGetDouble(const Json::Value& node, const std::string& key, double default_val = 0.0);
    static std::string safeGetString(const Json::Value& node, const std::string& key, const std::string& default_val = "");
    static bool safeGetBool(const Json::Value& node, const std::string& key, bool default_val = false);
};

} // namespace agv_protocol
