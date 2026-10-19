#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <memory>
#include <mutex>

namespace agv_bridge {

class BridgeManager {
public:
    static void init(std::shared_ptr<rclcpp::Node> node);

    // 发布接口
    static void publishChassisCmd(double vx, double vy, double wz);
    static void publishRobotStatus(const std::string& status_json);

    // 状态查询接口 (新增)
    static bool isRadarOnline();
    static bool isCameraOnline();
    static double getBatteryLevel();

private:
    BridgeManager() = default;
    static BridgeManager& getInstance();

    void setupPublishers();
    void setupSubscribers();

    // 订阅者回调 (新增)
    void radarHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg);
    void cameraHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr chassis_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // 订阅者 (新增)
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr radar_sub_;
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr camera_sub_;

    // 状态数据 (新增)
    rclcpp::Time last_radar_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_camera_time_{0, 0, RCL_ROS_TIME};
    double battery_level_ = 100.0;

    std::mutex mutex_;
};

} // namespace agv_bridge
