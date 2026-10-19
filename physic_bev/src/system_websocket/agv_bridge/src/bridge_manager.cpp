#include "agv_bridge/bridge_manager.hpp"

namespace agv_bridge {

BridgeManager& BridgeManager::getInstance() {
    static BridgeManager instance;
    return instance;
}

void BridgeManager::init(std::shared_ptr<rclcpp::Node> node) {
    auto& instance = getInstance();
    instance.node_ = node;
    instance.setupPublishers();
    instance.setupSubscribers();
}

void BridgeManager::setupPublishers() {
    if (!node_) return;
    chassis_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_vel", 10);
    status_pub_ = node_->create_publisher<std_msgs::msg::String>("robot_status", 10);
}

void BridgeManager::setupSubscribers() {
    if (!node_) return;
    
    // 订阅雷达和相机的心跳/数据话题 (此处假设为标准 Header，可根据实际驱动调整)
    radar_sub_ = node_->create_subscription<std_msgs::msg::Header>(
        "radar_heartbeat", 10, std::bind(&BridgeManager::radarHeartbeatCallback, this, std::placeholders::_1));
        
    camera_sub_ = node_->create_subscription<std_msgs::msg::Header>(
        "camera_heartbeat", 10, std::bind(&BridgeManager::cameraHeartbeatCallback, this, std::placeholders::_1));
}

void BridgeManager::radarHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg) {
    (void)msg;
    std::lock_guard<std::mutex> lock(mutex_);
    last_radar_time_ = node_->now();
}

void BridgeManager::cameraHeartbeatCallback(const std_msgs::msg::Header::SharedPtr msg) {
    (void)msg;
    std::lock_guard<std::mutex> lock(mutex_);
    last_camera_time_ = node_->now();
}

bool BridgeManager::isRadarOnline() {
    auto& instance = getInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    if (!instance.node_) return false;
    auto now = instance.node_->now();
    return (now - instance.last_radar_time_).seconds() < 2.0; // 2秒内有数据认为在线
}

bool BridgeManager::isCameraOnline() {
    auto& instance = getInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    if (!instance.node_) return true;
    auto now = instance.node_->now();
    return (now - instance.last_camera_time_).seconds() < 2.0;
}

// ... 其余 publish 函数保持不变 ...
void BridgeManager::publishChassisCmd(double vx, double vy, double wz) {
    auto& instance = getInstance();
    if (!instance.chassis_pub_) return;
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {vx, vy, wz};
    instance.chassis_pub_->publish(msg);
}

void BridgeManager::publishRobotStatus(const std::string& status_json) {
    auto& instance = getInstance();
    if (!instance.status_pub_) return;
    auto msg = std_msgs::msg::String();
    msg.data = status_json;
    instance.status_pub_->publish(msg);
}

double BridgeManager::getBatteryLevel() {
    auto& instance = getInstance();
    std::lock_guard<std::mutex> lock(instance.mutex_);
    return instance.battery_level_;
}

} // namespace agv_bridge
