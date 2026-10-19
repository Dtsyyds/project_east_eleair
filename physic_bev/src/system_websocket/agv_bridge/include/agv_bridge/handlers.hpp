#pragma once

#include "agv_bridge/device_handler.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <mutex>
#include <cmath>
#include <algorithm>
#include <condition_variable>
#include <chrono>
#include <websocketpp/base64/base64.hpp>

namespace agv_bridge {

/**
 * @brief 通用设备处理器基类，封装心跳判断逻辑
 */
class BaseHandler : public DeviceHandler {
public:
    void updateHeartbeat() {
        std::lock_guard<std::mutex> lock(mutex_);
        last_update_ = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    bool isOnline() override {
        if (!node_) return false;
        std::lock_guard<std::mutex> lock(mutex_);
        return (node_->now() - last_update_).seconds() < 2.0;
    }

protected:
    rclcpp::Node* node_ = nullptr;
    rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
    std::mutex mutex_;
};

// ==========================================
// 底盘部分 (Chassis)
// ==========================================

class BaseChassisHandler : public BaseHandler {
protected:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sys_pub_;
};

/**
 * @brief 墙壁机器人底盘 (Vacuum Adsorption)
 */
class WallChassisHandler : public BaseChassisHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        manual_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_manual", 10);
        sys_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/sys_command", 10);
        
        sub_ = node->create_subscription<std_msgs::msg::String>(
            "/chassis/serial_status", 10, [this](const std_msgs::msg::String::SharedPtr) { updateHeartbeat(); });

        // 🌟 订阅任务事件：当主控发起作业请求时，我们需要转发给上位机
        event_sub_ = node->create_subscription<std_msgs::msg::String>(
            "/mission/events", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
                if (msg->data == "capture" || msg->data == "save" || msg->data == "work_complete") {
                    std::lock_guard<std::mutex> lock(event_mutex_);
                    AsyncEvent ev;
                    ev.msg_type = "command";
                    ev.payload["command"] = msg->data;
                    pending_events_.push_back(ev);
                    RCLCPP_INFO(node_->get_logger(), "📥 捕获到 ROS 事件 [%s]，准备向本地/远程上位机申请授权...", msg->data.c_str());
                }
            });
    }

    std::vector<AsyncEvent> getPendingEvents() override {
        std::lock_guard<std::mutex> lock(event_mutex_);
        auto events = std::move(pending_events_);
        pending_events_.clear();
        return events;
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader& header) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        
        // 🌟 处理上位机的回复 (Response)
        if (header.msg_type == "response") {
            if (data.isMember("result")) {
                std::string result = data["result"].asString();
                RCLCPP_INFO(node_->get_logger(), "📤 收到上位机对 [%s] 的响应: %s", cmd.c_str(), result.c_str());
                
                // 如果上位机确认成功，我们将确认信号发布回 ROS
                auto msg = std_msgs::msg::String();
                msg.data = result; // 例如 "capture_successed"
                sys_pub_->publish(msg);
            }
            return Json::Value(); // 回复包不需要再回复
        }

        if (cmd == "move") {
            const auto& p = data.isMember("parameters") ? data["parameters"] : data["data"];
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x = p.isMember("vx") ? p["vx"].asDouble() : (p.isMember("x") ? p["x"].asDouble() : 0.0);
            msg.linear.y = p.isMember("vy") ? p["vy"].asDouble() : (p.isMember("y") ? p["y"].asDouble() : 0.0);
            msg.angular.z = p.isMember("wz") ? p["wz"].asDouble() : (p.isMember("yaw") ? p["yaw"].asDouble() : 0.0);
            manual_pub_->publish(msg);
            res["result"] = "yes";
        } 
        else if (cmd == "single_scan") {
            Json::Value cmd_json;
            cmd_json["command"] = "single_scan";
            const auto& p = data.isMember("parameters") ? data["parameters"] : data;
            
            cmd_json["ig35_start"] = p.isMember("ig35_start") ? p["ig35_start"] : 11;
            cmd_json["ig35_end"] = p.isMember("ig35_end") ? p["ig35_end"] : 0;
            cmd_json["scan_speed"] = p.isMember("scan_speed") ? p["scan_speed"] : 20;
            
            auto msg = std_msgs::msg::String();
            Json::StreamWriterBuilder writer;
            msg.data = Json::writeString(writer, cmd_json);
            sys_pub_->publish(msg);
            res["result"] = "single_scan_successed";
        }
        else if (cmd == "capture" || cmd == "save" || cmd == "work_complete") {
            // 这里处理的是上位机主动下发的指令（非 Response）
            auto msg = std_msgs::msg::String();
            msg.data = cmd; 
            sys_pub_->publish(msg);
            res["result"] = msg.data;
        }

        else {
            res["result"] = cmd + "_successed";
        }
        res["command"] = cmd;
        return res;
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    
    // 🌟 异步确认机制
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr event_sub_;
    std::vector<AsyncEvent> pending_events_;
    std::mutex event_mutex_;
};

/**
 * @brief 复合机器人底盘 (Mobile Dual Arm)
 */
class AgvChassisHandler : public BaseChassisHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        agv_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_vel", 10);
        sys_pub_ = node->create_publisher<std_msgs::msg::String>("/mission/sys_command", 10);
        
        sub_ = node->create_subscription<geometry_msgs::msg::Pose2D>(
            "/agv_pose", 10, [this](const geometry_msgs::msg::Pose2D::SharedPtr) { updateHeartbeat(); });
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "move") {
            const auto& p = data.isMember("parameters") ? data["parameters"] : data["data"];
            auto msg = std_msgs::msg::Float64MultiArray();
            double vx = p.isMember("vx") ? p["vx"].asDouble() : (p.isMember("x") ? p["x"].asDouble() : 0.0);
            double vy = p.isMember("vy") ? p["vy"].asDouble() : (p.isMember("y") ? p["y"].asDouble() : 0.0);
            double wz = p.isMember("wz") ? p["wz"].asDouble() : (p.isMember("yaw") ? p["yaw"].asDouble() : 0.0);
            msg.data = {vx, vy, wz};
            agv_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = cmd + "_successed";
        }
        res["command"] = cmd;
        return res;
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr agv_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sub_;
};

// ==========================================
// 机械臂部分 (Arms)
// ==========================================

class ArmHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        joint_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_joint", 10);
        cart_pub_ = node->create_publisher<std_msgs::msg::Float64MultiArray>("cmd_arm_cartesian", 10);
        
        auto qos = rclcpp::SensorDataQoS();
        sub_joint_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_joint_states", qos, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                updateHeartbeat();
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_joint_ = msg;
            });

        sub_cart_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/arm_cartesian_pose", qos, [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                updateHeartbeat();
                std::lock_guard<std::mutex> lock(data_mutex_);
                last_cart_ = msg;
            });
            
        // 🌟 墙壁机器人可能使用标准的 /joint_states
        sub_standard_ = node->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr) { updateHeartbeat(); });
    }

    Json::Value getReport() override {
        auto reports = getReports();
        return reports.empty() ? Json::Value(Json::arrayValue) : reports.front();
    }

    std::vector<Json::Value> getReports() override {
        std::lock_guard<std::mutex> lock(data_mutex_);
        std::vector<Json::Value> reports;
        if (last_joint_) reports.push_back(buildArmReport(last_joint_->data, "joint"));
        if (last_cart_) reports.push_back(buildArmReport(last_cart_->data, "cartesian"));
        return reports;
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res_array(Json::arrayValue);
        if (data.isArray()) {
            auto joint_msg = std_msgs::msg::Float64MultiArray();
            auto cart_msg = std_msgs::msg::Float64MultiArray();

            for (const auto& arm : data) {
                std::string cmd = arm["command"].asString();
                const auto& p = arm["parameters"];
                if (cmd == "joint_move") {
                    appendJointPoints(p, joint_msg.data);
                } else {
                    appendCartesianPoints(p, cart_msg.data);
                }
                Json::Value arm_res;
                arm_res["arm_id"] = arm["arm_id"];
                arm_res["command"] = cmd;
                arm_res["result"] = cmd + "_successed";
                res_array.append(arm_res);
            }

            if (!joint_msg.data.empty()) joint_pub_->publish(joint_msg);
            if (!cart_msg.data.empty()) cart_pub_->publish(cart_msg);
        }
        return res_array;
    }

private:
    static double numericOrZero(const Json::Value& value) { return value.isNumeric() ? value.asDouble() : 0.0; }

    static void appendJointPoint(const Json::Value& point, std::vector<double>& out) {
        for (int i = 1; i <= 6; ++i) out.push_back(numericOrZero(point["val" + std::to_string(i)]));
    }

    static void appendCartesianPoint(const Json::Value& point, std::vector<double>& out) {
        out.push_back(numericOrZero(point["x"]));
        out.push_back(numericOrZero(point["y"]));
        out.push_back(numericOrZero(point["z"]));
        out.push_back(point.isMember("rx") ? numericOrZero(point["rx"]) : numericOrZero(point["roll"]));
        out.push_back(point.isMember("ry") ? numericOrZero(point["ry"]) : numericOrZero(point["pitch"]));
        out.push_back(point.isMember("rz") ? numericOrZero(point["rz"]) : numericOrZero(point["yaw"]));
    }

    static void appendJointPoints(const Json::Value& parameters, std::vector<double>& out) {
        if (parameters.isArray()) { for (const auto& point : parameters) appendJointPoint(point, out); return; }
        if (parameters.isObject() && parameters["points"].isArray()) { for (const auto& point : parameters["points"]) appendJointPoint(point, out); return; }
        appendJointPoint(parameters, out);
    }

    static void appendCartesianPoints(const Json::Value& parameters, std::vector<double>& out) {
        if (parameters.isArray()) { for (const auto& point : parameters) appendCartesianPoint(point, out); return; }
        if (parameters.isObject() && parameters["points"].isArray()) { for (const auto& point : parameters["points"]) appendCartesianPoint(point, out); return; }
        appendCartesianPoint(parameters, out);
    }

    static double valueAt(const std::vector<double>& data, size_t index) { return index < data.size() ? data[index] : 0.0; }

    static Json::Value buildOneArm(const std::vector<double>& data, const std::string& arm_id, const std::string& pos_type, size_t offset) {
        Json::Value arm;
        arm["arm_id"] = arm_id;
        arm["pos_type"] = pos_type;
        if (pos_type == "joint") {
            for (int i = 0; i < 6; ++i) arm["joint" + std::to_string(i + 1)] = valueAt(data, offset + i);
        } else {
            arm["x"] = valueAt(data, offset + 0); arm["y"] = valueAt(data, offset + 1); arm["z"] = valueAt(data, offset + 2);
            arm["rx"] = valueAt(data, offset + 3); arm["ry"] = valueAt(data, offset + 4); arm["rz"] = valueAt(data, offset + 5);
        }
        return arm;
    }

    static Json::Value buildArmReport(const std::vector<double>& data, const std::string& pos_type) {
        Json::Value arms(Json::arrayValue);
        arms.append(buildOneArm(data, "left_arm", pos_type, 0));
        arms.append(buildOneArm(data, "right_arm", pos_type, 6));
        return arms;
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cart_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_joint_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_cart_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_standard_;
    std_msgs::msg::Float64MultiArray::SharedPtr last_joint_;
    std_msgs::msg::Float64MultiArray::SharedPtr last_cart_;
    std::mutex data_mutex_;
};

// ==========================================
// 雷达部分 (Radar)
// ==========================================

class BaseRadarHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        auto_pub_ = node->create_publisher<std_msgs::msg::String>("/cmd_vel_auto", 10);
        
        // 🌟 订阅主控规划好的路径话题
        path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
            "/mission/planned_path", 10, 
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                RCLCPP_INFO(node_->get_logger(), "📍 [ServerNode] 成功捕获主控下发的规划路径！点数: %zu", msg->poses.size());
                std::lock_guard<std::mutex> lock(path_mutex_);
                last_path_ = msg;
                new_path_available_ = true;
            });
    }

    Json::Value getPathReport() {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!new_path_available_ || !last_path_) return Json::Value();
        
        Json::Value lidar;
        lidar["command"] = "nav_path";
        Json::Value path_array(Json::arrayValue);
        
        for (size_t i = 0; i < last_path_->poses.size(); ++i) {
            const auto& pose = last_path_->poses[i].pose;
            Json::Value p;
            p["x"] = pose.position.x;
            p["y"] = pose.position.y;
            
            // 四元数转偏航角 (简单 2D 情况)
            double qz = pose.orientation.z;
            double qw = pose.orientation.w;
            double theta = 2.0 * std::atan2(qz, qw);
            p["theta"] = theta;
            
            // 默认类型逻辑：最后一个点为 target，其余为 normal
            p["type"] = (i == last_path_->poses.size() - 1) ? "target" : "normal";
            path_array.append(p);
        }
        
        lidar["path"] = path_array;
        new_path_available_ = false;
        return lidar;
    }

    Json::Value getReport() override {
        Json::Value lidar;
        if (last_odom_) {
            lidar["x"] = last_odom_->pose.pose.position.x;
            lidar["y"] = last_odom_->pose.pose.position.y;
            lidar["angle"] = last_odom_->pose.pose.orientation.z; 
            lidar["target_idx"] = 0;
            lidar["loc_stat"] = isOnline() ? 1 : 0;
            lidar["state"] = isOnline() ? "running" : "stopped";
        }
        return lidar;
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "nav_path") {
            auto msg = std_msgs::msg::String();
            Json::StreamWriterBuilder writer;
            msg.data = Json::writeString(writer, data);
            auto_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = "yes";
        }
        res["command"] = cmd;
        return res;
    }

protected:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr auto_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    
    // 🌟 新增：路径缓存
    nav_msgs::msg::Path::SharedPtr last_path_;
    bool new_path_available_ = false;
    std::mutex path_mutex_;
};

class RadarHandler : public BaseRadarHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseRadarHandler::init(node);
        sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                updateHeartbeat();
                last_odom_ = msg;
            });
    }
};

// ==========================================
// 相机部分 (Camera)
// ==========================================

class BaseCameraHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        click_pub_ = node->create_publisher<geometry_msgs::msg::PointStamped>("/click_point", 10);
    }

    Json::Value handleCommand(const Json::Value& data, const agv_protocol::MessageHeader&) override {
        Json::Value res;
        std::string cmd = data["command"].asString();
        if (cmd == "image_pos") {
            auto msg = geometry_msgs::msg::PointStamped();
            msg.header.stamp = node_->now();
            msg.header.frame_id = "camera_color_optical_frame";
            msg.point.x = data["x"].asDouble();
            msg.point.y = data["y"].asDouble();
            msg.point.z = 0.0;
            click_pub_->publish(msg);
            res["result"] = "yes";
        } else {
            res["result"] = "yes";
        }
        res["command"] = cmd;
        return res;
    }

protected:
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr click_pub_;
};

class WallCameraHandler : public BaseCameraHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseCameraHandler::init(node);
        auto qos = rclcpp::SensorDataQoS();
        sub_wall_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
        sub_rect_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/camera/camera/color/image_rect_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_wall_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_rect_;
};

class AgvCameraHandler : public BaseCameraHandler {
public:
    void init(rclcpp::Node* node) override {
        BaseCameraHandler::init(node);
        auto qos = rclcpp::SensorDataQoS();
        sub_agv_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/color/image_raw", qos, [this](const sensor_msgs::msg::Image::SharedPtr) { updateHeartbeat(); });
    }
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_agv_;
};

/**
 * @brief 涡流/点云处理器 (Eddy Current / Point Cloud)
 * 
 * 功能：订阅 /path_planner/path_3d 话题，转换格式并 Base64 编码，主动作为 query 发送
 */
class EddyCurrentHandler : public BaseHandler {
public:
    void init(rclcpp::Node* node) override {
        node_ = node;
        // 获取话题名称，默认为 /path_planner/path_3d
        if (!node_->has_parameter("output_3d_path_topic")) {
            node_->declare_parameter("output_3d_path_topic", "/path_planner/path_3d");
        }
        std::string topic = node_->get_parameter("output_3d_path_topic").as_string();

        sub_pc_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            topic, 10, std::bind(&EddyCurrentHandler::pointCloudCallback, this, std::placeholders::_1));
    }

    Json::Value handleCommand(const Json::Value&, const agv_protocol::MessageHeader&) override {
        return Json::Value(); 
    }

    std::vector<AsyncEvent> getPendingEvents() override {
        std::lock_guard<std::mutex> lock(mutex_);
        auto events = std::move(pending_events_);
        pending_events_.clear();
        return events;
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        updateHeartbeat();
        RCLCPP_INFO(node_->get_logger(), "[EddyCurrentHandler] 收到点云数据, 点数: %u", msg->width * msg->height);

        // 🌟 按照 C# 端的点结构转换：24字节 (float x,y,z, r,g,b 或 rx,ry,rz)
        // 注意：PointCloud2 中的 RGB 可能是打包的，也可能是分开的
        
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        
        // 尝试获取姿态字段 (rx, ry, rz) 或颜色字段 (rgb, r, g, b)
        bool has_pose = false;
        bool has_rgb = false;
        bool has_separate_rgb = false;
        
        for (const auto& field : msg->fields) {
            if (field.name == "rx") has_pose = true;
            if (field.name == "rgb") has_rgb = true;
            if (field.name == "r") has_separate_rgb = true;
        }

        RCLCPP_INFO(node_->get_logger(), "[EddyCurrentHandler] 处理模式: %s", 
            has_pose ? "6D 路径 (Pose)" : (has_rgb || has_separate_rgb ? "彩色点云 (Color)" : "仅位置 (XYZ)"));

        std::vector<float> packed_data;
        packed_data.reserve(msg->width * msg->height * 6);

        if (has_pose) {
            sensor_msgs::PointCloud2ConstIterator<float> iter_rx(*msg, "rx");
            sensor_msgs::PointCloud2ConstIterator<float> iter_ry(*msg, "ry");
            sensor_msgs::PointCloud2ConstIterator<float> iter_rz(*msg, "rz");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rx, ++iter_ry, ++iter_rz) {
                packed_data.push_back(*iter_x);
                packed_data.push_back(*iter_y);
                packed_data.push_back(*iter_z);
                packed_data.push_back(*iter_rx);
                packed_data.push_back(*iter_ry);
                packed_data.push_back(*iter_rz);
            }
        } else if (has_rgb) {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_rgb(*msg, "rgb");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
                packed_data.push_back(*iter_x);
                packed_data.push_back(*iter_y);
                packed_data.push_back(*iter_z);
                // RGB 打包通常是 BGRA 或 RGBA，取决于偏移。假设标准 offset 16-18
                packed_data.push_back(static_cast<float>(iter_rgb[2])); // R
                packed_data.push_back(static_cast<float>(iter_rgb[1])); // G
                packed_data.push_back(static_cast<float>(iter_rgb[0])); // B
            }
        } else if (has_separate_rgb) {
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(*msg, "r");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(*msg, "g");
            sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(*msg, "b");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
                packed_data.push_back(*iter_x);
                packed_data.push_back(*iter_y);
                packed_data.push_back(*iter_z);
                packed_data.push_back(static_cast<float>(*iter_r));
                packed_data.push_back(static_cast<float>(*iter_g));
                packed_data.push_back(static_cast<float>(*iter_b));
            }
        } else {
            // 无颜色/姿态，填 0
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                packed_data.push_back(*iter_x);
                packed_data.push_back(*iter_y);
                packed_data.push_back(*iter_z);
                packed_data.push_back(0.0f); packed_data.push_back(0.0f); packed_data.push_back(0.0f);
            }
        }

        // 转换二进制并 Base64 编码
        std::string binary_str(reinterpret_cast<const char*>(packed_data.data()), packed_data.size() * sizeof(float));
        std::string base64_str = websocketpp::base64_encode(binary_str);

        RCLCPP_INFO(node_->get_logger(), "[EddyCurrentHandler] 数据打包完成, Base64长度: %zu, 放入待发送队列", base64_str.size());

        // 封装为异步 query 事件 (按照新协议格式)
        AsyncEvent ev;
        ev.msg_type = "query";
        ev.flat_payload = true; // 🌟 关键：不被 handler name (eddy_current) 包装
        
        Json::Value path_cloud_point;
        path_cloud_point["command"] = "cloud_point";
        path_cloud_point["result"] = base64_str;
        
        ev.payload["path_cloud_point"] = path_cloud_point;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_events_.push_back(ev);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    std::vector<AsyncEvent> pending_events_;
};

} // namespace agv_bridge
