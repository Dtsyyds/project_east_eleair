#pragma once

#include <rclcpp/rclcpp.hpp>
#include <jsoncpp/json/json.h>
#include "agv_protocol/command_parser.hpp"
#include "agv_bridge/device_handler.hpp"
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace agv_bridge {

/**
 * @brief 机器人抽象基类
 */
class Robot {
public:
    using Ptr = std::shared_ptr<Robot>;
    virtual ~Robot() = default;
    virtual void tick() {}

    virtual void init(rclcpp::Node* node) = 0;
    
    // 获取启动该机器人的 shell 命令
    virtual std::string getLaunchCommand() = 0;

    // 处理常规业务指令
    virtual Json::Value handleCommand(const agv_protocol::RawMessage& msg) = 0;

    // 处理自检指令
    virtual Json::Value handleCheck(const agv_protocol::RawMessage& msg) = 0;

    // 生成周期性状态报告（支持多条消息，如复合机器人机械臂的分开上报）
    virtual std::vector<std::string> generateReports(agv_protocol::CommandParser& parser) = 0;

    // 🌟 新增：生成主动发送给上位机的异步指令消息 (msg_type: "command")
    virtual std::vector<std::string> generateEvents(agv_protocol::CommandParser& parser) = 0;

    virtual std::string getRobotId() const = 0;
};

/**
 * @brief 基础机器人实现，通过组合 DeviceHandler 来完成工作
 */
class BaseRobot : public Robot {
public:
    BaseRobot(const std::string& robot_id) : robot_id_(robot_id) {}

    void init(rclcpp::Node* node) override {
        node_ = node;
        for (auto& [name, handler] : handlers_) {
            handler->init(node);
        }
    }

    void addHandler(const std::string& name, DeviceHandler::Ptr handler) {
        handlers_[name] = handler;
    }

    Json::Value handleCommand(const agv_protocol::RawMessage& msg) override {
        Json::Value response_payload;
        for (const auto& key : msg.payload.getMemberNames()) {
            if (handlers_.count(key)) {
                response_payload[key] = handlers_[key]->handleCommand(msg.payload[key], msg.header);
            }
        }
        return response_payload;
    }

    Json::Value handleCheck(const agv_protocol::RawMessage& msg) override {
        Json::Value response_payload;
        for (const auto& key : msg.payload.getMemberNames()) {
            if (handlers_.count(key)) {
                Json::Value dev_res;
                dev_res["command"] = msg.payload[key]["command"].asString();
                dev_res["result"] = handlers_[key]->isOnline() ? "yes" : "no";
                response_payload[key] = dev_res;
            }
        }
        return response_payload;
    }

    std::vector<std::string> generateReports(agv_protocol::CommandParser& parser) override {
        Json::Value payload;
        for (auto const& [name, handler] : handlers_) {
            Json::Value r = handler->getReport();
            if (!r.empty()) payload[name] = r;
        }

        if (payload.empty()) return {};
        return { parser.buildRawMessage(robot_id_, "status_update", payload) };
    }

    std::vector<std::string> generateEvents(agv_protocol::CommandParser& parser) override {
        std::vector<std::string> event_msgs;
        for (auto const& [name, handler] : handlers_) {
            auto events = handler->getPendingEvents();
            for (const auto& ev : events) {
                Json::Value payload;
                if (ev.flat_payload) {
                    payload = ev.payload;
                } else {
                    payload[name] = ev.payload;
                }
                event_msgs.push_back(parser.buildRawMessage(robot_id_, ev.msg_type, payload));
            }
        }
        return event_msgs;
    }

    std::string getRobotId() const override { return robot_id_; }

protected:
    std::string robot_id_;
    rclcpp::Node* node_ = nullptr;
    std::map<std::string, DeviceHandler::Ptr> handlers_;
};

} // namespace agv_bridge
