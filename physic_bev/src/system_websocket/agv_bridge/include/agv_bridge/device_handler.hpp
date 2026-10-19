#pragma once

#include <rclcpp/rclcpp.hpp>
#include <jsoncpp/json/json.h>
#include "agv_protocol/command_parser.hpp"
#include <memory>
#include <string>
#include <vector>

namespace agv_bridge {

/**
 * @brief 异步事件结构
 */
struct AsyncEvent {
    std::string msg_type = "command"; // 默认为 command，支持 query 等
    Json::Value payload;
    bool flat_payload = false;        // 如果为 true，则不被 handler name 包装，直接放入 payload
};

/**
 * @brief 设备处理器基类
 */
class DeviceHandler {
public:
    using Ptr = std::shared_ptr<DeviceHandler>;
    virtual ~DeviceHandler() = default;

    // 初始化：由 Node 调用，用于 Handler 创建自己的 Publisher/Subscriber
    virtual void init(rclcpp::Node* node) = 0;

    // 命令处理：处理 payload 中对应自己的部分
    // 返回值：处理结果 Json，将直接放入响应包的对应设备 Key 下
    virtual Json::Value handleCommand(const Json::Value& device_payload, const agv_protocol::MessageHeader& header) = 0;

    // 状态检查：处理 msg_type 为 "check" 时的在线/离线检测逻辑
    virtual bool isOnline() = 0;

    // 获取实时状态报告 (周期性回传)
    virtual Json::Value getReport() {
        return Json::Value(Json::objectValue);
    }

    // 🌟 新增：获取待发送的异步事件 (可能作为 "command" 或 "query" 发送)
    virtual std::vector<AsyncEvent> getPendingEvents() {
        return {};
    }

    // 获取多个实时状态报告。默认兼容单报告处理器。
    virtual std::vector<Json::Value> getReports() {
        Json::Value report = getReport();
        if (report.empty()) {
            return {};
        }
        return {report};
    }
};

} // namespace agv_bridge
