#pragma once

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/common/connection_hdl.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <string>
#include <memory>

namespace agv_protocol {

/**
 * @brief WebSocket 客户端封装类
 *
 * 负责与上位机服务器建立 WebSocket 连接，发送关节位置数据，接收控制命令
 */
class WebSocketClientWrapper {
public:
    using WebSocketConfig = websocketpp::config::asio_client;
    using Client = websocketpp::client<WebSocketConfig>;
    using MessagePtr = Client::message_ptr;
    using ConnectionHdl = websocketpp::connection_hdl;

    /// @brief 消息接收回调函数类型
    using MessageHandler = std::function<void(const std::string&)>;

    /// @brief 连接状态变化回调函数类型
    using ConnectionHandler = std::function<void(bool)>;

    /**
     * @brief 构造函数
     * @param server_uri 服务器 URI（如 "ws://192.168.1.163:9000"）
     * @param reconnect_interval_ms 重连间隔（毫秒）
     */
    explicit WebSocketClientWrapper(const std::string& server_uri, int reconnect_interval_ms);

    /// @brief 析构函数
    ~WebSocketClientWrapper();

    /**
     * @brief 启动 WebSocket 客户端（开始连接线程）
     */
    void start();

    /**
     * @brief 停止 WebSocket 客户端
     */
    void stop();

    /**
     * @brief 发送消息到服务器
     * @param message 要发送的 JSON 字符串
     */
    void send(const std::string& message);
    bool trySend(const std::string& message);

    /**
     * @brief 设置消息接收处理器
     * @param handler 回调函数
     */
    void setMessageHandler(MessageHandler handler);

    /**
     * @brief 设置连接状态变化处理器
     * @param handler 回调函数
     */
    void setConnectionHandler(ConnectionHandler handler);

    /**
     * @brief 检查是否已连接
     * @return true=已连接，false=未连接
     */
    bool isConnected() const { return is_connected_; }

private:
    // WebSocket 回调函数
    void onOpen(ConnectionHdl hdl);
    void onFail(ConnectionHdl hdl);
    void onClose(ConnectionHdl hdl);
    void onMessage(ConnectionHdl hdl, MessagePtr msg);

    /**
     * @brief 运行事件循环（在独立线程中执行）
     */
    void runEventLoop();

    /**
     * @brief 重置客户端以准备重连
     */
    void resetClient();

    std::string server_uri_;
    int reconnect_interval_ms_;

    std::unique_ptr<Client> client_;
    std::unique_ptr<std::thread> thread_;

    std::atomic<bool> is_connected_{false};
    std::atomic<bool> running_{false};

    ConnectionHdl connection_hdl_;
    std::mutex mutex_;

    MessageHandler message_handler_;
    ConnectionHandler connection_handler_;
};

} // namespace agv_protocol
