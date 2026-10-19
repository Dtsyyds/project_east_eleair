#pragma once

#include <websocketpp/server.hpp>
#include <websocketpp/config/asio.hpp>
#include <functional>
#include <thread>
#include <atomic>
#include <string>
#include <memory>
#include <set>
#include <mutex>

namespace agv_protocol {

/**
 * @brief 本地 WebSocket 服务器封装类
 *
 * 负责在本地启动 WebSocket 服务器，接收本地控制指令
 */
class WebSocketServerWrapper {
public:
    using Server = websocketpp::server<websocketpp::config::asio>;
    using MessagePtr = Server::message_ptr;
    using ConnectionHdl = websocketpp::connection_hdl;

    /// @brief 消息接收回调函数类型
    using MessageHandler = std::function<void(const std::string&)>;

    /**
     * @brief 构造函数
     * @param port 监听端口
     */
    explicit WebSocketServerWrapper(int port);

    /// @brief 析构函数
    ~WebSocketServerWrapper();

    /**
     * @brief 启动服务器
     */
    void start();

    /**
     * @brief 停止服务器
     */
    void stop();

    // Bounded best-effort fanout to local wireless clients.
    bool send(const std::string& message);
    bool hasClients() const;

    /**
     * @brief 设置消息接收处理器
     * @param handler 回调函数
     */
    void setMessageHandler(MessageHandler handler);

private:
    /**
     * @brief 运行服务器（在独立线程中执行）
     */
    void runServer();

    int port_;

    std::unique_ptr<Server> server_;
    std::unique_ptr<std::thread> thread_;

    std::atomic<bool> running_{false};

    MessageHandler message_handler_;
    mutable std::mutex connections_mutex_;
    std::set<ConnectionHdl, std::owner_less<ConnectionHdl>> connections_;
};

} // namespace agv_protocol
