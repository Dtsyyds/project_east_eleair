#include "agv_protocol/websocket_server.hpp"
#include <iostream>
#include <chrono>
#include <stdexcept>

namespace agv_protocol {

WebSocketServerWrapper::WebSocketServerWrapper(int port)
    : port_(port)
    , server_(std::make_unique<Server>())
{
    // 初始化 ASIO
    server_->init_asio();
    server_->set_reuse_addr(true);
    server_->set_max_message_size(1024 * 1024);

    // 设置日志级别
    server_->clear_access_channels(websocketpp::log::alevel::all);
    server_->set_access_channels(
        websocketpp::log::alevel::connect |
        websocketpp::log::alevel::disconnect
    );

    server_->clear_error_channels(websocketpp::log::elevel::all);
}

WebSocketServerWrapper::~WebSocketServerWrapper()
{
    stop();
}

void WebSocketServerWrapper::start()
{
    if (running_) {
        return;
    }


    // 设置消息处理器
    server_->set_message_handler([this](ConnectionHdl hdl, MessagePtr msg) {
        (void)hdl;  // 避免未使用参数警告
        try {
            std::string payload = msg->get_payload();
            if (message_handler_) {
                message_handler_(payload);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "[WSServer] 处理本地消息异常：" << e.what() << std::endl;
        }
    });

    // 设置连接处理器
    server_->set_open_handler([this](ConnectionHdl hdl) {
        { std::lock_guard<std::mutex> lock(connections_mutex_);
          if(connections_.size()>=8) {
              websocketpp::lib::error_code ec;
              server_->close(hdl,websocketpp::close::status::policy_violation,"local client limit",ec);
              return;
          }
          connections_.insert(hdl);
        }
        Server::connection_ptr con = server_->get_con_from_hdl(hdl);
        std::cout << "[WSServer] 本地客户端连接：" << con->get_remote_endpoint() << std::endl;
    });

    server_->set_close_handler([this](ConnectionHdl hdl) {
        { std::lock_guard<std::mutex> lock(connections_mutex_); connections_.erase(hdl); }
        std::cout << "[WSServer] 本地客户端断开连接" << std::endl;
    });

    // ASIO copies endpoint handlers when the accept connection is created.
    // Register all handlers before start_accept, including the very first peer.
    websocketpp::lib::error_code listen_error;
    server_->listen(port_,listen_error);
    if(listen_error)throw std::runtime_error("WebSocket listen failed: "+listen_error.message());
    server_->start_accept(listen_error);
    if(listen_error)throw std::runtime_error("WebSocket accept failed: "+listen_error.message());
    running_ = true;
    thread_ = std::make_unique<std::thread>(&WebSocketServerWrapper::runServer, this);
}

void WebSocketServerWrapper::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;

    if(server_) {
        websocketpp::lib::error_code ec;server_->stop_listening(ec);
        {
            std::lock_guard<std::mutex> lock(connections_mutex_);
            for(auto hdl:connections_)server_->close(hdl,websocketpp::close::status::normal,"Server shutdown",ec);
        }
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
        while(hasClients() && std::chrono::steady_clock::now()<deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        server_->stop();
    }

    // 等待线程结束
    if (thread_ && thread_->joinable()) {
        try {
            thread_->join();
            thread_.reset();
        }
        catch (const std::exception& e) {
            std::cerr << "[WSServer] 等待线程结束时出错：" << e.what() << std::endl;
        }
    }
}

void WebSocketServerWrapper::setMessageHandler(MessageHandler handler)
{
    message_handler_ = std::move(handler);
}

bool WebSocketServerWrapper::hasClients() const {
    std::lock_guard<std::mutex> lock(connections_mutex_);
    return !connections_.empty();
}

bool WebSocketServerWrapper::send(const std::string& message) {
    if (message.size() > 1024 * 1024) return false;
    std::lock_guard<std::mutex> lock(connections_mutex_);
    bool sent = false;
    for (auto hdl : connections_) {
        websocketpp::lib::error_code ec;
        auto con = server_->get_con_from_hdl(hdl, ec);
        if (ec || con->get_buffered_amount() + message.size() > 2 * 1024 * 1024) continue;
        server_->send(hdl, message, websocketpp::frame::opcode::text, ec);
        sent = sent || !ec;
    }
    return sent;
}

void WebSocketServerWrapper::runServer()
{
    try {
        std::cout << "[WSServer] 本地 WebSocket 服务器已启动，监听端口：" << port_ << std::endl;

        // 运行服务器
        server_->run();
    }
    catch (const std::exception& e) {
        std::cerr << "[WSServer] 本地 WebSocket 服务器异常：" << e.what() << std::endl;
    }
}

} // namespace agv_protocol
