#include "agv_protocol/websocket_client.hpp"
#include <jsoncpp/json/json.h>
#include <iostream>
#include <chrono>

namespace agv_protocol {

WebSocketClientWrapper::WebSocketClientWrapper(const std::string& server_uri, int reconnect_interval_ms)
    : server_uri_(server_uri)
    , reconnect_interval_ms_(reconnect_interval_ms)
    , client_(std::make_unique<Client>())
{
    // 初始化 ASIO
    client_->init_asio();
    client_->set_max_message_size(1024 * 1024);

    // 设置日志级别
    client_->clear_access_channels(websocketpp::log::alevel::all);
    client_->set_access_channels(
        websocketpp::log::alevel::connect |
        websocketpp::log::alevel::disconnect |
        websocketpp::log::alevel::app
    );

    client_->clear_error_channels(websocketpp::log::elevel::all);
    client_->set_error_channels(
        websocketpp::log::elevel::rerror |
        websocketpp::log::elevel::fatal
    );

    // 设置处理器
    client_->set_open_handler(std::bind(&WebSocketClientWrapper::onOpen, this, std::placeholders::_1));
    client_->set_fail_handler(std::bind(&WebSocketClientWrapper::onFail, this, std::placeholders::_1));
    client_->set_close_handler(std::bind(&WebSocketClientWrapper::onClose, this, std::placeholders::_1));
    client_->set_message_handler(std::bind(&WebSocketClientWrapper::onMessage, this,
        std::placeholders::_1, std::placeholders::_2));
}

WebSocketClientWrapper::~WebSocketClientWrapper()
{
    stop();
}

void WebSocketClientWrapper::start()
{
    if (running_) {
        return;
    }

    running_ = true;
    is_connected_ = false;

    thread_ = std::make_unique<std::thread>(&WebSocketClientWrapper::runEventLoop, this);
}

void WebSocketClientWrapper::stop()
{
    if (!running_) {
        return;
    }

    running_ = false;

    std::unique_lock<std::mutex> transport_lock(mutex_);
    // 关闭连接
    if (client_ && is_connected_) {
        try {
            websocketpp::lib::error_code ec;
            client_->close(connection_hdl_, websocketpp::close::status::normal, "Client shutdown", ec);
            if (ec) {
                std::cout << "[WSClient] 关闭连接失败：" << ec.message() << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cout << "[WSClient] 关闭连接异常：" << e.what() << std::endl;
        }
    }

    // Let ASIO write the close frame and receive the peer acknowledgement.
    // Bound this wait; never stop the event loop before a healthy peer replies.
    transport_lock.unlock();
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(300);
    while(is_connected_ && std::chrono::steady_clock::now()<deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(client_)client_->stop();
    }

    // 等待线程结束
    if (thread_ && thread_->joinable()) {
        try {
            thread_->join();
            thread_.reset();
        }
        catch (const std::exception& e) {
            // std::cerr << "[WSClient] 等待线程结束时出错：" << e.what() << std::endl;
        }
    }

    is_connected_ = false;
}

void WebSocketClientWrapper::send(const std::string& message) { (void)trySend(message); }

bool WebSocketClientWrapper::trySend(const std::string& message) {
    if (message.size() > 1024 * 1024) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_connected_ || !client_) return false;
    websocketpp::lib::error_code ec;
    auto con = client_->get_con_from_hdl(connection_hdl_, ec);
    if (ec || con->get_buffered_amount() + message.size() > 2 * 1024 * 1024) return false;
    client_->send(connection_hdl_, message, websocketpp::frame::opcode::text, ec);
    return !ec;
}

void WebSocketClientWrapper::setMessageHandler(MessageHandler handler)
{
    message_handler_ = std::move(handler);
}

void WebSocketClientWrapper::setConnectionHandler(ConnectionHandler handler)
{
    connection_handler_ = std::move(handler);
}

void WebSocketClientWrapper::onOpen(ConnectionHdl hdl)
{
    std::lock_guard<std::mutex> lock(mutex_);
    is_connected_ = true;
    connection_hdl_ = hdl;

    if (connection_handler_) {
        connection_handler_(true);
    }

    // 发送连接确认消息（新协议格式）
    // try {
    //     Json::Value root;
    //     root["header"]["robot_id"] = "mobile_dual_arm_robot";
    //     root["header"]["msg_type"] = "check";
    //     root["payload"]["agv"]["command"] = "check";
    //     root["payload"]["arm1"]["command"] = "check";
    //     root["payload"]["arm2"]["command"] = "check";

    //     Json::StreamWriterBuilder writer;
    //     std::string json_str = Json::writeString(writer, root);

    //     websocketpp::lib::error_code ec;
    //     client_->send(hdl, json_str, websocketpp::frame::opcode::text, ec);

    //     if (ec) {
    //         std::cerr << "[WSClient] 发送连接确认失败：" << ec.message() << std::endl;
    //     } else {
    //         std::cout << "[WSClient] 发送连接确认消息" << std::endl;
    //     }
    // }
    // catch (const std::exception& e) {
    //     std::cerr << "[WSClient] 发送连接确认异常：" << e.what() << std::endl;
    // }
}

void WebSocketClientWrapper::onFail(ConnectionHdl hdl)
{
    std::lock_guard<std::mutex> lock(mutex_);
    is_connected_ = false;

    if (connection_handler_) {
        connection_handler_(false);
    }

    Client::connection_ptr con = client_->get_con_from_hdl(hdl);
    // std::cerr << "[WSClient] 连接失败：" << con->get_ec().message() << std::endl;
}

void WebSocketClientWrapper::onClose(ConnectionHdl hdl)
{
    std::lock_guard<std::mutex> lock(mutex_);
    is_connected_ = false;

    if (connection_handler_) {
        connection_handler_(false);
    }

    Client::connection_ptr con = client_->get_con_from_hdl(hdl);
    // std::cerr << "[WSClient] 连接关闭：" << con->get_remote_close_reason() << std::endl;
}

void WebSocketClientWrapper::onMessage(ConnectionHdl hdl, MessagePtr msg)
{
    (void)hdl;

    try {
        std::string payload = msg->get_payload();
        if (message_handler_) {
            message_handler_(payload);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "[WSClient] 处理消息异常：" << e.what() << std::endl;
    }
}

void WebSocketClientWrapper::runEventLoop()
{
    while (running_) {
        try {
            websocketpp::lib::error_code ec;

            // 创建连接对象
            Client::connection_ptr con = client_->get_connection(server_uri_, ec);
            if (!ec) {
                // 设置连接超时
                con->set_open_handshake_timeout(5000);  // 5 秒
                con->set_close_handshake_timeout(3000); // 3 秒
            }
            if (ec) {
                std::cerr << "[WSClient] 创建连接失败：" << ec.message() << std::endl;
                for (int elapsed = 0; running_ && elapsed < reconnect_interval_ms_; elapsed += 20)
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            // 设置连接参数
            con->append_header("User-Agent", "AGV-Hardware-Client/1.0");

            // 保存连接句柄
            { std::lock_guard<std::mutex> lock(mutex_); connection_hdl_ = con->get_handle(); }

            // 连接到服务器
            client_->connect(con);

            // 运行事件循环（阻塞直到连接断开）
            client_->run();

        }
        catch (const websocketpp::exception& e) {
            std::cerr << "[WSClient] WebSocket 异常：" << e.what() << std::endl;
        }
        catch (const std::exception& e) {
            std::cerr << "[WSClient] 连接线程异常：" << e.what() << std::endl;
        }

        // 如果还在运行状态，等待后重连
        if (running_) {
            for (int elapsed = 0; running_ && elapsed < reconnect_interval_ms_; elapsed += 20)
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
            resetClient();
        }
    }
}

void WebSocketClientWrapper::resetClient()
{
    try {
        std::lock_guard<std::mutex> lock(mutex_);
        if (client_ && running_) {
            client_->reset();


            // 重新设置处理器
            client_->set_open_handler(std::bind(&WebSocketClientWrapper::onOpen, this, std::placeholders::_1));
            client_->set_fail_handler(std::bind(&WebSocketClientWrapper::onFail, this, std::placeholders::_1));
            client_->set_close_handler(std::bind(&WebSocketClientWrapper::onClose, this, std::placeholders::_1));
            client_->set_message_handler(std::bind(&WebSocketClientWrapper::onMessage, this,
                std::placeholders::_1, std::placeholders::_2));
        }
    }
    catch (const std::exception& e) {
        std::cout << "[WSClient] 重置客户端失败：" << e.what() << std::endl;
    }
}

} // namespace agv_protocol
