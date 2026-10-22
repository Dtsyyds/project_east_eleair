#include <rclcpp/rclcpp.hpp>
#include "agv_protocol/websocket_client.hpp"
#include "agv_protocol/websocket_server.hpp"
#include "agv_protocol/command_parser.hpp"
#include "agv_bridge/robots.hpp"
#include <deque>
#include <mutex>
#include <csignal>
#include <sys/wait.h>
#include <unistd.h>
#include <spawn.h>
#include <stdexcept>
extern char** environ;

namespace agv_bridge {
class WebSocketBridgeNode : public rclcpp::Node {
public:
    WebSocketBridgeNode() : Node("websocket_bridge_node") {
        declare_parameter("server_uri", "ws://192.168.1.102:9100");
        declare_parameter("local_server_port", 9001);
        const auto id=declare_parameter<std::string>("robot_id", "");
        ws_client_=std::make_unique<agv_protocol::WebSocketClientWrapper>(get_parameter("server_uri").as_string(),2000);
        ws_server_=std::make_unique<agv_protocol::WebSocketServerWrapper>(get_parameter("local_server_port").as_int());
        auto enqueue=[this](const std::string& message) {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if(message.size()>1024*1024 || commands_.size()>=64) {++command_drops_;return;}
            commands_.push_back(message);
        };
        ws_client_->setMessageHandler(enqueue);ws_server_->setMessageHandler(enqueue);
        if(!id.empty()) {
            robot_=RobotFactory::createRobot(id);
            if(!robot_)throw std::invalid_argument("unknown robot_id: "+id);
            robot_->init(this);
        }
        if(!get_parameter("server_uri").as_string().empty())ws_client_->start();
        if(get_parameter("local_server_port").as_int()>0)ws_server_->start();
        command_timer_=create_wall_timer(std::chrono::milliseconds(20),[this]() {
            std::deque<std::string> work;
            {std::lock_guard<std::mutex> lock(queue_mutex_);for(int i=0;i<8&&!commands_.empty();i++){work.push_back(std::move(commands_.front()));commands_.pop_front();}}
            for(const auto& message:work)handleCommand(message);
            if(robot_)robot_->tick();
        });
        const int period=declare_parameter<int>("report_period_ms",200);
        if(period<5 || period>1000)throw std::invalid_argument("report_period_ms must be 5..1000");
        report_timer_=create_wall_timer(std::chrono::milliseconds(period),[this]() {sendPeriodicReport();});
        RCLCPP_INFO(get_logger(),"WebSocket bridge ready; configured robot=%s",id.c_str());
    }
    ~WebSocketBridgeNode() {
        if(command_timer_)command_timer_->cancel();
        if(report_timer_)report_timer_->cancel();
        ws_client_->stop();ws_server_->stop();
        // Only stop this instance's owned legacy launch process group.
        if(launch_pid_>0) {
            kill(-launch_pid_,SIGINT);
            const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
            while(std::chrono::steady_clock::now()<deadline) {
                if(waitpid(launch_pid_,nullptr,WNOHANG)!=0)break;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    }
private:
    void send(const std::string& message) {
        bool remote=ws_client_->trySend(message);bool local=ws_server_->send(message);
        if(!remote&&!local)++transport_drops_;
    }
    void sendPeriodicReport() {
        if(!robot_||(!ws_client_->isConnected()&&!ws_server_->hasClients()))return;
        for(const auto& message:robot_->generateReports(parser_))send(message);
        for(const auto& message:robot_->generateEvents(parser_))send(message);
        const auto now=std::chrono::steady_clock::now();
        if(now<next_stats_)return;  // Faster video polling must not flood status/control traffic.
        next_stats_=now+std::chrono::milliseconds(200);
        Json::Value stats;stats["bridge"]["command_dropped"]=Json::UInt64(command_drops_.load());
        stats["bridge"]["transport_dropped"]=Json::UInt64(transport_drops_);
        stats["bridge"]["remote_connected"]=ws_client_->isConnected();
        stats["bridge"]["hardware_revalidated"]=false;
        send(parser_.buildRawMessage(robot_->getRobotId(),"status_update",stats));
    }
    void initialise(const std::string& id) {
        if(robot_)return;
        robot_=RobotFactory::createRobot(id);
        if(!robot_)return;
        robot_->init(this);
        const std::string cmd=robot_->getLaunchCommand();
        if(cmd.empty())return;  // inspection attaches; check never creates owners
        // No fork from transport threads, no arbitrary shell input: command is
        // supplied only by the registered legacy Robot implementation.
        std::string launch=cmd+" wait";
        char* argv[]={const_cast<char*>("bash"),const_cast<char*>("-c"),launch.data(),nullptr};
        posix_spawnattr_t attrs;posix_spawnattr_init(&attrs);
        posix_spawnattr_setflags(&attrs,POSIX_SPAWN_SETPGROUP);posix_spawnattr_setpgroup(&attrs,0);
        int error=posix_spawn(&launch_pid_,"/bin/bash",nullptr,&attrs,argv,environ);
        posix_spawnattr_destroy(&attrs);
        if(error) {launch_pid_=0;throw std::runtime_error("legacy launch spawn failed");}
    }
    void handleCommand(const std::string& json) {
        auto request=parser_.parseRaw(json);
        if(!request||request->header.robot_id.empty()||!request->payload.isObject())return;
        Json::Value result;
        try {
            if(robot_ && request->header.robot_id!=robot_->getRobotId()) {
                result["error"]["status"]="failed";result["error"]["reason"]="robot_id_mismatch";
            } else {
                if(request->header.msg_type=="check")initialise(request->header.robot_id);
                if(!robot_) {result["error"]["status"]="failed";result["error"]["reason"]="unknown_or_uninitialised_robot";}
                else if(request->header.msg_type=="check")result=robot_->handleCheck(*request);
                else if(request->header.msg_type=="command"||request->header.msg_type=="response")result=robot_->handleCommand(*request);
                else {result["error"]["status"]="failed";result["error"]["reason"]="unsupported_message_type";}
            }
        } catch(const std::exception& error) {result["error"]["status"]="failed";result["error"]["reason"]=error.what();}
        if(!result.empty())send(parser_.buildRawMessage(
            request->header.robot_id,"response",result,request->header.request_id));
    }
    agv_protocol::CommandParser parser_;
    std::unique_ptr<agv_protocol::WebSocketClientWrapper> ws_client_;
    std::unique_ptr<agv_protocol::WebSocketServerWrapper> ws_server_;
    Robot::Ptr robot_;
    rclcpp::TimerBase::SharedPtr report_timer_,command_timer_;
    std::deque<std::string> commands_;
    std::mutex queue_mutex_;
    std::atomic<uint64_t> command_drops_{0};
    uint64_t transport_drops_=0;
    pid_t launch_pid_=0;
    std::chrono::steady_clock::time_point next_stats_{};
};
}
int main(int argc,char** argv) {
    try {
        rclcpp::init(argc,argv);
        auto node=std::make_shared<agv_bridge::WebSocketBridgeNode>();
        rclcpp::spin(node);node.reset();if(rclcpp::ok())rclcpp::shutdown();return 0;
    } catch(const std::exception& error) {
        std::cerr << "WebSocket bridge failed: " << error.what() << std::endl;
        if(rclcpp::ok())rclcpp::shutdown();return 2;
    }
}
