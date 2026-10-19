#include "agv_bridge/tank_inspection_robot.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <eddy_driver/msg/eddy_frame.hpp>
#include <eddy_driver/msg/eddy_channel_mean.hpp>
#include <inspection_interfaces/msg/ultrasound_frame.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <websocketpp/base64/base64.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <deque>

namespace agv_bridge {
using Clock = std::chrono::steady_clock;
static int64_t stamp(const std_msgs::msg::Header& h) {
    return static_cast<int64_t>(h.stamp.sec)*1000000000LL+h.stamp.nanosec;
}
static Json::Value headerJson(const std_msgs::msg::Header& h) {
    Json::Value out; out["stamp_ns"]=std::to_string(stamp(h));out["stamp_sec"]=h.stamp.sec;out["stamp_nanosec"]=h.stamp.nanosec;out["frame_id"]=h.frame_id;return out;
}
struct TankInspectionRobot::Impl {
    struct State {
        std::string topic, type, error;
        Clock::time_point arrival{}, previous_arrival{};
        int64_t stamp_ns=0;
        uint64_t received=0, accepted=0, invalid=0, repeated=0, regressed=0;
        double rate_hz=0.;
        bool valid=false;
    };
    rclcpp::Node* node=nullptr;
    std::mutex mutex;
    std::map<std::string,State> states;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions;
    std::map<std::string,sensor_msgs::msg::Image::ConstSharedPtr> images;
    eddy_driver::msg::EddyFrame::ConstSharedPtr eddy_raw;
    eddy_driver::msg::EddyChannelMean::ConstSharedPtr eddy_mean;
    std::map<std::string,uint64_t> last_preview, preview_drops;
    nav_msgs::msg::Odometry::ConstSharedPtr pose;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr array_pub;
    bool control_enabled=false, moving=false, streaming=true;
    double timeout=3., watchdog=.5, max_linear=.2, max_angular=.5, max_video_fps=5., max_eddy_fps=10.;
    int preview_width=640,jpeg_quality=70;
    std::string control_type, token;
    Clock::time_point last_move{},next_video{},next_eddy{},next_status{};
    uint64_t last_eddy_raw=0,last_eddy_mean=0,eddy_drops=0;
    uint64_t watchdog_stops=0;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_pub;
    std::deque<std::string> operation_events;
    Json::Value operation_status;
    Clock::time_point operation_arrival{};
    template<class T> T param(const std::string& key,T fallback) {
        return node->declare_parameter<T>("tank_inspection."+key,fallback);
    }
    bool mark(const std::string& key, const std_msgs::msg::Header& h,bool valid,const std::string& reason="") {
        auto& s=states.at(key);s.received++;const auto now=Clock::now();
        s.previous_arrival=s.arrival;s.arrival=now;
        if(s.previous_arrival!=Clock::time_point{}) {
            const double interval=std::chrono::duration<double>(s.arrival-s.previous_arrival).count();
            if(interval>0.) {const double instant=1./interval;s.rate_hz=s.rate_hz>0.?s.rate_hz*.8+instant*.2:instant;}
        }
        const int64_t value=stamp(h);
        if(value<=0 || h.stamp.nanosec>=1000000000U) {valid=false;s.error="invalid_header_stamp";}
        else if(s.stamp_ns>0 && value<=s.stamp_ns) {
            valid=false; if(value==s.stamp_ns){s.repeated++;s.error="repeated_header_stamp";}
            else {s.regressed++;s.error="regressed_header_stamp";}
        } else {s.error=valid?"":reason;}
        if(value>0) s.stamp_ns=std::max(s.stamp_ns,value);
        s.valid=valid;if(valid)s.accepted++;else s.invalid++;return valid;
    }
    Json::Value checks() {
        Json::Value out(Json::objectValue);
        for(const auto& [name,s]:states) {
            const bool seen=s.received>0;
            const double age=seen?std::chrono::duration<double>(Clock::now()-s.arrival).count():0.;
            const bool valid=seen && s.valid && age<=timeout;
            auto& r=out[name];r["command"]="check";r["result"]=valid?"yes":"no";
            r["status"]=valid?"verified":seen?"failed":"unverified";
            r["reason"]=!seen?(s.topic.empty()?"interface_unconfigured":"no_valid_message_received"):
                !s.valid?s.error:age>timeout?"stale":"recent_valid_ros_message";
            r["topic"]=s.topic;r["message_type"]=s.type;r["received"]=Json::UInt64(s.received);
            r["accepted"]=Json::UInt64(s.accepted);r["invalid"]=Json::UInt64(s.invalid);r["repeated_stamps"]=Json::UInt64(s.repeated);
            r["regressed_stamps"]=Json::UInt64(s.regressed);r["stamp_ns"]=std::to_string(s.stamp_ns);
            r["arrival_age_seconds"]=seen?Json::Value(age):Json::Value();
            r["device_id"]=name;r["connection_state"]=s.topic.empty()?"not_configured":seen?"connected":"disconnected";
            r["running_state"]=valid?"running":seen?"degraded":"stopped";
            r["data_state"]=valid?"streaming":seen?"stale_or_invalid":"no_data";
            r["last_data_timestamp"]=seen?Json::Value(std::to_string(s.stamp_ns)):Json::Value();
            r["publish_rate_hz"]=s.rate_hz;r["error_code"]=valid?"":"DATA_UNAVAILABLE";
            r["error_message"]=valid?"":r["reason"].asString();
            r["timestamp_source"]=name=="chassis"?"host_monotonic_receipt":"message_header";
            if(name=="chassis")r["stamp_ns"]=Json::Value();
            r["evidence"]="ros_data_liveness_only";r["hardware_revalidated"]=false;
        }
        return out;
    }
    void velocity(double x,double y,double z) {
        if(twist_pub) {geometry_msgs::msg::Twist msg;msg.linear.x=x;msg.linear.y=y;msg.angular.z=z;twist_pub->publish(msg);}
        if(array_pub) {std_msgs::msg::Float64MultiArray msg;msg.data={x,y,z};array_pub->publish(msg);}
    }
    bool fresh(const std::string& key) const {
        const auto& s=states.at(key);return s.received && s.valid && std::chrono::duration<double>(Clock::now()-s.arrival).count()<=timeout;
    }
};
TankInspectionRobot::TankInspectionRobot():impl_(std::make_unique<Impl>()) {}
TankInspectionRobot::~TankInspectionRobot() {if(impl_->moving && rclcpp::ok()) {try{impl_->velocity(0,0,0);}catch(const std::exception&){}}}
void TankInspectionRobot::init(rclcpp::Node* node) {
    auto& d=*impl_;d.node=node;
    const auto operations=d.param<std::string>("operations_topic", "");
    if(!operations.empty()) {
        d.operation_pub=node->create_publisher<std_msgs::msg::String>(operations+"/command",rclcpp::QoS(32));
        d.subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(operations+"/events",rclcpp::QoS(64),
            [this](std_msgs::msg::String::ConstSharedPtr msg) {
                if(msg->data.size()>1024*1024)return;
                Json::Value packet;Json::Reader reader;
                if(!reader.parse(msg->data,packet)||!packet["payload"].isObject())return;
                auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
                if(packet["header"]["msg_type"].asString()=="status_update") {
                    d.operation_status=packet["payload"];d.operation_arrival=Clock::now();
                } else {
                    if(d.operation_events.size()>=128)d.operation_events.pop_front();
                    d.operation_events.push_back(msg->data);
                }
            }));
    }
    d.timeout=d.param<double>("device_timeout_sec",3.);d.watchdog=d.param<double>("command_timeout_sec",.5);
    d.max_linear=d.param<double>("max_linear_m_s",.2);d.max_angular=d.param<double>("max_angular_rad_s",.5);
    d.max_video_fps=d.param<double>("video_fps",5.);d.preview_width=d.param<int>("preview_width",640);
    d.max_eddy_fps=d.param<double>("eddy_fps",10.);
    d.jpeg_quality=d.param<int>("jpeg_quality",70);d.control_enabled=d.param<bool>("control_enabled",false);
    d.control_type=d.param<std::string>("chassis_command_type","geometry_msgs/msg/Twist");
    d.token=d.param<std::string>("control_token","");
    for(double v:{d.timeout,d.watchdog,d.max_linear,d.max_angular,d.max_video_fps,d.max_eddy_fps})
        if(!std::isfinite(v)||v<=0)throw std::invalid_argument("invalid tank timeout/rate/limit");
    if(d.max_video_fps>30.||d.preview_width<16||d.preview_width>1280||d.jpeg_quality<1||d.jpeg_quality>95)
        throw std::invalid_argument("tank preview limits: fps<=30, width16..1280, jpeg1..95");
    auto sensor_qos=rclcpp::SensorDataQoS().keep_last(1);
    for(auto spec:{std::pair<std::string,std::string>{"fisheye",""},{"mindvision","/camera/front/image_raw"},{"depth_camera","/d405/camera/depth/image_rect_raw"}}) {
        auto key=spec.first;auto topic=d.param<std::string>(key+"_topic",spec.second);
        d.states[key].topic=topic;d.states[key].type="sensor_msgs/msg/Image";
        if(topic.empty())continue;
        d.subscriptions.push_back(node->create_subscription<sensor_msgs::msg::Image>(topic,sensor_qos,
            [this,key](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
                size_t bytes=msg->encoding=="bgr8"||msg->encoding=="rgb8"?3:
                    msg->encoding=="mono8"?1:msg->encoding=="16UC1"?2:0;
                bool valid=bytes && msg->width>0 && msg->height>0 && msg->width<=16384 && msg->height<=16384 &&
                    msg->step>=static_cast<uint64_t>(msg->width)*bytes &&
                    msg->data.size()==static_cast<uint64_t>(msg->step)*msg->height && msg->data.size()<=64*1024*1024;
                if(d.mark(key,msg->header,valid,"invalid_image_layout_or_encoding"))d.images[key]=msg;
            }));
    }
    auto raw=d.param<std::string>("eddy_raw_topic","/eddy/data");auto mean=d.param<std::string>("eddy_mean_topic","/eddy/channel_mean");
    d.states["eddy_current"].topic=raw+" | "+mean;d.states["eddy_current"].type="eddy_driver/msg/EddyFrame | EddyChannelMean";
    // Separate journals prevent native raw/mean sharing one stamp being called a duplicate.
    d.states["eddy_raw"].topic=raw;d.states["eddy_raw"].type="eddy_driver/msg/EddyFrame";
    d.states["eddy_mean"].topic=mean;d.states["eddy_mean"].type="eddy_driver/msg/EddyChannelMean";
    if(!raw.empty())d.subscriptions.push_back(node->create_subscription<eddy_driver::msg::EddyFrame>(raw,sensor_qos,
        [this](eddy_driver::msg::EddyFrame::ConstSharedPtr msg) {auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
            d.mark("eddy_raw",msg->header,msg->channel_count==8&&msg->samples_per_channel==20,"invalid_channel_sample_contract");
            d.states["eddy_current"]=d.states["eddy_raw"];
            if(d.states["eddy_raw"].valid)d.eddy_raw=msg;
        }));
    if(!mean.empty())d.subscriptions.push_back(node->create_subscription<eddy_driver::msg::EddyChannelMean>(mean,sensor_qos,
        [this](eddy_driver::msg::EddyChannelMean::ConstSharedPtr msg) {auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
            bool valid=std::all_of(msg->channel_mean.begin(),msg->channel_mean.end(),[](double x){return std::isfinite(x);});
            d.mark("eddy_mean",msg->header,valid,"nonfinite_channel_mean");
            if(d.states["eddy_mean"].valid)d.eddy_mean=msg;
            if(!d.states["eddy_raw"].received || !d.fresh("eddy_raw"))d.states["eddy_current"]=d.states["eddy_mean"];
        }));
    auto emat=d.param<std::string>("emat_topic","");d.states["emat"].topic=emat;d.states["emat"].type="inspection_interfaces/msg/UltrasoundFrame";
    if(!emat.empty())d.subscriptions.push_back(node->create_subscription<inspection_interfaces::msg::UltrasoundFrame>(emat,sensor_qos,
        [this](inspection_interfaces::msg::UltrasoundFrame::ConstSharedPtr msg) {auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
            bool valid=msg->channel_count>0&&msg->sample_count>0&&msg->samples.size()==static_cast<uint64_t>(msg->channel_count)*msg->sample_count&&
                std::isfinite(msg->sampling_rate_hz)&&msg->sampling_rate_hz>0;
            d.mark("emat",msg->header,valid,"invalid_ultrasound_sample_contract");}));
    auto chassis=d.param<std::string>("chassis_status_topic","");d.states["chassis"].topic=chassis;
    auto chassis_type=d.param<std::string>("chassis_status_type","std_msgs/msg/String");d.states["chassis"].type=chassis_type;
    auto heartbeat=[this](bool valid){auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);auto& s=d.states["chassis"];
        const auto now=Clock::now();s.received++;s.previous_arrival=s.arrival;s.arrival=now;
        if(s.previous_arrival!=Clock::time_point{}) {const double interval=std::chrono::duration<double>(s.arrival-s.previous_arrival).count();
            if(interval>0.) {const double instant=1./interval;s.rate_hz=s.rate_hz>0.?s.rate_hz*.8+instant*.2:instant;}}
        s.valid=valid;s.error=valid?"":"chassis_reported_error_or_invalid_pose";if(valid)s.accepted++;else s.invalid++;};
    if(!chassis.empty()) {
        if(chassis_type=="std_msgs/msg/String")d.subscriptions.push_back(node->create_subscription<std_msgs::msg::String>(chassis,rclcpp::QoS(10).best_effort(),
            [heartbeat](std_msgs::msg::String::ConstSharedPtr msg){heartbeat(msg->data=="online"||msg->data=="connected"||msg->data=="ok");}));
        else if(chassis_type=="geometry_msgs/msg/Pose2D")d.subscriptions.push_back(node->create_subscription<geometry_msgs::msg::Pose2D>(chassis,rclcpp::QoS(10).best_effort(),
            [heartbeat](geometry_msgs::msg::Pose2D::ConstSharedPtr msg){heartbeat(std::isfinite(msg->x)&&std::isfinite(msg->y)&&std::isfinite(msg->theta));}));
        else throw std::invalid_argument("unsupported chassis_status_type");
    }
    auto lidar=d.param<std::string>("lidar_topic","/livox/lidar");d.states["lidar"].topic=lidar;d.states["lidar"].type="sensor_msgs/msg/PointCloud2";
    if(!lidar.empty())d.subscriptions.push_back(node->create_subscription<sensor_msgs::msg::PointCloud2>(lidar,sensor_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
            const bool valid=msg->height>0&&msg->width>0&&msg->point_step>0&&msg->row_step>=static_cast<uint64_t>(msg->point_step)*msg->width&&msg->data.size()==static_cast<uint64_t>(msg->row_step)*msg->height;
            d.mark("lidar",msg->header,valid,"invalid_pointcloud_layout");}));
    auto pose=d.param<std::string>("lidar_pose_topic","/aft_mapped_to_init");d.states["lidar_pose"].topic=pose;d.states["lidar_pose"].type="nav_msgs/msg/Odometry";
    if(!pose.empty())d.subscriptions.push_back(node->create_subscription<nav_msgs::msg::Odometry>(pose,rclcpp::QoS(10).best_effort(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
            auto& p=msg->pose.pose.position;auto& q=msg->pose.pose.orientation;
            const double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
            bool valid=std::isfinite(norm)&&std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)&&std::isfinite(q.x)&&std::isfinite(q.y)&&std::isfinite(q.z)&&std::isfinite(q.w)&&norm>1e-12;
            if(d.mark("lidar_pose",msg->header,valid,"invalid_odometry_pose"))d.pose=msg;}));
    const auto command=d.param<std::string>("chassis_command_topic","");
    if(d.control_enabled) {
        if(command.empty())throw std::invalid_argument("control enabled requires explicit chassis_command_topic");
        if(d.control_type=="geometry_msgs/msg/Twist")d.twist_pub=node->create_publisher<geometry_msgs::msg::Twist>(command,10);
        else if(d.control_type=="std_msgs/msg/Float64MultiArray")d.array_pub=node->create_publisher<std_msgs::msg::Float64MultiArray>(command,10);
        else throw std::invalid_argument("unsupported chassis_command_type");
    }
}
Json::Value TankInspectionRobot::handleCheck(const agv_protocol::RawMessage&) {
    std::lock_guard<std::mutex> lock(impl_->mutex);return impl_->checks();
}
Json::Value TankInspectionRobot::handleCommand(const agv_protocol::RawMessage& request) {
    auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);Json::Value out(Json::objectValue);
    if(request.header.msg_type!="command")return out;
    if(request.payload["action"].isString()) {
        if(d.operation_pub && d.operation_arrival!=Clock::time_point{} &&
           std::chrono::duration<double>(Clock::now()-d.operation_arrival).count()<3.) {
            std_msgs::msg::String msg;Json::StreamWriterBuilder writer;
            msg.data=Json::writeString(writer,request.payload);d.operation_pub->publish(msg);
            return out;
        }
        out["request_id"]=request.payload["request_id"];out["status"]="failed";
        out["error"]="采集服务未连接";return out;
    }
    for(const auto& key:request.payload.getMemberNames()) {
        if(key=="token")continue;
        const auto& value=request.payload[key];auto& r=out[key];
        const std::string cmd=value.isObject()&&value["command"].isString()?value["command"].asString():"";
        r["command"]=cmd;r["result"]="no";r["status"]="failed";
        r["request_state"]="completed";r["accepted"]=false;
        if(d.states.count(key) && (cmd=="GetStatus"||cmd=="get_status")) {
            r=d.checks()[key];r["command"]=cmd;r["request_state"]="completed";r["accepted"]=true;continue;
        }
        if(d.states.count(key) && (cmd=="GetConfig"||cmd=="get_config")) {
            r["result"]="yes";r["status"]="verified";r["accepted"]=true;
            r["topic"]=d.states[key].topic;r["message_type"]=d.states[key].type;continue;
        }
        if(d.states.count(key) && (cmd=="Start"||cmd=="Stop"||cmd=="Restart"||
                                   cmd=="start"||cmd=="stop"||cmd=="restart")) {
            r["reason"]="passive_bridge_does_not_own_device";continue;
        }
        if(d.states.count(key) && (cmd=="SetConfig"||cmd=="set_config")) {
            r["reason"]="runtime_config_not_supported";continue;
        }
        if(key=="camera" && (cmd=="start_stream"||cmd=="stop_stream")) {
            d.streaming=cmd=="start_stream";r["result"]="yes";r["status"]="verified";r["accepted"]=true;continue;
        }
        if(key!="agv"&&key!="chassis") {r["reason"]="unsupported_command";continue;}
        if(!d.control_enabled) {r["reason"]="control_disabled";continue;}
        if(!d.token.empty()&&(!request.payload["token"].isString()||request.payload["token"].asString()!=d.token)) {r["reason"]="invalid_control_token";continue;}
        if(cmd=="stop") {d.velocity(0,0,0);d.moving=false;r["result"]="yes";r["status"]="verified";r["accepted"]=true;r["execution"]="published_not_hardware_acknowledged";continue;}
        if(cmd!="move") {r["reason"]="unsupported_command";continue;}
        if(!d.fresh("chassis")) {r["reason"]="chassis_not_verified";continue;}
        const auto& p=value.isMember("parameters")?value["parameters"]:value["data"];
        if(!p.isObject()||!p["vx"].isNumeric()||!p["vy"].isNumeric()||!p["wz"].isNumeric()) {r["reason"]="velocity_fields_required";continue;}
        double x=p["vx"].asDouble(),y=p["vy"].asDouble(),z=p["wz"].asDouble();
        if(!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z)||std::hypot(x,y)>d.max_linear||std::abs(z)>d.max_angular) {r["reason"]="velocity_out_of_bounds";continue;}
        d.velocity(x,y,z);d.last_move=Clock::now();d.moving=true;r["result"]="yes";r["status"]="verified";
        r["accepted"]=true;
        r["execution"]="published_not_hardware_acknowledged";
    }
    return out;
}
void TankInspectionRobot::tick() {
    auto& d=*impl_;std::lock_guard<std::mutex> lock(d.mutex);
    if(d.moving && (std::chrono::duration<double>(Clock::now()-d.last_move).count()>d.watchdog || !d.fresh("chassis"))) {
        d.velocity(0,0,0);d.moving=false;d.watchdog_stops++;
    }
}
std::vector<std::string> TankInspectionRobot::generateReports(agv_protocol::CommandParser& parser) {
    auto& d=*impl_;std::vector<std::string> result;
    std::map<std::string,sensor_msgs::msg::Image::ConstSharedPtr> pending;
    std::map<std::string,uint64_t> pending_received,pending_accepted,pending_dropped;
    eddy_driver::msg::EddyFrame::ConstSharedPtr pending_raw;
    eddy_driver::msg::EddyChannelMean::ConstSharedPtr pending_mean;
    Json::Value payload;
    bool status_due=false;
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        const auto now=Clock::now();
        status_due=now>=d.next_status;
        if(status_due) {
        d.next_status=now+std::chrono::milliseconds(200);
        payload["devices"]=d.checks();
        payload["schema_version"]="tank_inspection_v2";
        payload["control_enabled"]=d.control_enabled;payload["watchdog_stops"]=Json::UInt64(d.watchdog_stops);
        payload["calibration_status"]="unverified";payload["hardware_revalidated"]=false;
        if(d.operation_arrival!=Clock::time_point{} &&
           std::chrono::duration<double>(now-d.operation_arrival).count()<3.) {
            for(const auto& key:d.operation_status.getMemberNames())payload[key]=d.operation_status[key];
        } else if(d.operation_pub) {
            payload["capabilities"]["operations"]=Json::Value(Json::objectValue);
        }
        if(d.pose && d.fresh("lidar_pose")) {
            auto& m=*d.pose;auto& p=m.pose.pose.position;auto& q=m.pose.pose.orientation;auto& o=payload["lidar"];
            o=headerJson(m.header);o["child_frame_id"]=m.child_frame_id;o["x"]=p.x;o["y"]=p.y;o["z"]=p.z;
            o["orientation"]["x"]=q.x;o["orientation"]["y"]=q.y;o["orientation"]["z"]=q.z;o["orientation"]["w"]=q.w;
            double norm=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w;
            o["angle"]=std::atan2(2*(q.w*q.z+q.x*q.y)/norm,1-2*(q.y*q.y+q.z*q.z)/norm);
            o["angle_unit"]="rad";o["position_unit"]="m";o["asset_coordinate_valid"]=false;
        }
        if(now>=d.next_eddy) {
            if(d.eddy_raw && d.states["eddy_raw"].accepted!=d.last_eddy_raw) {
                if(d.states["eddy_raw"].accepted>d.last_eddy_raw+1)d.eddy_drops+=d.states["eddy_raw"].accepted-d.last_eddy_raw-1;
                d.last_eddy_raw=d.states["eddy_raw"].accepted;pending_raw=d.eddy_raw;
            }
            if(d.eddy_mean && d.states["eddy_mean"].accepted!=d.last_eddy_mean) {
                if(d.states["eddy_mean"].accepted>d.last_eddy_mean+1)d.eddy_drops+=d.states["eddy_mean"].accepted-d.last_eddy_mean-1;
                d.last_eddy_mean=d.states["eddy_mean"].accepted;pending_mean=d.eddy_mean;
            }
            d.next_eddy=now+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/d.max_eddy_fps));
        }
        }
        // A report timer can wake a fraction early relative to an equal-rate
        // preview deadline (for example both are 200 ms at 5 Hz).  Without a
        // small scheduler tolerance that early wake skips a complete slot and
        // turns harmless timer jitter into a large rate drop.  Phase is still
        // advanced from next_video below, so this neither bursts nor duplicates.
        constexpr auto video_deadline_slack=std::chrono::milliseconds(5);
        if(d.streaming && (d.next_video==Clock::time_point{} ||
                           now+video_deadline_slack>=d.next_video)) {
            for(const auto& [key,msg]:d.images)if(d.fresh(key)&&d.states[key].accepted!=d.last_preview[key]) {
                const uint64_t count=d.states[key].accepted;
                if(count>d.last_preview[key]+1)d.preview_drops[key]+=count-d.last_preview[key]-1;
                d.last_preview[key]=count;pending[key]=msg;
                pending_received[key]=d.states[key].received;pending_accepted[key]=d.states[key].accepted;
                pending_dropped[key]=d.preview_drops[key];
            }
            const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1/d.max_video_fps));
            if(d.next_video==Clock::time_point{})d.next_video=now;
            // Keep phase across timer jitter; skip missed slots, never burst old frames.
            d.next_video+=period*((now-d.next_video)/period+1);
        }
    }
    {
        std::lock_guard<std::mutex> lock(d.mutex);
        while(!d.operation_events.empty()) {
            result.push_back(std::move(d.operation_events.front()));d.operation_events.pop_front();
        }
    }
    if(status_due)result.push_back(parser.buildRawMessage(getRobotId(),"status_update",payload));
    if(pending_raw || pending_mean) {
        Json::Value envelope;auto& e=envelope["eca"];
        e["preview_dropped"]=Json::UInt64(d.eddy_drops);e["visualization_only"]=true;
        if(pending_raw) {
            auto& raw=e["raw"];raw=headerJson(pending_raw->header);raw["sequence"]=pending_raw->sequence;
            raw["samples_per_channel"]=pending_raw->samples_per_channel;raw["channel_count"]=pending_raw->channel_count;
            raw["excitation_amplitude"]=pending_raw->excitation_amplitude;
            raw["excitation_frequency"]=pending_raw->excitation_frequency;
            raw["driver_dropped_frames"]=Json::UInt64(pending_raw->dropped_frames);
            for(const auto value:pending_raw->data)raw["data"].append(value);
            for(const auto value:pending_raw->channel_mean)raw["channel_mean"].append(value);
        }
        if(pending_mean) {
            auto& mean=e["mean"];mean=headerJson(pending_mean->header);
            for(const auto value:pending_mean->channel_mean)mean["channel_mean"].append(value);
        }
        result.push_back(parser.buildRawMessage(getRobotId(),"sensor_data",envelope));
    }
    for(const auto& [key,msg]:pending) {
        try {
            const auto encoding_started=Clock::now();
            const bool depth=msg->encoding=="16UC1";
            cv::Mat image(msg->height,msg->width,depth?CV_16UC1:msg->encoding=="mono8"?CV_8UC1:CV_8UC3,const_cast<uint8_t*>(msg->data.data()),msg->step);
            cv::Mat preview;double scale=std::min(1.,static_cast<double>(d.preview_width)/image.cols);
            cv::resize(image,preview,cv::Size(std::max(1,static_cast<int>(image.cols*scale)),std::max(1,static_cast<int>(image.rows*scale))),0,0,depth?cv::INTER_NEAREST:cv::INTER_AREA);
            if(depth) {
                if(msg->is_bigendian)for(int y=0;y<preview.rows;y++)for(int x=0;x<preview.cols;x++){auto& v=preview.at<uint16_t>(y,x);v=(v>>8)|(v<<8);}
                // Bounded preview, raw-unit display, never calibrated metric depth.
                preview.convertTo(preview,CV_8UC1,255./10000.);
            } else if(msg->encoding=="rgb8")cv::cvtColor(preview,preview,cv::COLOR_RGB2BGR);
            std::vector<uint8_t> bytes;if(!cv::imencode(".jpg",preview,bytes,{cv::IMWRITE_JPEG_QUALITY,d.jpeg_quality}))throw std::runtime_error("JPEG encode failed");
            if(bytes.size()>600000)throw std::runtime_error("preview exceeds bounded message budget");
            Json::Value envelope;auto& v=envelope["video"];v=headerJson(msg->header);v["camera_id"]=key;v["encoding"]="jpeg_base64";
            v["data"]=websocketpp::base64_encode(bytes.data(),bytes.size());v["width"]=preview.cols;v["height"]=preview.rows;
            v["source_encoding"]=msg->encoding;v["visualization_only"]=true;v["source_depth_unit"]="unavailable";
            v["preview_dropped"]=Json::UInt64(pending_dropped[key]);
            v["source_received"]=Json::UInt64(pending_received[key]);
            v["source_accepted"]=Json::UInt64(pending_accepted[key]);
            v["preview_target_fps"]=d.max_video_fps;
            v["encode_wall_ms"]=std::chrono::duration<double,std::milli>(Clock::now()-encoding_started).count();
            result.push_back(parser.buildRawMessage(getRobotId(),"status_sensor",envelope));
        } catch(const std::exception& error) {std::lock_guard<std::mutex> lock(d.mutex);d.states[key].valid=false;d.states[key].error=error.what();}
    }
    return result;
}
}
