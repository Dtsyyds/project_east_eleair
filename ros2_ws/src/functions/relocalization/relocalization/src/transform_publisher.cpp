/*
将建图完成先验初始化后的最终局部参考系位姿发布为静态 TF。
固定订阅 /fast_livo2/initial_map_pose，并常驻维护 map -> camera_init。
*/
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class TransformPublisherNode : public rclcpp::Node
{
public:
    TransformPublisherNode()
        : Node("transform_publisher_node")
    {
        odom_frame_id = this->declare_parameter<std::string>("odom_frame_id", "camera_init");
        map_frame_id = this->declare_parameter<std::string>("map_frame_id", "map");
        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/fast_livo2/initial_map_pose", rclcpp::QoS(1).reliable().transient_local(),
            std::bind(&TransformPublisherNode::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
            "Waiting on /fast_livo2/initial_map_pose for the map -> camera_init alignment.");
    }

private:
    void callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (received_) return;
        const auto &p=msg->pose.pose.position; auto q=msg->pose.pose.orientation;
        const double norm=std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w);
        if (msg->header.frame_id!=map_frame_id || !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) || norm<1e-12) {
            RCLCPP_WARN(get_logger(), "Ignoring invalid map alignment pose."); return;
        }
        q.x/=norm; q.y/=norm; q.z/=norm; q.w/=norm;
        received_=true;
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = map_frame_id;
        transform.child_frame_id = odom_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = q;

        broadcaster_->sendTransform(transform);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
    std::string odom_frame_id, map_frame_id;
    bool received_=false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TransformPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
