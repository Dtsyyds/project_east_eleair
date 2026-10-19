from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    parameters = PathJoinSubstitution([FindPackageShare("tank_vision_ros"), "config", "ros_topics.yaml"])
    return LaunchDescription(
        [
            Node(
                package="tank_vision_ros",
                executable="vision_node",
                name="tank_vision_node",
                output="screen",
                parameters=[parameters, {"use_sim_time": False}],
            )
        ]
    )
