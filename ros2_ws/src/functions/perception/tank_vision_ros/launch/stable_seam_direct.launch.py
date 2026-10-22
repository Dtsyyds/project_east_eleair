from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="tank_vision_ros", executable="stable_seam_node", name="stable_seam_node", output="screen",
            parameters=[{"input_mode": "mindvision_direct", "publish_debug_image": False}],
        )
    ])
