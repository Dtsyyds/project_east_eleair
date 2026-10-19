"""Launch the configurable V4L2 fisheye camera."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("fisheye_camera_driver"))
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(share / "config/fisheye.yaml")),
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("node_name", default_value="fisheye"),
        DeclareLaunchArgument("device", default_value=""),
        Node(package="fisheye_camera_driver", executable="fisheye_camera_node",
             namespace=LaunchConfiguration("namespace"), name=LaunchConfiguration("node_name"),
             output="screen", emulate_tty=True,
             parameters=[LaunchConfiguration("config_file"), {"device": LaunchConfiguration("device")}]),
    ])
