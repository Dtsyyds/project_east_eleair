"""Launch FAST-LIVO2 with the local Livox MID-360 and RealSense D405 setup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start mapping and, optionally, RViz with the local calibration files."""
    package_share = get_package_share_directory("fast_livo")

    parameters = [
        os.path.join(package_share, "config", "livo.yaml"),
        os.path.join(package_share, "config", "mid360.yaml"),
        os.path.join(package_share, "config", "d405.yaml"),
        os.path.join(package_share, "config", "log.yaml"),
    ]
    rviz_config = os.path.join(package_share, "rviz_cfg", "fast_livo2.rviz")

    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value="true",
        description="Whether to start RViz.",
    )

    return LaunchDescription([
        enable_rviz_arg,
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            output="screen",
            emulate_tty=True,
            parameters=parameters,
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_rviz")),
        ),
    ])
