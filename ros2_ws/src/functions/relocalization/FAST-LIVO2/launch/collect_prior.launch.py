#!/usr/bin/env python3
"""Collect a prior map with the Livox driver and FAST-LIVO2; all operating parameters remain in the shared YAML."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mapping_share = get_package_share_directory("fast_livo")
    driver_share = get_package_share_directory("livox_ros_driver2")

    # Each include gets its own configuration scope: the driver also calls its
    # YAML argument config_file, while mapping and registration share one file.
    driver = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(driver_share, "launch", "msg_MID360_launch.py")),
            launch_arguments={"config_file": LaunchConfiguration("driver_config_file")}.items(),
        ),
    ], condition=IfCondition(LaunchConfiguration("start_driver")))
    mapping = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_share, "launch", "mapping_relocation.launch.py")),
            launch_arguments={
                "config_file": LaunchConfiguration("config_file"),
                "enable_rviz": LaunchConfiguration("enable_rviz"),
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(mapping_share, "config", "mapping_relocation.yaml")),
        DeclareLaunchArgument("driver_config_file", default_value=os.path.join(driver_share, "config", "msg_MID360.yaml")),
        DeclareLaunchArgument("start_driver", default_value="true", description="Start the Livox driver; disable for bag replay."),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        driver,
        mapping,
    ])
