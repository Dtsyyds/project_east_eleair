#!/usr/bin/env python3
"""Mapping and optional visualization/image decompression; sensors start separately."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    config = LaunchConfiguration("config_file").perform(context)
    with open(config, encoding="utf-8") as stream:
        parameters = yaml.safe_load(stream)["/**"]["ros__parameters"]
    image_enabled = parameters.get("common", {}).get("img_en", 0)
    files = [config]
    if image_enabled:
        files.append(LaunchConfiguration("camera_file"))
    actions = [Node(package="fast_livo", executable="fastlivo_mapping", name="laserMapping",
                    parameters=files, output="screen", emulate_tty=True)]
    republish = LaunchConfiguration("republish_compressed").perform(context).lower() in ("true", "1")
    if republish:
        if not image_enabled:
            raise RuntimeError("republish_compressed requires common.img_en=1.")
        topic = parameters["common"]["img_topic"]
        source = LaunchConfiguration("image_input_topic").perform(context) or topic
        actions.append(Node(package="image_transport", executable="republish", name="image_republish",
                            parameters=[{"in_transport": "compressed", "out_transport": "raw"}],
                            remappings=[("in/compressed", source + "/compressed"), ("out", topic)],
                            respawn=LaunchConfiguration("use_respawn"), respawn_delay=2.0,
                            respawn_max_retries=5, output="screen"))
    actions.append(Node(package="rviz2", executable="rviz2", name="rviz2",
                        condition=IfCondition(LaunchConfiguration("enable_rviz")),
                        arguments=["-d", os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")],
                        output="screen"))
    return actions


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=os.path.join(config_dir, "mapping_relocation.yaml")),
        DeclareLaunchArgument("camera_file", default_value=os.path.join(config_dir, "camera_parameter.yaml")),
        DeclareLaunchArgument("enable_rviz", default_value="true"),
        DeclareLaunchArgument("republish_compressed", default_value="false"),
        DeclareLaunchArgument("image_input_topic", default_value="", description="Compressed input base topic, without /compressed."),
        DeclareLaunchArgument("use_respawn", default_value="false", description="Restart only the optional image republisher, at most 5 times."),
        OpaqueFunction(function=_setup),
    ])
