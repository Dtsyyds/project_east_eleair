"""传感器硬件驱动（Livox MID-360 雷达与 RealSense D405 相机）统一启动文件。"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_bringup_config():
    """从 top_pkg/config/bringup_sensor.yaml 中加载模块使能配置。"""
    config_path = os.path.join(
        get_package_share_directory("top_pkg"),
        "config",
        "bringup_sensor.yaml",
    )
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)["bringup"]


def generate_launch_description():
    """解析参数并包含各传感器官方启动脚本，实现传感器驱动集中调用。"""
    config = load_bringup_config()

    # 声明命令行可覆写使能参数（默认值读取自 bringup_sensor.yaml）
    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar",
        default_value=str(config["enable_lidar"]).lower(),
        description="是否启动 Livox MID-360 激光雷达驱动 (true/false)。",
    )
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera",
        default_value=str(config["enable_camera"]).lower(),
        description="是否启动 Intel RealSense D405 相机驱动 (true/false)。",
    )
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz",
        default_value=str(config["enable_rviz"]).lower(),
        description="是否启动 RViz2 可视化界面 (true/false)。",
    )

    # 包含 Livox 官方 MID360 启动文件 (msg_MID360_launch.py)
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("livox_ros_driver2"),
                "launch",
                "msg_MID360_launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_lidar")),
    )

    # 包含 RealSense 官方相机启动文件 (rs_launch.py)，配置为 D405 模式及 1280x720 彩色流
    realsense_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        condition=IfCondition(LaunchConfiguration("enable_camera")),
        launch_arguments={
            "camera_namespace": "camera",
            "camera_name": "camera",
            "device_type": "d405",
            "enable_depth": "false",
            "enable_color": "true",
            "enable_sync": "false",
            "align_depth.enable": "false",
            "pointcloud.enable": "false",
            "spatial_filter.enable": "false",
            "temporal_filter.enable": "false",
            # D405 的实际彩色流由 depth_module.color_profile 控制。
            # 640x360x30 是本机驱动已确认支持的 profile。
            "depth_module.color_profile": "640x360x30",
            # 当前未启用深度流；保留一个合法值以避免驱动初始化时回退并报错。
            "depth_module.depth_profile": "640x360x30",
            "log_level": "warn",
        }.items(),
    )

    # 启动传感器调试专用的 RViz2 可视化节点
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="calibration_rviz",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", os.path.join(
            get_package_share_directory("top_pkg"), "config", "bringup.rviz")],
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_rviz_arg,
        livox_driver,
        realsense_driver,
        rviz,
    ])
