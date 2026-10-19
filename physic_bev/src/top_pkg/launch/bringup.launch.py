"""传感器驱动与 FAST-LIVO2 建图定位一键联合启动文件。"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """同时启动传感器硬件驱动、FAST-LIVO2 建图定位算法及顶层 RViz 可视化。"""
    top_pkg_share = get_package_share_directory("top_pkg")
    fast_livo_share = get_package_share_directory("fast_livo")
    with open(os.path.join(top_pkg_share, "config", "bringup_sensor.yaml"),
              "r", encoding="utf-8") as config_file:
        sensor_config = yaml.safe_load(config_file)["bringup"]

    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar", default_value=str(sensor_config["enable_lidar"]).lower(),
        description="Whether to start the Livox MID-360 driver.")
    enable_camera_arg = DeclareLaunchArgument(
        "enable_camera", default_value=str(sensor_config["enable_camera"]).lower(),
        description="Whether to start the RealSense camera driver.")
    enable_rviz_arg = DeclareLaunchArgument(
        "enable_rviz", default_value=str(sensor_config["enable_rviz"]).lower(),
        description="Whether to start RViz on this host.")
    mapping_delay_arg = DeclareLaunchArgument(
        "mapping_delay", default_value="1.0",
        description="Seconds to wait for sensor streams before starting FAST-LIVO2.")

    # 传感器启动文件统一管理三个模块开关及 RViz。
    sensor_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(top_pkg_share, "launch", "bringup_sensor.launch.py")
        ),
        launch_arguments={
            "enable_lidar": LaunchConfiguration("enable_lidar"),
            "enable_camera": LaunchConfiguration("enable_camera"),
            "enable_rviz": LaunchConfiguration("enable_rviz"),
        }.items(),
    )

    # 包含 FAST-LIVO2 建图定位算法启动文件（同样屏蔽其内部单独打开的 RViz）
    fast_livo_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_livo_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={"enable_rviz": "false"}.items(),
    )
    delayed_fast_livo_mapping = TimerAction(
        period=LaunchConfiguration("mapping_delay"),
        actions=[
            LogInfo(
                msg="Sensor initialization wait complete; starting FAST-LIVO2 mapping."
            ),
            fast_livo_mapping,
        ],
    )

    return LaunchDescription([
        enable_lidar_arg,
        enable_camera_arg,
        enable_rviz_arg,
        mapping_delay_arg,
        sensor_bringup,
        delayed_fast_livo_mapping,
    ])
