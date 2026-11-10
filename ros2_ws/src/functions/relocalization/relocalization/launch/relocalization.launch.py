#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context):
    # Share the mode, map path and initial-pose convention with FAST-LIVO2.
    mapping_params_file = LaunchConfiguration("config_file").perform(context)
    with open(mapping_params_file, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    try:
        parameters = config["/**"]["ros__parameters"]
        locate_in_prior_map = parameters["locate_in_prior_map"]
        prior_map_path = parameters.get("prior_map_path", "")
        prior_map_name = parameters.get("prior_map_name", "")
    except (KeyError, TypeError) as error:
        raise RuntimeError(
            f"配置 {mapping_params_file} 的 /**.ros__parameters 中需要包含 "
            "locate_in_prior_map、prior_map_path 和 prior_map_name。"
        ) from error

    if not isinstance(locate_in_prior_map, bool):
        raise RuntimeError("locate_in_prior_map 必须是 YAML 布尔值 true 或 false。")
    extrinsic_R, extrinsic_T = [], []
    if locate_in_prior_map:
        if not isinstance(prior_map_path, str) or not isinstance(prior_map_name, str):
            raise RuntimeError("prior_map_path 必须是目录字符串，prior_map_name 必须是文件名字符串。")

        try:
            extrinsics = parameters["extrin_calib"]
            extrinsic_R = extrinsics["extrinsic_R"]
            extrinsic_T = extrinsics["extrinsic_T"]
        except (KeyError, TypeError) as error:
            raise RuntimeError(
                "定位模式需要建图 YAML 中的 extrin_calib.extrinsic_R 和 extrinsic_T。"
            ) from error
        for name, values, size in [
            ("extrin_calib.extrinsic_R", extrinsic_R, 9),
            ("extrin_calib.extrinsic_T", extrinsic_T, 3),
        ]:
            if not isinstance(values, list) or len(values) != size or any(
                isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                for value in values
            ):
                raise RuntimeError(f"{name} 必须包含 {size} 个有限数值。")

    start_delay = LaunchConfiguration("start_delay")

    # Direct mapping uses identity; prior mode waits for mapping's final alignment.
    map_odom_trans = Node(
        package="relocalization",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
        parameters=[
            {"map_frame_id": parameters.get("publish", {}).get("map_frame_id", "map")},
            {"locate_in_prior_map": locate_in_prior_map},
        ],
    )

    if not locate_in_prior_map:
        return [
            LogInfo(msg=f"直接建图：配准停用，map 与 camera_init 重合；配置：{mapping_params_file}"),
            map_odom_trans,
            Node(package="relocalization", executable="teaser_gicp_node",
                 name="teaser_gicp_node", output="screen",
                 parameters=[{"locate_in_prior_map": False}]),
        ]

    # Registration parameters follow old/launch_top/example_teaser_gicp.launch.py.
    teaser_gicp_node = Node(
        package="relocalization",
        executable="teaser_gicp_node",
        name="teaser_gicp_node",
        output="screen",
        parameters=[
            # 基础配置
            {"workspace_root": ParameterValue(parameters.get("workspace_root", "auto"), value_type=str)},
            {"prior_map_path": ParameterValue(prior_map_path, value_type=str)},
            {"prior_map_name": ParameterValue(prior_map_name, value_type=str)},
            {"map_frame_id": parameters.get("publish", {}).get("map_frame_id", "map")},
            {"pcl_type": "livox"},
            # 初值换算共用建图的 LiDAR -> IMU 外参；重力对齐由建图节点完成。
            {"extrin_calib.extrinsic_R": [float(value) for value in extrinsic_R]},
            {"extrin_calib.extrinsic_T": [float(value) for value in extrinsic_T]},

            # TEASER 降采样
            {"map_voxel_leaf_size": 0.4},
            {"cloud_voxel_leaf_size": 0.4},

            # GICP 降采样
            {"gicp_map_voxel_leaf_size": 0.2},
            {"gicp_cloud_voxel_leaf_size": 0.1},

            # TEASER++ 参数
            {"fpfh_normal_radius": 0.8},
            {"fpfh_feature_radius": 1.2},
            {"noise_bound": 0.3},
            {"teaser_solver_max_iter": 100},
            {"rotation_gnc_factor": 1.4},
            {"teaser_inlier_threshold": 5},
            {"teaser_success_count": 3},

            # GICP 参数
            {"gicp_solver_max_iter": 50},
            {"num_threads": 4},
            {"max_correspondence_distance": 5.0},
            {"fitness_score_thre": 0.2},
            {"converged_count_thre": 10},
            {"registration_type": "VGICP"},
            parameters.get("relocalization", {}),
            {"algorithm": LaunchConfiguration("algorithm")},
            # Shared mode cannot be overridden by nested registration options.
            {"locate_in_prior_map": locate_in_prior_map},
        ],
    )

    # This delay does not wait for localization success or start FAST-LIVO2.
    delayed_relocalization = TimerAction(
        period=start_delay,
        actions=[
            teaser_gicp_node,
        ],
    )

    return [
        LogInfo(msg=f"配准与坐标变换共用建图配置：{mapping_params_file}"),
        map_odom_trans,
        delayed_relocalization,
    ]


def generate_launch_description():
    # Use the same installed configuration as mapping_relocation.launch.py.
    mapping_config_cmd = os.path.join(
        get_package_share_directory("fast_livo"), "config", "mapping_relocation.yaml"
    )

    mapping_config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=mapping_config_cmd,
        description="与建图共用的 YAML；直接建图停用配准并发布单位 TF，先验模式读取地图与外参。",
    )

    start_delay_arg = DeclareLaunchArgument(
        "start_delay",
        default_value="5.0",
        description="启动 TEASER + GICP 节点前的延时秒数，沿用原示例默认值。",
    )

    return LaunchDescription([
        mapping_config_arg,
        start_delay_arg,
        DeclareLaunchArgument("algorithm", default_value="teaser_gicp",
                              description="teaser_gicp / teaser / gicp / icp / sac_ia_icp"),
        OpaqueFunction(function=_launch_setup),
    ])
