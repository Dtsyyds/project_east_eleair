from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LIDAR_CONFIGS = {
    'mid360': 'MID360_config.json',
    'mid360s': 'MID360s_config.json',
}


def _launch_driver(context):
    package_share = Path(get_package_share_directory('livox_ros_driver2'))
    config_path = Path(
        LaunchConfiguration('config_file').perform(context)
    ).expanduser()
    with config_path.open(encoding='utf-8') as stream:
        config = yaml.safe_load(stream)

    if not isinstance(config, dict):
        raise RuntimeError(f'启动配置必须是 YAML 映射：{config_path}')

    lidar_model = str(config.get('lidar_model', '')).strip().lower()
    if lidar_model not in LIDAR_CONFIGS:
        raise RuntimeError(
            f'不支持的 lidar_model：{lidar_model!r}；请选择 mid360 或 mid360s。'
        )

    parameters = config.get('driver_parameters')
    if not isinstance(parameters, dict):
        raise RuntimeError(f'driver_parameters 必须是 YAML 映射：{config_path}')

    lidar_config = package_share / 'config' / LIDAR_CONFIGS[lidar_model]
    if not lidar_config.is_file():
        raise RuntimeError(f'雷达 JSON 配置不存在：{lidar_config}')

    parameters = dict(parameters)
    parameters['user_config_path'] = str(lidar_config)

    return [
        LogInfo(msg=f'Livox 型号：{lidar_model}；设备配置：{lidar_config}'),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[parameters],
        ),
    ]


def generate_launch_description():
    config_path = (
        Path(get_package_share_directory('livox_ros_driver2'))
        / 'config' / 'msg_MID360.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=str(config_path),
            description='启动配置 YAML，包含 lidar_model 和 driver_parameters。',
        ),
        OpaqueFunction(function=_launch_driver),
    ])
