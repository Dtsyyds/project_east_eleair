"""One owner for FAST-LIVO, registration and map alignment; sensors are external."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _core_exited(event, context):
    # All three nodes are persistent, including disabled registration. A partial
    # stack must not keep reporting healthy mapping after a core node exits.
    if context.is_shutdown or not isinstance(event.action, Node):
        return []
    name = event.action.node_name.rsplit('/', 1)[-1]
    if name in ('laserMapping', 'teaser_gicp_node', 'transform_publisher'):
        reason = f'SLAM core node {name} exited ({event.returncode})'
        return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
    return []


def generate_launch_description():
    mapping_share = get_package_share_directory('fast_livo')
    relocation_share = get_package_share_directory('relocalization')
    mapping_args = ('config_file', 'camera_file', 'enable_rviz',
                    'republish_compressed', 'image_input_topic', 'use_respawn')
    relocation_args = ('config_file', 'start_delay', 'algorithm')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(mapping_share, 'config', 'mapping_relocation.yaml')),
        DeclareLaunchArgument('camera_file', default_value=os.path.join(mapping_share, 'config', 'camera_parameter.yaml')),
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('republish_compressed', default_value='false'),
        DeclareLaunchArgument('image_input_topic', default_value=''),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('start_delay', default_value='5.0'),
        DeclareLaunchArgument('algorithm', default_value='teaser_gicp'),
        # Match the console's save budget, including command-line use.
        DeclareLaunchArgument('sigterm_timeout', default_value='120'),
        DeclareLaunchArgument('sigkill_timeout', default_value='5'),
        RegisterEventHandler(OnProcessExit(on_exit=_core_exited)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(relocation_share, 'launch', 'relocalization.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in relocation_args}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mapping_share, 'launch', 'mapping_relocation.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in mapping_args}.items()),
    ])
