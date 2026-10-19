"""Offline wireless attachment only; no hardware driver/mapping is started."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def launch_bridge(context):
    # Only explicit CLI overrides replace values from the one config authority.
    # An omitted URI/port must not silently overwrite a custom config file.
    overrides={}
    for key,kind in [('server_uri',str),('local_server_port',int)]:
        value=LaunchConfiguration(key).perform(context)
        if value!='__config__':overrides[key]=ParameterValue(value,value_type=kind)
    return [Node(package='agv_bridge',executable='websocket_bridge_node',name='websocket_bridge_node',output='screen',
                 parameters=[LaunchConfiguration('config'),overrides])]

def generate_launch_description():
    config=str(Path(get_package_share_directory('agv_bridge'))/'config/tank_inspection.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=config),
        DeclareLaunchArgument('server_uri',default_value='__config__',description='Use config unless explicitly overridden'),
        DeclareLaunchArgument('local_server_port',default_value='__config__',description='Use config unless explicitly overridden'),
        OpaqueFunction(function=launch_bridge),
    ])
