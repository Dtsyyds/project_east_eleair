from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', description='Absolute path to independent experimental YAML'),
        DeclareLaunchArgument('stream', default_value='infra1'),
        DeclareLaunchArgument('depth_scale_m', default_value='0.001', description='Existing RealSense ROS 16UC1 millimetres, NOT SDK native units'),
        Node(package='d405_seam_ros', executable='d405_seam_node', output='screen',
             arguments=['--config', LaunchConfiguration('config'), '--stream', LaunchConfiguration('stream'),
                        '--depth-scale-m', LaunchConfiguration('depth_scale_m')])])
