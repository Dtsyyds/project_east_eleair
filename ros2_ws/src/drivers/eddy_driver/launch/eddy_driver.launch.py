import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('eddy_driver')
    config_file = os.path.join(pkg_share, 'config', 'eddy.yaml')

    return LaunchDescription([
        Node(
            package='eddy_driver',
            executable='eddy_driver_node',
            name='eddy_driver_node',
            output='screen',
            parameters=[config_file],
        )
    ])
