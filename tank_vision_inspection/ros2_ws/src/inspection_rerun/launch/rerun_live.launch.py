from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = Path(get_package_share_directory("inspection_rerun")) / "config" / "rerun.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=str(default_config)),
        DeclareLaunchArgument("output_mode", default_value="spawn"),
        DeclareLaunchArgument("save_path", default_value="output/rerun/inspection.rrd"),
        Node(
            package="inspection_rerun",
            executable="rerun_bridge_node",
            name="rerun_bridge",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "output_mode": ParameterValue(LaunchConfiguration("output_mode"), value_type=str),
                    "save_path": ParameterValue(LaunchConfiguration("save_path"), value_type=str),
                },
            ],
        ),
    ])
