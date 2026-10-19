from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _bag_player(context):
    bag_path = Path(LaunchConfiguration("bag_path").perform(context)).expanduser().resolve()
    if not bag_path.exists():
        raise RuntimeError(f"bag_path does not exist: {bag_path}")
    rate = float(LaunchConfiguration("playback_rate").perform(context))
    if rate <= 0.0:
        raise RuntimeError("playback_rate must be positive")
    command = ["ros2", "bag", "play", str(bag_path), "--rate", str(rate)]
    if LaunchConfiguration("loop").perform(context).lower() in {"1", "true", "yes", "on"}:
        command.append("--loop")
    if LaunchConfiguration("use_clock").perform(context).lower() in {"1", "true", "yes", "on"}:
        command.append("--clock")
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    default_config = Path(get_package_share_directory("inspection_rerun")) / "config" / "rerun.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("bag_path", description="Absolute path to an existing rosbag2/MCAP directory"),
        DeclareLaunchArgument("config_file", default_value=str(default_config)),
        DeclareLaunchArgument("output_mode", default_value="spawn"),
        DeclareLaunchArgument("save_path", default_value="output/rerun/replay.rrd"),
        DeclareLaunchArgument("playback_rate", default_value="1.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("use_clock", default_value="false"),
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
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_clock"), value_type=bool),
                },
            ],
        ),
        OpaqueFunction(function=_bag_player),
    ])
