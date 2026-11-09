"""Launch the standalone MindVision Image/CameraInfo driver."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _camera_node(context):
    """Apply CLI values only when supplied, preserving the selected YAML."""
    converters = {
        "resolution_index": int, "width": int, "height": int,
        "auto_exposure": lambda value: value.lower() in {"1", "true", "yes", "on"},
        "exposure_time_us": float, "analog_gain": float, "ae_target": int,
        "rgb_gain_r": int, "rgb_gain_g": int, "rgb_gain_b": int,
        "saturation": int, "gamma": int, "contrast": int,
        "color_temperature_index": int, "frame_speed_index": int,
        "target_fps": float, "warmup_frames": int, "timeout_ms": int,
    }
    names = [
        "sdk_root", "expected_model", "serial_number", "frame_id", "image_topic",
        "camera_info_topic", "pixel_format", "resolution_index", "width", "height",
        "auto_exposure", "exposure_time_us", "analog_gain", "ae_target",
        "rgb_gain_r", "rgb_gain_g", "rgb_gain_b", "saturation", "gamma", "contrast",
        "white_balance", "color_temperature_index", "frame_speed_index", "target_fps",
        "warmup_frames", "timeout_ms",
    ]
    overrides = {}
    for name in names:
        value = LaunchConfiguration(name).perform(context).strip()
        if value:
            # ROS launch may serialize an all-digit serial as a numeric YAML
            # scalar even though the node declares a string parameter.
            overrides[name] = (
                f"sn_{value}" if name == "serial_number" else converters.get(name, str)(value)
            )
    config = Path(LaunchConfiguration("config_file").perform(context)).expanduser().resolve()
    if not config.is_file():
        raise RuntimeError(f"MindVision config file does not exist: {config}")
    return [Node(
        package="mindvision_camera_driver",
        executable="mindvision_camera_node",
        namespace=LaunchConfiguration("namespace"),
        name=LaunchConfiguration("node_name"),
        output="screen",
        emulate_tty=True,
        parameters=[str(config), overrides],
    )]


def generate_launch_description():
    share = Path(get_package_share_directory("mindvision_camera_driver"))
    default_config = share / "config" / "mindvision_sus1207gc_t1v_c.yaml"
    names = [
        "sdk_root", "expected_model", "serial_number", "frame_id", "image_topic",
        "camera_info_topic", "resolution_index", "width", "height", "auto_exposure",
        "exposure_time_us", "analog_gain", "ae_target", "rgb_gain_r", "rgb_gain_g",
        "rgb_gain_b", "saturation", "gamma", "contrast", "white_balance",
        "color_temperature_index", "frame_speed_index", "target_fps", "warmup_frames",
        "timeout_ms",
    ]
    arguments = [
        DeclareLaunchArgument("namespace", default_value="camera"),
        DeclareLaunchArgument("node_name", default_value="front"),
        DeclareLaunchArgument("config_file", default_value=str(default_config)),
        DeclareLaunchArgument("pixel_format", default_value=""),
    ]
    arguments.extend(DeclareLaunchArgument(name, default_value="") for name in names)
    return LaunchDescription(arguments + [OpaqueFunction(function=_camera_node)])
