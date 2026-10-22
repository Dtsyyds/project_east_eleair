"""Launch the MID-360 + MindVision recording profile and optional hardware.

MID-360, MindVision and MCAP recording start by default. Eddy-current and
synchronization remain explicit opt-ins. Ultrasound and robot drivers remain
external adapters and must publish the documented topics.
"""

from datetime import datetime
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _default_bag_root() -> Path:
    """Use the dedicated persistent storage for all default MCAP recordings."""
    return Path(os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data")) / "mcap"


def _start_mid360(context):
    """Create the optional Livox node only after validating its explicit config."""
    if not _as_bool(LaunchConfiguration("start_mid360").perform(context)):
        return []

    config_value = LaunchConfiguration("mid360_config_path").perform(context).strip()
    if not config_value:
        raise RuntimeError(
            "start_mid360:=true requires mid360_config_path:=/absolute/path/MID360_config.json")
    config_path = Path(config_value).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"MID-360 config file does not exist: {config_path}")

    # Fail here, rather than at import/generation time, so the core launch remains
    # usable when this optional vendor package was not included in a partial build.
    get_package_share_directory("livox_ros_driver2")
    publish_freq = float(LaunchConfiguration("mid360_publish_freq").perform(context))
    if publish_freq <= 0.0:
        raise RuntimeError("mid360_publish_freq must be greater than zero")

    return [
        LogInfo(msg=(
            "Starting livox_ros_driver2 in PointCloud2 PointXYZRTLT mode; "
            f"config={config_path}"
        )),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": publish_freq,
                "output_data_type": 0,
                "frame_id": LaunchConfiguration("mid360_frame_id"),
                "lvx_file_path": "",
                "user_config_path": str(config_path),
                "cmdline_input_bd_code": "",
            }],
            remappings=[
                ("/livox/lidar", LaunchConfiguration("lidar_topic")),
                ("/livox/imu", "/livox/imu_raw"),
            ],
        ),
        Node(
            package="inspection_livox_adapter",
            executable="livox_imu_adapter_node",
            name="livox_imu_adapter",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "acceleration_scale": LaunchConfiguration(
                    "mid360_imu_acceleration_scale"),
            }],
            remappings=[
                ("/livox/imu", LaunchConfiguration("imu_topic")),
            ],
        ),
    ]


def _publish_rough_extrinsics(context):
    """Publish a traceable rough MID-360-to-camera transform when both devices run."""
    required_flags = ("start_mid360", "start_mindvision", "publish_rough_extrinsics")
    if not all(_as_bool(LaunchConfiguration(name).perform(context)) for name in required_flags):
        return []

    config_value = LaunchConfiguration("rough_extrinsics_path").perform(context).strip()
    if not config_value:
        raise RuntimeError("publish_rough_extrinsics:=true requires rough_extrinsics_path")
    config_path = Path(config_value).expanduser().resolve()
    if not config_path.is_file():
        raise RuntimeError(f"rough extrinsics file does not exist: {config_path}")

    document = yaml.safe_load(config_path.read_text()) or {}
    calibration = document.get("rough_extrinsics", {})
    if not calibration.get("enabled", False):
        return [LogInfo(msg=f"Rough extrinsics disabled in {config_path}")]
    if calibration.get("calibration_stage") != "rough":
        raise RuntimeError("rough extrinsics must declare calibration_stage: rough")
    if calibration.get("production_valid") is not False:
        raise RuntimeError("rough extrinsics must explicitly declare production_valid: false")

    transform = calibration.get("transform", {})
    expected_parent = str(transform.get("parent_frame", ""))
    parent_frame = LaunchConfiguration("mid360_frame_id").perform(context)
    if expected_parent != parent_frame:
        raise RuntimeError(
            f"rough extrinsics parent_frame {expected_parent!r} does not match "
            f"mid360_frame_id {parent_frame!r}")
    child_frame = str(transform.get("child_frame", ""))
    if not child_frame:
        raise RuntimeError("rough extrinsics child_frame must not be empty")

    translation = transform.get("translation_m", {})
    rotation = transform.get("rotation_rpy_rad", {})
    values = {
        "x": float(translation["x"]),
        "y": float(translation["y"]),
        "z": float(translation["z"]),
        "roll": float(rotation["roll"]),
        "pitch": float(rotation["pitch"]),
        "yaw": float(rotation["yaw"]),
    }
    calibration_id = str(calibration.get("calibration_id", "UNASSIGNED"))
    return [
        LogInfo(msg=(
            f"Publishing ROUGH extrinsics {calibration_id}: {parent_frame} -> {child_frame}; "
            f"xyz_m=({values['x']}, {values['y']}, {values['z']}), "
            f"rpy_rad=({values['roll']}, {values['pitch']}, {values['yaw']}); "
            "production_valid=false"
        )),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_to_mindvision_rough_tf",
            output="screen",
            arguments=[
                "--x", str(values["x"]),
                "--y", str(values["y"]),
                "--z", str(values["z"]),
                "--roll", str(values["roll"]),
                "--pitch", str(values["pitch"]),
                "--yaw", str(values["yaw"]),
                "--frame-id", parent_frame,
                "--child-frame-id", child_frame,
            ],
        ),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("inspection_bringup"))
    livox_share = Path(get_package_share_directory("livox_ros_driver2"))
    fisheye_share = Path(get_package_share_directory("fisheye_camera_driver"))
    default_mid360_config = livox_share / "config" / "MID360_config.json"
    default_rough_extrinsics = share / "config" / "rough_extrinsics.yaml"
    default_output_directory = (
        _default_bag_root() /
        f"mid360_mindvision_run_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
    )
    record_launch = PythonLaunchDescriptionSource(
        str(share / "launch" / "record_only.launch.py"))

    start_mindvision = LaunchConfiguration("start_mindvision")
    start_fisheye = LaunchConfiguration("start_fisheye")
    start_d405 = LaunchConfiguration("start_d405")
    start_eddy = LaunchConfiguration("start_eddy")
    start_sync = LaunchConfiguration("start_sync")
    record = LaunchConfiguration("record")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_mindvision", default_value="true",
            description="Start the connected MindVision stream (SUS1207GC BGR8 by default)."),
        DeclareLaunchArgument(
            "mindvision_sdk_root", default_value="",
            description="Official SDK root; empty uses MINDVISION_SDK_ROOT or workspace discovery."),
        DeclareLaunchArgument(
            "mindvision_serial_number", default_value="046041920527",
            description="Camera serial selector; empty is allowed only when exactly one camera is enumerated."),
        DeclareLaunchArgument("mindvision_expected_model", default_value="MV-SUS1207GC"),
        DeclareLaunchArgument("mindvision_resolution_index", default_value="-1"),
        DeclareLaunchArgument("mindvision_width", default_value="4096"),
        DeclareLaunchArgument("mindvision_height", default_value="3072"),
        DeclareLaunchArgument("mindvision_auto_exposure", default_value="false"),
        DeclareLaunchArgument("mindvision_exposure_time_us", default_value="70877.0"),
        DeclareLaunchArgument("mindvision_analog_gain", default_value="4.0"),
        DeclareLaunchArgument("mindvision_ae_target", default_value="70"),
        DeclareLaunchArgument("mindvision_rgb_gain_r", default_value="131"),
        DeclareLaunchArgument("mindvision_rgb_gain_g", default_value="110"),
        DeclareLaunchArgument("mindvision_rgb_gain_b", default_value="147"),
        DeclareLaunchArgument("mindvision_saturation", default_value="100"),
        DeclareLaunchArgument("mindvision_gamma", default_value="-1"),
        DeclareLaunchArgument("mindvision_contrast", default_value="-1"),
        DeclareLaunchArgument("mindvision_white_balance", default_value="off"),
        DeclareLaunchArgument("mindvision_color_temperature_index", default_value="-1"),
        DeclareLaunchArgument("mindvision_frame_speed_index", default_value="-1"),
        # Legacy SUF profile aliases remain accepted when explicitly supplied.
        DeclareLaunchArgument("mindvision_exposure_mode", default_value=""),
        DeclareLaunchArgument("mindvision_exposure_us", default_value="-1.0"),
        DeclareLaunchArgument("mindvision_ae_min_exposure_us", default_value="0.0"),
        DeclareLaunchArgument("mindvision_ae_max_exposure_us", default_value="0.0"),
        DeclareLaunchArgument("mindvision_ae_min_analog_gain_raw", default_value="0"),
        DeclareLaunchArgument("mindvision_ae_max_analog_gain_raw", default_value="0"),
        DeclareLaunchArgument("mindvision_warmup_frames", default_value="60"),
        DeclareLaunchArgument(
            "mindvision_publish_rate_hz", default_value="10.0",
            description=(
                "Published image rate; 10 Hz is the lossless full-resolution eMMC profile, "
                "zero requests the camera maximum.")),
        DeclareLaunchArgument("mindvision_timeout_ms", default_value="1000"),
        DeclareLaunchArgument("mindvision_frame_id", default_value="mindvision_optical_frame"),
        DeclareLaunchArgument("mindvision_raw_bayer_topic", default_value=""),
        DeclareLaunchArgument("mindvision_link_frame_id", default_value="camera_link"),
        DeclareLaunchArgument("start_fisheye", default_value="true"),
        DeclareLaunchArgument(
            "fisheye_config_file", default_value=str(fisheye_share / "config/fisheye.yaml")),
        DeclareLaunchArgument("fisheye_device", default_value=""),
        DeclareLaunchArgument("start_d405", default_value="true"),
        DeclareLaunchArgument("d405_serial_number", default_value=""),
        DeclareLaunchArgument("d405_rgb_profile", default_value="848x480x30"),
        DeclareLaunchArgument("d405_depth_profile", default_value="848x480x30"),
        DeclareLaunchArgument(
            "start_mid360", default_value="true",
            description="Start livox_ros_driver2 in PointCloud2 mode (enabled by default)."),
        DeclareLaunchArgument(
            "mid360_config_path", default_value=str(default_mid360_config),
            description="Vendor JSON config path; defaults to the verified local MID-360 config."),
        DeclareLaunchArgument("mid360_publish_freq", default_value="10.0"),
        DeclareLaunchArgument("mid360_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument(
            "publish_rough_extrinsics", default_value="true",
            description="Publish the traceable rough MID-360-to-camera static transform."),
        DeclareLaunchArgument(
            "rough_extrinsics_path", default_value=str(default_rough_extrinsics),
            description="YAML containing the non-production rough extrinsic measurement."),
        DeclareLaunchArgument(
            "mid360_imu_acceleration_scale", default_value="9.80665",
            description="Convert the vendor IMU acceleration from g to m/s^2."),
        DeclareLaunchArgument(
            "start_eddy", default_value="false",
            description="Start the local STM32 USB-UART eddy-current driver."),
        DeclareLaunchArgument("eddy_device", default_value="/dev/eddy"),
        DeclareLaunchArgument("eddy_baud", default_value="4000000"),
        DeclareLaunchArgument("eddy_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("eddy_settings_query_rate", default_value="1.0"),
        DeclareLaunchArgument("eddy_sensor_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("eddy_calibration_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("eddy_frame_id", default_value="eddy_probe_link"),
        DeclareLaunchArgument("eddy_sampling_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("eddy_gain_db", default_value="-1.0"),
        DeclareLaunchArgument("eddy_lift_off_mm", default_value="-1.0"),
        DeclareLaunchArgument("start_sync", default_value="false"),
        DeclareLaunchArgument(
            "anchor_decimation", default_value="1",
            description="Generate one FusionIndex for every N accepted anchor frames."),
        DeclareLaunchArgument(
            "software_clock_match_enable", default_value="false",
            description="Apply camera/Livox software time offsets against the anchor clock."),
        DeclareLaunchArgument(
            "software_clock_adaptive_enable", default_value="false",
            description="Continuously adapt offsets; experimental and disabled by default."),
        DeclareLaunchArgument("camera_clock_offset_ms", default_value="0.0"),
        DeclareLaunchArgument("livox_clock_offset_ms", default_value="0.0"),
        DeclareLaunchArgument("clock_offset_alpha", default_value="0.25"),
        DeclareLaunchArgument("clock_offset_max_ms", default_value="20.0"),
        DeclareLaunchArgument("clock_offset_update_gate_ms", default_value="45.0"),
        DeclareLaunchArgument("clock_offset_window_matches", default_value="50"),
        DeclareLaunchArgument(
            "anchor_topic", default_value="/inspection/eddy_current/raw",
            description="Fusion anchor: /inspection/eddy_current/raw or /inspection/ultrasound/raw."),
        DeclareLaunchArgument(
            "record", default_value="true",
            description="Record configured topics to split MCAP files (enabled by default)."),
        DeclareLaunchArgument("debug_mode", default_value="false"),
        DeclareLaunchArgument(
            "output_directory", default_value=str(default_output_directory),
            description="MCAP output directory; defaults to a new timestamped directory."),
        DeclareLaunchArgument("asset_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("course_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("plate_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument("weld_id", default_value="UNASSIGNED"),
        DeclareLaunchArgument(
            "camera_image_topic", default_value="/camera/front/image_raw"),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="/camera/front/camera_info"),
        DeclareLaunchArgument("lidar_topic", default_value="/livox/lidar"),
        DeclareLaunchArgument("imu_topic", default_value="/livox/imu"),
        DeclareLaunchArgument("odometry_topic", default_value="/robot/odometry"),
        DeclareLaunchArgument("robot_state_topic", default_value="/robot/state"),
        DeclareLaunchArgument(
            "ultrasound_topic", default_value="/inspection/ultrasound/raw"),
        DeclareLaunchArgument(
            "eddy_current_topic", default_value="/inspection/eddy_current/raw"),
        DeclareLaunchArgument(
            "probe_state_topic", default_value="/inspection/probe_state"),
        LogInfo(msg=[
            "Default profile: MID-360 + MindVision SUS1207GC BGR8 + IMU SI adapter + MCAP recording; output=",
            LaunchConfiguration("output_directory"),
            ". Eddy-current and synchronization remain opt-in.",
        ]),
        Node(
            package="mindvision_camera_driver",
            executable="mindvision_camera_node",
            namespace="camera",
            name="front",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_mindvision),
            parameters=[{
                "sdk_root": ParameterValue(
                    LaunchConfiguration("mindvision_sdk_root"), value_type=str),
                "serial_number": ParameterValue(
                    ["sn_", LaunchConfiguration("mindvision_serial_number")], value_type=str),
                "expected_model": ParameterValue(
                    LaunchConfiguration("mindvision_expected_model"), value_type=str),
                "resolution_index": ParameterValue(
                    LaunchConfiguration("mindvision_resolution_index"), value_type=int),
                "width": ParameterValue(
                    LaunchConfiguration("mindvision_width"), value_type=int),
                "height": ParameterValue(
                    LaunchConfiguration("mindvision_height"), value_type=int),
                "exposure_mode": ParameterValue(
                    LaunchConfiguration("mindvision_exposure_mode"), value_type=str),
                "exposure_us": ParameterValue(
                    LaunchConfiguration("mindvision_exposure_us"), value_type=float),
                "auto_exposure": ParameterValue(
                    LaunchConfiguration("mindvision_auto_exposure"), value_type=bool),
                "exposure_time_us": ParameterValue(
                    LaunchConfiguration("mindvision_exposure_time_us"), value_type=float),
                "analog_gain": ParameterValue(
                    LaunchConfiguration("mindvision_analog_gain"), value_type=float),
                "ae_target": ParameterValue(
                    LaunchConfiguration("mindvision_ae_target"), value_type=int),
                "rgb_gain_r": ParameterValue(
                    LaunchConfiguration("mindvision_rgb_gain_r"), value_type=int),
                "rgb_gain_g": ParameterValue(
                    LaunchConfiguration("mindvision_rgb_gain_g"), value_type=int),
                "rgb_gain_b": ParameterValue(
                    LaunchConfiguration("mindvision_rgb_gain_b"), value_type=int),
                "saturation": ParameterValue(
                    LaunchConfiguration("mindvision_saturation"), value_type=int),
                "gamma": ParameterValue(
                    LaunchConfiguration("mindvision_gamma"), value_type=int),
                "contrast": ParameterValue(
                    LaunchConfiguration("mindvision_contrast"), value_type=int),
                "white_balance": ParameterValue(
                    LaunchConfiguration("mindvision_white_balance"), value_type=str),
                "color_temperature_index": ParameterValue(
                    LaunchConfiguration("mindvision_color_temperature_index"), value_type=int),
                "frame_speed_index": ParameterValue(
                    LaunchConfiguration("mindvision_frame_speed_index"), value_type=int),
                "ae_min_exposure_us": ParameterValue(
                    LaunchConfiguration("mindvision_ae_min_exposure_us"), value_type=float),
                "ae_max_exposure_us": ParameterValue(
                    LaunchConfiguration("mindvision_ae_max_exposure_us"), value_type=float),
                "ae_min_analog_gain_raw": ParameterValue(
                    LaunchConfiguration("mindvision_ae_min_analog_gain_raw"), value_type=int),
                "ae_max_analog_gain_raw": ParameterValue(
                    LaunchConfiguration("mindvision_ae_max_analog_gain_raw"), value_type=int),
                "warmup_frames": ParameterValue(
                    LaunchConfiguration("mindvision_warmup_frames"), value_type=int),
                "publish_rate_hz": ParameterValue(
                    LaunchConfiguration("mindvision_publish_rate_hz"), value_type=float),
                "timeout_ms": ParameterValue(
                    LaunchConfiguration("mindvision_timeout_ms"), value_type=int),
                "frame_id": ParameterValue(
                    LaunchConfiguration("mindvision_frame_id"), value_type=str),
                "raw_bayer_topic": ParameterValue(
                    LaunchConfiguration("mindvision_raw_bayer_topic"), value_type=str),
            }],
        ),
        Node(
            package="fisheye_camera_driver",
            executable="fisheye_camera_node",
            namespace="camera",
            name="fisheye",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_fisheye),
            parameters=[LaunchConfiguration("fisheye_config_file"), {
                "device": ParameterValue(LaunchConfiguration("fisheye_device"), value_type=str),
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory("realsense2_camera"), "/launch/rs_launch.py"
            ]),
            condition=IfCondition(start_d405),
            launch_arguments={
                "camera_namespace": "d405",
                "camera_name": "camera",
                # Keep numeric-only serials as strings through the generated
                # parameter YAML. realsense2_camera removes this prefix.
                "serial_no": ["_", LaunchConfiguration("d405_serial_number")],
                "enable_color": "true",
                "enable_depth": "true",
                "enable_infra1": "true",
                "enable_infra2": "true",
                # D405 exposes its RGB stream through the depth module rather
                # than a separate RGB camera sensor.
                "depth_module.color_profile": LaunchConfiguration("d405_rgb_profile"),
                "depth_module.depth_profile": LaunchConfiguration("d405_depth_profile"),
                "depth_module.infra_profile": LaunchConfiguration("d405_depth_profile"),
                "diagnostics_period": "1.0",
            }.items(),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mindvision_optical_tf",
            output="screen",
            condition=IfCondition(start_mindvision),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "-1.5707963267948966",
                "--pitch", "0",
                "--yaw", "-1.5707963267948966",
                "--frame-id", LaunchConfiguration("mindvision_link_frame_id"),
                "--child-frame-id", LaunchConfiguration("mindvision_frame_id"),
            ],
        ),
        OpaqueFunction(function=_start_mid360),
        # This identity transform gives the otherwise standalone Livox frame a TF-tree
        # anchor for RViz. It is not a measured camera-to-MID360 extrinsic calibration.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_visualization_tf",
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_mid360")),
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "inspection_origin",
                "--child-frame-id", LaunchConfiguration("mid360_frame_id"),
            ],
        ),
        OpaqueFunction(function=_publish_rough_extrinsics),
        Node(
            package="eddy_driver",
            executable="eddy_driver_node",
            name="eddy_driver_node",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(start_eddy),
            parameters=[{
                "device": LaunchConfiguration("eddy_device"),
                "baud": LaunchConfiguration("eddy_baud"),
                "publish_rate": LaunchConfiguration("eddy_publish_rate"),
                "settings_query_rate": LaunchConfiguration("eddy_settings_query_rate"),
                "sensor_id": LaunchConfiguration("eddy_sensor_id"),
                "calibration_id": LaunchConfiguration("eddy_calibration_id"),
                "frame_id": LaunchConfiguration("eddy_frame_id"),
                "sampling_rate_hz": LaunchConfiguration("eddy_sampling_rate_hz"),
                "gain_db": LaunchConfiguration("eddy_gain_db"),
                "lift_off_mm": LaunchConfiguration("eddy_lift_off_mm"),
            }],
            remappings=[
                ("/inspection/eddy_current/raw", LaunchConfiguration("eddy_current_topic")),
            ],
        ),
        Node(
            package="inspection_sync",
            executable="inspection_sync_node",
            name="inspection_sync",
            output="screen",
            condition=IfCondition(start_sync),
            parameters=[
                str(share / "config" / "cache.yaml"),
                {
                    "anchor_topic": LaunchConfiguration("anchor_topic"),
                    "anchor_decimation": LaunchConfiguration("anchor_decimation"),
                    "software_clock_match_enable": LaunchConfiguration(
                        "software_clock_match_enable"),
                    "software_clock_adaptive_enable": LaunchConfiguration(
                        "software_clock_adaptive_enable"),
                    "camera_clock_offset_ms": LaunchConfiguration("camera_clock_offset_ms"),
                    "livox_clock_offset_ms": LaunchConfiguration("livox_clock_offset_ms"),
                    "clock_offset_alpha": LaunchConfiguration("clock_offset_alpha"),
                    "clock_offset_max_ms": LaunchConfiguration("clock_offset_max_ms"),
                    "clock_offset_update_gate_ms": LaunchConfiguration(
                        "clock_offset_update_gate_ms"),
                    "clock_offset_window_matches": LaunchConfiguration(
                        "clock_offset_window_matches"),
                    "asset_id": LaunchConfiguration("asset_id"),
                    "course_id": LaunchConfiguration("course_id"),
                    "plate_id": LaunchConfiguration("plate_id"),
                    "weld_id": LaunchConfiguration("weld_id"),
                    "asset_coordinate_valid": False,
                    "camera_reliable_qos": True,
                },
            ],
            remappings=[
                ("/camera/front/image_raw", LaunchConfiguration("camera_image_topic")),
                ("/livox/lidar", LaunchConfiguration("lidar_topic")),
                ("/livox/imu", LaunchConfiguration("imu_topic")),
                ("/robot/odometry", LaunchConfiguration("odometry_topic")),
                ("/robot/state", LaunchConfiguration("robot_state_topic")),
                ("/inspection/ultrasound/raw", LaunchConfiguration("ultrasound_topic")),
                ("/inspection/eddy_current/raw", LaunchConfiguration("eddy_current_topic")),
                ("/inspection/probe_state", LaunchConfiguration("probe_state_topic")),
            ],
        ),
        IncludeLaunchDescription(
            record_launch,
            condition=IfCondition(record),
            launch_arguments={
                "debug_mode": LaunchConfiguration("debug_mode"),
                "output_directory": LaunchConfiguration("output_directory"),
            }.items(),
        ),
    ])
