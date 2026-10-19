"""Single-owner MindVision + stable seam field-validation launch.

This launch intentionally starts the camera driver and the stable seam node in
ROS-topic mode.  It never starts ``stable_seam_direct``.
"""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from field_validation_common import prepare_run, write_manifest


def _bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _start(context):
    bringup_share = Path(get_package_share_directory("inspection_bringup"))
    camera_share = Path(get_package_share_directory("mindvision_camera_driver"))
    vision_share = Path(get_package_share_directory("tank_vision_ros"))
    data_root = LaunchConfiguration("data_root").perform(context)
    run_id = LaunchConfiguration("run_id").perform(context)
    tracker_config = Path(LaunchConfiguration("tracker_config").perform(context)).expanduser().resolve()
    tracker_reference = Path(LaunchConfiguration("tracker_reference").perform(context)).expanduser().resolve()
    if not tracker_config.is_file() or not tracker_reference.is_file():
        raise RuntimeError("tracker_config and tracker_reference must exist")
    run_dir, filesystem = prepare_run(data_root, run_id)
    run_id = run_dir.name
    frozen = run_dir / "frozen_configs"
    shutil.copy2(tracker_config, frozen / tracker_config.name)
    shutil.copy2(tracker_reference, frozen / tracker_reference.name)

    topics_doc = yaml.safe_load((bringup_share / "config" / "record_topics_visual.yaml").read_text())
    topics = list(topics_doc["topics"]["lightweight"])
    if _bool(LaunchConfiguration("record_raw_image").perform(context)):
        topics += list(topics_doc["topics"]["raw"])
    camera_topic = LaunchConfiguration("camera_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    roi_validation_output_root = LaunchConfiguration("roi_validation_output_root").perform(context).strip()
    if not roi_validation_output_root:
        roi_validation_output_root = str(run_dir / "roi_validation")
    topics = [camera_topic if value == "/camera/front/image_raw" else
              camera_info_topic if value == "/camera/front/camera_info" else value for value in topics]
    launch_parameters = {name: LaunchConfiguration(name).perform(context) for name in (
        "camera_serial", "resolution_index", "exposure_mode", "exposure_us",
        "ae_min_exposure_us", "ae_max_exposure_us", "ae_min_analog_gain_raw",
        "ae_max_analog_gain_raw", "camera_publish_rate_hz", "record_raw_image",
        "record_mcap", "start_mid360", "start_sync", "roi_validation_recording_enabled",
        "roi_validation_queue_size", "roi_validation_max_duration_sec",
        "geometry_provenance_sampling_enabled", "geometry_provenance_queue_size",
        "geometry_provenance_sample_period_sec", "geometry_provenance_max_events",
        "per_frame_metrics_log", "enable_tracker")}
    launch_parameters["roi_validation_output_root"] = roi_validation_output_root
    manifest_path = write_manifest(
        run_dir, run_id=run_id, data_root=data_root, tracker_config=tracker_config,
        tracker_reference=tracker_reference,
        camera={"model": "MindVision:runtime-enumerated", "serial_number": LaunchConfiguration("camera_serial").perform(context),
                "topic": camera_topic, "camera_info_topic": camera_info_topic,
                "resolution_index": int(LaunchConfiguration("resolution_index").perform(context)),
                "encoding": "runtime-enumerated"},
        exposure={"mode": LaunchConfiguration("exposure_mode").perform(context),
                  "requested_exposure_us": float(LaunchConfiguration("exposure_us").perform(context)),
                  "ae_min_exposure_us": float(LaunchConfiguration("ae_min_exposure_us").perform(context)),
                  "ae_max_exposure_us": float(LaunchConfiguration("ae_max_exposure_us").perform(context)),
                  "ae_min_analog_gain_raw": int(LaunchConfiguration("ae_min_analog_gain_raw").perform(context)),
                  "ae_max_analog_gain_raw": int(LaunchConfiguration("ae_max_analog_gain_raw").perform(context)),
                  "runtime_values": "from /diagnostics; gain_db unavailable unless SDK exposes it"},
        topics=topics, launch_parameters=launch_parameters, camera_info_valid=None)
    (run_dir / "run_id.txt").write_text(run_id + "\n", encoding="utf-8")
    (run_dir / "launch_parameters.json").write_text(json.dumps({"filesystem": filesystem, **launch_parameters}, indent=2) + "\n")

    actions = [
        Node(package="mindvision_camera_driver", executable="mindvision_camera_node",
             namespace="camera", name="front", output="screen", emulate_tty=True,
             parameters=[{
                 "sdk_root": LaunchConfiguration("sdk_root"),
                 # A numeric-looking serial must remain a ROS string; without
                 # an explicit type, launch converts 046081820650 to a DOUBLE.
                 "serial_number": ParameterValue(LaunchConfiguration("camera_serial"), value_type=str),
                 "resolution_index": ParameterValue(LaunchConfiguration("resolution_index"), value_type=int),
                 "width": 4096, "height": 3072,
                 "exposure_mode": LaunchConfiguration("exposure_mode"),
                 "exposure_us": ParameterValue(LaunchConfiguration("exposure_us"), value_type=float),
                 "ae_min_exposure_us": ParameterValue(LaunchConfiguration("ae_min_exposure_us"), value_type=float),
                 "ae_max_exposure_us": ParameterValue(LaunchConfiguration("ae_max_exposure_us"), value_type=float),
                 "ae_min_analog_gain_raw": ParameterValue(LaunchConfiguration("ae_min_analog_gain_raw"), value_type=int),
                 "ae_max_analog_gain_raw": ParameterValue(LaunchConfiguration("ae_max_analog_gain_raw"), value_type=int),
                 "warmup_frames": ParameterValue(LaunchConfiguration("warmup_frames"), value_type=int),
                 "publish_rate_hz": ParameterValue(LaunchConfiguration("camera_publish_rate_hz"), value_type=float),
                 "timeout_ms": ParameterValue(LaunchConfiguration("timeout_ms"), value_type=int),
                 "qos_reliability": ParameterValue(
                     LaunchConfiguration("camera_qos_reliability"), value_type=str),
             }]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="mindvision_optical_tf", output="screen",
             arguments=["--x", "0", "--y", "0", "--z", "0",
                        "--roll", "-1.5707963267948966", "--pitch", "0",
                        "--yaw", "-1.5707963267948966", "--frame-id", "camera_link",
                        "--child-frame-id", "mindvision_optical_frame"]),
    ]
    if _bool(LaunchConfiguration("enable_tracker").perform(context)):
        actions.append(Node(
             package="tank_vision_ros", executable="stable_seam_node", name="stable_seam_node",
             output="screen", parameters=[{
                 "input_mode": "ros_topic", "image_topic": camera_topic,
                 "camera_info_topic": camera_info_topic,
                 "config_file": str(tracker_config), "reference_file": str(tracker_reference),
                 "publish_debug_image": _bool(LaunchConfiguration("publish_debug_image").perform(context)),
                 "snapshot_output_dir": str(run_dir / "failure_snapshots"),
                 "live_session_output_root": str(run_dir / "live_session"),
                 "auto_save_failures": _bool(LaunchConfiguration("auto_save_failures").perform(context)),
                 "auto_save_full_frame": _bool(LaunchConfiguration("auto_save_full_frame").perform(context)),
                 "auto_save_context_jpeg": _bool(LaunchConfiguration("auto_save_context_jpeg").perform(context)),
                 "save_lossless_roi": _bool(LaunchConfiguration("save_lossless_roi").perform(context)),
                 "auto_save_policy": LaunchConfiguration("auto_save_policy").perform(context),
                 "per_frame_metrics_log": _bool(LaunchConfiguration("per_frame_metrics_log").perform(context)),
                 "failure_snapshot_cooldown_sec": float(LaunchConfiguration("failure_snapshot_cooldown_sec").perform(context)),
                 "max_failure_snapshots_per_session": int(LaunchConfiguration("max_failure_snapshots").perform(context)),
                 "roi_validation_recording_enabled": _bool(LaunchConfiguration("roi_validation_recording_enabled").perform(context)),
                 "roi_validation_output_root": roi_validation_output_root,
                 "roi_validation_queue_size": int(LaunchConfiguration("roi_validation_queue_size").perform(context)),
                 "roi_validation_max_duration_sec": float(LaunchConfiguration("roi_validation_max_duration_sec").perform(context)),
                 "geometry_provenance_sampling_enabled": _bool(LaunchConfiguration("geometry_provenance_sampling_enabled").perform(context)),
                 "geometry_provenance_output_root": str(run_dir / "geometry_provenance"),
                 "geometry_provenance_queue_size": int(LaunchConfiguration("geometry_provenance_queue_size").perform(context)),
                 "geometry_provenance_sample_period_sec": float(LaunchConfiguration("geometry_provenance_sample_period_sec").perform(context)),
                 "geometry_provenance_max_events": int(LaunchConfiguration("geometry_provenance_max_events").perform(context)),
             }]))
    if _bool(LaunchConfiguration("record_mcap").perform(context)):
        profile = "raw_short" if _bool(LaunchConfiguration("record_raw_image").perform(context)) else "storage"
        split = topics_doc[profile]
        # rosbag2 creates its output directory itself and refuses an existing
        # directory.  Remove only the empty directory just created by
        # prepare_run; never touch a non-empty historical run.
        mcap_dir = run_dir / "mcap"
        if mcap_dir.exists():
            if any(mcap_dir.iterdir()):
                raise RuntimeError(f"MCAP output directory is not empty: {mcap_dir}")
            mcap_dir.rmdir()
        actions.append(ExecuteProcess(cmd=[
            "ros2", "bag", "record", "--storage", "mcap", "--output", str(mcap_dir),
            "--max-bag-size", str(split["max_bag_size"]), "--max-bag-duration", str(split["max_bag_duration"]),
            "--max-cache-size", str(split["max_cache_size"]),
            "--storage-config-file", str(bringup_share / "config" / "mcap_writer_options.yaml"),
            "--qos-profile-overrides-path", str(bringup_share / "config" / "qos_overrides.yaml"), *topics,
        ], output="screen"))
    if _bool(LaunchConfiguration("start_mid360").perform(context)) or _bool(LaunchConfiguration("start_sync").perform(context)):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(bringup_share / "launch" / "hardware_pipeline.launch.py")),
            launch_arguments={
                "start_mindvision": "false", "start_mid360": LaunchConfiguration("start_mid360"),
                "start_sync": LaunchConfiguration("start_sync"), "record": "false",
                "mid360_config_path": LaunchConfiguration("mid360_config_path"),
                "camera_image_topic": camera_topic, "output_directory": str(run_dir / "mcap"),
            }.items()))
    return actions


def generate_launch_description():
    vision_share = Path(get_package_share_directory("tank_vision_ros"))
    return LaunchDescription([
        DeclareLaunchArgument("data_root", default_value="/media/mcap_data"),
        DeclareLaunchArgument("run_id", default_value=""),
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("camera_serial", default_value=""),
        DeclareLaunchArgument("camera_topic", default_value="/camera/front/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/front/camera_info"),
        DeclareLaunchArgument("resolution_index", default_value="-1"),
        DeclareLaunchArgument("exposure_mode", default_value="auto"),
        DeclareLaunchArgument("exposure_us", default_value="0.0"),
        DeclareLaunchArgument("ae_min_exposure_us", default_value="0.0"),
        DeclareLaunchArgument("ae_max_exposure_us", default_value="0.0"),
        DeclareLaunchArgument("ae_min_analog_gain_raw", default_value="0"),
        DeclareLaunchArgument("ae_max_analog_gain_raw", default_value="0"),
        DeclareLaunchArgument("warmup_frames", default_value="60"),
        DeclareLaunchArgument("camera_publish_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("timeout_ms", default_value="1000"),
        DeclareLaunchArgument(
            "camera_qos_reliability", default_value="best_effort",
            description=(
                "MindVision Image/CameraInfo publisher reliability. FIELD-W5 uses reliable "
                "only when the tested hobot_codec hardware encoder is enabled.")),
        DeclareLaunchArgument("tracker_config", default_value=str(vision_share / "config/s100p_guided_anchor_seam_4096x3072.yaml")),
        DeclareLaunchArgument("tracker_reference", default_value=str(vision_share / "config/seam_geometry_reference_s100p_live_v1.json")),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
        DeclareLaunchArgument(
            "enable_tracker", default_value="true",
            description="Start stable_seam_node. Vehicle sensor-only collection sets this false."),
        DeclareLaunchArgument("auto_save_failures", default_value="true"),
        DeclareLaunchArgument("auto_save_full_frame", default_value="false"),
        DeclareLaunchArgument("auto_save_context_jpeg", default_value="true"),
        DeclareLaunchArgument("save_lossless_roi", default_value="true"),
        DeclareLaunchArgument("auto_save_policy", default_value="event"),
        DeclareLaunchArgument("per_frame_metrics_log", default_value="true"),
        DeclareLaunchArgument("failure_snapshot_cooldown_sec", default_value="2.0"),
        DeclareLaunchArgument("max_failure_snapshots", default_value="100"),
        DeclareLaunchArgument("roi_validation_recording_enabled", default_value="false"),
        DeclareLaunchArgument("roi_validation_output_root", default_value=""),
        DeclareLaunchArgument("roi_validation_queue_size", default_value="8"),
        DeclareLaunchArgument("roi_validation_max_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("geometry_provenance_sampling_enabled", default_value="false"),
        DeclareLaunchArgument("geometry_provenance_queue_size", default_value="8"),
        DeclareLaunchArgument("geometry_provenance_sample_period_sec", default_value="5.0"),
        DeclareLaunchArgument("geometry_provenance_max_events", default_value="30"),
        DeclareLaunchArgument("record_mcap", default_value="true"),
        DeclareLaunchArgument("record_raw_image", default_value="false"),
        DeclareLaunchArgument("start_mid360", default_value="false"),
        DeclareLaunchArgument("mid360_config_path", default_value=""),
        DeclareLaunchArgument("start_sync", default_value="false"),
        OpaqueFunction(function=_start),
    ])
