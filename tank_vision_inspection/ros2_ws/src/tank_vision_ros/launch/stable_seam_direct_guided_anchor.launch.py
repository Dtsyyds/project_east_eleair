import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("tank_vision_ros")
    default_config = os.path.join(share, "config", "s100p_guided_anchor_seam_4096x3072.yaml")
    default_reference = os.path.join(share, "config", "seam_geometry_reference_s100p_live_v1.json")
    default_snapshots = "/media/mcap_data/vision/stable_seam_failure_snapshots"
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config,
                              description="Installed guided-anchor tracker YAML"),
        DeclareLaunchArgument("reference_file", default_value=default_reference,
                              description="Installed guided-anchor geometry reference JSON"),
        DeclareLaunchArgument("publish_debug_image", default_value="false",
                              description="Publish debug image only when a subscriber exists"),
        DeclareLaunchArgument("snapshot_output_root", default_value=default_snapshots,
                              description="Root directory for same-frame snapshots"),
        DeclareLaunchArgument("auto_save_failures", default_value="true",
                              description="Enable sparse automatic failure-event snapshots"),
        DeclareLaunchArgument("failure_snapshot_cooldown_sec", default_value="2.0",
                              description="Cooldown for repeated ordinary failure events"),
        DeclareLaunchArgument("max_failure_snapshots_per_session", default_value="100",
                              description="Maximum automatic failure snapshots in one session"),
        DeclareLaunchArgument("auto_save_full_frame", default_value="false",
                              description="Also save lossless full frame for automatic events"),
        DeclareLaunchArgument("auto_save_context_jpeg", default_value="true",
                              description="Save compressed full-frame event context"),
        DeclareLaunchArgument("save_lossless_roi", default_value="true",
                              description="Save lossless ROI for automatic events"),
        DeclareLaunchArgument("auto_save_policy", default_value="event",
                              description="Automatic policy: event, every_failure, or disabled"),
        DeclareLaunchArgument("live_session_output_root",
                              default_value="/media/mcap_data/vision/stable_seam_live_sessions",
                              description="Root for per-frame JSONL and failure events"),
        Node(
            package="tank_vision_ros", executable="stable_seam_node", name="stable_seam_node", output="screen",
            parameters=[{
                "input_mode": "mindvision_direct",
                "config_file": LaunchConfiguration("config_file"),
                "reference_file": LaunchConfiguration("reference_file"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                "snapshot_output_dir": LaunchConfiguration("snapshot_output_root"),
                "auto_save_failures": LaunchConfiguration("auto_save_failures"),
                "failure_snapshot_cooldown_sec": LaunchConfiguration("failure_snapshot_cooldown_sec"),
                "max_failure_snapshots_per_session": LaunchConfiguration("max_failure_snapshots_per_session"),
                "auto_save_full_frame": LaunchConfiguration("auto_save_full_frame"),
                "auto_save_context_jpeg": LaunchConfiguration("auto_save_context_jpeg"),
                "save_lossless_roi": LaunchConfiguration("save_lossless_roi"),
                "auto_save_policy": LaunchConfiguration("auto_save_policy"),
                "live_session_output_root": LaunchConfiguration("live_session_output_root"),
            }],
        ),
    ])
