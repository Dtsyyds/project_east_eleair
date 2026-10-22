"""Record the configured raw and derived topics to split MCAP files."""

from pathlib import Path
import os
import shutil

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

STORAGE_MOUNT = Path(os.environ.get("FIELD_DATA_ROOT", "/media/mcap_data"))
DEFAULT_MCAP_ROOT = STORAGE_MOUNT / "mcap"


def _ensure_storage_mounted(path: Path) -> None:
    resolved = path.expanduser().resolve(strict=False)
    if STORAGE_MOUNT == resolved or STORAGE_MOUNT in resolved.parents:
        if not STORAGE_MOUNT.is_mount():
            raise RuntimeError(
                f"persistent storage is not mounted at {STORAGE_MOUNT}; "
                "refusing to write capture data to the system disk"
            )


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _record_process(context):
    share = Path(get_package_share_directory("inspection_bringup"))
    output_directory = Path(
        LaunchConfiguration("output_directory").perform(context)
    ).expanduser()
    _ensure_storage_mounted(output_directory)
    free_bytes = shutil.disk_usage(output_directory.parent if output_directory.parent.exists() else STORAGE_MOUNT).free
    minimum_free = int(LaunchConfiguration("minimum_free_bytes").perform(context))
    if free_bytes < minimum_free:
        raise RuntimeError(f"insufficient recording space: free={free_bytes} required={minimum_free}")
    topics_file = Path(LaunchConfiguration("topics_file").perform(context)).expanduser().resolve()
    settings = yaml.safe_load(topics_file.read_text())
    profile = "debug" if _as_bool(LaunchConfiguration("debug_mode").perform(context)) else "production"
    split = settings[profile]
    return [ExecuteProcess(cmd=[
        "ros2", "bag", "record", "--storage", "mcap",
        "--output", LaunchConfiguration("output_directory"),
        "--max-bag-size", str(split["max_bag_size"]),
        "--max-bag-duration", str(split["max_bag_duration"]),
        "--max-cache-size", str(split["max_cache_size"]),
        "--storage-config-file", str(share / "config" / "mcap_writer_options.yaml"),
        "--qos-profile-overrides-path", str(share / "config" / "qos_overrides.yaml"),
        *settings["topics"],
    ], output="screen")]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_directory", default_value=str(DEFAULT_MCAP_ROOT / "record_only")),
        DeclareLaunchArgument("topics_file", default_value=str(
            Path(get_package_share_directory("inspection_bringup")) / "config/record_topics.yaml")),
        DeclareLaunchArgument("minimum_free_bytes", default_value=str(5 * 1024**3)),
        DeclareLaunchArgument("debug_mode", default_value="false"),
        OpaqueFunction(function=_record_process),
    ])
