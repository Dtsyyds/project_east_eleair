#!/usr/bin/env python3
"""Validate closed FIELD-W5 MCAP, video, timestamps, snapshots, and processes."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re
import struct
import subprocess
import sys
from typing import Any

import yaml
from inspection_tools.video_index import aligned_rows, load_rows, VideoAlignmentError

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import (  # noqa: E402
    atomic_json, atomic_write_text, command, load_json, process_matches, sha256_file, utc_now,
)


DIRECT_HEADER_TYPES = {
    "sensor_msgs/msg/CameraInfo", "sensor_msgs/msg/Image", "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/PointCloud2", "diagnostic_msgs/msg/DiagnosticArray",
    "nav_msgs/msg/Odometry", "nav_msgs/msg/Path",
    "realsense2_camera_msgs/msg/Metadata",
    "eddy_driver/msg/EddyFrame", "eddy_driver/msg/EddyChannelMean",
    "eddy_driver/msg/EddyStatus", "livox_ros_driver2/msg/CustomMsg",
}


def _cdr_header_stamp_ns(serialized: bytes | bytearray | memoryview) -> int | None:
    """Read a leading std_msgs/Header stamp without deserializing the payload.

    ROS 2 CDR messages start with a four-byte encapsulation header.  Every type
    in DIRECT_HEADER_TYPES has std_msgs/Header as its first field, so sec/nsec
    immediately follow it.  This avoids rebuilding multi-megabyte cumulative
    nav_msgs/Path messages merely to validate timestamp monotonicity.
    """
    view = memoryview(serialized)
    if len(view) < 12:
        raise ValueError("serialized message is too short for a CDR Header")
    identifier = int.from_bytes(view[:2], "big")
    if identifier not in (0, 1):
        raise ValueError(f"unsupported CDR encapsulation identifier: {identifier}")
    endian = "<" if identifier == 1 else ">"
    sec, nanosec = struct.unpack_from(f"{endian}iI", view, 4)
    if nanosec >= 1_000_000_000:
        raise ValueError(f"invalid Header nanosecond value: {nanosec}")
    value = sec * 1_000_000_000 + nanosec
    return value


def _odometry_record(message: Any, bag_timestamp: int, sequence: int) -> dict[str, Any]:
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        "sequence": sequence,
        "bag_timestamp_ns": int(bag_timestamp),
        "header_stamp_ns": _header_stamp_ns(message),
        "frame_id": str(message.header.frame_id),
        "child_frame_id": str(message.child_frame_id),
        "position": {
            "x": float(pose.position.x), "y": float(pose.position.y),
            "z": float(pose.position.z),
        },
        "orientation_xyzw": {
            "x": float(pose.orientation.x), "y": float(pose.orientation.y),
            "z": float(pose.orientation.z), "w": float(pose.orientation.w),
        },
        "linear_velocity": {
            "x": float(twist.linear.x), "y": float(twist.linear.y),
            "z": float(twist.linear.z),
        },
        "angular_velocity": {
            "x": float(twist.angular.x), "y": float(twist.angular.y),
            "z": float(twist.angular.z),
        },
    }


def _header_stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value


def _update_monotonic(item: dict[str, Any], stamp: int | None) -> None:
    if stamp is None:
        item["missing_header_count"] += 1
        return
    item["total_header_count"] += 1
    if stamp == 0:
        item["zero_header_count"] += 1
        return
    if stamp < 0:
        item["negative_header_count"] += 1
        return
    item["header_stamp_count"] += 1
    if item["last_header_stamp_ns"] is not None:
        delta = stamp - item["last_header_stamp_ns"]
        if delta == 0: item["duplicate_header_count"] += 1
        if delta > 0:
            item["interval_count"] += 1
            item["interval_sum_ns"] += delta
            item["interval_min_ns"] = min(item["interval_min_ns"] or delta, delta)
            item["interval_max_ns"] = max(item["interval_max_ns"] or delta, delta)
    if item["last_header_stamp_ns"] is not None and stamp < item["last_header_stamp_ns"]:
        item["header_stamp_regressions"] += 1
    item["first_header_stamp_ns"] = item["first_header_stamp_ns"] or stamp
    item["last_header_stamp_ns"] = stamp


def _empty_time_item(type_name: str = "") -> dict[str, Any]:
    return {
        "type": type_name, "count": 0,
        "first_bag_timestamp_ns": None, "last_bag_timestamp_ns": None,
        "bag_timestamp_regressions": 0,
        "total_header_count": 0, "missing_header_count": 0,
        "zero_header_count": 0, "negative_header_count": 0, "duplicate_header_count": 0,
        "interval_count": 0, "interval_sum_ns": 0, "interval_min_ns": None, "interval_max_ns": None,
        "header_stamp_count": 0, "first_header_stamp_ns": None,
        "last_header_stamp_ns": None, "header_stamp_regressions": 0,
        "serialized_bytes_total": 0, "max_serialized_bytes": 0,
        "first_serialized_bytes": None, "last_serialized_bytes": None,
        "first_path_pose_count": None, "last_path_pose_count": None,
        "max_path_pose_count": None,
    }


DYNAMIC_HEADER_TYPES = {"sensor_msgs/msg/Image", "sensor_msgs/msg/Imu",
    "sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/CameraInfo", "livox_ros_driver2/msg/CustomMsg",
    "eddy_driver/msg/EddyFrame", "eddy_driver/msg/EddyChannelMean", "nav_msgs/msg/Odometry"}


def timestamp_contract_valid(topic: str, value: dict[str, Any]) -> bool:
    if value.get("bag_timestamp_regressions", 0): return False
    if topic == "/tf_static": return True
    if value.get("type") in DYNAMIC_HEADER_TYPES:
        return (int(value.get("count", 0)) > 0
            and int(value.get("header_stamp_count", 0)) == int(value.get("count", 0))
            and int(value.get("zero_header_count", 0)) == 0
            and int(value.get("negative_header_count", 0)) == 0
            and int(value.get("duplicate_header_count", 0)) == 0
            and int(value.get("header_stamp_regressions", 0)) == 0)
    return topic == "/diagnostics" or value.get("header_stamp_regressions", 0) == 0


def inspect_bag_messages(
    path: Path, compact_trajectory_output: Path | None = None,
) -> dict[str, Any]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        return {"valid": False, "error": f"ROS bag Python imports unavailable: {exc}"}
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as exc:
        return {"valid": False, "error": f"open failed: {type(exc).__name__}: {exc}"}
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    stats = {name: _empty_time_item(kind) for name, kind in types.items()}
    message_types: dict[str, Any] = {}
    diagnostic_groups: dict[str, dict[str, Any]] = defaultdict(_empty_time_item)
    diagnostic_values: dict[str, dict[str, str]] = {}
    latest_eddy_status: dict[str, Any] = {}
    deserialize_errors: dict[str, int] = defaultdict(int)
    trajectory_records: list[dict[str, Any]] = []
    first_path_serialized: bytes | None = None
    last_path_serialized: bytes | None = None
    while reader.has_next():
        topic, serialized, bag_timestamp = reader.read_next()
        item = stats.setdefault(topic, _empty_time_item(types.get(topic, "")))
        item["count"] += 1
        item["serialized_bytes_total"] += len(serialized)
        item["max_serialized_bytes"] = max(item["max_serialized_bytes"], len(serialized))
        item["first_serialized_bytes"] = item["first_serialized_bytes"] or len(serialized)
        item["last_serialized_bytes"] = len(serialized)
        if item["last_bag_timestamp_ns"] is not None and bag_timestamp < item["last_bag_timestamp_ns"]:
            item["bag_timestamp_regressions"] += 1
        item["first_bag_timestamp_ns"] = item["first_bag_timestamp_ns"] or bag_timestamp
        item["last_bag_timestamp_ns"] = bag_timestamp
        try:
            type_name = types[topic]
            message = None
            if topic in {"/diagnostics", "/eddy/status", "/aft_mapped_to_init"} \
                    or type_name not in DIRECT_HEADER_TYPES:
                message_type = message_types.setdefault(topic, get_message(type_name))
                message = deserialize_message(serialized, message_type)
                stamp = _header_stamp_ns(message)
            else:
                stamp = _cdr_header_stamp_ns(serialized)
            _update_monotonic(item, stamp)
            if topic == "/path":
                if first_path_serialized is None:
                    first_path_serialized = bytes(serialized)
                last_path_serialized = bytes(serialized)
            if topic == "/aft_mapped_to_init" and message is not None:
                trajectory_records.append(
                    _odometry_record(message, bag_timestamp, len(trajectory_records))
                )
            if topic == "/diagnostics":
                for status in getattr(message, "status", []) or []:
                    keys = [
                        f"status.name={getattr(status, 'name', '') or '<empty>'}",
                        f"hardware_id={getattr(status, 'hardware_id', '') or '<empty>'}",
                    ]
                    for key in keys:
                        group = diagnostic_groups[key]
                        group["count"] += 1
                        _update_monotonic(group, stamp)
                        diagnostic_values[key] = {
                            str(getattr(value, "key", "")): str(getattr(value, "value", ""))
                            for value in (getattr(status, "values", []) or [])
                        }
            if topic == "/eddy/status":
                latest_eddy_status = {
                    "connected": bool(getattr(message, "connected", False)),
                    "received_frames": int(getattr(message, "received_frames", 0)),
                    "dropped_frames": int(getattr(message, "dropped_frames", 0)),
                    "queue_depth": int(getattr(message, "queue_depth", 0)),
                    "last_status_is_error": bool(getattr(message, "last_status_is_error", False)),
                }
        except Exception:
            deserialize_errors[topic] += 1
    if first_path_serialized is not None and last_path_serialized is not None:
        try:
            message_type = message_types.setdefault(
                "/path", get_message(types["/path"]),
            )
            first_path = deserialize_message(first_path_serialized, message_type)
            last_path = deserialize_message(last_path_serialized, message_type)
            path_item = stats["/path"]
            path_item["first_path_pose_count"] = len(first_path.poses)
            path_item["last_path_pose_count"] = len(last_path.poses)
            path_item["max_path_pose_count"] = max(
                path_item["first_path_pose_count"], path_item["last_path_pose_count"],
            )
        except Exception:
            deserialize_errors["/path"] += 1
    for item in stats.values():
        bag_span = (
            (item["last_bag_timestamp_ns"] - item["first_bag_timestamp_ns"]) / 1e9
            if item["first_bag_timestamp_ns"] is not None
            and item["last_bag_timestamp_ns"] is not None else None
        )
        header_span = (
            (item["last_header_stamp_ns"] - item["first_header_stamp_ns"]) / 1e9
            if item["first_header_stamp_ns"] is not None
            and item["last_header_stamp_ns"] is not None else None
        )
        item["timestamp_source"] = "header" if item["header_stamp_count"] else "bag_host_time"
        item["header_contract"] = "required_dynamic" if item["type"] in DYNAMIC_HEADER_TYPES else "headerless_or_static_exception"
        item["bag_span_sec"] = bag_span
        item["header_span_sec"] = header_span
        item["span_sec"] = header_span if header_span and header_span > 0 else bag_span
        item["rate_time_basis"] = (
            "header_stamp" if header_span and header_span > 0 else "bag_timestamp"
        )
        item["rate_hz"] = (
            (item["count"] - 1) / item["span_sec"]
            if item["span_sec"] and item["span_sec"] > 0 and item["count"] > 1 else None
        )
        item["average_serialized_bytes"] = (
            item["serialized_bytes_total"] / item["count"] if item["count"] else None
        )
    global_diag_regressions = stats.get("/diagnostics", {}).get("header_stamp_regressions", 0)
    same_source_regressions = sum(
        value["header_stamp_regressions"] for value in diagnostic_groups.values()
    )
    trajectory_summary = {
        "source_topic": "/aft_mapped_to_init",
        "format": "jsonl_pose_v1",
        "count": len(trajectory_records),
        "first_header_stamp_ns": (
            trajectory_records[0]["header_stamp_ns"] if trajectory_records else None
        ),
        "last_header_stamp_ns": (
            trajectory_records[-1]["header_stamp_ns"] if trajectory_records else None
        ),
    }
    if compact_trajectory_output is not None and trajectory_records:
        atomic_write_text(
            compact_trajectory_output,
            "".join(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n"
                    for value in trajectory_records),
        )
        trajectory_summary["path"] = str(compact_trajectory_output)
    return {
        "valid": all(value["count"] > 0 for value in stats.values()) and not deserialize_errors,
        "topics": stats,
        "deserialize_errors": dict(deserialize_errors),
        "compact_trajectory": trajectory_summary,
        "timestamp_scan_mode": "fast_leading_cdr_header_with_semantic_deserialization",
        "diagnostics": {
            "global_header_regressions": global_diag_regressions,
            "same_source_header_regressions": same_source_regressions,
            "by_status_name_or_hardware_id": dict(diagnostic_groups),
            "latest_values_by_status_name_or_hardware_id": diagnostic_values,
            "publisher_gid": "not stored by rosbag2 MCAP; unavailable post-recording",
            "classification": (
                "same_source_timestamp_regression" if same_source_regressions
                else "aggregate_multi_publisher_interleaving" if global_diag_regressions
                else "monotonic"
            ),
        },
        "latest_eddy_status": latest_eddy_status,
    }


def validate_bag(
    path: Path, required_topics: list[str],
    compact_trajectory_output: Path | None = None,
) -> dict[str, Any]:
    metadata_path = path / "metadata.yaml"
    info = command(["ros2", "bag", "info", str(path)], timeout=120.0)
    try:
        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError) as exc:
        metadata = {"error": str(exc)}
    bag_info = metadata.get("rosbag2_bagfile_information", {})
    relative_files = bag_info.get("relative_file_paths", [])
    files = []
    for relative in relative_files:
        item = path / relative
        files.append({
            "path": str(item), "exists": item.is_file(),
            "size": item.stat().st_size if item.is_file() else 0,
        })
    messages = inspect_bag_messages(path, compact_trajectory_output) if info["returncode"] == 0 else {
        "valid": False, "error": "ros2 bag info failed",
    }
    topics = messages.get("topics", {})
    missing = [name for name in required_topics if name not in topics]
    empty = [name for name in required_topics if int(topics.get(name, {}).get("count", 0)) <= 0]
    valid = (
        info["returncode"] == 0 and metadata_path.is_file()
        and bool(files) and all(item["exists"] and item["size"] > 0 for item in files)
        and not missing and not empty and messages.get("valid", False)
    )
    return {
        "path": str(path), "valid": valid, "bag_info": info,
        "metadata_present": metadata_path.is_file(), "files": files,
        "missing_required_topics": missing, "empty_required_topics": empty,
        "message_validation": messages,
    }


def _decode_frame(video: Path, index: int, seek_sec: float) -> dict[str, Any]:
    result = command([
        "ffmpeg", "-v", "error", "-ss", f"{seek_sec:.3f}", "-i", str(video),
        "-frames:v", "1", "-f", "null", "-",
    ], timeout=120.0)
    return {
        "frame_index": index, "seek_sec": seek_sec,
        "decodable": result["returncode"] == 0, "command": result,
    }


def validate_video(run_dir: Path) -> dict[str, Any]:
    segments = []
    for video in sorted((run_dir / "video/front").glob("front_*.mkv")):
        sidecar = video.with_suffix(".frames.jsonl")
        try:
            source = load_rows(sidecar)
            rows = aligned_rows(video, sidecar)
            if segments and segments[-1].get("last_frame") and rows and int(rows[0]["ros_header_stamp_ns"]) <= int(segments[-1]["last_frame"]["ros_header_stamp_ns"]):
                raise VideoAlignmentError("camera segment timestamps overlap or regress")
            segments.append({"video": str(video.relative_to(run_dir)),
                "sidecar": str(sidecar.relative_to(run_dir)), "valid": bool(rows),
                "strict_alignment": True, "video_frame_count": len(rows),
                "sidecar_frame_count": len(source), "unmapped_source_count": len(source)-len(rows),
                "first_frame": rows[0], "last_frame": rows[-1],
                "frame_count_explanation": "decoded-frame mapping verified against source hashes"})
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            segments.append({"video": str(video.relative_to(run_dir)), "valid": False,
                "strict_alignment": False, "video_frame_count": 0,
                "error": str(exc), "status": "not_strictly_alignable"})
    runtime = load_json(run_dir / "reports/video_runtime_status.json", {}) or {}
    received = int(runtime.get("received_frames", 0) or 0)
    encoded = sum(item["video_frame_count"] for item in segments)
    drops = int(runtime.get("queue_drop_count", 0) or 0)
    pre_idr = int(runtime.get("pre_idr_discard_count", 0) or 0)
    return {"schema_version": "field_w5_video_validation_v2",
        "valid": bool(segments) and all(item["valid"] for item in segments)
                 and received == encoded + drops + pre_idr
                 and int(runtime.get("write_error_count", 1)) == 0,
        "segments": segments, "input_frames": received, "encoded_frames": encoded,
        "encoded_to_input_ratio": encoded/received if received else None,
        "queue_drop_count": drops, "pre_idr_discard_count": pre_idr,
        "write_error_count": runtime.get("write_error_count"),
        "frame_conservation": {"received": received, "decoded": encoded, "queue_drop": drops,
                               "pre_idr_discard": pre_idr, "unaccounted": received-encoded-drops-pre_idr},
        "lossless_raw_equivalence": False,
        "note": "packet counts and 95% ratios never establish frame timestamp alignment"}


def validate_snapshots(run_dir: Path) -> dict[str, Any]:
    results = []
    filename_by_key = {
        "roi_raw_sha256": ("roi_raw.png", "roi_raw.bmp"),
        "context_sha256": ("context.jpg",),
        "overlay_sha256": ("overlay.png",),
        "seam_geometry_sha256": ("seam_geometry.json",),
        "failure_reason_sha256": ("failure_reason.json",),
    }
    metadata_paths = sorted(run_dir.rglob("failure_snapshots/*/metadata.json"))
    for metadata_path in metadata_paths:
        document = load_json(metadata_path, {}) or {}
        checks = []
        for key, filenames in filename_by_key.items():
            expected = document.get(key)
            if expected is None:
                continue
            candidate = next(
                (metadata_path.parent / name for name in filenames
                 if (metadata_path.parent / name).is_file()),
                metadata_path.parent / filenames[0],
            )
            checks.append({
                "key": key, "file": candidate.name, "exists": candidate.is_file(),
                "matches": candidate.is_file() and sha256_file(candidate) == expected,
            })
        results.append({
            "event": str(metadata_path.parent.relative_to(run_dir)),
            "complete": document.get("complete") is True,
            "hash_checks": checks,
            "valid": document.get("complete") is True and bool(checks)
            and all(item["matches"] for item in checks),
        })
    incomplete_metadata = [str(path.relative_to(run_dir)) for path in
                           run_dir.rglob("failure_snapshots/*/metadata.json.partial")]
    sessions = []
    for summary_path in sorted(run_dir.glob("live_session/*/live_summary.json")):
        summary = load_json(summary_path, {}) or {}
        counters = summary.get("counters", {})
        sessions.append({
            "path": str(summary_path.parent.relative_to(run_dir)),
            "complete": summary.get("status") == "complete",
            "auto_snapshot_requested": counters.get("auto_snapshot_requested"),
            "auto_snapshot_saved": counters.get("auto_snapshot_saved"),
            "auto_snapshot_rejected": counters.get("auto_snapshot_rejected"),
            "snapshot_writer_failed": counters.get("snapshot_writer_failed"),
            "snapshot_queue_high_watermark": counters.get("snapshot_queue_high_watermark"),
            "metrics_log_dropped": counters.get("metrics_log_dropped"),
            "event_log_dropped": counters.get("event_log_dropped"),
            "writer_errors": counters.get("writer_errors", []),
            "provenance_second_pass_count": counters.get("provenance_second_pass_count"),
        })
    sessions_valid = all(
        item["complete"] and int(item.get("snapshot_writer_failed") or 0) == 0
        and not item.get("writer_errors") for item in sessions
    )
    return {
        "schema_version": "field_w5_snapshot_integrity_v1",
        "valid": all(item["valid"] for item in results) and not incomplete_metadata
        and sessions_valid,
        "event_count": len(results), "events": results,
        "sessions": sessions,
        "incomplete_metadata": incomplete_metadata,
        "no_events_is_valid": True,
    }


def orphan_check(run_dir: Path) -> dict[str, Any]:
    registry = load_json(run_dir / "logs" / "process_registry.json", {}) or {}
    guard_finalizing = (run_dir / "logs" / "disk_guard_finalize_in_progress").is_file()
    alive = [
        item for item in registry.get("processes", [])
        if process_matches(item)
        and not (guard_finalizing and item.get("name") == "disk_guard")
    ]
    return {
        "schema_version": "field_w5_orphan_check_v1",
        "valid": not alive, "alive_registered_processes": alive,
        "disk_guard_excluded_during_its_own_finalize": guard_finalizing,
    }


def _extract_rates(mcap: dict[str, Any]) -> dict[str, Any]:
    rates: dict[str, Any] = {}
    for bag in mcap.get("bags", {}).values():
        for topic, value in bag.get("message_validation", {}).get("topics", {}).items():
            rates[topic] = value.get("rate_hz")
    return rates


def _extract_topic_stats(mcap: dict[str, Any]) -> dict[str, dict[str, Any]]:
    """Return the most complete per-topic record across split bags."""
    stats: dict[str, dict[str, Any]] = {}
    for bag in mcap.get("bags", {}).values():
        for topic, value in bag.get("message_validation", {}).get("topics", {}).items():
            if int(value.get("count", 0) or 0) > int(stats.get(topic, {}).get("count", 0) or 0):
                stats[topic] = value
    return stats


def _effective_topic_metrics(
    topic_stats: dict[str, dict[str, Any]], topics: list[str],
    reference_first_ns: int | None, reference_last_ns: int | None,
) -> dict[str, dict[str, float | int | None]]:
    values: dict[str, dict[str, float | int | None]] = {}
    reference_span_sec = (
        (reference_last_ns - reference_first_ns) / 1e9
        if reference_first_ns is not None and reference_last_ns is not None
        and reference_last_ns > reference_first_ns else 0.0
    )
    for topic in topics:
        item = topic_stats.get(topic, {})
        first = item.get("first_bag_timestamp_ns")
        last = item.get("last_bag_timestamp_ns")
        count = int(item.get("count", 0) or 0)
        overlap_sec = (
            max(0.0, (min(int(last), int(reference_last_ns))
                      - max(int(first), int(reference_first_ns))) / 1e9)
            if first is not None and last is not None and reference_span_sec > 0 else 0.0
        )
        values[topic] = {
            "count": count,
            "effective_rate_hz": ((count - 1) / reference_span_sec)
            if count > 1 and reference_span_sec > 0 else 0.0,
            "run_coverage_ratio": overlap_sec / reference_span_sec
            if reference_span_sec > 0 else 0.0,
            "first_bag_timestamp_ns": first,
            "last_bag_timestamp_ns": last,
        }
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--report-dir", type=Path)
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    report_dir = (args.report_dir or (run_dir / "reports")).resolve()
    if run_dir.is_relative_to(Path("/media/mcap_data/runs")) and report_dir.is_relative_to(run_dir):
        raise ValueError("historical runs are read-only; supply --report-dir in analysis/<run_id>/<analysis_id>/metrics")
    report_dir.mkdir(parents=True, exist_ok=True)
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    is_mapping_eddy = manifest.get("collection_profile") == "mapping_eddy"
    enabled = manifest.get("enabled_writers", {})
    inventory = load_json(run_dir / "topic_inventory.json", {}) or {}
    groups = inventory.get("record_groups", {})
    bags: dict[str, Any] = {}
    for name in ("core", "mapping", "mid360", "d405", "eddy", "raw_burst"):
        required = list(groups.get(name, {}))
        if name == "raw_burst":
            required = ["/camera/front/image_raw", "/camera/front/camera_info"]
        enabled_name = "front_raw_burst" if name == "raw_burst" else name
        if enabled.get(enabled_name) and (name != "raw_burst" or enabled.get("raw_burst_mode") == "full"):
            bags[name] = validate_bag(
                run_dir / "mcap" / name, required,
                report_dir / "compact_trajectory.jsonl" if name == "mapping" else None,
            )
    mcap = {
        "schema_version": ("field_w6_mcap_validation_v1" if is_mapping_eddy
                           else "field_w5_mcap_validation_v1"),
        "valid": bool(bags) and all(value["valid"] for value in bags.values()),
        "bags": bags,
    }
    atomic_json(report_dir / "mcap_validation.json", mcap)
    trajectory = bags.get("mapping", {}).get("message_validation", {}).get(
        "compact_trajectory", {}
    )
    trajectory_topic = bags.get("mapping", {}).get("message_validation", {}).get(
        "topics", {}
    ).get("/aft_mapped_to_init", {})
    trajectory_report = {
        "schema_version": "field_w6_compact_trajectory_v1",
        "created_at": utc_now(),
        "source_topic": "/aft_mapped_to_init",
        "output": "reports/compact_trajectory.jsonl",
        "format": "jsonl_pose_v1",
        "count": int(trajectory.get("count", 0) or 0),
        "source_message_count": int(trajectory_topic.get("count", 0) or 0),
        "first_header_stamp_ns": trajectory.get("first_header_stamp_ns"),
        "last_header_stamp_ns": trajectory.get("last_header_stamp_ns"),
    }
    trajectory_report["valid"] = (
        not enabled.get("mapping") or (
            trajectory_report["count"] > 0
            and trajectory_report["count"] == trajectory_report["source_message_count"]
            and (report_dir / "compact_trajectory.jsonl").is_file()
        )
    )
    atomic_json(report_dir / "compact_trajectory_summary.json", trajectory_report)
    timestamp = {
        "schema_version": ("field_w6_timestamp_validation_v1" if is_mapping_eddy
                           else "field_w5_timestamp_validation_v1"),
        "valid": mcap["valid"] and all(
            all(
                timestamp_contract_valid(topic, value)
                for topic, value in bag.get("message_validation", {}).get("topics", {}).items()
            )
            and bag.get("message_validation", {}).get("diagnostics", {}).get(
                "same_source_header_regressions", 0
            ) == 0
            for bag in bags.values()
        ),
        "bags": {
            name: value.get("message_validation", {}) for name, value in bags.items()
        },
        "claim": "transport timestamp consistency and host/ROS time alignment only",
        "hardware_synchronization_claimed": False,
        "spatial_synchronization_claimed": False,
    }
    atomic_json(report_dir / "timestamp_validation.json", timestamp)
    mapping_bag = bags.get("mapping", {})
    path_stats = mapping_bag.get("message_validation", {}).get("topics", {}).get("/path", {})
    path_rate = float(path_stats.get("rate_hz") or 0.0)
    first_poses = int(path_stats.get("first_path_pose_count") or 0)
    last_poses = int(path_stats.get("last_path_pose_count") or 0)
    first_bytes = int(path_stats.get("first_serialized_bytes") or 0)
    last_bytes = int(path_stats.get("last_serialized_bytes") or 0)
    bytes_per_pose = ((last_bytes - first_bytes) / (last_poses - first_poses)
                      if last_poses > first_poses and last_bytes >= first_bytes else None)
    pose_growth_hz = ((last_poses - first_poses) / float(path_stats.get("span_sec") or 0)
                      if last_poses > first_poses and float(path_stats.get("span_sec") or 0) > 0
                      else None)
    projections = {}
    if bytes_per_pose and pose_growth_hz and path_rate > 0:
        for minutes in (10, 30, 60):
            horizon = minutes * 60.0
            projected = path_rate * (
                last_bytes * horizon
                + 0.5 * bytes_per_pose * pose_growth_hz * horizon * horizon
            )
            projections[f"{minutes}_minutes"] = {
                "estimated_messages": path_rate * horizon,
                "estimated_last_message_bytes": last_bytes + bytes_per_pose * pose_growth_hz * horizon,
                "estimated_recording_bytes": projected,
                "estimated_recording_GB_decimal": projected / 1e9,
            }
    mapping_profile = manifest.get("mapping_capture_profile", {})
    compact_policy = mapping_profile.get("storage_policy") == "compact_odometry_only"
    path_growth = {
        "schema_version": "field_w6_path_growth_analysis_v1",
        "created_at": utc_now(), "observed": bool(path_stats),
        "first_pose_count": path_stats.get("first_path_pose_count"),
        "last_pose_count": path_stats.get("last_path_pose_count"),
        "max_pose_count": path_stats.get("max_path_pose_count"),
        "average_message_bytes": path_stats.get("average_serialized_bytes"),
        "max_message_bytes": path_stats.get("max_serialized_bytes"),
        "receive_hz": path_rate, "pose_growth_hz": pose_growth_hz,
        "estimated_bytes_per_pose": bytes_per_pose,
        "growing_cumulative_path": last_poses > int(path_stats.get("first_path_pose_count") or last_poses),
        "projected_raw_path_storage": projections,
        "conclusion": "compact_odometry_only" if compact_policy else (
            "B" if projections else "A_or_insufficient_duration"
        ),
        "policy": ("/path is online-health-only and is not recorded; only "
                   "/aft_mapped_to_init is stored") if compact_policy else
                  "historical run retained raw /path at publisher frequency",
        "recorded_mapping_topics": list(mapping_profile.get("recorded_topics", {})),
        "offline_alternative": "reports/compact_trajectory.jsonl reconstructed from /aft_mapped_to_init",
    }
    atomic_json(report_dir / "path_growth_analysis.json", path_growth)
    if enabled.get("front_video"):
        video = validate_video(run_dir)
    else:
        video = {"schema_version": "field_w5_video_validation_v2", "valid": None, "enabled": False, "status": "disabled"}
    if is_mapping_eddy:
        video["schema_version"] = "field_w6_video_validation_v1"
    atomic_json(report_dir / "video_validation.json", video)
    snapshots = validate_snapshots(run_dir)
    snapshots["enabled"] = bool(enabled.get("failure_snapshots"))
    snapshots["status"] = ("passed" if snapshots["valid"] else "failed") if snapshots["event_count"] else (
        "not_executed" if snapshots["enabled"] else "disabled")
    atomic_json(report_dir / "snapshot_integrity.json", snapshots)
    orphans = orphan_check(run_dir)
    atomic_json(report_dir / "orphan_process_check.json", orphans)
    status = load_json(run_dir / "collection_status.json", {}) or {}
    rates = _extract_rates(mcap)
    topic_stats = _extract_topic_stats(mcap)
    reference = topic_stats.get("/field/collection_status", {})
    reference_first_ns = reference.get("first_bag_timestamp_ns")
    reference_last_ns = reference.get("last_bag_timestamp_ns")
    camera_log = (run_dir / "logs" / "visual_launch.log").read_text(errors="replace") \
        if (run_dir / "logs" / "visual_launch.log").is_file() else ""
    timeout_values = [int(value) for value in re.findall(r"timeout_count=(\d+)", camera_log)]
    error_values = [int(value) for value in re.findall(r"capture_error_count=(\d+)", camera_log)]
    for bag in bags.values():
        sources = bag.get("message_validation", {}).get("diagnostics", {}).get(
            "latest_values_by_status_name_or_hardware_id", {}
        )
        for source, values in sources.items():
            if "mindvision" not in source.lower() and source != "hardware_id=046081820650":
                continue
            try:
                timeout_values.append(int(values["timeout_count"]))
            except (KeyError, TypeError, ValueError):
                pass
            try:
                error_values.append(int(values["capture_error_count"]))
            except (KeyError, TypeError, ValueError):
                pass
    camera_timeout = timeout_values[-1] if timeout_values else None
    camera_errors = error_values[-1] if error_values else None
    lidar_topics = [topic for topic in topic_stats if topic.startswith("/livox/") and "lidar" in topic]
    imu_topics = [topic for topic in topic_stats if topic.startswith("/livox/") and "imu" in topic]
    d405_image_topics = [
        topic for topic in topic_stats if topic.startswith("/d405/") and "image" in topic
    ]
    eddy_topics = [
        topic for topic, value in topic_stats.items()
        if value.get("type") in {
            "eddy_driver/msg/EddyFrame", "eddy_driver/msg/EddyChannelMean",
        }
    ]
    recorded_mapping_topics = list(mapping_profile.get("recorded_topics") or (
        topic for topic in ("/cloud_registered", "/path", "/aft_mapped_to_init")
        if topic in groups.get("mapping", {})
    ))
    live_topics = sorted(set(lidar_topics + imu_topics + d405_image_topics + eddy_topics + [
        "/camera/front/camera_info", *recorded_mapping_topics,
    ]))
    effective = _effective_topic_metrics(
        topic_stats, live_topics, reference_first_ns, reference_last_ns,
    )
    reference_span_sec = (
        (int(reference_last_ns) - int(reference_first_ns)) / 1e9
        if reference_first_ns is not None and reference_last_ns is not None
        and int(reference_last_ns) > int(reference_first_ns) else 0.0
    )
    video_frames = [
        frame for segment in video.get("segments", [])
        for frame in (segment.get("first_frame"), segment.get("last_frame")) if frame
    ]
    video_first_ns = min(
        (int(frame["ros_header_stamp_ns"]) for frame in video_frames), default=None,
    )
    video_last_ns = max(
        (int(frame["ros_header_stamp_ns"]) for frame in video_frames), default=None,
    )
    video_overlap_sec = (
        max(0.0, (min(video_last_ns, int(reference_last_ns))
                  - max(video_first_ns, int(reference_first_ns))) / 1e9)
        if video_first_ns is not None and video_last_ns is not None
        and reference_span_sec > 0 else 0.0
    )
    video_effective = {
        "count": int(video.get("encoded_frames", 0) or 0),
        "effective_rate_hz": int(video.get("encoded_frames", 0) or 0) / reference_span_sec
        if reference_span_sec > 0 else 0.0,
        "run_coverage_ratio": video_overlap_sec / reference_span_sec
        if reference_span_sec > 0 else 0.0,
        "first_ros_header_stamp_ns": video_first_ns,
        "last_ros_header_stamp_ns": video_last_ns,
    }
    lidar_metrics = [effective[topic] for topic in lidar_topics]
    imu_metrics = [effective[topic] for topic in imu_topics]
    d405_image_metrics = {topic: effective[topic] for topic in d405_image_topics}
    d405_image_rates = {
        topic: float(value["effective_rate_hz"] or 0) for topic, value in d405_image_metrics.items()
    }
    eddy_metrics = {topic: effective[topic] for topic in eddy_topics}
    d405_profile = manifest.get("d405_capture_profile", {})
    d405_expected_fps = float(d405_profile.get("expected_fps", 30.0) or 30.0)
    d405_minimum_fps = d405_expected_fps * 0.95
    tracker_rate = rates.get("/vision/front/seam_state")
    tracker_enabled = bool(enabled.get("tracker", True))
    requires_eddy_status = manifest.get("collection_profile") == "mapping_eddy" and enabled.get("eddy")
    latest_eddy = {}
    if "eddy" in bags:
        latest_eddy = bags["eddy"].get("message_validation", {}).get("latest_eddy_status", {})
    recorder_log_errors = []
    for log_path in (run_dir / "logs").glob("*_recorder.log"):
        text = log_path.read_text(encoding="utf-8", errors="replace")
        if re.search(r"failed to write|write error|storage error", text, re.IGNORECASE):
            recorder_log_errors.append(str(log_path.relative_to(run_dir)))
    monitored_mapping = status.get("topics", {})
    online_mapping_topics = list(mapping_profile.get("online_required_topics", {}))
    mapping_sources_observed = not enabled.get("mapping") or all(
        int(monitored_mapping.get(topic, {}).get("count", 0) or 0) > 1
        and float(monitored_mapping.get(topic, {}).get("receive_hz", 0) or 0) > 0
        for topic in online_mapping_topics
    )
    observed_eddy = status.get("eddy_status", {}) or status.get("eddy", {})
    if not isinstance(observed_eddy, dict): observed_eddy = {}
    # Monitor's persisted topic record contains the real latest driver status.
    observed_eddy = status.get("topics", {}).get("/eddy/status", {}).get("device_status", observed_eddy)
    eddy_evidence = [x for x in [latest_eddy, observed_eddy] if isinstance(x, dict) and x]
    eddy_error_seen = any(x.get("last_status_is_error") is True for x in eddy_evidence)
    checks = {
        "camera_timeout_zero": camera_timeout == 0,
        "camera_capture_error_zero": camera_errors == 0,
        "video_at_least_95_percent": not enabled.get("front_video")
        or (
            video.get("valid") is True
            and float(video.get("encoded_to_input_ratio") or 0) >= 0.95
            and float(video_effective["effective_rate_hz"] or 0) >= 9.5
            and float(video_effective["run_coverage_ratio"] or 0) >= 0.95
        ),
        "front_camera_near_10_hz_and_covers_run": not enabled.get("core") or (
            float(effective.get("/camera/front/camera_info", {}).get("effective_rate_hz", 0) or 0) >= 9.5
            and float(effective.get("/camera/front/camera_info", {}).get("run_coverage_ratio", 0) or 0) >= 0.95
        ),
        "mid360_lidar_near_10_hz": not enabled.get("mid360") or bool(lidar_metrics) and max(
            float(value["effective_rate_hz"] or 0) for value in lidar_metrics
        ) >= 8.0 and max(float(value["run_coverage_ratio"] or 0) for value in lidar_metrics) >= 0.95,
        "mid360_imu_near_200_hz": not enabled.get("mid360") or bool(imu_metrics) and max(
            float(value["effective_rate_hz"] or 0) for value in imu_metrics
        ) >= 180.0 and max(float(value["run_coverage_ratio"] or 0) for value in imu_metrics) >= 0.95,
        "d405_streams_at_least_95_percent": not enabled.get("d405")
        or bool(d405_image_metrics) and all(
            float(value["effective_rate_hz"] or 0) >= d405_minimum_fps
            and float(value["run_coverage_ratio"] or 0) >= 0.95
            for value in d405_image_metrics.values()
        ),
        "eddy_raw_and_channel_mean_cover_run": not enabled.get("eddy")
        or bool(eddy_metrics) and all(
            int(value["count"] or 0) > 1
            and float(value["effective_rate_hz"] or 0) > 0
            and float(value["run_coverage_ratio"] or 0) >= 0.95
            for value in eddy_metrics.values()
        ),
        "mapping_recorded_topics_cover_run": not enabled.get("mapping") or all(
            int(effective.get(topic, {}).get("count", 0) or 0) > 1
            and float(effective.get(topic, {}).get("effective_rate_hz", 0) or 0) > 0
            and float(effective.get(topic, {}).get("run_coverage_ratio", 0) or 0) >= 0.95
            for topic in recorded_mapping_topics
        ),
        "mapping_online_sources_observed": mapping_sources_observed,
        "eddy_driver_connected": not requires_eddy_status or latest_eddy.get("connected") is True,
        "eddy_driver_drop_zero": not requires_eddy_status or int(latest_eddy.get("dropped_frames", -1)) == 0,
        "eddy_driver_status_error_zero": not enabled.get("eddy") or (not eddy_error_seen and (not requires_eddy_status or latest_eddy.get("last_status_is_error") is False)),
        "recorder_write_error_zero": not recorder_log_errors,
        "compact_trajectory_rebuilt": trajectory_report["valid"],
        "tracker_at_least_6_hz": not tracker_enabled or (
            tracker_rate is not None and tracker_rate >= 6.0
        ),
        "video_queue_drop_zero": not enabled.get("front_video") or int(video.get("queue_drop_count", 1) or 0) == 0,
        "mcap_valid": mcap["valid"], "video_valid": not enabled.get("front_video") or video["valid"],
        "timestamps_valid": timestamp["valid"], "snapshots_valid": snapshots["valid"],
        "no_orphans": orphans["valid"], "provenance_disabled": not manifest.get(
            "geometry_provenance_sampling_enabled", True
        ),
    }
    module_for_check = {"camera": "core", "front_camera": "core", "video": "front_video",
        "mid360": "mid360", "d405": "d405", "eddy": "eddy", "mapping": "mapping", "tracker": "tracker"}
    check_results = {}
    for name, passed in checks.items():
        module = next((value for prefix,value in module_for_check.items() if name.startswith(prefix)), None)
        active = bool(enabled.get(module)) if module else True
        check_results[name] = {"status": ("passed" if passed else "failed") if active else "disabled",
                               "passed": bool(passed) if active else None}
    if not snapshots["event_count"]:
        check_results["snapshots_valid"] = {"status": snapshots["status"], "passed": None}
    check_results["provenance_disabled"] = {"status": "not_applicable", "passed": None,
        "reason": "provenance is an optional diagnostic switch, not an acquisition quality gate"}
    checks.pop("provenance_disabled")
    elapsed = float(status.get("elapsed_sec", 0) or 0)
    acceptance = {
        "schema_version": ("field_w6_system_acceptance_v1" if is_mapping_eddy
                           else "field_w5_system_acceptance_v1"),
        "created_at": utc_now(),
        "valid": all(checks.values()),
        "checks": checks,
        "check_results": check_results,
        "acceptance": {"collection": "passed" if all(checks.values()) else "failed",
            "transport": "passed" if mcap["valid"] and timestamp["valid"] else "failed",
            "algorithm": "not_executed", "calibration": "not_calibrated",
            "replay": "not_executed", "viewer": "not_executed"},
        "rates_hz": rates,
        "run_reference": {
            "topic": "/field/collection_status",
            "first_bag_timestamp_ns": reference_first_ns,
            "last_bag_timestamp_ns": reference_last_ns,
        },
        "effective_topic_metrics": effective,
        "video_effective_metrics": video_effective,
        "d405_image_rates_hz": d405_image_rates,
        "d405_expected_fps": d405_expected_fps,
        "d405_minimum_fps": d405_minimum_fps,
        "camera_timeout_count": camera_timeout,
        "camera_capture_error_count": camera_errors,
        "elapsed_sec": elapsed,
        "stage_results": {
            "S0_source_and_static_checks": "not_executed",
            "S1_60_second_full_chain": all(checks.values()) and elapsed >= 60,
            "S2_10_minute": all(checks.values()) and elapsed >= 600,
            "S3_30_minute": all(checks.values()) and elapsed >= 1800,
            "S4_finalize_sha_replay": "not_executed",
        },
        "camera_info_calibrated": False,
        "spatial_fusion_complete": False,
        "tracker_enabled": tracker_enabled,
        "latest_eddy_status": latest_eddy,
        "recorder_log_errors": recorder_log_errors,
    }
    atomic_json(report_dir / "system_acceptance.json", acceptance)
    diagnostic = {
        "schema_version": ("field_w6_diagnostic_summary_v1" if is_mapping_eddy
                           else "field_w5_diagnostic_summary_v1"),
        "created_at": utc_now(), "rates_hz": rates,
        "collection_status": status, "acceptance_checks": checks,
    }
    atomic_json(report_dir / "diagnostic_summary.json", diagnostic)
    return 0 if acceptance["valid"] else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("vehicle collection validation interrupted", file=sys.stderr)
        raise SystemExit(130)
