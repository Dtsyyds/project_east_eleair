#!/usr/bin/env python3
"""Collate auditable FIELD-W5 source, encoder, storage and stage reports."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
VISION_ROOT = ROOT.parent / "tank_vision_inspection"
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, load_json, sha256_file, utc_now  # noqa: E402


def _git_status(path: Path) -> dict[str, Any]:
    result = subprocess.run(
        ["git", "-C", str(path), "status", "--short"],
        text=True, capture_output=True, check=False,
    )
    return {
        "path": str(path), "is_git_worktree": result.returncode == 0,
        "dirty": bool(result.stdout.strip()), "status_short": result.stdout.splitlines(),
    }


def _file(path: Path, role: str, action: str) -> dict[str, Any]:
    return {
        "path": str(path), "role": role, "action": action,
        "exists": path.is_file(), "sha256": sha256_file(path) if path.is_file() else None,
    }


def _topic_rate(mcap: dict[str, Any], topic: str) -> float | None:
    for bag in mcap.get("bags", {}).values():
        item = bag.get("message_validation", {}).get("topics", {}).get(topic)
        if item:
            return item.get("rate_hz")
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--summary-dir", type=Path,
        default=VISION_ROOT / "output" / "field_w5_vehicle_collection",
    )
    args = parser.parse_args()
    run_dir = args.run_dir.resolve()
    reports = run_dir / "reports"
    summary = args.summary_dir.resolve()
    reports.mkdir(parents=True, exist_ok=True)
    summary.mkdir(parents=True, exist_ok=True)
    created = utc_now()

    reused = [
        (ROOT / "scripts/start_field_visual.sh", "existing field visual startup"),
        (ROOT / "scripts/stop_field_visual.sh", "existing exact-stop/finalize precedent"),
        (ROOT / "scripts/finalize_field_run.py", "existing manifest/SHA precedent"),
        (ROOT / "scripts/field_visual_preflight.py", "existing device/storage preflight precedent"),
        (ROOT / "scripts/replay_visual_mcap.py", "existing MCAP replay precedent"),
        (ROOT / "src/inspection_bringup/launch/visual_field_validation.launch.py", "single MindVision owner and tracker launch"),
        (VISION_ROOT / "scripts/run_field_visual_e2e.sh", "existing visual E2E workflow"),
        (VISION_ROOT / "scripts/stop_field_visual_e2e.sh", "existing visual stop workflow"),
        (VISION_ROOT / "scripts/record_seam_validation_session.py", "existing validation recording"),
        (VISION_ROOT / "scripts/replay_seam_validation_session.py", "existing validation replay"),
        (VISION_ROOT / "scripts/seam_validation_common.py", "existing manifest/hash helpers"),
        (VISION_ROOT / "tank_vision/roi_validation_recorder.py", "bounded ROI recording"),
        (VISION_ROOT / "tank_vision/failure_event_recorder.py", "bounded atomic failure snapshots"),
        (VISION_ROOT / "scripts/field_w4_multisensor_e2e.py", "W4 test evidence and probes"),
    ]
    created_files = [
        "vehicle_collection_common.py", "vehicle_collection_manager.py",
        "front_h264_recorder.py", "front_video_recorder.py",
        "vehicle_collection_monitor.py", "vehicle_collection_disk_guard.py",
        "stop_vehicle_collection.py", "validate_vehicle_collection.py",
        "finalize_vehicle_collection.py", "replay_vehicle_collection.py",
        "generate_vehicle_collection_summary.py", "start_vehicle_collection.sh",
        "status_vehicle_collection.sh", "stop_vehicle_collection.sh",
        "replay_vehicle_collection.sh",
    ]
    source_inventory = {
        "schema_version": "field_w5_source_inventory_v2", "created_at": created,
        "run_id": run_dir.name,
        "reused": [_file(path, role, "reused") for path, role in reused],
        "field_w5_files": [
            _file(SCRIPT_DIR / name, "FIELD-W5 collection component", "created")
            for name in created_files
        ],
        "modified_existing": [
            _file(
                ROOT / "src/inspection_bringup/launch/visual_field_validation.launch.py",
                "added camera_qos_reliability passthrough only", "modified",
            )
        ],
        "algorithm_files_modified": [],
        "build_install_log_direct_edits": False,
        "workspace": {
            "physic_bev_source": _git_status(ROOT / "src"),
            "tank_vision_inspection": _git_status(VISION_ROOT),
        },
        "resolution": {
            "inspection_bringup_source": str(ROOT / "src/inspection_bringup"),
            "inspection_bringup_install": str(ROOT / "install/inspection_bringup"),
            "tank_vision_ros_source": str(VISION_ROOT / "ros2_ws/src/tank_vision_ros"),
            "tank_vision_ros_install": str(VISION_ROOT / "ros2_ws/install_s100p_humble/tank_vision_ros"),
            "inspection_bringup_rebuilt_after_source_change": True,
        },
    }
    atomic_json(reports / "source_inventory.json", source_inventory)

    video = load_json(reports / "video_validation.json", {}) or {}
    encoder_inventory = {
        "schema_version": "field_w5_encoder_inventory_v2", "created_at": created,
        "selected": {
            "encoder": "hobot_codec S100P VPU H.264", "package_version": "2.3.5",
            "input": "/camera/front/image_raw BGR8 4096x3072 reliable",
            "output": "/camera/front/image_h264 img_msgs/msg/H26XFrame",
            "container": "Matroska (MKV)", "tested": True,
            "measured_bitrate_mbps": (
                load_json(reports / "video_runtime_status.json", {}) or {}
            ).get("measured_bitrate_mbps"),
            "validation": video,
        },
        "alternatives_tested": {
            "ffmpeg_libx264": "available and synthetic encode succeeded; not selected due CPU cost",
            "ffmpeg_h264_v4l2m2m": "failed: no valid V4L2 encoder device",
            "ffmpeg_h264_omx": "failed: OMX library unavailable",
            "gstreamer_h264": "no x264enc/openh264enc/v4l2h264enc plugin detected",
            "opencv": "available but no validated hardware H.264 path; not selected",
        },
        "timestamp_limit": "vendor H26XFrame preserves source ROS stamp; internal VPU monotonic enqueue timestamp is not exposed and is recorded as null, not fabricated",
    }
    atomic_json(reports / "encoder_inventory.json", encoder_inventory)

    preflight = load_json(reports / "preflight_report.json", {}) or {}
    mcap = load_json(reports / "mcap_validation.json", {}) or {}
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    acceptance = load_json(reports / "system_acceptance.json", {}) or {}
    acceptance_checks = acceptance.get("checks", {})
    tracker_enabled = bool(manifest.get("enabled_writers", {}).get("tracker", False))
    failed_checks = [name for name, passed in acceptance_checks.items() if not passed]
    bag_bytes = sum(
        int(item.get("size", 0))
        for bag in mcap.get("bags", {}).values() for item in bag.get("files", [])
    )
    video_bytes = sum(path.stat().st_size for path in (run_dir / "video/front").glob("*.mkv"))
    live_session_bytes = sum(
        path.stat().st_size for path in (run_dir / "live_session").rglob("*")
        if path.is_file()
    )
    spans = [
        float(item.get("span_sec") or 0)
        for bag in mcap.get("bags", {}).values()
        for item in bag.get("message_validation", {}).get("topics", {}).values()
    ]
    observed_span = max(spans, default=0.0)
    total_bytes = bag_bytes + video_bytes + live_session_bytes
    observed_MBps = total_bytes / observed_span / 1_000_000 if observed_span else None
    storage = {
        "schema_version": "field_w5_storage_bandwidth_v1", "created_at": created,
        "mount": preflight.get("mount"), "benchmark": preflight.get("storage_benchmark"),
        "planning": preflight.get("bandwidth_plan"), "free_bytes_at_start": preflight.get("free_bytes"),
        "observed": {
            "span_sec": observed_span, "mcap_bytes": bag_bytes, "video_bytes": video_bytes,
            "live_session_and_failure_snapshot_bytes": live_session_bytes,
            "total_bytes": total_bytes, "average_MBps": observed_MBps,
            "projected_GB_per_hour": observed_MBps * 3.6 if observed_MBps is not None else None,
            "note": (
                "projection includes the 42-event snapshot load from a failed, continuously "
                "LOST tracker run; it is not a 10/30 minute normal-operation stability result"
            ),
        },
    }
    atomic_json(reports / "storage_bandwidth_report.json", storage)

    performance = {
        "schema_version": "field_w5_performance_matrix_v1", "created_at": created,
        "tests": {
            **{f"T{i}": {"status": "not_executed", "reason": "no independent result for this run"}
               for i in range(6)},
            "T6": {"status": "failed", "run_id": run_dir.name,
                   "camera_hz": _topic_rate(mcap, "/camera/front/camera_info"),
                   "tracker_hz": _topic_rate(mcap, "/vision/front/seam_state"),
                   "mid360_lidar_hz": _topic_rate(mcap, "/livox/lidar"),
                   "mid360_imu_hz": _topic_rate(mcap, "/livox/imu"),
                   "eddy_channel_mean_hz": _topic_rate(mcap, "/eddy/channel_mean"),
                   "tracker_enabled": tracker_enabled,
                   "primary_failure": (
                       "tracker below 6 Hz" if "tracker_at_least_6_hz" in failed_checks
                       else (failed_checks[0] if failed_checks else None)
                   )},
            "T7": {"status": "included_in_T6",
                   "failure_snapshots_enabled": bool(
                       manifest.get("enabled_writers", {}).get("failure_snapshots", False)
                   ),
                   "complete_events": (load_json(reports / "snapshot_integrity.json", {}) or {}).get("event_count")},
            "T8": {"status": "not_run", "reason": "optional raw burst disabled and S1 failed"},
        },
        "stop_rule": "acceptance determines this run; independent duration gates require their own evidence",
    }
    performance["tests"]["T6"]["status"] = (
        "passed" if acceptance.get("valid") is True else "failed"
    )
    atomic_json(reports / "performance_matrix.json", performance)

    replay_path = reports / "replay_validation.json"
    existing_replay = load_json(replay_path, {}) or {}
    if not (
        existing_replay.get("executed") is True
        or (existing_replay.get("valid") is True and existing_replay.get("mode"))
    ):
        replay = {
            "schema_version": "field_w5_replay_validation_v1", "created_at": created,
            "valid": False, "executed": False, "run_id": run_dir.name,
            "reason": (
                "replay E2E has not been executed"
                if acceptance.get("valid") is True
                else "S1 acceptance failed; S2/S3 and replay E2E were not authorized by the stage gate"
            ),
            "topic_replay_counts": {}, "original_span_sec": observed_span,
            "replay_span_sec": None, "detected_state_consistency": None,
            "geometry_comparison": None, "first_mismatch": None,
            "video_frame_header_alignment": "sidecar exists; replay comparison not executed",
        }
        atomic_json(replay_path, replay)

    # Preserve the runtime-discovered inventory as the authoritative final inventory.
    shutil.copy2(run_dir / "topic_inventory.json", reports / "topic_inventory.json")
    for name in (
        "source_inventory.json", "topic_inventory.json", "encoder_inventory.json",
        "storage_bandwidth_report.json", "performance_matrix.json", "replay_validation.json",
    ):
        shutil.copy2(reports / name, summary / name)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
