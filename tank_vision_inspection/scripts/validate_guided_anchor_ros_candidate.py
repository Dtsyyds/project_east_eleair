#!/usr/bin/env python3
"""Offline and topic-injected validation for the S1-P2F0 ROS candidate.

The ROS check uses ``input_mode=ros_topic`` and the frozen image as an injected
topic sample.  It never opens the MindVision device or starts the direct launch.
"""

from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_anchor_width_tail import pstats  # noqa: E402
from scripts.diagnose_reference_selection_error import manual_at, manual_points  # noqa: E402
from tank_vision.seam_tracking_types import StableSeamResult, TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402
from tank_vision.snapshot_writer import SnapshotTask, sha256_file, write_snapshot  # noqa: E402

OUT = ROOT / "output/s1_p2f0_guided_anchor_ros_candidate"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
GUIDED = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
ANCHOR_CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
STABLE = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
GEOMETRY_REF = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
WIDTH_REF = ROOT / "configs/seam_width_reference_s100p_v1.json"
LAUNCH = ROOT / "ros2_ws/src/tank_vision_ros/launch/stable_seam_direct_guided_anchor.launch.py"
DEFAULT_LAUNCH = ROOT / "ros2_ws/src/tank_vision_ros/launch/stable_seam_direct.launch.py"
SETUP = ROOT / "ros2_ws/src/tank_vision_ros/setup.py"
_INSTALL_PREFIX = ROOT / "ros2_ws/install_s100p_humble"
INSTALL_SHARE = (_INSTALL_PREFIX / "tank_vision_ros/share/tank_vision_ros"
                 if (_INSTALL_PREFIX / "tank_vision_ros/share/tank_vision_ros").is_dir()
                 else _INSTALL_PREFIX / "share/tank_vision_ros")
EXPECTED_RAW = "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02"


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(x) for x in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        x = float(value)
        return x if np.isfinite(x) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def write_json(name: str, value: Any) -> None:
    (OUT / name).write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def curve_stats(curves: list[dict[str, float]], points: tuple[np.ndarray, np.ndarray], ys: list[float] | None = None) -> dict[str, Any]:
    manuals = [manual_at(float(y), points) for y in (ys or [c["y"] for c in curves])]
    fields = {}
    for field in ("left", "right", "center", "width"):
        errors = [curve[field] - manual[field] for curve, manual in zip(curves, manuals)]
        fields[field] = {"signed": pstats(errors), "absolute": pstats([abs(x) for x in errors])}
    return fields


def offline_reproduction(image: np.ndarray) -> dict[str, Any]:
    points = manual_points()
    tracker = StableSeamTracker(str(ANCHOR_CONFIG), str(GEOMETRY_REF))
    results = []
    for _ in range(3):
        results.append(tracker.process(image))
    final = results[-1]
    curves = [{"left": l[0], "right": r[0], "center": c[0], "width": w, "y": c[1]}
              for l, r, c, w in zip(final.left_boundary_original_px, final.right_boundary_original_px,
                                     final.centerline_original_px, final.width_profile_px)]
    y_values = [x["y"] for x in curves]
    errors = curve_stats(curves, points, y_values)
    crossing = sum(int(l[0] >= r[0]) for l, r in zip(final.left_boundary_original_px, final.right_boundary_original_px))
    negative_tracker = StableSeamTracker(str(ANCHOR_CONFIG), str(GEOMETRY_REF))
    negative = negative_tracker.process(np.full_like(image, int(np.mean(image))))
    return {
        "status": "matched" if (len(final.centerline_original_px) == 32 and final.accepted_scan_bands == 117
                                  and abs(final.coverage - 1.0) < 1e-12
                                  and abs(final.fit_inlier_ratio - 0.9401709401709402) < 1e-9
                                  and [r.tracking_state.name for r in results] == ["ACQUIRING", "ACQUIRING", "TRACKING"]
                                  and errors["left"]["absolute"]["p95"] < 12.0
                                  and errors["right"]["absolute"]["p95"] < 15.0
                                  and errors["center"]["absolute"]["p95"] < 12.0
                                  and errors["width"]["absolute"]["p95"] < 12.0
                                  and crossing == 0 and final.abrupt_left_jump_count == 0 and final.abrupt_right_jump_count == 0)
                   else "offline_mismatch",
        "config": str(ANCHOR_CONFIG), "reference": str(GEOMETRY_REF),
        "selected_path": final.accepted_scan_bands, "coverage": final.coverage,
        "joint_inlier_ratio": final.fit_inlier_ratio,
        "state_sequence": [r.tracking_state.name for r in results],
        "final_message_sampling": errors, "final_message_point_count": len(curves),
        "crossing": crossing, "abrupt_left": final.abrupt_left_jump_count,
        "abrupt_right": final.abrupt_right_jump_count,
        "processing_ms": [r.processing_ms for r in results],
        "hard_no_seam_negative": {"detected": negative.detected, "state": negative.tracking_state.name,
                                  "crossing": 0, "pass": not negative.detected and negative.tracking_state == TrackingState.LOST},
    }


def default_regression(image: np.ndarray) -> dict[str, Any]:
    tracker = StableSeamTracker(str(STABLE), str(WIDTH_REF))
    results = [tracker.process(image) for _ in range(3)]
    config = yaml.safe_load(STABLE.read_text(encoding="utf-8"))
    return {"status": "checked", "launch": str(DEFAULT_LAUNCH), "config": str(STABLE), "reference": str(WIDTH_REF),
            "config_reference_path": config["reference"]["path"],
            "anchor_enabled": bool(config.get("guided_search", {}).get("reference_anchor_enabled", False)),
            "state_sequence": [r.tracking_state.name for r in results],
            "accepted_scan_bands": results[0].accepted_scan_bands,
            "coverage": results[0].coverage, "detected": results[0].detected,
            "default_launch_source_sha256": sha256(DEFAULT_LAUNCH),
            "behavior": "existing stable launch file not modified; default node parameters remain unchanged"}


def runtime_resolution() -> dict[str, Any]:
    source = LAUNCH.read_text(encoding="utf-8")
    installed = {
        "config_file": INSTALL_SHARE / "config/s100p_guided_anchor_seam_4096x3072.yaml",
        "reference_file": INSTALL_SHARE / "config/seam_geometry_reference_s100p_live_v1.json",
        "launch_file": INSTALL_SHARE / "launch/stable_seam_direct_guided_anchor.launch.py",
    }
    args = [name for name in ("config_file", "reference_file", "publish_debug_image", "snapshot_output_root")
            if f'DeclareLaunchArgument("{name}"' in source]
    ros_env = os.environ.copy()
    ros_env["ROS_LOG_DIR"] = str(OUT / "ros_logs")
    show = subprocess.run(["ros2", "launch", "tank_vision_ros", "stable_seam_direct_guided_anchor.launch.py", "--show-args"],
                          capture_output=True, text=True, check=False, env=ros_env)
    return {"launch": str(LAUNCH), "node": "tank_vision_ros/stable_seam_node", "input_mode": "mindvision_direct",
            "declared_arguments": args, "default_config_file": str(installed["config_file"]),
            "default_reference_file": str(installed["reference_file"]), "publish_debug_image_default": False,
            "show_args_returncode": show.returncode, "show_args_stdout": show.stdout, "show_args_stderr": show.stderr,
            "show_args_has_required": all(name in show.stdout for name in ("config_file", "publish_debug_image", "snapshot_output_root")),
            "installed_paths_exist": {key: path.is_file() for key, path in installed.items()}}


def source_install_hashes() -> dict[str, Any]:
    pairs = {
        "config": (ANCHOR_CONFIG, INSTALL_SHARE / "config/s100p_guided_anchor_seam_4096x3072.yaml"),
        "reference": (GEOMETRY_REF, INSTALL_SHARE / "config/seam_geometry_reference_s100p_live_v1.json"),
        "launch": (LAUNCH, INSTALL_SHARE / "launch/stable_seam_direct_guided_anchor.launch.py"),
    }
    output = {}
    for name, (source, install) in pairs.items():
        output[name] = {"source": str(source), "install": str(install),
                        "source_sha256": sha256(source),
                        "install_sha256": sha256(install) if install.is_file() else None,
                        "match": install.is_file() and sha256(source) == sha256(install)}
    return output


class _Stamp:
    sec = 1788922627
    nanosec = 983752476


class _Header:
    stamp = _Stamp()
    frame_id = "offline_injected_camera"


def ros_topic_injected_e2e(image: np.ndarray) -> dict[str, Any]:
    """Run the ROS node in topic mode; no camera driver is constructed."""
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    from tank_vision_interfaces.msg import SeamGeometry
    from tank_vision_ros.stable_seam_node import StableSeamNode

    snapshot_root = OUT / "ros_e2e_snapshot_probe"
    if snapshot_root.exists() and any(snapshot_root.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty {snapshot_root}")
    snapshot_root.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=["--ros-args", "-p", "input_mode:=ros_topic", "-p", f"config_file:={ANCHOR_CONFIG}",
                     "-p", f"reference_file:={GEOMETRY_REF}", "-p", f"snapshot_output_dir:={snapshot_root}",
                     "-p", "publish_debug_image:=false"])
    executor = SingleThreadedExecutor()
    node = None
    probe = None
    try:
        node = StableSeamNode()
        probe = Node("guided_anchor_ros_e2e_probe")
        bridge = CvBridge()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
        publisher = probe.create_publisher(Image, "/camera/front/image_raw", qos)
        geometry_messages = []
        debug_messages = []
        probe.create_subscription(SeamGeometry, "/vision/front/seam_geometry", geometry_messages.append, 10)
        probe.create_subscription(Image, "/vision/front/debug_image", debug_messages.append, qos)
        executor.add_node(node)
        executor.add_node(probe)
        def spin_until(predicate: Any, timeout: float = 30.0) -> None:
            end = time.monotonic() + timeout
            while not predicate():
                if time.monotonic() >= end:
                    raise RuntimeError("ROS topic-injected E2E timed out")
                executor.spin_once(timeout_sec=0.1)
        def publish(stamp: int) -> None:
            message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            message.header.stamp.sec = stamp
            message.header.stamp.nanosec = 1
            message.header.frame_id = "offline_injected_camera"
            publisher.publish(message)
            spin_until(lambda: len(geometry_messages) >= stamp - 1788922626)
        spin_until(lambda: publisher.get_subscription_count() > 0)
        for index in range(1, 4):
            publish(1788922626 + index)
        debug_off_count = len(debug_messages)
        node.set_parameters([Parameter("publish_debug_image", Parameter.Type.BOOL, True)])
        publish(1788922630)
        spin_until(lambda: len(debug_messages) >= 1)
        client = probe.create_client(Trigger, "/stable_seam_node/save_snapshot")
        spin_until(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        spin_until(lambda: future.done())
        service_ok = bool(future.result() and future.result().success)
        publish(1788922631)
        spin_until(lambda: any((p / "metadata.json").is_file() for p in snapshot_root.iterdir()))
        snapshot_dirs = [p for p in snapshot_root.iterdir() if (p / "metadata.json").is_file()]
        snapshot = max(snapshot_dirs, key=lambda p: p.stat().st_mtime)
        metadata = json.loads((snapshot / "metadata.json").read_text(encoding="utf-8"))
        geometry = geometry_messages[-1]
        points_ok = len(geometry.left_boundary) == len(geometry.right_boundary) == len(geometry.centerline) == 32
        ordered = all(l.x < c.x < r.x and l.z == 0.0 and c.z == 0.0 and r.z == 0.0
                      for l, c, r in zip(geometry.left_boundary, geometry.centerline, geometry.right_boundary))
        return {"status": "passed" if (service_ok and points_ok and ordered and debug_off_count == 0
                                          and len(debug_messages) >= 1 and metadata.get("complete") is True) else "failed",
                "camera_started": False, "input_mode": "ros_topic", "geometry_messages": len(geometry_messages),
                "last_geometry": {"header_frame_id": geometry.header.frame_id, "detected": geometry.detected,
                                   "tracking_state": int(geometry.tracking_state), "confidence": float(geometry.confidence),
                                   "point_counts": [len(geometry.left_boundary), len(geometry.right_boundary), len(geometry.centerline)],
                                   "left_center_right_ordered": ordered, "z_zero": ordered},
                "debug_disabled_message_count": debug_off_count, "debug_enabled_message_count": len(debug_messages),
                "save_snapshot_service_success": service_ok, "snapshot_directory": str(snapshot),
                "snapshot_complete": metadata.get("complete"),
                "snapshot_hash_checks": {"raw": metadata.get("raw_sha256") == sha256_file(snapshot / "raw.bmp"),
                                          "geometry": metadata.get("seam_geometry_sha256") == sha256_file(snapshot / "seam_geometry.json")}}
    finally:
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(OUT / "ros_logs"))
    if sha256(RAW) != EXPECTED_RAW:
        raise RuntimeError("fixed raw SHA mismatch")
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read fixed raw")
    offline = offline_reproduction(image)
    write_json("offline_reproduction.json", offline)
    if offline["status"] != "matched":
        return 2
    write_json("runtime_config_resolution.json", runtime_resolution())
    write_json("source_install_hashes.json", source_install_hashes())
    write_json("default_stable_regression.json", default_regression(image))
    ros_result = ros_topic_injected_e2e(image)
    write_json("ros_e2e_result.json", ros_result)
    hashes = json.loads((OUT / "source_install_hashes.json").read_text(encoding="utf-8"))
    runtime = json.loads((OUT / "runtime_config_resolution.json").read_text(encoding="utf-8"))
    summary = {"task": "S1-P2F0", "status": "passed" if (ros_result["status"] == "passed"
                and all(item["match"] for item in hashes.values())
                and runtime["show_args_returncode"] == 0
                and runtime["show_args_has_required"])
                else "failed", "candidate_status": "field-validation candidate",
               "offline_status": offline["status"], "ros_e2e_status": ros_result["status"],
               "default_production_path_replaced": False, "camera_started": False,
               "fixed_raw_sha256": sha256(RAW)}
    write_json("validation_summary.json", summary)
    return 0 if summary["status"] == "passed" else 3


if __name__ == "__main__":
    raise SystemExit(main())
