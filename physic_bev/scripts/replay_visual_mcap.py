#!/usr/bin/env python3
"""Replay raw visual MCAP through StableSeamNode and compare header-aligned output."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from tank_vision_interfaces.msg import SeamGeometry


def sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def stamp(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def geometry(message) -> dict[str, object]:
    return {"detected": bool(message.detected), "state": int(message.tracking_state),
            "confidence": float(message.confidence), "fit_residual_px": float(message.fit_residual_px),
            "centerline": [(float(p.x), float(p.y)) for p in message.centerline],
            "left": [(float(p.x), float(p.y)) for p in message.left_boundary],
            "right": [(float(p.x), float(p.y)) for p in message.right_boundary]}


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("visual_replay_compare")
        self.expected: dict[int, dict[str, object]] = {}
        self.replay: dict[int, dict[str, object]] = {}
        self.create_subscription(SeamGeometry, "/vision/front/seam_geometry", self._expected, 10)
        self.create_subscription(SeamGeometry, "/vision/replay/seam_geometry", self._replay, 10)

    def _expected(self, message) -> None:
        self.expected[stamp(message)] = geometry(message)

    def _replay(self, message) -> None:
        self.replay[stamp(message)] = geometry(message)


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            process.terminate()
            process.wait(timeout=10)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    args = parser.parse_args(argv)
    if not (args.bag / "metadata.yaml").is_file():
        raise SystemExit("replay requires a closed MCAP directory with metadata.yaml")
    rclpy.init()
    collector = Collector()
    tracker = subprocess.Popen([
        "ros2", "run", "tank_vision_ros", "stable_seam_node", "--ros-args",
        "-p", "input_mode:=ros_topic", "-p", f"config_file:={args.config.resolve()}",
        "-p", f"reference_file:={args.reference.resolve()}",
        "-p", "geometry_topic:=/vision/replay/seam_geometry",
        "-p", "seam_state_topic:=/vision/replay/seam_state",
        "-p", "camera_info_topic:=/camera/front/camera_info",
        "-p", "publish_debug_image:=false", "-p", "auto_save_failures:=false",
        "-p", "live_session_output_root:=/tmp/visual_replay_live_session",
    ])
    bag = subprocess.Popen(["ros2", "bag", "play", str(args.bag), "--clock"])
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(collector, timeout_sec=0.1)
            if bag.poll() is not None and time.monotonic() + 3.0 > deadline:
                break
            if bag.poll() is not None and collector.expected and collector.replay:
                time.sleep(0.2)
                break
    finally:
        stop_process(bag)
        stop_process(tracker)
        collector.destroy_node()
        rclpy.shutdown()
    common = sorted(set(collector.expected) & set(collector.replay))
    mismatches = []
    for key in common:
        expected, replay = collector.expected[key], collector.replay[key]
        if expected["detected"] != replay["detected"] or expected["state"] != replay["state"]:
            mismatches.append({"stamp_ns": key, "field": "detected/state", "expected": expected, "replay": replay})
            continue
        for field in ("centerline", "left", "right"):
            if len(expected[field]) != len(replay[field]):
                mismatches.append({"stamp_ns": key, "field": field, "expected_count": len(expected[field]), "replay_count": len(replay[field])})
                break
            if any(abs(a - b) > 1e-3 for lhs, rhs in zip(expected[field], replay[field]) for a, b in zip(lhs, rhs)):
                mismatches.append({"stamp_ns": key, "field": field, "expected": expected[field], "replay": replay[field]})
                break
    report = {"frame_count": len(common), "expected_count": len(collector.expected), "replay_count": len(collector.replay),
              "detected_state_consistency_rate": (sum(collector.expected[key]["detected"] == collector.replay[key]["detected"] and
                                                         collector.expected[key]["state"] == collector.replay[key]["state"] for key in common) / len(common)
                                                    if common else 0.0),
              "geometry_key_mismatch_count": len(mismatches), "first_mismatch": mismatches[0] if mismatches else None,
              "config_sha256": sha(args.config), "reference_sha256": sha(args.reference),
              "deterministic_claim": False, "note": "temporal state and dropped frames are compared only by matching header stamp",
              "passed": bool(common) and not mismatches}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
