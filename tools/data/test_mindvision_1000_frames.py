#!/usr/bin/env python3
"""Observe exactly 1000 MindVision ROS frames without starting another camera."""

from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class FrameProbe(Node):
    def __init__(self, topic: str, target: int) -> None:
        super().__init__("mindvision_1000_frame_probe")
        self.target = target
        self.samples: list[dict[str, object]] = []
        self.errors: list[str] = []
        self.started = time.monotonic()
        self._last_stamp = None
        self._last_host = None
        self.diagnostics: list[dict[str, str]] = []
        self.source_read_samples_ms: list[float] = []
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, topic, self._callback, sensor_qos)
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostic_callback, 10)

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "mindvision_camera":
                values = {item.key: item.value for item in status.values}
                self.diagnostics.append(values)
                try:
                    self.source_read_samples_ms.append(float(values["source_read_ms"]))
                except (KeyError, ValueError):
                    pass

    def _callback(self, message: Image) -> None:
        if len(self.samples) >= self.target:
            return
        stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
        host = time.monotonic_ns()
        if self._last_stamp is not None and stamp <= self._last_stamp:
            self.errors.append("device/header timestamp is not strictly monotonic")
        if self._last_host is not None and host <= self._last_host:
            self.errors.append("host timestamp is not monotonic")
        self._last_stamp, self._last_host = stamp, host
        self.samples.append({"stamp_ns": stamp, "host_ns": host, "width": int(message.width),
                             "height": int(message.height), "encoding": str(message.encoding),
                             "step": int(message.step), "data_bytes": len(message.data)})


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/front/image_raw")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    args = parser.parse_args(argv)
    rclpy.init()
    probe = FrameProbe(args.topic, 1000)
    deadline = time.monotonic() + args.timeout_sec
    try:
        while rclpy.ok() and len(probe.samples) < probe.target and time.monotonic() < deadline:
            rclpy.spin_once(probe, timeout_sec=0.2)
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    samples = probe.samples
    inter_arrival_ms = []
    if len(samples) > 1:
        inter_arrival_ms = [(samples[i]["host_ns"] - samples[i - 1]["host_ns"]) / 1e6 for i in range(1, len(samples))]
    source_read = sorted(probe.source_read_samples_ms)
    last_diagnostics = probe.diagnostics[-1] if probe.diagnostics else {}
    shapes = {(row["width"], row["height"], row["encoding"], row["step"], row["data_bytes"]) for row in samples}
    report = {"captured": len(samples), "timeout": int(last_diagnostics.get("timeout_count", 0)),
              "capture_error": int(last_diagnostics.get("capture_error_count", len(probe.errors))),
              "errors": probe.errors, "shape_encoding_constant": len(shapes) <= 1,
              "shape_encoding_values": [list(value) for value in sorted(shapes)],
              "actual_fps": ((len(samples) - 1) / ((samples[-1]["host_ns"] - samples[0]["host_ns"]) / 1e9)
                             if len(samples) > 1 and samples[-1]["host_ns"] > samples[0]["host_ns"] else 0.0),
              "source_read_sample_count": len(source_read),
              "source_read_p50_ms": source_read[len(source_read) // 2] if source_read else None,
              "source_read_p95_ms": source_read[max(0, int(len(source_read) * .95) - 1)] if source_read else None,
              "source_read_max_ms": max(source_read) if source_read else None,
              "inter_arrival_p50_ms": sorted(inter_arrival_ms)[len(inter_arrival_ms) // 2] if inter_arrival_ms else None,
              "inter_arrival_p95_ms": sorted(inter_arrival_ms)[max(0, int(len(inter_arrival_ms) * .95) - 1)] if inter_arrival_ms else None,
              "exposure_gain_samples": len(probe.diagnostics),
              "exposure_gain_last": probe.diagnostics[-1] if probe.diagnostics else None,
              "usb_connection": "see lsusb/udevadm evidence captured by preflight",
              "passed": len(samples) == 1000 and not probe.errors and len(shapes) == 1}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
