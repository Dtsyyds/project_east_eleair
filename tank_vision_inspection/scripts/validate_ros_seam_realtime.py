#!/usr/bin/env python3
"""Measure ROS seam transport, header association, and output freshness.

The validator intentionally subscribes to CameraInfo instead of Image.  The
MindVision driver publishes CameraInfo with the same Header as Image, so this
provides a bounded, low-load input-header evidence stream without importing
cv_bridge or copying image payloads.

The statistical core is ROS-independent so it can be unit tested on a
development machine without rclpy.  ROS imports are intentionally lazy in
``main`` for Python 3.10/Humble deployment.
"""

from __future__ import annotations

import argparse
from collections import deque
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable


DEFAULT_CAMERA_INFO_TOPIC = "/camera/front/camera_info"
DEFAULT_SEAM_TOPIC = "/vision/front/seam_state"
DEFAULT_QUALITY_TOPIC = "/vision/front/quality"
DEFAULT_MAX_CACHE_ENTRIES = 10_000


def percentile(values: Iterable[float], percent: float) -> float | None:
    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return None
    position = (len(ordered) - 1) * percent / 100.0
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def finite_values(values: Iterable[float | None]) -> list[float]:
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


def header_key(stamp_ns: int, frame_id: str) -> tuple[int, str]:
    return int(stamp_ns), str(frame_id)


class BoundedKeys:
    """Bounded exact key membership with an eviction signal."""

    def __init__(self, max_entries: int) -> None:
        if max_entries < 1:
            raise ValueError("max_entries must be positive")
        self.max_entries = int(max_entries)
        self._order: deque[tuple[int, str]] = deque()
        self._keys: set[tuple[int, str]] = set()
        self.evicted = False

    def add(self, key: tuple[int, str]) -> bool:
        if key in self._keys:
            return False
        if len(self._order) >= self.max_entries:
            oldest = self._order.popleft()
            self._keys.remove(oldest)
            self.evicted = True
        self._order.append(key)
        self._keys.add(key)
        return True

    def contains(self, key: tuple[int, str]) -> bool:
        return key in self._keys

    def __len__(self) -> int:
        return len(self._order)


class SeamRealtimeAccumulator:
    """Bounded, deterministic statistics for CameraInfo/SeamState/Quality."""

    def __init__(self, scenario: str, max_cache_entries: int = DEFAULT_MAX_CACHE_ENTRIES) -> None:
        self.scenario = str(scenario)
        self.max_cache_entries = int(max_cache_entries)
        self.camera_info_count = 0
        self.seam_count = 0
        self.quality_count = 0
        self._camera_stamps = BoundedKeys(max_cache_entries)
        self._seam_stamps = BoundedKeys(max_cache_entries)
        self._camera_headers = BoundedKeys(max_cache_entries)
        self._last_camera_stamp: int | None = None
        self._last_seam_stamp: int | None = None
        self.camera_stamp_monotonic = True
        self.seam_stamp_monotonic = True
        self.duplicate_seam_stamp_count = 0
        self.exact_header_match_count = 0
        self.unmatched_seam_count = 0
        self.seam_detected_count = 0
        self._confidence: list[float] = []
        self._lateral_error_px: list[float] = []
        self._heading_error_deg: list[float] = []
        self._output_age_ms: list[float] = []
        self._output_age_available = True
        self._output_age_reason: str | None = None
        self.invalid_quality_count = 0
        self._warnings: list[str] = []

    def _warn(self, warning: str) -> None:
        if warning not in self._warnings:
            self._warnings.append(warning)

    def _check_monotonic(self, stamp_ns: int, previous: int | None, stream: str) -> bool:
        if previous is not None and int(stamp_ns) <= previous:
            self._warn(f"{stream}_stamp_not_strictly_monotonic")
            return False
        return True

    def observe_camera_info(self, stamp_ns: int, frame_id: str) -> None:
        stamp_ns = int(stamp_ns)
        self.camera_info_count += 1
        self.camera_stamp_monotonic &= self._check_monotonic(
            stamp_ns, self._last_camera_stamp, "camera"
        )
        self._last_camera_stamp = stamp_ns
        self._camera_stamps.add(header_key(stamp_ns, ""))
        self._camera_headers.add(header_key(stamp_ns, frame_id))
        if self._camera_stamps.evicted or self._camera_headers.evicted:
            self._warn("camera_header_cache_evicted_association_is_bounded")

    def _observe_output_age(self, stamp_ns: int, receive_ns: int | None) -> None:
        if not self._output_age_available:
            return
        if receive_ns is None or int(receive_ns) <= 0 or int(stamp_ns) <= 0:
            self._output_age_available = False
            self._output_age_reason = "ROS receive clock or Header stamp is unavailable"
            self._warn("output_age_unavailable:clock_or_header_stamp_missing")
            return
        age_ms = (int(receive_ns) - int(stamp_ns)) / 1_000_000.0
        # A negative age beyond normal scheduling jitter or a multi-day age is
        # strong evidence that the two clocks cannot be compared reliably.
        if age_ms < -1000.0 or abs(age_ms) > 86_400_000.0:
            self._output_age_available = False
            self._output_age_reason = "ROS clock and Header stamp are not reliably comparable"
            self._warn("output_age_unavailable:incompatible_clock_basis")
            return
        self._output_age_ms.append(age_ms)

    def observe_seam(
        self,
        stamp_ns: int,
        frame_id: str,
        detected: bool,
        confidence: float | None,
        lateral_error_px: float | None,
        heading_error_deg: float | None,
        receive_ns: int | None = None,
    ) -> None:
        stamp_ns = int(stamp_ns)
        key = header_key(stamp_ns, frame_id)
        self.seam_count += 1
        self.seam_stamp_monotonic &= self._check_monotonic(stamp_ns, self._last_seam_stamp, "seam")
        self._last_seam_stamp = stamp_ns
        if not self._seam_stamps.add(header_key(stamp_ns, "")):
            self.duplicate_seam_stamp_count += 1
        if self._seam_stamps.evicted:
            self._warn("seam_stamp_cache_evicted_duplicate_count_is_bounded")
        if self._camera_headers.contains(key):
            self.exact_header_match_count += 1
        else:
            self.unmatched_seam_count += 1
            self._warn("unmatched_seam_header")
        if bool(detected):
            self.seam_detected_count += 1
        self._confidence.extend(finite_values([confidence]))
        self._lateral_error_px.extend(finite_values([lateral_error_px]))
        self._heading_error_deg.extend(finite_values([heading_error_deg]))
        self._observe_output_age(stamp_ns, receive_ns)

    def observe_quality(
        self,
        valid_image: bool,
        valid_for_measurement: bool,
        numeric_values: Iterable[float | None],
        warnings: Iterable[str] = (),
    ) -> None:
        self.quality_count += 1
        numeric_valid = all(value is not None and math.isfinite(float(value)) for value in numeric_values)
        if not bool(valid_image) or not bool(valid_for_measurement) or not numeric_valid:
            self.invalid_quality_count += 1
        for warning in warnings:
            self._warn(f"quality:{warning}")

    @staticmethod
    def _distribution(values: list[float]) -> dict[str, float | int | None]:
        return {
            "valid_count": len(values),
            "mean": sum(values) / len(values) if values else None,
            "p50": percentile(values, 50),
            "p95": percentile(values, 95),
        }

    def report(self, duration_s: float) -> dict[str, Any]:
        duration_s = max(0.0, float(duration_s))
        if self.camera_info_count == 0:
            self._warn("no_camera_info_messages:CameraInfo_is_only_lightweight_Header_evidence")
        if self.seam_count == 0:
            self._warn("no_seam_messages")
        if self.quality_count == 0:
            self._warn("no_quality_messages")
        if self._camera_stamps.evicted or self._camera_headers.evicted:
            self._warn("camera_info_count_is_not_Image_count")
        if not self._output_age_ms:
            if self.seam_count and self._output_age_available:
                self._output_age_available = False
                self._output_age_reason = "no_valid_output age samples"
                self._warn("output_age_unavailable:no_valid_samples")
            elif not self.seam_count and self._output_age_reason is None:
                self._output_age_available = False
                self._output_age_reason = "no SeamState samples"
        exact_ratio = (
            self.exact_header_match_count / self.seam_count if self.seam_count else None
        )
        output_age: dict[str, Any] = {
            "available": bool(self._output_age_available and self._output_age_ms),
            "mean": None,
            "p50": None,
            "p95": None,
            "max": None,
            "valid_count": 0,
            "reason": self._output_age_reason,
        }
        if output_age["available"]:
            output_age.update(
                {
                    "mean": sum(self._output_age_ms) / len(self._output_age_ms),
                    "p50": percentile(self._output_age_ms, 50),
                    "p95": percentile(self._output_age_ms, 95),
                    "max": max(self._output_age_ms),
                    "valid_count": len(self._output_age_ms),
                }
            )
        return {
            "scenario": self.scenario,
            "duration_s": duration_s,
            "camera_info_count": self.camera_info_count,
            "camera_info_rate_hz": self.camera_info_count / duration_s if duration_s else 0.0,
            "seam_count": self.seam_count,
            "seam_rate_hz": self.seam_count / duration_s if duration_s else 0.0,
            "quality_count": self.quality_count,
            "unique_camera_stamp_count": len(self._camera_stamps),
            "unique_seam_stamp_count": len(self._seam_stamps),
            "duplicate_seam_stamp_count": self.duplicate_seam_stamp_count,
            "camera_stamp_monotonic": self.camera_stamp_monotonic,
            "seam_stamp_monotonic": self.seam_stamp_monotonic,
            "exact_header_match_count": self.exact_header_match_count,
            "exact_header_match_ratio": exact_ratio,
            "unmatched_seam_count": self.unmatched_seam_count,
            "seam_detected_count": self.seam_detected_count,
            "seam_detected_ratio": self.seam_detected_count / self.seam_count if self.seam_count else None,
            "confidence": self._distribution(self._confidence),
            "lateral_error_px": self._distribution(self._lateral_error_px),
            "heading_error_deg": self._distribution(self._heading_error_deg),
            "output_age_ms": output_age,
            "invalid_quality_count": self.invalid_quality_count,
            "camera_info_is_lightweight_header_evidence": True,
            "camera_info_not_image_count": True,
            "max_cache_entries": self.max_cache_entries,
            "warnings": list(self._warnings),
        }


def _stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--camera-info-topic", default=DEFAULT_CAMERA_INFO_TOPIC)
    parser.add_argument("--seam-topic", default=DEFAULT_SEAM_TOPIC)
    parser.add_argument("--quality-topic", default=DEFAULT_QUALITY_TOPIC)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--overwrite", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.duration <= 0:
        raise SystemExit("--duration must be positive")
    if args.output and args.output.exists() and not args.overwrite:
        raise SystemExit(f"output exists; pass --overwrite to replace it: {args.output}")

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo
        from tank_vision_interfaces.msg import SeamState, VisionQuality
    except ImportError as exc:
        raise SystemExit(f"ROS 2 Humble Python dependencies are required: {exc}") from exc

    accumulator = SeamRealtimeAccumulator(args.scenario)

    class ValidatorNode(Node):
        def __init__(self) -> None:
            super().__init__("validate_ros_seam_realtime")
            reliable_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(CameraInfo, args.camera_info_topic, self._camera_info, qos_profile_sensor_data)
            self.create_subscription(SeamState, args.seam_topic, self._seam, reliable_qos)
            self.create_subscription(VisionQuality, args.quality_topic, self._quality, reliable_qos)

        def _camera_info(self, message: Any) -> None:
            accumulator.observe_camera_info(_stamp_ns(message), message.header.frame_id)

        def _seam(self, message: Any) -> None:
            accumulator.observe_seam(
                _stamp_ns(message),
                message.header.frame_id,
                bool(message.detected),
                float(message.confidence),
                float(message.lateral_error_px),
                float(message.heading_error_deg),
                self.get_clock().now().nanoseconds,
            )

        def _quality(self, message: Any) -> None:
            accumulator.observe_quality(
                bool(message.valid_image),
                bool(message.valid_for_measurement),
                (
                    float(message.mean_brightness),
                    float(message.overexposed_ratio),
                    float(message.underexposed_ratio),
                    float(message.laplacian_variance),
                    float(message.contrast_std),
                    float(message.confidence_scale),
                ),
                message.warnings,
            )

    rclpy.init(args=None)
    node = ValidatorNode()
    started = time.monotonic()
    try:
        deadline = started + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
    finally:
        elapsed = time.monotonic() - started
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    report = accumulator.report(elapsed)
    output = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
