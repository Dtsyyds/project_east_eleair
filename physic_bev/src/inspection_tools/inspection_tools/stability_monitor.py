#!/usr/bin/env python3
"""Bounded-overhead ROS topic timing and host resource monitor."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import struct
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import yaml


def percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    pos = (len(ordered) - 1) * q
    low = int(pos)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (pos - low)


def parse_cdr_header(raw: bytes) -> tuple[int | None, str]:
    """Parse a leading std_msgs/Header without deserializing a large payload."""
    view = memoryview(raw)
    if len(view) < 16:
        return None, ""
    # CDR encapsulation is 0x0001 for little endian and 0x0000 for big endian.
    endian = "<" if view[1] & 1 else ">"
    try:
        sec, nanosec, length = struct.unpack_from(endian + "iII", view, 4)
        if nanosec >= 1_000_000_000 or length < 1 or 16 + length > len(view):
            return None, ""
        frame = bytes(view[16:16 + length - 1]).decode("utf-8", errors="replace")
        value = int(sec) * 1_000_000_000 + int(nanosec)
        return (value if value > 0 else None), frame
    except (struct.error, ValueError):
        return None, ""


def _proc_cpu_snapshot() -> tuple[int, int]:
    values = [int(value) for value in Path("/proc/stat").read_text().splitlines()[0].split()[1:]]
    return sum(values), sum(values[3:5])


def _memory_available() -> tuple[int, int]:
    values: dict[str, int] = {}
    for line in Path("/proc/meminfo").read_text().splitlines():
        key, value = line.split(":", 1)
        values[key] = int(value.strip().split()[0]) * 1024
    return values.get("MemTotal", 0), values.get("MemAvailable", 0)


def _process_rows(patterns: list[str]) -> tuple[int, int, str]:
    total_rss = 0
    matched = 0
    details: list[str] = []
    own_pid = os.getpid()
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own_pid:
            continue
        try:
            command = entry.joinpath("cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            names = [pattern for pattern in patterns if pattern in command]
            if not names:
                continue
            status = entry.joinpath("status").read_text()
            rss_kib = int(next(line.split()[1] for line in status.splitlines() if line.startswith("VmRSS:")))
            total_rss += rss_kib * 1024
            matched += 1
            details.append(f"{entry.name}:{names[0]}:{rss_kib}")
        except (OSError, StopIteration, ValueError):
            continue
    return matched, total_rss, ";".join(details)


class TopicState:
    def __init__(self, spec: dict[str, Any]) -> None:
        self.spec = spec
        self.count = 0
        self.invalid = 0
        self.regressions = 0
        self.duplicates = 0
        self.first_steady_ns: int | None = None
        self.last_steady_ns: int | None = None
        self.last_header_ns: int | None = None
        self.intervals_ms: list[float] = []
        self.header_intervals_ms: list[float] = []
        self.latencies_ms: list[float] = []
        self.last_frame_id = ""
        self.last_sequence: int | None = None

    def update(self, header_ns: int | None, frame_id: str, receive_system_ns: int,
               receive_steady_ns: int, sequence: int | None) -> None:
        self.count += 1
        if self.first_steady_ns is None:
            self.first_steady_ns = receive_steady_ns
        if self.last_steady_ns is not None:
            self.intervals_ms.append((receive_steady_ns - self.last_steady_ns) / 1e6)
        self.last_steady_ns = receive_steady_ns
        if header_ns is None:
            self.invalid += 1
        else:
            if self.last_header_ns is not None:
                delta = header_ns - self.last_header_ns
                if delta < 0:
                    self.regressions += 1
                elif delta == 0:
                    self.duplicates += 1
                else:
                    self.header_intervals_ms.append(delta / 1e6)
            self.last_header_ns = header_ns
            self.latencies_ms.append((receive_system_ns - header_ns) / 1e6)
        self.last_frame_id = frame_id
        self.last_sequence = sequence

    def report(self, observation_sec: float, ended_steady_ns: int) -> dict[str, Any]:
        active_elapsed = 0.0
        if self.first_steady_ns is not None and self.last_steady_ns is not None:
            active_elapsed = (self.last_steady_ns - self.first_steady_ns) / 1e9
        expected = float(self.spec.get("expected_min_hz", 0.0))
        # Acceptance uses the complete requested observation window.  Computing
        # only over first-to-last receipt would hide a device that stopped near
        # the end of a long run while making its earlier active rate look good.
        rate = self.count / observation_sec if observation_sec > 0 else 0.0
        active_rate = (
            (self.count - 1) / active_elapsed
            if active_elapsed > 0 and self.count > 1 else 0.0
        )
        end_lag = (
            (ended_steady_ns - self.last_steady_ns) / 1e9
            if self.last_steady_ns is not None else observation_sec
        )
        def dist(values: list[float]) -> dict[str, float | None]:
            return {
                "min": min(values) if values else None,
                "p50": percentile(values, 0.50),
                "p95": percentile(values, 0.95),
                "p99": percentile(values, 0.99),
                "max": max(values) if values else None,
            }
        return {
            "topic": self.spec["topic"],
            "monitored_topic": self.spec.get("monitored_topic", self.spec["topic"]),
            "message_type": self.spec["message_type"],
            "source_device": self.spec["source_device"],
            "timestamp_class": self.spec.get("timestamp_class", "UNKNOWN"),
            "received_count": self.count,
            "invalid_timestamp_count": self.invalid,
            "timestamp_regression_count": self.regressions,
            "duplicate_timestamp_count": self.duplicates,
            "actual_rate_hz": rate,
            "active_span_rate_hz": active_rate,
            "active_span_sec": active_elapsed,
            "end_of_window_lag_sec": end_lag,
            "expected_min_hz": expected,
            "rate_pass": rate >= expected if expected > 0 else None,
            "receive_interval_ms": dist(self.intervals_ms),
            "header_interval_ms": dist(self.header_intervals_ms),
            "header_to_receive_system_ms": dist(self.latencies_ms),
            "frame_id": self.last_frame_id,
            "sequence": self.last_sequence,
            "subscriber_queue_overflow": None,
            "subscriber_queue_overflow_note": "DDS does not expose per-subscription overflow in this path",
        }


class StabilityMonitor(Node):
    def __init__(self, config: dict[str, Any], output: Path, duration_sec: float) -> None:
        super().__init__("stability_monitor")
        self.config = config
        self.output = output
        self.duration_sec = duration_sec
        self.started_system_ns = time.time_ns()
        self.started_steady_ns = time.monotonic_ns()
        self.stop_reason = "duration_complete"
        self.done = False
        self.states: dict[str, TopicState] = {}
        self.diagnostics: dict[str, dict[str, Any]] = {}
        self.resource_samples: list[dict[str, float | int]] = []
        # Do not shadow rclpy.node.Node._subscriptions; Node owns that list.
        self._field_subscriptions: list[Any] = []
        self.output.mkdir(parents=True, exist_ok=True)
        self.timing_file = (output / "topic_timing.csv").open("w", newline="", encoding="utf-8", buffering=1)
        self.timing_writer = csv.writer(self.timing_file)
        self.timing_writer.writerow([
            "topic", "monitored_topic", "message_type", "source_device",
            "source_timestamp_ns", "header_timestamp_ns", "receive_system_timestamp_ns",
            "receive_steady_timestamp_ns", "sequence", "frame_id",
        ])
        self.resource_file = (output / "resource_usage.csv").open("w", newline="", encoding="utf-8", buffering=1)
        self.resource_writer = csv.writer(self.resource_file)
        self.resource_writer.writerow([
            "system_timestamp_ns", "steady_timestamp_ns", "elapsed_sec", "cpu_percent",
            "memory_total_bytes", "memory_available_bytes", "disk_total_bytes",
            "disk_free_bytes", "matched_process_count", "matched_process_rss_bytes", "process_details",
        ])
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        for spec in config.get("topics", []):
            state = TopicState(spec)
            self.states[spec["topic"]] = state
            message_type = get_message(spec["message_type"])
            callback = self._make_raw_callback(state, message_type)
            self._field_subscriptions.append(self.create_subscription(
                message_type, spec.get("monitored_topic", spec["topic"]), callback, qos, raw=True))
        diag_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self._field_subscriptions.append(self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostic_callback, diag_qos))
        period = float(config.get("resources", {}).get("sample_period_sec", 2.0))
        self._previous_cpu = _proc_cpu_snapshot()
        self._resource_timer = self.create_timer(period, self._resource_tick)
        self._stop_timer = self.create_timer(0.25, self._check_duration)

    def _make_raw_callback(self, state: TopicState, message_type: Any):
        fields = list(state.spec.get("deserialize_fields", []))
        source_from_header = bool(state.spec.get("source_timestamp_from_header", False))
        def callback(raw: bytes) -> None:
            receive_system_ns = time.time_ns()
            receive_steady_ns = time.monotonic_ns()
            header_ns, frame_id = parse_cdr_header(raw)
            sequence = None
            if fields:
                try:
                    message = deserialize_message(raw, message_type)
                    sequence = int(getattr(message, fields[0]))
                except (AttributeError, TypeError, ValueError):
                    sequence = None
            state.update(header_ns, frame_id, receive_system_ns, receive_steady_ns, sequence)
            self.timing_writer.writerow([
                state.spec["topic"], state.spec.get("monitored_topic", state.spec["topic"]),
                state.spec["message_type"], state.spec["source_device"],
                header_ns if source_from_header and header_ns is not None else "",
                header_ns if header_ns is not None else "", receive_system_ns,
                receive_steady_ns, sequence if sequence is not None else "", frame_id,
            ])
        return callback

    def _diagnostic_callback(self, message: DiagnosticArray) -> None:
        received = time.time_ns()
        for status in message.status:
            level = status.level
            if isinstance(level, (bytes, bytearray)):
                level = level[0] if level else 0
            self.diagnostics[status.name] = {
                "level": int(level), "message": status.message,
                "hardware_id": status.hardware_id,
                "values": {item.key: item.value for item in status.values},
                "receive_system_timestamp_ns": received,
            }

    def _resource_tick(self) -> None:
        now_system = time.time_ns()
        now_steady = time.monotonic_ns()
        current_total, current_idle = _proc_cpu_snapshot()
        old_total, old_idle = self._previous_cpu
        delta = current_total - old_total
        cpu = 100.0 * (1.0 - (current_idle - old_idle) / delta) if delta > 0 else 0.0
        self._previous_cpu = current_total, current_idle
        memory_total, memory_available = _memory_available()
        root = Path(self.config.get("resources", {}).get("data_root", "/media/mcap_data"))
        stat = os.statvfs(root)
        patterns = list(self.config.get("resources", {}).get("process_patterns", []))
        process_count, rss, details = _process_rows(patterns)
        elapsed_sec = (now_steady - self.started_steady_ns) / 1e9
        disk_free = stat.f_bavail * stat.f_frsize
        self.resource_samples.append({
            "elapsed_sec": elapsed_sec,
            "cpu_percent": cpu,
            "memory_available_bytes": memory_available,
            "disk_free_bytes": disk_free,
            "matched_process_count": process_count,
            "matched_process_rss_bytes": rss,
        })
        self.resource_writer.writerow([
            now_system, now_steady, elapsed_sec, cpu,
            memory_total, memory_available, stat.f_blocks * stat.f_frsize,
            disk_free, process_count, rss, details,
        ])

    def _check_duration(self) -> None:
        if (time.monotonic_ns() - self.started_steady_ns) / 1e9 >= self.duration_sec:
            # Shutting the context down from inside an executor callback can
            # deadlock in Humble. Let the outer spin_once loop stop cleanly.
            self.done = True

    def finish(self) -> dict[str, Any]:
        ended_system_ns = time.time_ns()
        ended_steady_ns = time.monotonic_ns()
        self.timing_file.flush()
        self.resource_file.flush()
        os.fsync(self.timing_file.fileno())
        os.fsync(self.resource_file.fileno())
        self.timing_file.close()
        self.resource_file.close()
        observation_sec = (ended_steady_ns - self.started_steady_ns) / 1e9
        topics = {
            topic: state.report(observation_sec, ended_steady_ns)
            for topic, state in self.states.items()
        }
        cpu_values = [float(row["cpu_percent"]) for row in self.resource_samples]
        rss_values = [int(row["matched_process_rss_bytes"]) for row in self.resource_samples]
        memory_values = [int(row["memory_available_bytes"]) for row in self.resource_samples]
        disk_values = [int(row["disk_free_bytes"]) for row in self.resource_samples]
        elapsed_values = [float(row["elapsed_sec"]) for row in self.resource_samples]
        rss_slope = None
        if len(rss_values) > 1 and elapsed_values[-1] > elapsed_values[0]:
            rss_slope = (rss_values[-1] - rss_values[0]) / (elapsed_values[-1] - elapsed_values[0])
        resource_summary = {
            "sample_count": len(self.resource_samples),
            "cpu_percent_p50": percentile(cpu_values, 0.50),
            "cpu_percent_p95": percentile(cpu_values, 0.95),
            "cpu_percent_max": max(cpu_values) if cpu_values else None,
            "memory_available_min_bytes": min(memory_values) if memory_values else None,
            "disk_free_min_bytes": min(disk_values) if disk_values else None,
            "disk_free_change_bytes": disk_values[-1] - disk_values[0] if len(disk_values) > 1 else None,
            "matched_process_rss_first_bytes": rss_values[0] if rss_values else None,
            "matched_process_rss_last_bytes": rss_values[-1] if rss_values else None,
            "matched_process_rss_max_bytes": max(rss_values) if rss_values else None,
            "matched_process_rss_change_bytes": rss_values[-1] - rss_values[0] if len(rss_values) > 1 else None,
            "matched_process_rss_endpoint_slope_bytes_per_sec": rss_slope,
            "trend_note": "Endpoint RSS slope is observational; it is not by itself proof of a leak.",
        }
        return {
            "schema_version": "tank_stability_report_v1",
            "started_system_timestamp_ns": self.started_system_ns,
            "ended_system_timestamp_ns": ended_system_ns,
            "actual_duration_sec": observation_sec,
            "requested_duration_sec": self.duration_sec,
            "stop_reason": self.stop_reason,
            "topics": topics,
            "diagnostic_status": self.diagnostics,
            "resource_summary": resource_summary,
            "pass": all(item["received_count"] > 1 and item["timestamp_regression_count"] == 0
                        and item["rate_pass"] is not False
                        for item in topics.values() if item["expected_min_hz"] > 0),
            "notes": [
                "Camera rates use same-stamp CameraInfo companions to avoid full-image copies.",
                "A missing message sequence cannot be interpreted as a measured source frame drop.",
                "Preview throttling/drops are not sensor acquisition failures.",
            ],
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration-sec", type=float, required=True)
    args = parser.parse_args(argv)
    if args.duration_sec <= 0:
        parser.error("--duration-sec must be positive")
    config = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    output = Path(args.output).resolve()
    rclpy.init(args=None)
    node = StabilityMonitor(config, output, args.duration_sec)
    def interrupted(_signum, _frame):
        node.stop_reason = "signal"
        if rclpy.ok():
            rclpy.shutdown()
    signal.signal(signal.SIGTERM, interrupted)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        node.stop_reason = "keyboard_interrupt"
    finally:
        report = node.finish()
        (output / "stability_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if report["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
