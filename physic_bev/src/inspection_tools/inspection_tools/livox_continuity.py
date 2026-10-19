#!/usr/bin/env python3
"""Low-overhead independent Livox observer and three-way classifier."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import time
from typing import Any

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2


def _status_values(status: Any) -> dict[str, str]:
    return {item.key: item.value for item in status.values}


def classify_livox_continuity(
    monitor_topics: dict[str, dict[str, Any]], probe: dict[str, Any], stale_sec: float = 2.0,
) -> dict[str, Any]:
    diagnostics = probe.get("diagnostics", {})
    publisher = diagnostics.get("livox_ros_driver2/publisher", {})
    adapter = diagnostics.get("inspection_livox_adapter/imu_units", {})
    publisher_class = publisher.get("values", {}).get("classification", publisher.get("message", ""))
    adapter_class = adapter.get("values", {}).get("classification", adapter.get("message", ""))
    probe_topics = probe.get("topics", {})
    names = ("/livox/lidar", "/livox/imu")
    monitor_lags = [
        float(monitor_topics.get(name, {}).get("end_of_window_lag_sec", float("inf")))
        for name in names
    ]
    probe_lags = [
        float(probe_topics.get(name, {}).get("end_of_window_lag_sec", float("inf")))
        for name in names
    ]
    if "SOURCE_STOPPED" in publisher_class or "SOURCE_STOPPED" in adapter_class:
        classification = "SOURCE_STOPPED"
        reason = "publisher-side or adapter-input heartbeat reports stale source data"
    elif max(probe_lags) <= stale_sec and max(monitor_lags) > stale_sec:
        classification = "MONITOR_STALLED"
        reason = "independent subscriber remained current while stability monitor became stale"
    elif "SOURCE_STREAMING" in publisher_class and min(probe_lags) > stale_sec:
        classification = "DDS_DELIVERY_INTERRUPTED"
        reason = "publisher heartbeat remained current while both independent data paths became stale"
    elif max(probe_lags) <= stale_sec and max(monitor_lags) <= stale_sec:
        classification = "SOURCE_STREAMING"
        reason = "publisher, independent subscriber, and stability monitor reached the end window"
    else:
        classification = "UNKNOWN"
        reason = "available observations do not uniquely identify the failed layer"
    return {
        "classification": classification,
        "reason": reason,
        "stale_threshold_sec": stale_sec,
        "publisher_classification": publisher_class or "UNKNOWN",
        "adapter_classification": adapter_class or "UNKNOWN",
        "independent_probe_end_lag_sec": dict(zip(("lidar", "imu"), probe_lags)),
        "stability_monitor_end_lag_sec": dict(zip(("lidar", "imu"), monitor_lags)),
    }


class LivoxContinuityProbe(Node):
    def __init__(self, output: Path, duration_sec: float) -> None:
        super().__init__("livox_continuity_probe")
        self.output = output
        self.duration_sec = duration_sec
        self.started_steady_ns = time.monotonic_ns()
        self.started_system_ns = time.time_ns()
        self.done = False
        # The adapter diagnostic already reports its /livox/imu_raw input
        # count and age.  A second Python subscription at 200 Hz adds load but
        # no independent observation layer, so observe only public ROS output.
        self.counts = {name: 0 for name in ("/livox/lidar", "/livox/imu")}
        self.first_ns: dict[str, int] = {}
        self.last_ns: dict[str, int] = {}
        self.diagnostics: dict[str, dict[str, Any]] = {}
        self.heartbeat_count = 0
        output.mkdir(parents=True, exist_ok=True)
        self.timeline_file = (output / "livox_observer.csv").open(
            "w", newline="", encoding="utf-8", buffering=1)
        self.timeline = csv.writer(self.timeline_file)
        self.timeline.writerow([
            "system_timestamp_ns", "steady_timestamp_ns", "observer", "message",
            "lidar_publish_count", "imu_publish_count", "adapter_received_count",
            "source_age_sec",
        ])
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        # rclpy.Node exposes ``subscriptions`` as a read-only property.
        # Retain our handles under a private name so they stay alive.
        self._subscriptions = [
            self.create_subscription(PointCloud2, "/livox/lidar", self._count("/livox/lidar"), qos, raw=True),
            self.create_subscription(Imu, "/livox/imu", self._count("/livox/imu"), qos, raw=True),
            self.create_subscription(DiagnosticArray, "/diagnostics", self._diagnostic, 10),
        ]
        self.timer = self.create_timer(1.0, self._tick)

    def _count(self, topic: str):
        def callback(_raw: bytes) -> None:
            now = time.monotonic_ns()
            self.counts[topic] += 1
            self.first_ns.setdefault(topic, now)
            self.last_ns[topic] = now
        return callback

    def _diagnostic(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name not in {
                "livox_ros_driver2/publisher", "inspection_livox_adapter/imu_units"
            }:
                continue
            level = status.level[0] if isinstance(status.level, bytes) else status.level
            self.diagnostics[status.name] = {
                "level": int(level), "message": status.message,
                "values": _status_values(status),
                "receive_system_timestamp_ns": time.time_ns(),
            }

    def _tick(self) -> None:
        now_system = time.time_ns()
        now_steady = time.monotonic_ns()
        self.heartbeat_count += 1
        for name, item in self.diagnostics.items():
            values = item["values"]
            self.timeline.writerow([
                now_system, now_steady, name, item["message"],
                values.get("lidar_publish_count", ""), values.get("imu_publish_count", ""),
                values.get("received_count", ""),
                values.get("source_last_receive_age_sec", values.get("imu_last_publish_age_sec", "")),
            ])
        if now_steady - self.started_steady_ns >= int(self.duration_sec * 1e9):
            self.done = True

    def report(self) -> dict[str, Any]:
        ended_steady_ns = time.monotonic_ns()
        duration = (ended_steady_ns - self.started_steady_ns) / 1e9
        topics = {}
        for topic, count in self.counts.items():
            first = self.first_ns.get(topic)
            last = self.last_ns.get(topic)
            active = (last - first) / 1e9 if first is not None and last is not None else 0.0
            topics[topic] = {
                "received_count": count,
                "actual_rate_hz": count / duration if duration else 0.0,
                "active_span_rate_hz": (count - 1) / active if count > 1 and active > 0 else 0.0,
                "end_of_window_lag_sec": (ended_steady_ns - last) / 1e9 if last else duration,
            }
        return {
            "schema_version": "livox_continuity_probe_v1",
            "started_system_timestamp_ns": self.started_system_ns,
            "actual_duration_sec": duration,
            "heartbeat_count": self.heartbeat_count,
            "topics": topics,
            "diagnostics": self.diagnostics,
            "overhead_note": (
                "Callbacks only increment counters; payloads are not deserialized or written. "
                "The adapter diagnostic supplies /livox/imu_raw input count/age."
            ),
        }


def observe(args: argparse.Namespace) -> int:
    rclpy.init(args=None)
    output = Path(args.output).resolve()
    node = LivoxContinuityProbe(output, args.duration_sec)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        report = node.report()
        node.timeline_file.close()
        (output / "livox_probe_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def classify(args: argparse.Namespace) -> int:
    monitor = json.loads(Path(args.monitor_report).read_text(encoding="utf-8"))
    probe = json.loads(Path(args.probe_report).read_text(encoding="utf-8"))
    result = classify_livox_continuity(monitor.get("topics", {}), probe, args.stale_sec)
    Path(args.output).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0 if result["classification"] == "SOURCE_STREAMING" else 2


def _interface_stats(interface: str) -> dict[str, int]:
    root = Path("/sys/class/net") / interface / "statistics"
    return {
        name: int((root / name).read_text(encoding="ascii").strip())
        for name in ("rx_bytes", "rx_packets", "rx_errors", "rx_dropped")
    }


def observe_network(args: argparse.Namespace) -> int:
    """Record the device-facing NIC independently of ROS and DDS."""
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    started_system_ns = time.time_ns()
    started_steady_ns = time.monotonic_ns()
    first = _interface_stats(args.interface)
    last = first
    samples = 0
    with (output / "livox_network.csv").open(
        "w", newline="", encoding="utf-8", buffering=1
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "system_timestamp_ns", "steady_timestamp_ns", "interface",
            "rx_bytes", "rx_packets", "rx_errors", "rx_dropped",
        ])
        while (time.monotonic_ns() - started_steady_ns) / 1e9 < args.duration_sec:
            last = _interface_stats(args.interface)
            writer.writerow([time.time_ns(), time.monotonic_ns(), args.interface,
                             last["rx_bytes"], last["rx_packets"],
                             last["rx_errors"], last["rx_dropped"]])
            samples += 1
            time.sleep(min(1.0, args.duration_sec))
    report = {
        "schema_version": "livox_network_continuity_v1",
        "interface": args.interface,
        "started_system_timestamp_ns": started_system_ns,
        "actual_duration_sec": (time.monotonic_ns() - started_steady_ns) / 1e9,
        "sample_count": samples,
        "first": first,
        "last": last,
        "delta": {key: last[key] - first[key] for key in first},
        "semantics": "kernel NIC receive counters; independent of Livox SDK, ROS publisher and DDS",
    }
    (output / "livox_network_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    observe_parser = sub.add_parser("observe")
    observe_parser.add_argument("--output", required=True)
    observe_parser.add_argument("--duration-sec", required=True, type=float)
    observe_parser.set_defaults(function=observe)
    network_parser = sub.add_parser("network")
    network_parser.add_argument("--output", required=True)
    network_parser.add_argument("--duration-sec", required=True, type=float)
    network_parser.add_argument("--interface", default="eth0")
    network_parser.set_defaults(function=observe_network)
    classify_parser = sub.add_parser("classify")
    classify_parser.add_argument("--monitor-report", required=True)
    classify_parser.add_argument("--probe-report", required=True)
    classify_parser.add_argument("--output", required=True)
    classify_parser.add_argument("--stale-sec", type=float, default=2.0)
    classify_parser.set_defaults(function=classify)
    args = parser.parse_args(argv)
    return int(args.function(args))


if __name__ == "__main__":
    raise SystemExit(main())
