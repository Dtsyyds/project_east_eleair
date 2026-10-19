#!/usr/bin/env python3
"""Bounded live audit for FIELD-W6 mapping topics and their offered QoS."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import atomic_json, atomic_write_text, utc_now  # noqa: E402


EXPECTED = {
    "/cloud_registered": "sensor_msgs/msg/PointCloud2",
    "/path": "nav_msgs/msg/Path",
    "/aft_mapped_to_init": "nav_msgs/msg/Odometry",
}

DEPTH = {"/cloud_registered": 100, "/path": 10, "/aft_mapped_to_init": 10}


def _fastlivo_snapshot() -> dict[str, Any] | None:
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        try:
            cmdline = proc.joinpath("cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
            if "fastlivo_mapping" not in cmdline:
                continue
            stat = proc.joinpath("stat").read_text().rsplit(")", 1)[1].split()
            status = proc.joinpath("status").read_text()
            rss_line = next((line for line in status.splitlines() if line.startswith("VmRSS:")), "")
            rss_bytes = int(rss_line.split()[1]) * 1024 if rss_line else None
            return {"pid": int(proc.name), "cmdline": cmdline.strip(),
                    "cpu_ticks": int(stat[11]) + int(stat[12]), "rss_bytes": rss_bytes}
        except (OSError, ValueError, IndexError):
            continue
    return None


def _temperatures() -> list[dict[str, Any]]:
    values = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            values.append({"path": str(path), "celsius": int(path.read_text()) / 1000.0})
        except (OSError, ValueError):
            pass
    return values


def _enum_name(value: Any) -> str:
    return str(getattr(value, "name", value)).split(".")[-1].upper()


def _stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else None


def _message_size(message: Any) -> int:
    if hasattr(message, "data") and hasattr(message, "point_step"):
        return len(message.data) + 256 + 48 * len(getattr(message, "fields", []))
    return len(serialize_message(message))


class Probe(Node):
    def __init__(self, seconds: float) -> None:
        super().__init__("field_w6_mapping_live_probe")
        self.seconds = seconds
        self.started: float | None = None
        self.stats: dict[str, dict[str, Any]] = {}
        self._field_subscriptions = []

    def _graph_snapshot(self) -> dict[str, tuple[list[str], list[Any]]]:
        names_and_types = dict(self.get_topic_names_and_types())
        return {
            topic: (names_and_types.get(topic, []), self.get_publishers_info_by_topic(topic))
            for topic in EXPECTED
        }

    def discover_and_configure(self, timeout_sec: float) -> tuple[bool, float]:
        """Wait for DDS discovery before deciding that a required publisher is absent."""
        discovery_started = time.monotonic()
        deadline = discovery_started + timeout_sec
        graph = self._graph_snapshot()
        laser_mapping_publishers = self._laser_mapping_publishers()
        while rclpy.ok() and not (
                _graph_ready(graph) and _mapping_node_ready(laser_mapping_publishers)):
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
            graph = self._graph_snapshot()
            laser_mapping_publishers = self._laser_mapping_publishers()
        self._configure(graph, laser_mapping_publishers)
        self.started = time.monotonic()
        ready = _graph_ready(graph) and _mapping_node_ready(laser_mapping_publishers)
        return ready, self.started - discovery_started

    def _laser_mapping_publishers(self) -> dict[str, list[str]]:
        try:
            return dict(
                self.get_publisher_names_and_types_by_node("laserMapping", "/")
            )
        except RuntimeError:
            return {}

    def _configure(
            self, graph: dict[str, tuple[list[str], list[Any]]],
            laser_mapping_publishers: dict[str, list[str]]) -> None:
        for topic, expected in EXPECTED.items():
            types, offered = graph[topic]
            entry = {
                "expected_type": expected,
                "types": types,
                "publisher_count": len(offered),
                "laser_mapping_node_declares_topic": (
                    laser_mapping_publishers.get(topic) == [expected]
                ),
                "publishers": [],
                "count": 0,
                "first_receive_wall_ns": None,
                "last_receive_wall_ns": None,
                "first_receive_monotonic_ns": None,
                "last_receive_monotonic_ns": None,
                "first_header_stamp_ns": None,
                "last_header_stamp_ns": None,
                "header_regressions": 0,
                "header_stamp_changes": 0,
                "interval_ns": [],
                "message_sizes": [],
                "path_size_samples": [],
                "last_deserialize_monotonic_ns": None,
                "frame_ids": [],
                "child_frame_ids": [],
                "path_pose_counts": [],
            }
            for endpoint in offered:
                qos = endpoint.qos_profile
                entry["publishers"].append({
                    "node_name": endpoint.node_name,
                    "node_namespace": endpoint.node_namespace,
                    "topic_type": endpoint.topic_type,
                    "endpoint_gid": bytes(endpoint.endpoint_gid).hex(),
                    "qos": {
                        "reliability": _enum_name(qos.reliability),
                        "durability": _enum_name(qos.durability),
                        "history": _enum_name(qos.history),
                        "depth": int(qos.depth),
                    },
                })
            self.stats[topic] = entry
            if len(types) != 1 or types[0] != expected or not offered:
                continue
            qos = offered[0].qos_profile
            profile = QoSProfile(
                history=(qos.history if _enum_name(qos.history) in {"KEEP_LAST", "KEEP_ALL"}
                         else HistoryPolicy.KEEP_LAST),
                depth=max(1, int(qos.depth or DEPTH[topic])),
                reliability=qos.reliability, durability=qos.durability,
            )
            message_type = get_message(expected)
            self._field_subscriptions.append(self.create_subscription(
                message_type, topic,
                lambda serialized, current=topic, current_type=message_type:
                    self._update_raw(current, serialized, current_type),
                profile, raw=True,
            ))

    def _update_raw(self, topic: str, serialized: bytes, message_type: Any) -> None:
        item = self.stats[topic]
        wall = time.time_ns()
        mono = time.monotonic_ns()
        if item["last_receive_monotonic_ns"] is not None:
            item["interval_ns"].append(mono - item["last_receive_monotonic_ns"])
        item["count"] += 1
        item["first_receive_wall_ns"] = item["first_receive_wall_ns"] or wall
        item["last_receive_wall_ns"] = wall
        item["first_receive_monotonic_ns"] = item["first_receive_monotonic_ns"] or mono
        item["last_receive_monotonic_ns"] = mono
        item["message_sizes"].append(len(serialized))
        last_sample = item["last_deserialize_monotonic_ns"]
        if last_sample is not None and mono - last_sample < 1_000_000_000:
            return
        item["last_deserialize_monotonic_ns"] = mono
        message = deserialize_message(serialized, message_type)
        stamp = _stamp_ns(message)
        if stamp is not None and item["last_header_stamp_ns"] is not None \
                and stamp < item["last_header_stamp_ns"]:
            item["header_regressions"] += 1
        if stamp is not None and item["last_header_stamp_ns"] is not None \
                and stamp != item["last_header_stamp_ns"]:
            item["header_stamp_changes"] += 1
        item["first_header_stamp_ns"] = item["first_header_stamp_ns"] or stamp
        item["last_header_stamp_ns"] = stamp
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        if frame_id and frame_id not in item["frame_ids"]:
            item["frame_ids"].append(frame_id)
        child = str(getattr(message, "child_frame_id", ""))
        if child and child not in item["child_frame_ids"]:
            item["child_frame_ids"].append(child)
        if topic == "/path":
            item["path_pose_counts"].append(len(message.poses))
            item["path_size_samples"].append(len(serialized))

    def done(self) -> bool:
        return self.started is not None and time.monotonic() - self.started >= self.seconds


def _graph_ready(graph: dict[str, tuple[list[str], list[Any]]]) -> bool:
    return all(
        graph.get(topic, ([], []))[0] == [expected]
        and len(graph.get(topic, ([], []))[1]) == 1
        for topic, expected in EXPECTED.items()
    )


def _mapping_node_ready(publishers: dict[str, list[str]]) -> bool:
    return all(publishers.get(topic) == [expected] for topic, expected in EXPECTED.items())


def _publisher_identity_verified(item: dict[str, Any]) -> bool:
    names = [publisher.get("node_name") for publisher in item.get("publishers", [])]
    if "laserMapping" in names:
        return True
    return bool(names) and all(name == "_NODE_NAME_UNKNOWN_" for name in names) \
        and bool(item.get("laser_mapping_node_declares_topic"))


def _finish(raw: dict[str, dict[str, Any]]) -> dict[str, Any]:
    now = time.time_ns()
    output: dict[str, Any] = {}
    for topic, item in raw.items():
        intervals = item.pop("interval_ns")
        sizes = item.pop("message_sizes")
        path_sizes = item.pop("path_size_samples")
        item.pop("last_deserialize_monotonic_ns")
        elapsed = ((item["last_receive_monotonic_ns"] - item["first_receive_monotonic_ns"]) / 1e9
                   if item["count"] > 1 else 0.0)
        item.update({
            "receive_hz": (item["count"] - 1) / elapsed if elapsed > 0 else 0.0,
            "max_interval_sec": max(intervals) / 1e9 if intervals else None,
            "last_message_age_sec": ((now - item["last_receive_wall_ns"]) / 1e9
                                      if item["last_receive_wall_ns"] else None),
            "average_message_bytes": sum(sizes) / len(sizes) if sizes else None,
            "max_message_bytes": max(sizes) if sizes else None,
            "message_size_samples": path_sizes if topic == "/path" else [],
            "header_stamp_span_sec": (
                (item["last_header_stamp_ns"] - item["first_header_stamp_ns"]) / 1e9
                if item["last_header_stamp_ns"] is not None
                and item["first_header_stamp_ns"] is not None else None
            ),
        })
        output[topic] = item
    return output


def _qos_yaml(topics: dict[str, Any]) -> str:
    document = {}
    for topic, item in topics.items():
        if not item["publishers"]:
            continue
        qos = item["publishers"][0]["qos"]
        history = qos["history"] if qos["history"] in {"KEEP_LAST", "KEEP_ALL"} else "KEEP_LAST"
        document[topic] = {
            "history": history.lower(),
            "depth": max(1, int(qos["depth"] or DEPTH[topic])),
            "reliability": qos["reliability"].lower(),
            "durability": qos["durability"].lower(),
        }
    document["/eddy/channel_mean"] = {
        "history": "keep_last", "depth": 100,
        "reliability": "best_effort", "durability": "volatile",
    }
    document["/eddy/data"] = {
        "history": "keep_last", "depth": 100,
        "reliability": "best_effort", "durability": "volatile",
    }
    document["/eddy/status"] = {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    }
    document["/camera/front/camera_info"] = {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    }
    document["/diagnostics"] = {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    }
    document["/field/collection_status"] = {
        "history": "keep_last", "depth": 10,
        "reliability": "reliable", "durability": "volatile",
    }
    document["/tf"] = {
        "history": "keep_last", "depth": 100,
        "reliability": "reliable", "durability": "volatile",
    }
    document["/tf_static"] = {
        "history": "keep_all",
        "reliability": "reliable", "durability": "transient_local",
    }
    return yaml.safe_dump(document, sort_keys=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--qos-output", type=Path, required=True)
    parser.add_argument("--path-output", type=Path, required=True)
    parser.add_argument("--performance-output", type=Path)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--discovery-timeout", type=float, default=15.0)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = Probe(args.seconds)
    discovered, discovery_seconds = node.discover_and_configure(args.discovery_timeout)
    process_before = _fastlivo_snapshot()
    probe_started = time.monotonic()
    try:
        while discovered and rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        topics = _finish(node.stats)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    errors = []
    for topic, expected in EXPECTED.items():
        item = topics[topic]
        if item["types"] != [expected]:
            errors.append(f"{topic} type={item['types']!r}, expected {expected}")
        if item["publisher_count"] != 1:
            errors.append(f"{topic} publisher_count={item['publisher_count']}, expected exactly 1")
        if item["publishers"] and not _publisher_identity_verified(item):
            errors.append(f"{topic} is not published by laserMapping")
        if item["count"] < 1:
            errors.append(f"{topic} delivered no message during probe")
        if item["header_regressions"]:
            errors.append(f"{topic} header timestamp regressed")
        if item["last_message_age_sec"] is not None and item["last_message_age_sec"] > 2.0:
            errors.append(f"{topic} last message is stale")
    report = {
        "schema_version": "field_w6_mapping_topic_inventory_v1",
        "created_at": utc_now(), "probe_seconds": args.seconds,
        "discovery_timeout_sec": args.discovery_timeout,
        "discovery_elapsed_sec": discovery_seconds,
        "discovery_complete": discovered,
        "valid": not errors, "errors": errors, "topics": topics,
    }
    atomic_json(args.output, report)
    atomic_write_text(args.qos_output, _qos_yaml(topics))
    path = topics["/path"]
    counts = path.get("path_pose_counts", [])
    sizes = path.get("message_size_samples", [])
    growing = len(counts) > 1 and counts[-1] > counts[0]
    rate = float(path.get("receive_hz") or 0.0)
    bytes_per_pose = None
    pose_growth_hz = None
    if growing and len(sizes) == len(counts):
        bytes_per_pose = max(0.0, (sizes[-1] - sizes[0]) / (counts[-1] - counts[0]))
        receive_span = (
            path["last_receive_monotonic_ns"] - path["first_receive_monotonic_ns"]
        ) / 1e9
        if receive_span > 0:
            pose_growth_hz = (counts[-1] - counts[0]) / receive_span
    projections = {}
    if growing and rate > 0 and sizes and pose_growth_hz is not None \
            and bytes_per_pose is not None:
        current_size = float(sizes[-1])
        for minutes in (10, 30, 60):
            horizon = minutes * 60.0
            projected_bytes = rate * (
                current_size * horizon
                + 0.5 * bytes_per_pose * pose_growth_hz * horizon * horizon
            )
            projections[f"{minutes}_minutes"] = {
                "estimated_messages": rate * horizon,
                "estimated_last_message_bytes": (
                    current_size + bytes_per_pose * pose_growth_hz * horizon
                ),
                "estimated_recording_bytes": projected_bytes,
                "estimated_recording_GB_decimal": projected_bytes / 1e9,
            }
    atomic_json(args.path_output, {
        "schema_version": "field_w6_path_growth_analysis_v1",
        "created_at": utc_now(), "observed": bool(counts),
        "pose_counts": counts, "growing_cumulative_path": growing,
        "average_message_bytes": path.get("average_message_bytes"),
        "max_message_bytes": path.get("max_message_bytes"),
        "receive_hz": rate, "pose_growth_hz": pose_growth_hz,
        "estimated_bytes_per_pose": bytes_per_pose,
        "header_stamp_changes": path.get("header_stamp_changes"),
        "header_stamp_span_sec": path.get("header_stamp_span_sec"),
        "projected_raw_path_storage": projections,
        "conclusion": "B" if growing else "insufficient_live_samples",
        "policy": "observe /path online but do not record it; record only /aft_mapped_to_init",
        "recorded_mapping_topics": ["/aft_mapped_to_init"],
        "offline_alternative": "reconstruct reports/compact_trajectory.jsonl from /aft_mapped_to_init",
    })
    process_after = _fastlivo_snapshot()
    elapsed = max(1e-9, time.monotonic() - probe_started)
    cpu_percent = None
    if process_before and process_after and process_before["pid"] == process_after["pid"]:
        cpu_percent = ((process_after["cpu_ticks"] - process_before["cpu_ticks"])
                       / os.sysconf(os.sysconf_names["SC_CLK_TCK"]) / elapsed * 100.0)
    performance = {
        "schema_version": "field_w6_baseline_performance_v1",
        "created_at": utc_now(), "executed": True, "duration_sec": elapsed,
        "mapping_topics": {name: {key: value.get(key) for key in (
            "count", "receive_hz", "max_interval_sec", "average_message_bytes",
            "max_message_bytes") } for name, value in topics.items()},
        "fast_livo": {"before": process_before, "after": process_after,
                      "cpu_percent_one_core_basis": cpu_percent},
        "temperatures": _temperatures(),
        "ssd_MBps": 0.0,
        "note": "mapping-only baseline; no collection writer was started",
        "valid": not errors and process_after is not None,
    }
    if args.performance_output:
        atomic_json(args.performance_output, performance)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if not errors else 2


if __name__ == "__main__":
    raise SystemExit(main())
