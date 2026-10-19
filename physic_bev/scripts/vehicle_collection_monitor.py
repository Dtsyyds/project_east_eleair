#!/usr/bin/env python3
"""Low-rate runtime status publisher and resource/disk monitor for FIELD-W5."""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
from pathlib import Path
import statistics
import sys
import time
from typing import Any

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import (  # noqa: E402
    atomic_json, directory_bytes, load_json, process_matches, utc_now,
)


def _percentile(values: list[float], q: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    low = int(position)
    high = min(low + 1, len(ordered) - 1)
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def _header_stamp_ns(message: Any) -> int | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return value if value > 0 else None


class TopicStats:
    def __init__(self, type_name: str) -> None:
        self.type_name = type_name
        self.count = 0
        self.first_monotonic_ns: int | None = None
        self.last_monotonic_ns: int | None = None
        self.last_header_ns: int | None = None
        self.header_regressions = 0
        self.intervals_ms: deque[float] = deque(maxlen=6000)
        self.latency_ms: deque[float] = deque(maxlen=6000)
        self.message_bytes: deque[int] = deque(maxlen=6000)
        self.path_pose_count: int | None = None
        self.device_status: dict[str, Any] | None = None
        self.last_raw_deserialize_monotonic_ns: int | None = None

    def update(self, message: Any) -> None:
        now_mono = time.monotonic_ns()
        now_wall = time.time_ns()
        stamp = _header_stamp_ns(message)
        self.count += 1
        if self.first_monotonic_ns is None:
            self.first_monotonic_ns = now_mono
        if self.last_monotonic_ns is not None:
            self.intervals_ms.append((now_mono - self.last_monotonic_ns) / 1e6)
        self.last_monotonic_ns = now_mono
        if stamp is not None:
            if self.last_header_ns is not None and stamp < self.last_header_ns:
                self.header_regressions += 1
            self.last_header_ns = stamp
            self.latency_ms.append((now_wall - stamp) / 1e6)
        if hasattr(message, "point_step") and hasattr(message, "data"):
            self.message_bytes.append(len(message.data) + 256)
        elif hasattr(message, "poses"):
            self.path_pose_count = len(message.poses)
            self.message_bytes.append(128 + len(message.poses) * 96)
        if self.type_name == "eddy_driver/msg/EddyStatus":
            self.device_status = {
                "connected": bool(message.connected),
                "received_frames": int(message.received_frames),
                "dropped_frames": int(message.dropped_frames),
                "queue_depth": int(message.queue_depth),
                "last_status_is_error": bool(message.last_status_is_error),
            }

    def update_raw(self, serialized: bytes, message_type: Any) -> None:
        """Count every sample but deserialize large messages at most once per second."""
        now_mono = time.monotonic_ns()
        self.count += 1
        if self.first_monotonic_ns is None:
            self.first_monotonic_ns = now_mono
        if self.last_monotonic_ns is not None:
            self.intervals_ms.append((now_mono - self.last_monotonic_ns) / 1e6)
        self.last_monotonic_ns = now_mono
        self.message_bytes.append(len(serialized))
        if self.last_raw_deserialize_monotonic_ns is not None \
                and now_mono - self.last_raw_deserialize_monotonic_ns < 1_000_000_000:
            return
        self.last_raw_deserialize_monotonic_ns = now_mono
        message = deserialize_message(serialized, message_type)
        stamp = _header_stamp_ns(message)
        if stamp is not None:
            if self.last_header_ns is not None and stamp < self.last_header_ns:
                self.header_regressions += 1
            self.last_header_ns = stamp
            self.latency_ms.append((time.time_ns() - stamp) / 1e6)
        if hasattr(message, "poses"):
            self.path_pose_count = len(message.poses)

    def document(self) -> dict[str, Any]:
        elapsed = (
            (self.last_monotonic_ns - self.first_monotonic_ns) / 1e9
            if self.last_monotonic_ns and self.first_monotonic_ns
            and self.last_monotonic_ns > self.first_monotonic_ns else 0.0
        )
        intervals = list(self.intervals_ms)
        latencies = list(self.latency_ms)
        sizes = list(self.message_bytes)
        return {
            "type": self.type_name,
            "count": self.count,
            "receive_hz": (self.count - 1) / elapsed if elapsed > 0 else None,
            "interval_ms": {
                "p50": _percentile(intervals, 0.50),
                "p95": _percentile(intervals, 0.95),
                "max": max(intervals) if intervals else None,
            },
            "header_to_host_receive_ms": {
                "p50": _percentile(latencies, 0.50),
                "p95": _percentile(latencies, 0.95),
                "max": max(latencies) if latencies else None,
            },
            "header_stamp_regressions": self.header_regressions,
            "last_message_age_sec": (
                (time.monotonic_ns() - self.last_monotonic_ns) / 1e9
                if self.last_monotonic_ns is not None else None
            ),
            "average_message_bytes": statistics.mean(sizes) if sizes else None,
            "max_message_bytes": max(sizes) if sizes else None,
            "path_pose_count": self.path_pose_count,
            "device_status": self.device_status,
        }


def _cpu_snapshot() -> dict[str, tuple[int, int]]:
    result: dict[str, tuple[int, int]] = {}
    try:
        lines = Path("/proc/stat").read_text().splitlines()
    except OSError:
        return result
    for line in lines:
        fields = line.split()
        if not fields or not fields[0].startswith("cpu"):
            continue
        values = [int(value) for value in fields[1:]]
        idle = sum(values[3:5])
        result[fields[0]] = (sum(values), idle)
    return result


def _cpu_percent(previous: dict[str, tuple[int, int]], current: dict[str, tuple[int, int]]) -> dict[str, float]:
    output: dict[str, float] = {}
    for name, (total, idle) in current.items():
        if name not in previous:
            continue
        old_total, old_idle = previous[name]
        delta = total - old_total
        output[name] = 100.0 * (1.0 - (idle - old_idle) / delta) if delta > 0 else 0.0
    return output


def _memory() -> dict[str, int]:
    result: dict[str, int] = {}
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            key, value = line.split(":", 1)
            result[key] = int(value.strip().split()[0]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    return result


def _temperatures() -> list[dict[str, Any]]:
    values = []
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            values.append({"path": str(path), "celsius": int(path.read_text()) / 1000.0})
        except (OSError, ValueError):
            pass
    return values


def _frequencies() -> list[dict[str, Any]]:
    values = []
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            values.append({"cpu": path.parts[-3], "khz": int(path.read_text())})
        except (OSError, ValueError):
            pass
    return values


class Monitor(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("field_w5_collection_monitor")
        self.args = args
        self.run_dir = Path(args.run_dir).resolve()
        self.status_path = self.run_dir / "collection_status.json"
        self.samples_path = self.run_dir / "reports" / "runtime_samples.jsonl"
        self.samples = self.samples_path.open("a", encoding="utf-8", buffering=1)
        self.started_monotonic = time.monotonic()
        self.previous_cpu = _cpu_snapshot()
        self.previous_sizes: dict[str, tuple[float, int]] = {}
        self.previous_process_cpu: dict[int, tuple[float, int]] = {}
        self.topic_stats: dict[str, TopicStats] = {}
        inventory = load_json(Path(args.topic_inventory), {}) or {}
        topics = inventory.get("all_topics", {})
        raw_eddy_available = topics.get("/eddy/data") == "eddy_driver/msg/EddyFrame"
        # CameraInfo is emitted once per image by both camera drivers and is a
        # low-bandwidth input-rate proxy. Avoid adding another subscriber to
        # either high-bandwidth image stream merely for status accounting.
        wanted = {
            name: type_name for name, type_name in topics.items()
            if (
                name == "/camera/front/camera_info"
                or (name.startswith("/d405/") and "CameraInfo" in type_name)
                or (name.startswith("/livox/") and ("PointCloud2" in type_name or "Imu" in type_name))
                or (type_name == "eddy_driver/msg/EddyChannelMean" and not raw_eddy_available)
                or type_name == "eddy_driver/msg/EddyFrame"
                or type_name == "eddy_driver/msg/EddyStatus"
                or name in {"/cloud_registered", "/path", "/aft_mapped_to_init"}
                or name.startswith("/vision/front/")
            )
        }
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._field_subscriptions = []
        for topic, type_name in wanted.items():
            try:
                message_type = get_message(type_name)
                stats = TopicStats(type_name)
                self.topic_stats[topic] = stats
                use_raw = type_name in {
                    "sensor_msgs/msg/PointCloud2", "nav_msgs/msg/Path",
                    "eddy_driver/msg/EddyFrame", "eddy_driver/msg/EddyChannelMean",
                }
                if use_raw:
                    self._field_subscriptions.append(self.create_subscription(
                        message_type, topic,
                        lambda serialized, current=stats, current_type=message_type:
                            current.update_raw(serialized, current_type),
                        qos, raw=True,
                    ))
                else:
                    self._field_subscriptions.append(self.create_subscription(
                        message_type, topic,
                        lambda message, current=stats: current.update(message), qos,
                    ))
            except Exception as exc:
                self.get_logger().warning(f"cannot monitor {topic}: {exc}")
        self.publisher = self.create_publisher(String, "/field/collection_status", 10)
        self.timer = self.create_timer(args.interval_sec, self.tick)

    def _writer_rates(self) -> dict[str, Any]:
        now = time.monotonic()
        roots = {
            "core": self.run_dir / ".partial" / "core",
            "mid360": self.run_dir / ".partial" / "mid360",
            "d405": self.run_dir / ".partial" / "d405",
            "eddy": self.run_dir / ".partial" / "eddy",
            "mapping": self.run_dir / ".partial" / "mapping",
            "raw_burst": self.run_dir / ".partial" / "raw_burst",
            "front_video": self.run_dir / "video" / "front",
        }
        output: dict[str, Any] = {}
        for name, root in roots.items():
            size = directory_bytes(root)
            previous_time, previous_size = self.previous_sizes.get(name, (now, size))
            elapsed = now - previous_time
            rate = (size - previous_size) / elapsed / 1_000_000 if elapsed > 0 else 0.0
            self.previous_sizes[name] = (now, size)
            output[name] = {
                "bytes": size, "write_MBps": max(0.0, rate),
                "current_segment": max(0, len(list(root.glob("*.mcap"))) - 1)
                if root.is_dir() else 0,
            }
        return output

    def _processes(self) -> dict[str, Any]:
        registry = load_json(self.run_dir / "logs" / "process_registry.json", {}) or {}
        output = {}
        for item in registry.get("processes", []):
            alive = process_matches(item)
            current: dict[str, Any] = {"pid": item.get("pid"), "alive": alive}
            if alive:
                pid = int(item["pid"])
                try:
                    status = Path(f"/proc/{pid}/status").read_text()
                    for line in status.splitlines():
                        if line.startswith(("VmRSS:", "Threads:", "voluntary_ctxt_switches:", "nonvoluntary_ctxt_switches:")):
                            key, value = line.split(":", 1)
                            current[key] = value.strip()
                    stat_fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
                    ticks = int(stat_fields[11]) + int(stat_fields[12])
                    now = time.monotonic()
                    old_time, old_ticks = self.previous_process_cpu.get(pid, (now, ticks))
                    current["cpu_percent_one_core_basis"] = (
                        (ticks - old_ticks) / os.sysconf(os.sysconf_names["SC_CLK_TCK"])
                        / (now - old_time) * 100.0 if now > old_time else 0.0
                    )
                    self.previous_process_cpu[pid] = (now, ticks)
                except (OSError, ValueError, IndexError):
                    pass
            output[str(item.get("name"))] = current
        for proc in Path("/proc").iterdir():
            if not proc.name.isdigit():
                continue
            try:
                cmdline = proc.joinpath("cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
                if "fastlivo_mapping" not in cmdline:
                    continue
                pid = int(proc.name)
                status = proc.joinpath("status").read_text()
                rss = next((line.split(":", 1)[1].strip() for line in status.splitlines()
                            if line.startswith("VmRSS:")), None)
                stat_fields = proc.joinpath("stat").read_text().rsplit(")", 1)[1].split()
                ticks = int(stat_fields[11]) + int(stat_fields[12])
                now = time.monotonic()
                old_time, old_ticks = self.previous_process_cpu.get(pid, (now, ticks))
                cpu = ((ticks - old_ticks) / os.sysconf(os.sysconf_names["SC_CLK_TCK"])
                       / (now - old_time) * 100.0 if now > old_time else 0.0)
                self.previous_process_cpu[pid] = (now, ticks)
                output["external_fast_livo"] = {
                    "pid": pid, "alive": True, "VmRSS": rss,
                    "cpu_percent_one_core_basis": cpu, "cmdline": cmdline.strip(),
                }
                break
            except (OSError, ValueError, IndexError):
                continue
        return output

    def tick(self) -> None:
        stat = os.statvfs(self.run_dir)
        free = stat.f_bavail * stat.f_frsize
        total = stat.f_blocks * stat.f_frsize
        writers = self._writer_rates()
        total_rate = sum(float(value["write_MBps"]) for value in writers.values())
        remaining_sec = free / (total_rate * 1_000_000) if total_rate > 0 else None
        cpu_now = _cpu_snapshot()
        cpu = _cpu_percent(self.previous_cpu, cpu_now)
        self.previous_cpu = cpu_now
        video = load_json(self.run_dir / "reports" / "video_runtime_status.json", {}) or {}
        complete_snapshots = sum(
            1 for path in self.run_dir.rglob("failure_snapshots/*/metadata.json")
            if (load_json(path, {}) or {}).get("complete") is True
        )
        warning = free < self.args.warning_free_gb * 1_000_000_000
        hard_stop = free < self.args.hard_stop_free_gb * 1_000_000_000
        if hard_stop:
            request = self.run_dir / "logs" / "hard_stop.request"
            if not request.exists():
                atomic_json(request, {
                    "requested_at": utc_now(), "reason": "disk_below_hard_stop_threshold",
                    "free_bytes": free, "threshold_gb": self.args.hard_stop_free_gb,
                })
        document = {
            "schema_version": "field_w5_collection_status_v1",
            "updated_at": utc_now(),
            "elapsed_sec": time.monotonic() - self.started_monotonic,
            "disk": {
                "free_bytes": free, "total_bytes": total,
                "warning": warning, "hard_stop_requested": hard_stop,
                "estimated_remaining_sec": remaining_sec,
            },
            "topics": {name: value.document() for name, value in self.topic_stats.items()},
            "writers": writers,
            "video": video,
            "processes": self._processes(),
            "resources": {
                "cpu_percent": cpu, "memory": _memory(),
                "temperatures": _temperatures(), "cpu_frequencies": _frequencies(),
                "thermal_throttling_claimed": False,
                "thermal_note": "temperatures/frequencies observed; throttling requires platform evidence",
            },
            "failure_snapshot_count": complete_snapshots,
        }
        atomic_json(self.status_path, document)
        self.samples.write(json.dumps(document, ensure_ascii=False) + "\n")
        message = String()
        message.data = json.dumps(document, ensure_ascii=False, separators=(",", ":"))
        if rclpy.ok():
            try:
                self.publisher.publish(message)
            except Exception:  # rclpy raises an implementation-specific RCLError on shutdown
                # SIGINT may invalidate the context between rclpy.ok() and the
                # publish call; the atomically written status remains valid.
                pass

    def close(self) -> None:
        try:
            self.tick()
        finally:
            self.samples.flush()
            os.fsync(self.samples.fileno())
            self.samples.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--topic-inventory", required=True)
    parser.add_argument("--interval-sec", type=float, default=2.0)
    parser.add_argument("--warning-free-gb", type=float, default=100.0)
    parser.add_argument("--hard-stop-free-gb", type=float, default=50.0)
    args = parser.parse_args(argv)
    if args.interval_sec < 1 or args.interval_sec > 10:
        parser.error("interval-sec must be between 1 and 10")
    if args.hard_stop_free_gb <= 0 or args.warning_free_gb <= args.hard_stop_free_gb:
        parser.error("warning threshold must be greater than the positive hard-stop threshold")
    rclpy.init(args=None)
    node = Monitor(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
