#!/usr/bin/env python3
"""Read-only USB/ROS lifecycle evidence collection for camera restart tests."""

from __future__ import annotations

import argparse
from functools import partial
import json
import os
from pathlib import Path
import re
import subprocess
import time
from typing import Any, Iterable


USB_EVENT = re.compile(
    r"usb|uvc|xhci|descriptor|disconnect|reset|timeout|8086|0b5b|realsense",
    re.IGNORECASE,
)
DRIVER_EVENTS: tuple[tuple[str, re.Pattern[str]], ...] = (
    ("PIPELINE_UP", re.compile(r"RealSense Node Is Up", re.IGNORECASE)),
    ("SENSOR_START", re.compile(r"Starting Sensor|Open profile", re.IGNORECASE)),
    ("SENSOR_STOP", re.compile(r"Stopping Sensor|Stop Sensor", re.IGNORECASE)),
    ("SENSOR_CLOSE", re.compile(r"Close Sensor", re.IGNORECASE)),
    ("UVC_CONTROL_ERROR", re.compile(r"UVCIOC_CTRL_QUERY|\b[gs]et_xu\(", re.IGNORECASE)),
    ("USB_TIMEOUT", re.compile(r"Connection timed out|error -110|timeout", re.IGNORECASE)),
    ("DEVICE_RESET", re.compile(r"hardware reset|device reset", re.IGNORECASE)),
    ("PROCESS_EXIT", re.compile(r"process (has finished|has died)", re.IGNORECASE)),
    ("MINDVISION_ENUMERATION_ERROR", re.compile(
        r"MindVision.*no camera|SDK found no camera", re.IGNORECASE)),
)
LOG_EPOCH = re.compile(r"(?:^|\[)(1[0-9]{9}(?:\.[0-9]+)?)(?:\s|\])")


def dual_timestamp() -> dict[str, int]:
    return {"system_timestamp_ns": time.time_ns(), "steady_timestamp_ns": time.monotonic_ns()}


def run_bounded(command: list[str], timeout_sec: float) -> dict[str, Any]:
    started = dual_timestamp()
    started_steady = time.monotonic_ns()
    try:
        result = subprocess.run(
            command, text=True, capture_output=True, timeout=timeout_sec, check=False)
        return {
            **started,
            "command": command,
            "returncode": result.returncode,
            "timed_out": False,
            "duration_ms": (time.monotonic_ns() - started_steady) / 1e6,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
    except subprocess.TimeoutExpired as exc:
        return {
            **started,
            "command": command,
            "returncode": None,
            "timed_out": True,
            "duration_ms": (time.monotonic_ns() - started_steady) / 1e6,
            "stdout": (
                exc.stdout.decode(errors="replace")
                if isinstance(exc.stdout, bytes) else (exc.stdout or "")),
            "stderr": (
                exc.stderr.decode(errors="replace")
                if isinstance(exc.stderr, bytes) else (exc.stderr or "")),
        }
    except OSError as exc:
        return {
            **started,
            "command": command,
            "returncode": None,
            "timed_out": False,
            "duration_ms": (time.monotonic_ns() - started_steady) / 1e6,
            "stdout": "",
            "stderr": str(exc),
        }


def read_text(path: Path) -> str | None:
    try:
        return path.read_text(errors="replace").strip()
    except OSError:
        return None


def enumerate_usb(
        vendor: str, product: str,
        sysfs_root: Path = Path("/sys/bus/usb/devices")) -> list[dict[str, Any]]:
    devices: list[dict[str, Any]] = []
    if not sysfs_root.is_dir():
        return devices
    for entry in sorted(sysfs_root.iterdir()):
        if (read_text(entry / "idVendor") != vendor.lower()
                or read_text(entry / "idProduct") != product.lower()):
            continue
        item: dict[str, Any] = {"sysfs_path": str(entry.resolve())}
        for key in (
                "busnum", "devnum", "speed", "manufacturer", "product",
                "serial", "authorized"):
            item[key] = read_text(entry / key)
        video_nodes: list[str] = []
        for node in entry.rglob("video*"):
            if node.parent.name == "video4linux" and node.name.startswith("video"):
                video_nodes.append("/dev/" + node.name)
        item["video_nodes"] = sorted(set(video_nodes))
        devices.append(item)
    return devices


def process_evidence(video_nodes: Iterable[str]) -> dict[str, Any]:
    targets = set(video_nodes)
    processes: list[dict[str, Any]] = []
    open_fds: list[dict[str, Any]] = []
    own_pid = os.getpid()
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) == own_pid:
            continue
        try:
            raw_arguments = (proc / "cmdline").read_bytes().split(b"\0")
            arguments = [value.decode(errors="replace") for value in raw_arguments if value]
            command = " ".join(arguments)
        except OSError:
            continue
        executable_names = " ".join(Path(value).name.lower() for value in arguments[:2])
        process_tokens = ("realsense", "rs-enumerate", "mindvision_camera")
        if any(token in executable_names for token in process_tokens):
            processes.append({"pid": int(proc.name), "cmdline": command})
        if not targets:
            continue
        try:
            fds = list((proc / "fd").iterdir())
        except OSError:
            continue
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if target in targets:
                open_fds.append({
                    "pid": int(proc.name), "fd": fd.name,
                    "target": target, "cmdline": command})
    return {"matching_processes": processes, "open_video_fds": open_fds}


def kernel_usb_events() -> dict[str, Any]:
    result = run_bounded(["dmesg", "--time-format", "iso"], 5.0)
    result["matching_lines"] = [
        line for line in result["stdout"].splitlines()
        if USB_EVENT.search(line)]
    result.pop("stdout", None)
    return result


def sdk_enumeration(timeout_sec: float) -> dict[str, Any]:
    result = run_bounded(["rs-enumerate-devices", "-s"], timeout_sec)
    text = result["stdout"] + "\n" + result["stderr"]
    result["d405_found"] = "D405" in text or "260322276866" in text
    return result


def snapshot(vendor: str, product: str, with_sdk: bool, sdk_timeout_sec: float) -> dict[str, Any]:
    devices = enumerate_usb(vendor, product)
    video_nodes = [node for device in devices for node in device["video_nodes"]]
    report: dict[str, Any] = {
        "schema_version": "tank_usb_lifecycle_v1",
        **dual_timestamp(),
        "target": {"vendor_id": vendor, "product_id": product},
        "usb_present": bool(devices),
        "usb_devices": devices,
        "process_evidence": process_evidence(video_nodes),
        "kernel_usb_events": kernel_usb_events(),
    }
    report["sdk_enumeration"] = (
        sdk_enumeration(sdk_timeout_sec) if with_sdk else {"status": "SKIP"})
    return report


def classify_log_line(line: str) -> list[str]:
    return [name for name, pattern in DRIVER_EVENTS if pattern.search(line)]


def analyze_logs(paths: Iterable[Path]) -> dict[str, Any]:
    events: list[dict[str, Any]] = []
    for path in paths:
        try:
            lines = path.read_text(errors="replace").splitlines()
        except OSError as exc:
            events.append({"event": "LOG_READ_ERROR", "path": str(path), "detail": str(exc)})
            continue
        for number, line in enumerate(lines, 1):
            kinds = classify_log_line(line)
            if not kinds:
                continue
            epoch = LOG_EPOCH.search(line)
            for kind in kinds:
                events.append({
                    "event": kind,
                    "system_timestamp_ns": int(float(epoch.group(1)) * 1e9) if epoch else None,
                    "steady_timestamp_ns": None,
                    "path": str(path),
                    "line_number": number,
                    "detail": line,
                })
    events.sort(key=lambda item: (item.get("system_timestamp_ns") is None,
                                  item.get("system_timestamp_ns") or 0,
                                  item.get("path", ""), item.get("line_number", 0)))
    return {"schema_version": "tank_lifecycle_log_analysis_v1", "events": events}


def observe_ros(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
    except ImportError as exc:
        raise SystemExit(f"ROS Python dependencies unavailable: {exc}")

    class Observer(Node):
        def __init__(self) -> None:
            super().__init__("d405_lifecycle_observer")
            self.started = dual_timestamp()
            self.counts = {args.color_topic: 0, args.depth_topic: 0}
            self.first: dict[str, dict[str, int] | None] = {topic: None for topic in self.counts}
            self.last: dict[str, dict[str, int] | None] = {topic: None for topic in self.counts}
            self.usb_samples: list[dict[str, Any]] = []
            self.usb_transitions: list[dict[str, Any]] = []
            self.last_present: bool | None = None
            self._image_subscriptions = []
            for topic in self.counts:
                self._image_subscriptions.append(self.create_subscription(
                    Image, topic, partial(self.frame, topic), qos_profile_sensor_data))
            self.create_timer(args.usb_poll_sec, self.usb_tick)

        def frame(self, topic: str, _message: Any = None) -> None:
            stamp = dual_timestamp()
            self.counts[topic] += 1
            if self.first[topic] is None:
                self.first[topic] = stamp
            self.last[topic] = stamp

        def usb_tick(self) -> None:
            stamp = dual_timestamp()
            devices = enumerate_usb(args.vendor, args.product)
            present = bool(devices)
            sample = {**stamp, "present": present, "devices": devices}
            self.usb_samples.append(sample)
            if self.last_present is None or present != self.last_present:
                self.usb_transitions.append(sample)
            self.last_present = present

    rclpy.init()
    node = Observer()
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.2, max(0.0, deadline - time.monotonic())))
    finally:
        ended = dual_timestamp()
        actual = (ended["steady_timestamp_ns"] - node.started["steady_timestamp_ns"]) / 1e9
        topics = {}
        for topic, count in node.counts.items():
            topics[topic] = {
                "received_count": count,
                "actual_rate_hz": count / actual if actual else 0.0,
                "first_receive": node.first[topic],
                "last_receive": node.last[topic],
            }
        report = {
            "schema_version": "tank_d405_lifecycle_observation_v1",
            "started": node.started,
            "ended": ended,
            "actual_duration_sec": actual,
            "topics": topics,
            "usb_samples": node.usb_samples,
            "usb_transitions": node.usb_transitions,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--vendor", default="8086")
    common.add_argument("--product", default="0b5b")

    snap = sub.add_parser("snapshot", parents=[common])
    snap.add_argument("--output", type=Path, required=True)
    snap.add_argument("--sdk", action="store_true")
    snap.add_argument("--sdk-timeout-sec", type=float, default=15.0)

    logs = sub.add_parser("analyze-logs")
    logs.add_argument("logs", nargs="+", type=Path)
    logs.add_argument("--output", type=Path, required=True)

    observe = sub.add_parser("observe", parents=[common])
    observe.add_argument("--output", type=Path, required=True)
    observe.add_argument("--duration-sec", type=float, required=True)
    observe.add_argument("--usb-poll-sec", type=float, default=1.0)
    observe.add_argument("--color-topic", default="/d405/camera/color/image_raw")
    observe.add_argument("--depth-topic", default="/d405/camera/depth/image_rect_raw")

    args = parser.parse_args(argv)
    if args.command == "snapshot":
        report = snapshot(args.vendor, args.product, args.sdk, args.sdk_timeout_sec)
    elif args.command == "analyze-logs":
        report = analyze_logs(args.logs)
    else:
        return observe_ros(args)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
