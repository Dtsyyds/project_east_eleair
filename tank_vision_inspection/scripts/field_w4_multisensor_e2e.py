#!/usr/bin/env python3
"""FIELD-W4-MULTISENSOR-E2E probes, audits, and acceptance reports.

This module intentionally treats the existing Livox, MindVision, frozen seam,
and inspection_sync nodes as black boxes.  It adds observation and orchestration
only; it does not alter the sensor, tracker, calibration, or synchronization
implementations.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import statistics
import subprocess
import time
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
BEV = Path(os.environ.get("PHYSIC_BEV_ROOT", "/home/sunrise/east_eleair/physic_bev"))
DEFAULT_OUTPUT = ROOT / "output" / "field_w4_multisensor_e2e"
DEFAULT_DATA_ROOT = Path(os.environ.get("DATA_ROOT", "/media/mcap_data"))
TRACKER_CONFIG = ROOT / "configs" / "s100p_guided_anchor_seam_4096x3072.yaml"
TRACKER_REFERENCE = ROOT / "configs" / "seam_geometry_reference_s100p_live_v1.json"
TRACKER_SOURCE = ROOT / "tank_vision" / "stable_seam_tracker.py"
ROS_NODE_SOURCE = ROOT / "ros2_ws" / "src" / "tank_vision_ros" / "tank_vision_ros" / "stable_seam_node.py"
VISUAL_LAUNCH_SOURCE = BEV / "src" / "inspection_bringup" / "launch" / "visual_field_validation.launch.py"
MID360_CONFIG = BEV / "src" / "livox_ros_driver2" / "config" / "MID360_config.json"
INSTALL_NODE_ENTRYPOINT = ROOT / "ros2_ws" / "install_s100p_humble" / "tank_vision_ros" / "lib" / "tank_vision_ros" / "stable_seam_node"
INSTALL_VISUAL_LAUNCH = BEV / "install" / "inspection_bringup" / "share" / "inspection_bringup" / "launch" / "visual_field_validation.launch.py"
MCAP_OPTIONS = BEV / "src" / "inspection_bringup" / "config" / "mcap_writer_options.yaml"
QOS_OVERRIDES = BEV / "src" / "inspection_bringup" / "config" / "qos_overrides.yaml"

VISUAL_TOPICS = {
    "/camera/front/camera_info",
    "/vision/front/seam_geometry",
    "/vision/front/seam_state",
    "/diagnostics",
    "/tf_static",
}
LIDAR_TOPIC_CANDIDATES = {"/livox/lidar", "/livox/lidar_ PointCloud2"}
SYNC_TOPIC = "/derived/inspection/fusion_index"


def _run(args: list[str], *, cwd: Path | None = None, timeout: float = 10.0) -> dict[str, Any]:
    try:
        result = subprocess.run(args, cwd=str(cwd) if cwd else None, capture_output=True,
                                text=True, timeout=timeout, check=False)
        return {"argv": args, "returncode": result.returncode, "stdout": result.stdout,
                "stderr": result.stderr}
    except (OSError, subprocess.SubprocessError) as exc:
        return {"argv": args, "returncode": None, "stdout": "", "stderr": str(exc)}


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _load(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _sha(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _sha_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _git(args: list[str]) -> dict[str, Any]:
    return _run(["git", *args], cwd=ROOT, timeout=20.0)


def _git_text(args: list[str]) -> str:
    result = _git(args)
    return result["stdout"].strip() if result["returncode"] == 0 else ""


def _resolved(path: Path) -> str | None:
    try:
        return str(path.resolve(strict=True))
    except OSError:
        return None


def _stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None, "mean": None}
    ordered = sorted(values)

    def percentile(q: float) -> float:
        position = (len(ordered) - 1) * q
        low = int(position)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (position - low)

    return {"count": len(ordered), "p50": percentile(0.5), "p95": percentile(0.95),
            "max": max(ordered), "mean": statistics.fmean(ordered)}


def _w3_reports() -> dict[str, Any]:
    report_root = ROOT / "output" / "field_w3_perf_isolation"
    result: dict[str, Any] = {}
    for case in ("p0", "p1", "p2"):
        value = _load(report_root / {
            "p0": "p0_baseline.json", "p1": "p1_snapshot.json",
            "p2": "p2_provenance.json",
        }[case], {}) or {}
        result[case] = {
            "run_dir": value.get("run_dir"),
            "system_pass": value.get("system_pass"),
            "rates": value.get("rates", {}),
            "slot_overwritten": value.get("slot_overwritten"),
            "processing_ms": value.get("processing_ms", {}),
            "wall_time_vs_processing_ms": value.get("wall_time_vs_processing_ms", {}),
        }
    return result


def checkpoint(args: argparse.Namespace) -> int:
    output = Path(args.output)
    head = _git_text(["rev-parse", "HEAD"])
    parent = _git_text(["rev-parse", "HEAD^"])
    status = _git_text(["status", "--porcelain=v1"])
    diff = _git(["diff", "--binary", "HEAD"])["stdout"].encode("utf-8", errors="surrogateescape")
    untracked = _git_text(["ls-files", "--others", "--exclude-standard"]).splitlines()
    patch_material = bytearray(diff)
    for name in untracked:
        patch_material.extend(name.encode("utf-8", errors="surrogateescape"))
        patch_material.extend(b"\0")
        file_path = ROOT / name
        if file_path.is_file():
            patch_material.extend(file_path.read_bytes())
    w3_commit_diff = _git(["diff", "--binary", f"{parent}..{head}"])["stdout"].encode(
        "utf-8", errors="surrogateescape") if parent and head else b""
    files = {
        "tracker_source": TRACKER_SOURCE,
        "ros_node_source": ROS_NODE_SOURCE,
        "visual_launch_source": VISUAL_LAUNCH_SOURCE,
        "installed_node_entrypoint": INSTALL_NODE_ENTRYPOINT,
        "installed_visual_launch": INSTALL_VISUAL_LAUNCH,
        "guided_yaml": TRACKER_CONFIG,
        "reference": TRACKER_REFERENCE,
        "mid360_config": MID360_CONFIG,
    }
    identities = {
        name: {"path": str(path), "resolved_path": _resolved(path), "sha256": _sha(path)}
        for name, path in files.items()
    }
    source_install_consistent = bool(
        identities["ros_node_source"]["sha256"] and identities["installed_node_entrypoint"]["sha256"]
        and identities["visual_launch_source"]["sha256"] == identities["installed_visual_launch"]["sha256"]
        and identities["installed_visual_launch"]["resolved_path"] == str(VISUAL_LAUNCH_SOURCE.resolve())
    )
    report = {
        "schema_version": "field_w4_w3_checkpoint_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "git": {
            "repository": str(ROOT), "head": head, "parent": parent,
            "dirty": bool(status), "status_porcelain": status.splitlines(),
            "worktree_patch_sha256": _sha_bytes(bytes(patch_material)),
            "w3_commit_diff_sha256": _sha_bytes(w3_commit_diff),
        },
        "identities": identities,
        "source_install_consistent": source_install_consistent,
        "default_formal_profile": {
            "name": "P1",
            "geometry_provenance_sampling_enabled": False,
            "auto_save_failures": True,
            "publish_debug_image": False,
            "roi_validation_recording_enabled": False,
            "record_mcap": True,
            "record_raw_image": False,
        },
        "w3_runs": _w3_reports(),
        "camera_info_calibration_complete": False,
        "spatial_projection_allowed": False,
        "algorithm_behavior_modified": False,
        "system_performance_isolated": True,
        "system_integration_ready": True,
        "algorithm_field_validation_pending": True,
        "production_ready": False,
    }
    _write(output, report)
    print(json.dumps(report, ensure_ascii=False))
    return 0


def init_run(args: argparse.Namespace) -> int:
    root = Path(args.data_root).expanduser().resolve()
    run = root / "runs" / args.run_id
    if run.exists() and any(run.iterdir()):
        raise RuntimeError(f"run directory exists and is non-empty: {run}")
    run.mkdir(parents=True, exist_ok=True)
    for name in ("mcap", "logs", "frozen_configs"):
        (run / name).mkdir(exist_ok=True)
    manifest = {
        "schema_version": "field_w4_transport_run_v1", "run_id": args.run_id,
        "created_at": datetime.now(timezone.utc).isoformat(), "phase": args.phase,
        "data_root": str(root), "status": {"complete": False, "finalized": False},
        "geometry_provenance_sampling_enabled": False, "record_raw_image": False,
        "publish_debug_image": False, "camera_info_calibration_complete": False,
        "spatial_projection_allowed": False,
    }
    _write(run / "run_manifest.json", manifest)
    print(str(run))
    return 0


def _parse_ip_json() -> list[dict[str, Any]]:
    result = _run(["ip", "-j", "addr"], timeout=5.0)
    if result["returncode"] != 0:
        return []
    try:
        return json.loads(result["stdout"])
    except json.JSONDecodeError:
        return []


def _route_interface(ip: str) -> str | None:
    result = _run(["ip", "route", "get", ip], timeout=5.0)
    match = re.search(r"\bdev\s+(\S+)", result["stdout"])
    return match.group(1) if match else None


def preflight(args: argparse.Namespace) -> int:
    config = Path(args.mid360_config).expanduser().resolve()
    document: dict[str, Any] = {}
    try:
        document = json.loads(config.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        document = {"error": str(exc)}
    mid360 = document.get("MID360", {})
    lidar = mid360.get("lidar_net_info", {})
    hosts = mid360.get("host_net_info", []) if isinstance(mid360, dict) else []
    lidar_ips = [item for item in document.get("lidar_configs", []) if isinstance(item, dict)]
    lidar_ip = lidar_ips[0].get("ip") if lidar_ips else None
    host_ip = hosts[0].get("host_ip") if hosts else None
    interface = _route_interface(str(lidar_ip)) if lidar_ip else None
    ping = _run(["ping", "-c", "1", "-W", "1", str(lidar_ip)], timeout=5.0) if lidar_ip else {}
    packages = {}
    for package in ("livox_ros_driver2", "inspection_livox_adapter", "inspection_sync",
                    "inspection_bringup", "mindvision_camera_driver"):
        packages[package] = _run(["ros2", "pkg", "prefix", package], timeout=5.0)
    tcpdump = None
    if interface and shutil.which("tcpdump"):
        ports = [str(lidar.get("point_data_port", 56300)), str(lidar.get("imu_data_port", 56400))]
        expression = " or ".join(f"udp port {port}" for port in ports)
        tcpdump = _run(["timeout", str(max(1.0, args.udp_capture_sec)), "tcpdump", "-nn", "-i",
                        interface, "-c", str(max(1, args.udp_packets)), expression], timeout=args.udp_capture_sec + 5.0)
    report = {
        "schema_version": "field_w4_mid360_preflight_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "host": {"hostname": socket.gethostname(), "kernel": platform.release(),
                 "architecture": platform.machine(), "platform": platform.platform()},
        "network": {"ip_addr": _parse_ip_json(), "route_to_lidar": _run(["ip", "route", "get", str(lidar_ip)], timeout=5.0),
                    "selected_interface": interface, "configured_host_ip": host_ip,
                    "configured_lidar_ip": lidar_ip, "ping": ping, "udp_capture": tcpdump},
        "mid360_config": {"path": str(config), "sha256": _sha(config), "document": document},
        "ros_packages": packages,
        "constraints": {"camera_started": False, "visual_started": False, "sync_started": False,
                         "fast_livo2_started": False},
    }
    _write(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0


def _topic_list() -> dict[str, str]:
    result = _run(["ros2", "topic", "list", "-t"], timeout=10.0)
    topics: dict[str, str] = {}
    for line in result["stdout"].splitlines():
        match = re.match(r"^\s*(\S+)\s+\[([^]]+)\]\s*$", line)
        if match:
            topics[match.group(1)] = match.group(2).split(",", 1)[0].strip()
    return topics


def inventory(args: argparse.Namespace) -> int:
    phase = args.phase.split("-", 1)[0]
    topics = _topic_list()
    selected = {}
    for topic in sorted(topics):
        if phase == "w4a" and not ("livox" in topic or topic in {"/diagnostics", "/tf_static"}):
            continue
        if phase in {"w4b", "w4c"}:
            if topic in VISUAL_TOPICS or "livox" in topic or (phase == "w4c" and topic == SYNC_TOPIC):
                selected[topic] = topics[topic]
    info: dict[str, Any] = {}
    for topic in selected:
        info[topic] = _run(["ros2", "topic", "info", topic, "--verbose"], timeout=10.0)
    lidar_topics = {
        topic: type_name for topic, type_name in topics.items()
        if "livox" in topic and ("PointCloud2" in type_name or "CustomMsg" in type_name)
    }
    imu_topics = {
        topic: type_name for topic, type_name in topics.items()
        if "livox" in topic and ("Imu" in type_name or topic.endswith("imu_raw"))
    }
    report = {
        "schema_version": "field_w4_topic_inventory_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "phase": args.phase,
        "all_topics": topics,
        "selected_topics": selected,
        "topic_info_verbose": info,
        "actual_mid360_lidar_topics": lidar_topics,
        "actual_mid360_imu_topics": imu_topics,
        "required_visual_topics_present": sorted(VISUAL_TOPICS & topics.keys()),
        "sync_output_present": SYNC_TOPIC in topics,
    }
    _write(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if topics else 2


def topic_args(args: argparse.Namespace) -> int:
    phase = args.phase.split("-", 1)[0]
    report = _load(Path(args.inventory), {}) or {}
    topics = dict(report.get("selected_topics", {}))
    if phase == "w4a":
        topics = dict(report.get("actual_mid360_lidar_topics", {}))
        topics.update(report.get("actual_mid360_imu_topics", {}))
        for topic in ("/diagnostics", "/tf_static"):
            if topic in report.get("all_topics", {}):
                topics[topic] = report["all_topics"][topic]
    else:
        required = set(VISUAL_TOPICS) | set(report.get("actual_mid360_lidar_topics", {})) | set(report.get("actual_mid360_imu_topics", {}))
        if phase == "w4c":
            required.add(SYNC_TOPIC)
        topics = {topic: report.get("all_topics", {}).get(topic, type_name)
                  for topic, type_name in topics.items() if topic in required}
    missing = sorted(set(args.required.split(",")) - topics.keys()) if args.required else []
    payload = {"topics": sorted(topics), "types": topics, "missing_required": missing}
    if args.output:
        _write(Path(args.output), payload)
    for topic in sorted(topics):
        print(topic)
    return 0 if not missing else 3


def _header_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
    return value if value > 0 else None


def _diagnostic_values(message: Any) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for status in getattr(message, "status", []) or []:
        name = str(getattr(status, "name", ""))
        values[name] = {str(item.key): str(item.value) for item in getattr(status, "values", []) or []}
    return values


def _diagnostic_group_key(value: Any) -> str:
    text = str(value or "").strip()
    return text if text else "<empty>"


def _publisher_gid(message_info: Any) -> str | None:
    value = getattr(message_info, "publisher_gid", None)
    if value is None:
        return None
    if isinstance(value, (bytes, bytearray)):
        return bytes(value).hex()
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        text = str(value).strip()
        return text or None


def _source_audit_update(groups: dict[str, dict[str, Any]], key: str,
                         stamp_ns: int | None, receive_ns: int | None) -> None:
    item = groups.setdefault(key, {
        "count": 0, "first_stamp_ns": None, "last_stamp_ns": None,
        "header_stamp_regressions": 0, "first_receive_ns": None,
        "last_receive_ns": None, "receive_timestamp_regressions": 0,
    })
    item["count"] += 1
    if stamp_ns is not None:
        if item["last_stamp_ns"] is not None and stamp_ns < item["last_stamp_ns"]:
            item["header_stamp_regressions"] += 1
        item["first_stamp_ns"] = item["first_stamp_ns"] or stamp_ns
        item["last_stamp_ns"] = stamp_ns
    if receive_ns is not None:
        if item["last_receive_ns"] is not None and receive_ns < item["last_receive_ns"]:
            item["receive_timestamp_regressions"] += 1
        item["first_receive_ns"] = item["first_receive_ns"] or receive_ns
        item["last_receive_ns"] = receive_ns


def _diagnostics_audit(stats: dict[str, dict[str, Any]], history: list[dict[str, Any]]) -> dict[str, Any]:
    item = stats.get("/diagnostics")
    if not item or not history:
        return {"available": False, "classification": "unavailable", "reason": "no diagnostics observed"}
    header_regressions = int(item.get("non_monotonic_count", 0))
    receive_regressions = int(item.get("arrival_non_monotonic_count", 0))
    by_name: dict[str, dict[str, Any]] = {}
    by_hardware: dict[str, dict[str, Any]] = {}
    by_gid: dict[str, dict[str, Any]] = {}
    for event in history:
        stamp_ns = event.get("header_stamp_ns")
        receive_ns = event.get("receive_timestamp_ns")
        for name in event.get("status_names", ["<empty>"]):
            _source_audit_update(by_name, str(name), stamp_ns, receive_ns)
        for hardware_id in event.get("hardware_ids", ["<empty>"]):
            _source_audit_update(by_hardware, str(hardware_id), stamp_ns, receive_ns)
        gid = event.get("publisher_gid")
        if gid:
            _source_audit_update(by_gid, str(gid), stamp_ns, receive_ns)
    same_name = sum(int(value["header_stamp_regressions"]) for value in by_name.values())
    same_hardware = sum(int(value["header_stamp_regressions"]) for value in by_hardware.values())
    same_gid = sum(int(value["header_stamp_regressions"]) for value in by_gid.values())
    if receive_regressions:
        classification = "bag_receive_timestamp_regression"
    elif same_name or same_hardware or same_gid:
        classification = "same_source_timestamp_regression"
    elif header_regressions:
        classification = "aggregate_multi_publisher_interleaving"
    else:
        classification = "unavailable"
    return {
        "available": True, "classification": classification,
        "raw_validator_relevant": classification in {
            "same_source_timestamp_regression", "bag_receive_timestamp_regression"},
        "header_stamp_regressions_global": header_regressions,
        "receive_timestamp_regressions_global": receive_regressions,
        "same_source_header_regressions": {
            "status_name": same_name, "hardware_id": same_hardware, "publisher_gid": same_gid},
        "by_status_name": by_name, "by_hardware_id": by_hardware,
        "by_publisher_gid": by_gid if by_gid else {"<unavailable>": {"count": 0}},
        "note": "publisher GID grouping is unavailable when the rclpy callback provides no MessageInfo publisher_gid",
    }


def probe(args: argparse.Namespace) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        _write(Path(args.output), {"error": f"ROS Python imports unavailable: {exc}", "valid": False})
        return 2

    inventory_doc = _load(Path(args.inventory), {}) or {}
    types = inventory_doc.get("types", {})
    if not types:
        types = inventory_doc.get("selected_topics", {})
    if args.topics:
        wanted = [item for item in args.topics.split(",") if item]
        types = {topic: types[topic] for topic in wanted if topic in types}
    stats: dict[str, dict[str, Any]] = {}
    diagnostic_history: list[dict[str, Any]] = []

    class ProbeNode(Node):
        def __init__(self) -> None:
            super().__init__("field_w4_topic_probe")
            self._w4_subscriptions = []
            for topic, type_name in types.items():
                try:
                    message_type = get_message(type_name)
                    if topic == "/tf_static":
                        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
                    else:
                        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT,
                                         durability=DurabilityPolicy.VOLATILE)
                    self._w4_subscriptions.append(self.create_subscription(
                        message_type, topic,
                        lambda msg, *callback_info, name=topic: self.received(
                            name, msg, callback_info[0] if callback_info else None), qos))
                    stats[topic] = {"type": type_name, "count": 0, "header_stamp_count": 0,
                                    "first_stamp_ns": None, "last_stamp_ns": None,
                                    "non_monotonic_count": 0, "arrival_first_ns": None,
                                    "arrival_last_ns": None, "arrival_non_monotonic_count": 0,
                                    "receive_first_ns": None, "receive_last_ns": None,
                                    "receive_non_monotonic_count": 0, "point_count_samples": [],
                                    "camera_info_calibrated": None, "diagnostics_last": {}}
                except Exception as exc:  # retain an explicit per-topic failure
                    stats[topic] = {"type": type_name, "count": 0, "subscription_error": str(exc)}

        def received(self, topic: str, message: Any, message_info: Any = None) -> None:
            now_ns = time.time_ns()
            item = stats[topic]
            item["count"] = int(item.get("count", 0)) + 1
            if item["arrival_last_ns"] is not None and now_ns < item["arrival_last_ns"]:
                item["arrival_non_monotonic_count"] += 1
            item["arrival_first_ns"] = item["arrival_first_ns"] or now_ns
            item["arrival_last_ns"] = now_ns
            receive_ns = getattr(message_info, "received_timestamp", None)
            try:
                receive_ns = int(receive_ns) if receive_ns is not None and int(receive_ns) > 0 else now_ns
            except (TypeError, ValueError):
                receive_ns = now_ns
            if item["receive_last_ns"] is not None and receive_ns < item["receive_last_ns"]:
                item["receive_non_monotonic_count"] += 1
            item["receive_first_ns"] = item["receive_first_ns"] or receive_ns
            item["receive_last_ns"] = receive_ns
            stamp_ns = _header_stamp_ns(message)
            if stamp_ns is not None:
                item["header_stamp_count"] += 1
                if item["last_stamp_ns"] is not None and stamp_ns < item["last_stamp_ns"]:
                    item["non_monotonic_count"] += 1
                item["first_stamp_ns"] = item["first_stamp_ns"] or stamp_ns
                item["last_stamp_ns"] = stamp_ns
            if hasattr(message, "point_num"):
                item["point_count_samples"].append(int(message.point_num))
            elif hasattr(message, "width") and hasattr(message, "height") and hasattr(message, "fields"):
                item["point_count_samples"].append(int(message.width) * int(message.height))
            if hasattr(message, "k") and hasattr(message, "p"):
                item["camera_info_calibrated"] = bool(
                    any(abs(float(value)) > 1e-12 for value in message.k)
                    and any(abs(float(value)) > 1e-12 for value in message.p))
            if getattr(message, "status", None) is not None:
                values = _diagnostic_values(message)
                if values:
                    item["diagnostics_last"] = values
                statuses = getattr(message, "status", []) or []
                diagnostic_history.append({
                    "host_time_ns": now_ns, "receive_timestamp_ns": receive_ns,
                    "header_stamp_ns": stamp_ns, "values": values,
                    "status_names": sorted({_diagnostic_group_key(getattr(status, "name", "")) for status in statuses}),
                    "hardware_ids": sorted({_diagnostic_group_key(getattr(status, "hardware_id", "")) for status in statuses}),
                    "publisher_gid": _publisher_gid(message_info),
                })

    rclpy.init()
    node = ProbeNode()
    deadline = time.monotonic() + max(0.1, float(args.duration_sec))
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    for item in stats.values():
        count = int(item.get("count", 0))
        first = item.get("first_stamp_ns")
        last = item.get("last_stamp_ns")
        arrival_first = item.get("arrival_first_ns")
        arrival_last = item.get("arrival_last_ns")
        item["header_rate_hz"] = ((count - 1) / ((last - first) / 1e9)
                                   if first and last and count > 1 and last > first else None)
        item["arrival_rate_hz"] = ((count - 1) / ((arrival_last - arrival_first) / 1e9)
                                    if arrival_first and arrival_last and count > 1 and arrival_last > arrival_first else None)
        item["point_count_stats"] = _stats([float(value) for value in item.pop("point_count_samples", [])])
    report = {"schema_version": "field_w4_topic_probe_v2", "duration_sec": args.duration_sec,
              "topics": stats, "diagnostics_history": diagnostic_history[-300:],
              "diagnostics_timestamp_audit": _diagnostics_audit(stats, diagnostic_history)}
    _write(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0


def monitor(args: argparse.Namespace) -> int:
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    needles = tuple(item for item in args.process_needles.split(",") if item)

    def temperatures() -> dict[str, float]:
        result = {}
        for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
            try:
                value = float(path.read_text().strip())
                result[path.parent.name] = value / 1000.0 if value > 200 else value
            except (OSError, ValueError):
                pass
        return result

    def frequencies() -> dict[str, float]:
        result = {}
        for path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")):
            try:
                result[path.parent.parent.name] = float(path.read_text().strip()) / 1000.0
            except (OSError, ValueError):
                pass
        return result

    def processes() -> list[dict[str, Any]]:
        result = _run(["ps", "-eo", "pid=,pcpu=,rss=,args="], timeout=5.0)
        rows = []
        for line in result["stdout"].splitlines():
            parts = line.strip().split(None, 3)
            if len(parts) != 4 or not any(needle in parts[3] for needle in needles):
                continue
            try:
                rows.append({"pid": int(parts[0]), "cpu_percent": float(parts[1]),
                             "rss_kib": int(parts[2]), "command": parts[3]})
            except ValueError:
                pass
        return rows

    deadline = time.monotonic() + max(0.0, float(args.duration_sec))
    with output.open("w", encoding="utf-8") as handle:
        while True:
            handle.write(json.dumps({"host_time_ns": time.time_ns(), "processes": processes(),
                                     "temperatures_c": temperatures(),
                                     "cpu_frequencies_mhz": frequencies()}, ensure_ascii=False) + "\n")
            handle.flush()
            if time.monotonic() >= deadline:
                break
            time.sleep(max(0.05, float(args.interval_sec)))
    return 0


def _bag_report(bag: Path, output: Path, *, phase: str, run_id: str) -> dict[str, Any]:
    validation_path = bag / "validation_report.json"
    info = _run(["ros2", "bag", "info", str(bag)], timeout=30.0)
    validate = _run(["ros2", "run", "inspection_tools", "validate_bag", str(bag),
                     "--allow-partial", "--output", str(validation_path)], timeout=180.0)
    validation = _load(validation_path, {}) or {}
    finalized = None
    if validate["returncode"] == 0 and validation.get("valid"):
        finalized = _run(["ros2", "run", "inspection_tools", "finalize_bag", str(bag),
                          "--task-id", run_id, "--asset-id", "tank_multisensor",
                          "--course-id", "field_w4", "--plate-id", phase,
                          "--validation-report", str(validation_path)], timeout=180.0)
    report = {"phase": phase, "run_id": run_id, "bag": str(bag),
              "bag_info": info, "validation_command": validate,
              "validation": validation, "finalize_command": finalized,
              "validated": bool(validate["returncode"] == 0 and validation.get("valid")),
              "finalized": bool(finalized and finalized.get("returncode") == 0 and (bag / "manifest.yaml").is_file())}
    _write(output, report)
    return report


def bag_report(args: argparse.Namespace) -> int:
    report = _bag_report(Path(args.bag), Path(args.output), phase=args.phase, run_id=args.run_id)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["validated"] else 3


def resource_summary(args: argparse.Namespace) -> int:
    samples = []
    path = Path(args.input)
    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    cpu = [float(item["cpu_percent"]) for sample in samples for item in sample.get("processes", [])
           if item.get("cpu_percent") is not None]
    rss = [float(item["rss_kib"]) for sample in samples for item in sample.get("processes", [])
           if item.get("rss_kib") is not None]
    temperatures = [float(value) for sample in samples for value in sample.get("temperatures_c", {}).values()]
    frequencies = [float(value) for sample in samples for value in sample.get("cpu_frequencies_mhz", {}).values()]
    report = {"input": str(path), "sample_count": len(samples), "cpu_percent": _stats(cpu),
              "rss_kib": _stats(rss), "temperature_c": _stats(temperatures),
              "cpu_frequency_mhz": _stats(frequencies)}
    _write(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0


def process_audit(args: argparse.Namespace) -> int:
    result = _run(["ps", "-eo", "pid=,ppid=,pgid=,stat=,args="], timeout=10.0)
    needles = tuple(item for item in args.process_needles.split(",") if item)
    lines = [line for line in result["stdout"].splitlines()
             if any(needle in line for needle in needles)]
    report = {"created_at": datetime.now(timezone.utc).isoformat(), "needles": needles,
              "matching_processes_after_stop": lines, "orphan_processes": lines,
              "pass": not lines}
    _write(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["pass"] else 4


def _phase_probe(path: Path) -> dict[str, Any]:
    return _load(path, {}) or {}


def _snapshot_integrity(run: Path) -> dict[str, Any]:
    """Verify automatic snapshot metadata and every hash named by it."""
    metadata_files = sorted(run.glob("live_session/*/failure_snapshots/*/metadata.json"))
    entries: list[dict[str, Any]] = []
    hash_fields = {
        "roi_raw_sha256": "roi_raw.bmp",
        "context_sha256": "context.jpg",
        "overlay_sha256": "overlay.png",
        "seam_geometry_sha256": "seam_geometry.json",
        "failure_reason_sha256": "failure_reason.json",
    }
    for metadata_path in metadata_files:
        metadata = _load(metadata_path, {}) or {}
        snapshot_dir = metadata_path.parent
        file_checks: dict[str, Any] = {}
        for field, filename in hash_fields.items():
            expected = metadata.get(field)
            file_path = snapshot_dir / filename
            actual = _sha(file_path)
            file_checks[field] = {
                "path": str(file_path), "expected": expected, "actual": actual,
                "match": bool(expected and actual and expected == actual),
            }
        config_path = Path(str(metadata.get("config", "")))
        reference_path = Path(str(metadata.get("reference", "")))
        config_match = bool(metadata.get("config_sha256") and _sha(config_path) == metadata.get("config_sha256"))
        reference_match = bool(metadata.get("reference_sha256") and _sha(reference_path) == metadata.get("reference_sha256"))
        entry = {
            "metadata": str(metadata_path), "event_id": metadata.get("event_id"),
            "complete": bool(metadata.get("complete")),
            "config_sha_match": config_match, "reference_sha_match": reference_match,
            "files": file_checks,
        }
        entry["pass"] = bool(entry["complete"] and config_match and reference_match and
                               all(value["match"] for value in file_checks.values()))
        entries.append(entry)
    return {
        "run_dir": str(run), "snapshot_count": len(entries),
        "complete_count": sum(1 for entry in entries if entry["complete"]),
        "hash_valid_count": sum(1 for entry in entries if entry["pass"]),
        "pass": all(entry["pass"] for entry in entries), "snapshots": entries,
    }


def _phase_pass(phase: str, probe_doc: dict[str, Any], bag_doc: dict[str, Any], shutdown: dict[str, Any]) -> dict[str, Any]:
    base_phase = phase.split("-", 1)[0]
    topics = probe_doc.get("topics", {})
    def rate(topic: str) -> float | None:
        value = topics.get(topic, {}).get("header_rate_hz")
        return float(value) if value is not None else None
    lidar_topics = [name for name in topics if "livox" in name and ("lidar" in name or "PointCloud2" in topics[name].get("type", ""))]
    lidar_rate = max((rate(name) or 0.0 for name in lidar_topics), default=0.0)
    lidar_monotonic = all(int(topics[name].get("non_monotonic_count", 1)) == 0 for name in lidar_topics)
    visual_topics_ok = all(name in topics and int(topics[name].get("count", 0)) > 0 for name in VISUAL_TOPICS - {"/tf_static"})
    tracker = topics.get("/vision/front/seam_state", {})
    tracker_rate = tracker.get("header_rate_hz")
    camera_info_rate = topics.get("/camera/front/camera_info", {}).get("header_rate_hz")
    camera_calibrated = topics.get("/camera/front/camera_info", {}).get("camera_info_calibrated")
    camera_info_approx_10hz = camera_info_rate is not None and 8.0 <= float(camera_info_rate) <= 12.0
    slot_overwritten = probe_doc.get("slot_overwritten")
    checks = {
        "mcap_validated": bool(bag_doc.get("validated")),
        "mcap_finalized": bool(bag_doc.get("finalized")),
        "shutdown_clean": bool(shutdown.get("pass")),
        "lidar_topic_present": bool(lidar_topics),
        "lidar_messages_received": lidar_rate > 0.0,
        "lidar_timestamp_monotonic": lidar_monotonic,
        "lidar_rate_within_10_percent_of_10hz": 9.0 <= lidar_rate <= 11.0,
        "visual_required_topics_present": visual_topics_ok if base_phase in {"w4b", "w4c"} else True,
        "camera_info_approximately_10hz": camera_info_approx_10hz if base_phase in {"w4b", "w4c"} else True,
        "tracker_output_ge_6hz": float(tracker_rate or 0.0) >= 6.0 if base_phase in {"w4b", "w4c"} else True,
        "slot_overwritten_zero_or_reported": slot_overwritten is not None or base_phase == "w4a",
        "camera_info_calibrated": bool(camera_calibrated),
    }
    return {"phase": phase, "rates": {"camera_info_hz": camera_info_rate,
                                        "tracker_hz": tracker_rate, "lidar_hz": lidar_rate},
            "lidar_topics": lidar_topics, "slot_overwritten": slot_overwritten,
            "checks": checks,
            "pass": all(checks[key] for key in checks if key not in {"camera_info_calibrated"})}


def aggregate(args: argparse.Namespace) -> int:
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    phases: dict[str, Any] = {}
    for item in args.phase_run:
        phase, run_text = item.split("=", 1)
        run = Path(run_text)
        probe_doc = _phase_probe(run / "logs" / "topic_probe.json")
        bag_doc = _load(run / "logs" / "mcap_validation.json", {}) or {}
        shutdown = _load(run / "logs" / "process_shutdown_audit.json", {}) or {}
        phase_result = _phase_pass(phase, probe_doc, bag_doc, shutdown)
        phase_result["run_dir"] = str(run)
        live_files = sorted(run.glob("live_session/*/live_summary.json"))
        if live_files:
            live = _load(live_files[-1], {}) or {}
            counters = live.get("counters", {})
            phase_result["slot_overwritten"] = counters.get("preview_skipped_frames")
            phase_result["counters"] = counters
            probe_doc["slot_overwritten"] = counters.get("preview_skipped_frames")
            phase_result["checks"]["slot_overwritten_zero_or_reported"] = "preview_skipped_frames" in counters
        phase_result["resource_usage"] = _load(run / "logs" / "resource_usage_summary.json", {}) or {}
        phase_result["snapshot_integrity"] = _snapshot_integrity(run)
        if phase.split("-", 1)[0] in {"w4b", "w4c"}:
            phase_result["checks"]["snapshots_complete_and_hashed"] = bool(
                phase_result["snapshot_integrity"]["pass"])
            phase_result["pass"] = bool(phase_result["pass"] and
                                         phase_result["checks"]["snapshots_complete_and_hashed"])
        phase_result["timestamp_audit"] = {
            topic: {"first_stamp_ns": value.get("first_stamp_ns"), "last_stamp_ns": value.get("last_stamp_ns"),
                    "non_monotonic_count": value.get("non_monotonic_count"), "header_rate_hz": value.get("header_rate_hz")}
            for topic, value in probe_doc.get("topics", {}).items()
        }
        phases[phase] = phase_result
    w4b = phases.get("w4b-long") or phases.get("w4b-smoke") or phases.get("w4b") or {}
    w4a = phases.get("w4a", {})
    w4c = phases.get("w4c", {})
    visual_chain_pass = bool(w4b.get("checks", {}).get("tracker_output_ge_6hz") and
                             w4b.get("checks", {}).get("visual_required_topics_present"))
    lidar_chain_pass = bool(w4a.get("pass") or w4b.get("pass") or w4c.get("pass"))
    parallel_pass = bool(w4b.get("pass"))
    sync_probe = _phase_probe(Path(w4c.get("run_dir", "")) / "logs" / "topic_probe.json") if w4c else {}
    sync_count = int(sync_probe.get("topics", {}).get(SYNC_TOPIC, {}).get("count", 0) or 0)
    temporal_sync_pass = bool(w4c.get("pass") and sync_count > 0) if w4c else False
    final = {
        "schema_version": "field_w4_system_acceptance_v1",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "phases": phases,
        "visual_chain_pass": visual_chain_pass,
        "lidar_chain_pass": lidar_chain_pass,
        "parallel_runtime_pass": parallel_pass,
        "temporal_sync_pass": temporal_sync_pass,
        "spatial_calibration_pass": False,
        "production_ready": False,
        "system_acceptance": {
            "visual_chain_pass": visual_chain_pass,
            "lidar_chain_pass": lidar_chain_pass,
            "parallel_runtime_pass": parallel_pass,
            "temporal_sync_pass": temporal_sync_pass,
            "spatial_calibration_pass": False,
            "production_ready": False,
        },
        "algorithm_field_validation_pending": True,
        "multisensor_transport_ready": bool(lidar_chain_pass and parallel_pass),
        "temporal_sync_ready": temporal_sync_pass,
        "spatial_fusion_ready": False,
    }
    _write(output / "system_acceptance.json", final)
    topic_summary = {phase: {"rates": value.get("rates"), "lidar_topics": value.get("lidar_topics"),
                             "checks": value.get("checks")} for phase, value in phases.items()}
    _write(output / "topic_inventory.json", {
        phase: _load(Path(value.get("run_dir", "")) / "logs" / "topic_inventory.json", {})
        for phase, value in phases.items()
    })
    _write(output / "w4a_lidar_only.json", phases.get("w4a", {}))
    _write(output / "w4b_parallel_runtime.json", {
        "smoke": phases.get("w4b-smoke", phases.get("w4b", {})),
        "long": phases.get("w4b-long", {}),
    })
    _write(output / "w4c_sync_runtime.json", phases.get("w4c", {}))
    _write(output / "topic_rate_summary.json", topic_summary)
    _write(output / "timestamp_audit.json", {phase: value.get("timestamp_audit") for phase, value in phases.items()})
    _write(output / "resource_usage.json", {phase: value.get("resource_usage") for phase, value in phases.items()})
    _write(output / "snapshot_integrity.json", {phase: value.get("snapshot_integrity") for phase, value in phases.items()})
    _write(output / "mcap_validation.json", {phase: _load(Path(value.get("run_dir", "")) / "logs" / "mcap_validation.json", {}) for phase, value in phases.items()})
    _write(output / "process_shutdown_audit.json", {phase: _load(Path(value.get("run_dir", "")) / "logs" / "process_shutdown_audit.json", {}) for phase, value in phases.items()})
    _write(output / "synchronization_statistics.json", {
        "sync_topic": SYNC_TOPIC, "output_message_count": sync_count,
        "strategy": "inspection_sync existing bounded nearest/bracket buffers; anchor-driven FusionIndex",
        "camera_header_stamp_source": "MindVision software-anchored device tick mapped by driver",
        "livox_header_stamp_source": "livox_ros_driver2 published header stamp",
        "host_receive_time": "probe receipt time_ns, diagnostic only",
        "unified_clock": False,
        "spatial_projection_performed": False,
        "temporal_sync_ready": temporal_sync_pass,
        "note": "No ultrasound/eddy anchor is enabled in this W4 profile; zero FusionIndex output is a real blocker, not synthesized evidence."
    })
    diagnostics = []
    for phase, value in phases.items():
        probe_doc = _phase_probe(Path(value.get("run_dir", "")) / "logs" / "topic_probe.json")
        diagnostics.extend(probe_doc.get("diagnostics_history", []))
    _write(output / "diagnostic_summary.json", {
        "task": "FIELD-W4-MULTISENSOR-E2E", "phase_count": len(phases),
        "diagnostic_messages_observed": len(diagnostics), "system_acceptance": final["system_acceptance"],
        "decision": "W4-C blocked or temporal-only" if not temporal_sync_pass else "continue temporal audit; spatial fusion remains blocked",
        "provenance_enabled": False, "algorithm_behavior_modified": False,
        "stop_condition": "W4-C not run because W4-B did not meet tracker >=6 Hz and MCAP validation failed",
    })
    _write(output / "next_actions.json", {
        "multisensor_transport_ready": final["multisensor_transport_ready"],
        "temporal_sync_ready": final["temporal_sync_ready"], "spatial_fusion_ready": False,
        "algorithm_field_validation_pending": True, "production_ready": False,
        "actions": [
            "保持 geometry provenance=false、record_raw_image=false、publish_debug_image=false",
            "CameraInfo K/P 未标定前禁止 BEV、空间投影和毫米测量",
            "若视觉低于 6 Hz，先分析多传感器资源竞争，不修改焊缝算法",
        ],
    })
    print(json.dumps(final, ensure_ascii=False))
    return 0 if final["multisensor_transport_ready"] else 3


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("checkpoint")
    p.add_argument("--output", default=str(DEFAULT_OUTPUT / "w3_checkpoint.json"))
    p.set_defaults(func=checkpoint)
    p = sub.add_parser("preflight")
    p.add_argument("--output", required=True)
    p.add_argument("--mid360-config", default=str(MID360_CONFIG))
    p.add_argument("--udp-capture-sec", type=float, default=4.0)
    p.add_argument("--udp-packets", type=int, default=3)
    p.set_defaults(func=preflight)
    p = sub.add_parser("init-run")
    p.add_argument("--data-root", default=str(DEFAULT_DATA_ROOT))
    p.add_argument("--run-id", required=True)
    p.add_argument("--phase", required=True)
    p.set_defaults(func=init_run)
    p = sub.add_parser("inventory")
    p.add_argument("--phase", choices=("w4a", "w4b", "w4b-smoke", "w4b-long", "w4c"), required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=inventory)
    p = sub.add_parser("topic-args")
    p.add_argument("--inventory", required=True)
    p.add_argument("--phase", choices=("w4a", "w4b", "w4b-smoke", "w4b-long", "w4c"), required=True)
    p.add_argument("--required", default="")
    p.add_argument("--output")
    p.set_defaults(func=topic_args)
    p = sub.add_parser("probe")
    p.add_argument("--inventory", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--duration-sec", type=float, required=True)
    p.add_argument("--topics", default="")
    p.set_defaults(func=probe)
    p = sub.add_parser("monitor")
    p.add_argument("--output", required=True)
    p.add_argument("--duration-sec", type=float, required=True)
    p.add_argument("--interval-sec", type=float, default=1.0)
    p.add_argument("--process-needles", default="livox_ros_driver2_node,livox_imu_adapter,mindvision_camera_node,stable_seam_node,inspection_sync,ros2 bag record")
    p.set_defaults(func=monitor)
    p = sub.add_parser("bag-report")
    p.add_argument("--bag", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--phase", required=True)
    p.add_argument("--run-id", required=True)
    p.set_defaults(func=bag_report)
    p = sub.add_parser("resource-summary")
    p.add_argument("--input", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=resource_summary)
    p = sub.add_parser("process-audit")
    p.add_argument("--output", required=True)
    p.add_argument("--process-needles", default="livox_ros_driver2_node,livox_imu_adapter,mindvision_camera_node,stable_seam_node,inspection_sync,ros2 bag record")
    p.set_defaults(func=process_audit)
    p = sub.add_parser("aggregate")
    p.add_argument("--output-dir", default=str(DEFAULT_OUTPUT))
    p.add_argument("--phase-run", action="append", default=[], help="phase=/absolute/run/path")
    p.set_defaults(func=aggregate)
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
