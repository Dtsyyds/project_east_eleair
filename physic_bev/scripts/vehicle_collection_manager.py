#!/usr/bin/env python3
"""Preflight, initialize, discover, register, and seal FIELD-W5 runs."""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import re
import shutil
import socket
import subprocess
import sys
import time
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parent
VISION_ROOT = ROOT.parent / "tank_vision_inspection"
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import (  # noqa: E402
    atomic_json, command, discover_topics, load_json, parse_bool, process_identity,
    process_matches, sha256_file, topic_verbose, update_manifest, utc_now, validate_run_id,
)


def _findmnt(data_root: Path) -> dict[str, Any]:
    result = command([
        "findmnt", "-J", "-T", str(data_root),
        "-o", "SOURCE,TARGET,FSTYPE,OPTIONS,SIZE,AVAIL,USE%",
    ])
    try:
        item = json.loads(result["stdout"])["filesystems"][0]
    except (json.JSONDecodeError, KeyError, IndexError):
        item = {}
    item["probe"] = result
    return item


def _interface_ipv4(interface: str) -> list[str]:
    result = command(["ip", "-j", "addr", "show", interface])
    try:
        values = json.loads(result["stdout"])
    except json.JSONDecodeError:
        return []
    return [
        item.get("local", "")
        for link in values
        for item in link.get("addr_info", [])
        if item.get("family") == "inet"
    ]


def _video_usb_id(device: Path) -> str | None:
    """Return the USB VID:PID owning a video node, if sysfs exposes it."""
    sysfs = Path("/sys/class/video4linux") / device.name / "device"
    try:
        current = sysfs.resolve(strict=True)
    except OSError:
        return None
    for parent in (current, *current.parents):
        vendor, product = parent / "idVendor", parent / "idProduct"
        if vendor.is_file() and product.is_file():
            try:
                return f"{vendor.read_text().strip()}:{product.read_text().strip()}".lower()
            except OSError:
                return None
    return None


def _storage_benchmark(data_root: Path, size_mib: int) -> dict[str, Any]:
    target = data_root / f".field_w5_storage_probe_{os.getpid()}.partial"
    started = time.monotonic()
    result = command([
        "dd", "if=/dev/zero", f"of={target}", "bs=16M",
        f"count={max(1, size_mib // 16)}", "oflag=direct", "conv=fsync",
    ], timeout=max(60.0, size_mib / 2.0))
    elapsed = time.monotonic() - started
    size = target.stat().st_size if target.is_file() else 0
    try:
        target.unlink()
    except OSError:
        pass
    return {
        "requested_mib": size_mib,
        "written_bytes": size,
        "elapsed_sec": elapsed,
        "write_MBps": size / elapsed / 1_000_000 if elapsed > 0 else None,
        "write_MiBps": size / elapsed / 1024 / 1024 if elapsed > 0 else None,
        "direct_and_fsync": True,
        "success": result["returncode"] == 0 and size > 0,
        "command": result,
    }


def _planned_bandwidth(args: argparse.Namespace) -> dict[str, Any]:
    # Conservative pre-discovery planning values. The live monitor replaces
    # these with measured writer rates once all enabled streams are running.
    components_MBps = {
        "core": 2.0 if args.record_core else 0.0,
        # W6 records only the compact odometry stream.  The large registered
        # cloud and cumulative Path remain live health signals, not bag data.
        "mapping": 0.05 if getattr(args, "record_mapping", False) else 0.0,
        "mid360": 20.0 if args.record_mid360 else 0.0,
        "d405": 70.0 if args.record_d405 else 0.0,
        "eddy": 0.05 if args.record_eddy else 0.0,
        "front_video": args.video_bitrate_mbps / 8.0 if args.record_front_video else 0.0,
        "front_raw_burst": (4096 * 3072 * 3 * 10 / 1_000_000)
        if args.record_front_raw_burst and args.raw_burst_mode == "full" else 0.0,
        "roi_raw_burst": 25.0
        if args.record_front_raw_burst and args.raw_burst_mode == "roi" else 0.0,
    }
    total = sum(components_MBps.values())
    return {
        "basis": "conservative pre-discovery upper bound; replaced by measured writer rates",
        "components_MBps": components_MBps,
        "estimated_total_MBps": total,
        "safety_factor": args.storage_safety_factor,
        "required_sustained_MBps": total * args.storage_safety_factor,
        "estimated_GB_per_hour": total * 3600 / 1000,
    }


def preflight(args: argparse.Namespace) -> int:
    data_root = Path(args.data_root).resolve()
    run_id = validate_run_id(args.run_id)
    run_dir = data_root / "runs" / run_id
    errors: list[str] = []
    warnings: list[str] = []
    mount = _findmnt(data_root)
    source = str(mount.get("source", ""))
    options = str(mount.get("options", "")).split(",")
    if source != "/dev/nvme0n1p1":
        errors.append(f"data root source is {source!r}, expected /dev/nvme0n1p1")
    if mount.get("fstype") != "ext4":
        errors.append(f"data root fstype is {mount.get('fstype')!r}, expected ext4")
    if "rw" not in options:
        errors.append("data root is not mounted rw")
    if not os.access(data_root, os.W_OK):
        errors.append("data root is not writable")
    if source.startswith("/dev/mmc") or mount.get("target") == "/":
        errors.append("data root resolves to root/eMMC instead of the dedicated SSD")
    if run_dir.exists():
        errors.append(f"run path already exists; overwrite is forbidden: {run_dir}")

    route = command(["ip", "route", "get", args.mid360_ip])
    ipv4 = _interface_ipv4(args.mid360_interface)
    network_ready = (
        args.mid360_host_ip in ipv4
        and re.search(rf"\bdev\s+{re.escape(args.mid360_interface)}\b", route["stdout"]) is not None
    )
    if args.record_mid360 and not network_ready:
        errors.append(
            f"MID-360 network is not ready: {args.mid360_interface} addresses={ipv4}, "
            f"route={route['stdout']!r}; expected {args.mid360_host_ip} and direct route"
        )

    eddy_device = Path(args.eddy_device)
    eddy_users = {"argv": ["fuser", str(eddy_device)], "returncode": 1, "stdout": "", "stderr": ""}
    if args.record_eddy and not eddy_device.exists():
        errors.append(f"Eddy device is missing: {eddy_device}")
    elif args.record_eddy and not eddy_device.is_char_device():
        errors.append(f"Eddy device is not a character device: {eddy_device}")
    elif args.record_eddy and not os.access(eddy_device, os.R_OK | os.W_OK):
        errors.append(f"Eddy device is not readable/writable: {eddy_device}")
    elif args.record_eddy:
        eddy_users = command(["fuser", str(eddy_device)])
        owner_pids = eddy_users["stdout"].split()
        if eddy_users["returncode"] == 0 and owner_pids:
            errors.append(
                f"Eddy device is already in use by PID(s) {', '.join(owner_pids)}: "
                f"{eddy_device}"
            )

    d405_nodes = command(["pgrep", "-af", "realsense2_camera_node|rs_launch.py"])
    d405_video_users: dict[str, dict[str, Any]] = {}
    if args.record_d405:
        for device in sorted(Path("/dev").glob("video*")):
            if _video_usb_id(device) != "8086:0b5b":
                continue
            owner = command(["fuser", str(device)])
            if owner["returncode"] == 0 and owner["stdout"].strip():
                d405_video_users[str(device)] = owner
        if d405_nodes["returncode"] == 0 and d405_nodes["stdout"].strip():
            errors.append("a RealSense launch/node is already running; D405 ownership must be single-instance")
        if d405_video_users:
            owners = ", ".join(
                f"{device}={value['stdout'].strip()}" for device, value in d405_video_users.items())
            errors.append(f"one or more video devices are already in use before D405 startup: {owners}")

    for path, label in (
        (Path(args.tracker_config), "tracker config"),
        (Path(args.tracker_reference), "tracker reference"),
        (Path(args.mid360_config), "MID-360 config"),
    ):
        if not path.is_file():
            errors.append(f"missing {label}: {path}")
    if args.provenance_enabled:
        errors.append("provenance_enabled=true is forbidden for formal collection")
    if args.raw_burst_duration_sec > 30:
        errors.append("raw burst duration exceeds the 30 second hard limit")
    if args.record_front_raw_burst and args.raw_burst_duration_sec < 1:
        errors.append("raw burst duration must be at least one second")

    plan = _planned_bandwidth(args)
    benchmark = _storage_benchmark(data_root, args.benchmark_size_mib)
    if not benchmark["success"]:
        errors.append("direct SSD write benchmark failed")
    elif float(benchmark["write_MBps"] or 0) < float(plan["required_sustained_MBps"]):
        errors.append(
            f"SSD benchmark {benchmark['write_MBps']:.1f} MB/s is below planned "
            f"{plan['required_sustained_MBps']:.1f} MB/s with safety margin"
        )
    stat = os.statvfs(data_root)
    free_bytes = stat.f_bavail * stat.f_frsize
    safe_hours = (
        free_bytes / (plan["estimated_total_MBps"] * 1_000_000) / 3600
        if plan["estimated_total_MBps"] > 0 else None
    )
    required_packages = [
        "mindvision_camera_driver", "inspection_bringup", "tank_vision_ros",
        "eddy_driver",
    ]
    if args.record_d405:
        required_packages.append("realsense2_camera")
    if args.record_mid360:
        required_packages.append("livox_ros_driver2")
    packages = {
        name: command(["ros2", "pkg", "prefix", name])
        for name in required_packages
    }
    if any(value["returncode"] != 0 for value in packages.values()):
        errors.append("one or more required ROS packages are not discoverable")
    report = {
        "schema_version": "field_w5_preflight_v1",
        "created_at": utc_now(),
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "run_id": run_id,
        "run_dir": str(run_dir),
        "mount": mount,
        "storage_benchmark": benchmark,
        "bandwidth_plan": plan,
        "free_bytes": free_bytes,
        "estimated_safe_recording_hours": safe_hours,
        "network": {
            "interface": args.mid360_interface, "ipv4": ipv4,
            "host_ip_required": args.mid360_host_ip, "lidar_ip": args.mid360_ip,
            "route": route, "ready": network_ready,
        },
        "eddy": {
            "enabled": args.record_eddy,
            "device": str(eddy_device),
            "exists": eddy_device.exists(),
            "character_device": eddy_device.is_char_device() if eddy_device.exists() else False,
            "users": eddy_users,
        },
        "d405_ownership": {
            "enabled": args.record_d405,
            "process_probe": d405_nodes,
            "video_device_users": d405_video_users,
            "single_instance_ready": not (
                (d405_nodes["returncode"] == 0 and d405_nodes["stdout"].strip())
                or d405_video_users
            ),
        },
        "packages": packages,
        "formal_defaults": {
            "provenance_enabled": args.provenance_enabled,
            "publish_debug_image": False,
            "roi_validation_recorder": args.record_front_raw_burst and args.raw_burst_mode == "roi",
            "raw_image_mcap": args.record_front_raw_burst and args.raw_burst_mode == "full",
            "auto_failure_snapshots": args.auto_save_failures,
        },
    }
    atomic_json(Path(args.output), report)
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["valid"] else 2


def initialize(args: argparse.Namespace) -> int:
    data_root = Path(args.data_root).resolve()
    run_dir = data_root / "runs" / validate_run_id(args.run_id)
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.is_file():
        raise RuntimeError(f"visual launch has not initialized the run: {manifest_path}")
    for relative in (
        "mcap", ".partial", "video/front", "failure_snapshots", "roi_validation",
        "logs/pids", "reports", "frozen_configs",
    ):
        (run_dir / relative).mkdir(parents=True, exist_ok=True)
    notes = run_dir / "field_notes.md"
    if not notes.exists():
        notes.write_text(f"# Field notes: {args.run_id}\n\n", encoding="utf-8")
    source_files = [Path(value).resolve() for value in args.freeze_file]
    frozen_sources = []
    for source in source_files:
        if not source.is_file():
            continue
        destination = run_dir / "frozen_configs" / "sources" / source.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        if destination.exists():
            destination = destination.with_name(f"{source.parent.name}_{source.name}")
        shutil.copy2(source, destination)
        frozen_sources.append({
            "source": str(source),
            "frozen": str(destination.relative_to(run_dir)),
            "sha256": sha256_file(destination),
        })
    inventory = {
        "created_at": utc_now(),
        "hostname": socket.gethostname(),
        "platform": platform.platform(),
        "machine": platform.machine(),
        "kernel": platform.release(),
        "camera_serial": args.camera_serial.removeprefix("sn_"),
        "mid360_ip": args.mid360_ip,
        "d405": "runtime enumeration required",
        "eddy_device": args.eddy_device,
        "mount": _findmnt(data_root),
        "usb": command(["lsusb"]),
    }
    atomic_json(run_dir / "device_inventory.json", inventory)
    enabled = {
        "core": args.record_core,
        "mapping": args.record_mapping,
        "mid360": args.record_mid360,
        "d405": args.record_d405,
        "eddy": args.record_eddy,
        "front_video": args.record_front_video,
        "failure_snapshots": args.auto_save_failures,
        "tracker": args.tracker_enabled,
        "front_raw_burst": args.record_front_raw_burst,
        "raw_burst_mode": args.raw_burst_mode,
    }
    update_manifest(
        run_dir,
        schema_version=("field_w6_mapping_eddy_collection_run_v1"
                        if args.collection_profile == "mapping_eddy"
                        else "field_w5_vehicle_collection_run_v1"),
        collection_profile=args.collection_profile,
        collection_ros_domain_id=args.ros_domain_id,
        enabled_writers=enabled,
        frozen_sources=frozen_sources,
        camera_info_calibration_complete=False,
        spatial_calibration_complete=False,
        hardware_synchronization_proven=False,
        geometry_provenance_sampling_enabled=False,
        d405_capture_profile={
            "enable_depth": args.record_d405,
            "enable_infra1": args.d405_enable_infra1,
            "enable_infra2": args.d405_enable_infra2,
            "depth_profile": args.d405_depth_profile,
            "infra_profile": args.d405_infra_profile,
            "expected_fps": args.d405_expected_fps,
            "global_time_enabled": args.d405_global_time_enabled,
            "error_polling_enabled": args.d405_error_polling_enabled,
        },
        eddy_capture_profile={
            "device": args.eddy_device,
            "recorded_topic": args.eddy_channel_mean_topic,
            "recorded_type": "eddy_driver/msg/EddyChannelMean",
            "raw_frame_topic": "/eddy/data" if args.collection_profile == "mapping_eddy" else None,
            "raw_frame_type": ("eddy_driver/msg/EddyFrame"
                               if args.collection_profile == "mapping_eddy" else None),
            "record_raw_frame": args.collection_profile == "mapping_eddy",
            "record_status": args.collection_profile == "mapping_eddy",
            "fields": ["header", "channel_mean"],
        },
        mapping_capture_profile={
            "attach_to_existing_mapping": args.attach_to_existing_mapping,
            "start_mapping": args.start_mapping,
            "online_required_topics": {
                "/cloud_registered": "sensor_msgs/msg/PointCloud2",
                "/path": "nav_msgs/msg/Path",
                "/aft_mapped_to_init": "nav_msgs/msg/Odometry",
            },
            "recorded_topics": {
                "/aft_mapped_to_init": "nav_msgs/msg/Odometry",
            },
            "storage_policy": "compact_odometry_only",
            "offline_trajectory": "reports/compact_trajectory.jsonl",
            "record_mid360_raw": False if args.collection_profile == "mapping_eddy" else args.record_mid360,
        },
        status={"complete": False, "finalized": False, "validation": "pending"},
    )
    atomic_json(run_dir / "complete.json", {
        "complete": False, "finalized": False, "validation": "pending",
        "updated_at": utc_now(),
    })
    print(run_dir)
    return 0


def inventory(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir).resolve()
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    mapping_profile = manifest.get("collection_profile") == "mapping_eddy"
    topics = discover_topics()
    selected_verbose = [
        topic for topic in sorted(topics)
        if topic.startswith(("/camera/front/", "/d405/", "/livox/", "/eddy/", "/vision/front/"))
        or topic in {"/cloud_registered", "/path", "/aft_mapped_to_init"}
        or topic in {"/diagnostics", "/tf", "/tf_static", "/field/collection_status"}
    ]
    verbose: dict[str, Any] = {}
    # ROS graph queries can each wait for daemon discovery. Querying them in a
    # bounded pool keeps startup deterministic without skipping actual QoS.
    with ThreadPoolExecutor(max_workers=8) as pool:
        futures = {pool.submit(topic_verbose, topic): topic for topic in selected_verbose}
        for future in as_completed(futures):
            topic = futures[future]
            try:
                verbose[topic] = future.result()
            except Exception as exc:
                verbose[topic] = {"returncode": None, "stdout": "", "stderr": str(exc)}
    d405 = {
        name: kind for name, kind in topics.items()
        if name.startswith("/d405/")
        and ("Image" in kind or "CameraInfo" in kind or "Metadata" in kind
             or name.endswith("depth_scale") or "/extrinsics/" in name)
    }
    mid = {
        name: kind for name, kind in topics.items()
        if name.startswith("/livox/")
    }
    eddy = {
        name: kind for name, kind in topics.items()
        if kind == "eddy_driver/msg/EddyChannelMean"
    }
    if mapping_profile and topics.get("/eddy/data") == "eddy_driver/msg/EddyFrame":
        eddy["/eddy/data"] = topics["/eddy/data"]
    if mapping_profile and topics.get("/eddy/status") == "eddy_driver/msg/EddyStatus":
        eddy["/eddy/status"] = topics["/eddy/status"]
    mapping_sources = {
        name: topics[name] for name in ("/cloud_registered", "/path", "/aft_mapped_to_init")
        if name in topics
    }
    # The W6 compact policy deliberately records only odometry.  The cloud and
    # cumulative Path are still inventoried and monitored to prove FAST-LIVO is
    # healthy, but writing them would reintroduce unbounded Path growth.
    mapping = ({
        "/aft_mapped_to_init": topics["/aft_mapped_to_init"],
    } if mapping_profile and "/aft_mapped_to_init" in topics else mapping_sources)
    core_candidates = {
        "/vision/front/seam_geometry", "/vision/front/seam_state", "/diagnostics",
        "/tf", "/tf_static", "/camera/front/camera_info", "/field/collection_status",
    }
    if mapping_profile:
        core_candidates.difference_update({"/tf", "/tf_static"})
    core_candidates.update(name for name, kind in d405.items() if "CameraInfo" in kind)
    report = {
        "schema_version": "field_w5_runtime_topic_inventory_v1",
        "created_at": utc_now(),
        "all_topics": topics,
        "topic_info_verbose": verbose,
        "record_groups": {
            "core": {name: topics[name] for name in sorted(core_candidates) if name in topics},
            "mapping": mapping,
            "mid360": mid,
            "d405": d405,
            "eddy": eddy,
        },
        "mapping_sources": mapping_sources,
        "mapping_storage_policy": (
            "compact_odometry_only" if mapping_profile else "record_discovered_mapping_topics"
        ),
        "d405_imu_assumed": False,
        "diagnostics_validation_grouping": [
            "status.name", "hardware_id", "publisher_gid_when_available",
        ],
    }
    atomic_json(run_dir / "topic_inventory.json", report)
    for group, values in report["record_groups"].items():
        target = run_dir / "logs" / f"{group}_topics.txt"
        target.write_text("".join(f"{name}\n" for name in sorted(values)), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False))
    return 0


def add_runtime_status(args: argparse.Namespace) -> int:
    """Add the monitor topic after it exists without repeating full graph probes."""
    run_dir = Path(args.run_dir).resolve()
    path = run_dir / "topic_inventory.json"
    report = load_json(path, {}) or {}
    discovered = discover_topics()
    type_name = discovered.get("/field/collection_status")
    if type_name != "std_msgs/msg/String":
        raise RuntimeError(
            "runtime status topic is unavailable or has unexpected type: "
            f"{type_name!r}"
        )
    report.setdefault("all_topics", {})["/field/collection_status"] = type_name
    report.setdefault("record_groups", {}).setdefault("core", {})[
        "/field/collection_status"
    ] = type_name
    report.setdefault("topic_info_verbose", {})[
        "/field/collection_status"
    ] = topic_verbose("/field/collection_status")
    report["runtime_status_added_at"] = utc_now()
    atomic_json(path, report)
    core = report["record_groups"]["core"]
    (run_dir / "logs" / "core_topics.txt").write_text(
        "".join(f"{name}\n" for name in sorted(core)), encoding="utf-8"
    )
    print(json.dumps({
        "topic": "/field/collection_status", "type": type_name,
        "added": True,
    }, ensure_ascii=False))
    return 0


def register(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir).resolve()
    registry_path = run_dir / "logs" / "process_registry.json"
    registry = load_json(registry_path, {"schema_version": "field_w5_process_registry_v1", "processes": []})
    item = process_identity(args.pid, args.name)
    registry["processes"] = [
        value for value in registry.get("processes", []) if value.get("name") != args.name
    ] + [item]
    atomic_json(registry_path, registry)
    (run_dir / "logs" / "pids" / f"{args.name}.pid").write_text(f"{args.pid}\n", encoding="ascii")
    print(json.dumps(item, ensure_ascii=False))
    return 0


def seal(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir).resolve()
    results: dict[str, Any] = {}
    for name in args.writer:
        partial = run_dir / ".partial" / name
        final = run_dir / "mcap" / name
        already_sealed = final.is_dir()
        conflict = already_sealed and partial.exists()
        candidate = final if already_sealed else partial
        result = command(["ros2", "bag", "info", str(candidate)], timeout=120.0)
        metadata = candidate / "metadata.yaml"
        valid = (
            candidate.is_dir() and metadata.is_file()
            and result["returncode"] == 0 and not conflict
        )
        if valid and not already_sealed:
            os.replace(partial, final)
        results[name] = {
            "valid": valid, "sealed": valid and final.is_dir(),
            "already_sealed": already_sealed, "partial_final_conflict": conflict,
            "partial": str(partial), "final": str(final), "bag_info": result,
        }
    atomic_json(run_dir / "reports" / "seal_report.json", results)
    print(json.dumps(results, ensure_ascii=False))
    return 0 if all(value["sealed"] for value in results.values()) else 2


def active_run(args: argparse.Namespace) -> int:
    """Resolve exactly one run which still owns a registered live process."""
    runs_dir = Path(args.data_root).resolve() / "runs"
    active: list[dict[str, Any]] = []
    if runs_dir.is_dir():
        for run_dir in sorted(runs_dir.iterdir()):
            registry = load_json(run_dir / "logs" / "process_registry.json", {}) or {}
            live = [
                item.get("name") for item in registry.get("processes", [])
                if process_matches(item)
            ]
            if live:
                active.append({"run_id": run_dir.name, "live_processes": live})
    if len(active) != 1:
        print(json.dumps({"active_runs": active}, ensure_ascii=False), file=sys.stderr)
        if not active:
            print("no active vehicle-collection run found", file=sys.stderr)
        else:
            print("multiple active runs found; specify RUN_ID explicitly", file=sys.stderr)
        return 2
    print(active[0]["run_id"])
    return 0


def assert_no_active(args: argparse.Namespace) -> int:
    runs_dir = Path(args.data_root).resolve() / "runs"
    active = []
    if runs_dir.is_dir():
        for run_dir in sorted(runs_dir.iterdir()):
            registry = load_json(run_dir / "logs" / "process_registry.json", {}) or {}
            live = [item.get("name") for item in registry.get("processes", []) if process_matches(item)]
            if live:
                active.append({"run_id": run_dir.name, "live_processes": live})
    print(json.dumps({"active_runs": active}, ensure_ascii=False))
    return 0 if not active else 2


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--data-root", default="/media/mcap_data")
    parser.add_argument("--run-id", required=True)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("preflight")
    _add_common(p)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--tracker-config", required=True)
    p.add_argument("--tracker-reference", required=True)
    p.add_argument("--mid360-config", required=True)
    p.add_argument("--mid360-interface", default="eth0")
    p.add_argument("--mid360-host-ip", default="192.168.2.100")
    p.add_argument("--mid360-ip", default="192.168.2.3")
    p.add_argument("--eddy-device", default="/dev/eddy")
    for name in ("record-core", "record-mid360", "record-d405", "record-eddy", "record-front-video",
                 "auto-save-failures", "provenance-enabled", "record-front-raw-burst"):
        p.add_argument(f"--{name}", type=parse_bool, required=True)
    p.add_argument("--record-mapping", type=parse_bool, default=False)
    p.add_argument("--raw-burst-mode", choices=("full", "roi"), default="full")
    p.add_argument("--raw-burst-duration-sec", type=int, default=10)
    p.add_argument("--video-bitrate-mbps", type=float, default=12.0)
    p.add_argument("--storage-safety-factor", type=float, default=1.5)
    p.add_argument("--benchmark-size-mib", type=int, default=512)
    p.set_defaults(func=preflight)
    p = sub.add_parser("initialize")
    _add_common(p)
    p.add_argument("--camera-serial", required=True)
    p.add_argument("--mid360-ip", default="192.168.2.3")
    for name in ("record-core", "record-mid360", "record-d405", "record-eddy", "record-front-video",
                 "auto-save-failures", "record-front-raw-burst", "tracker-enabled"):
        p.add_argument(f"--{name}", type=parse_bool, required=True)
    p.add_argument("--record-mapping", type=parse_bool, default=False)
    p.add_argument("--raw-burst-mode", choices=("full", "roi"), default="full")
    p.add_argument("--d405-enable-infra1", type=parse_bool, default=True)
    p.add_argument("--d405-enable-infra2", type=parse_bool, default=True)
    p.add_argument("--d405-depth-profile", default="848,480,30")
    p.add_argument("--d405-infra-profile", default="848,480,30")
    p.add_argument("--d405-expected-fps", type=float, default=30.0)
    p.add_argument("--d405-global-time-enabled", type=parse_bool, default=False)
    p.add_argument("--d405-error-polling-enabled", type=parse_bool, default=False)
    p.add_argument("--eddy-device", default="/dev/eddy")
    p.add_argument("--eddy-channel-mean-topic", default="/eddy/channel_mean")
    p.add_argument("--collection-profile", choices=("vehicle_multisensor_split_recorders", "mapping_eddy"),
                   default="vehicle_multisensor_split_recorders")
    p.add_argument("--ros-domain-id", type=int, default=0)
    p.add_argument("--attach-to-existing-mapping", type=parse_bool, default=True)
    p.add_argument("--start-mapping", type=parse_bool, default=False)
    p.add_argument("--freeze-file", action="append", default=[])
    p.set_defaults(func=initialize)
    p = sub.add_parser("inventory")
    p.add_argument("--run-dir", required=True)
    p.set_defaults(func=inventory)
    p = sub.add_parser("add-runtime-status")
    p.add_argument("--run-dir", required=True)
    p.set_defaults(func=add_runtime_status)
    p = sub.add_parser("register")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--name", required=True)
    p.add_argument("--pid", type=int, required=True)
    p.set_defaults(func=register)
    p = sub.add_parser("seal")
    p.add_argument("--run-dir", required=True)
    p.add_argument("--writer", action="append", default=[])
    p.set_defaults(func=seal)
    p = sub.add_parser("active-run")
    p.add_argument("--data-root", default="/media/mcap_data")
    p.set_defaults(func=active_run)
    p = sub.add_parser("assert-no-active")
    p.add_argument("--data-root", default="/media/mcap_data")
    p.set_defaults(func=assert_no_active)
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
