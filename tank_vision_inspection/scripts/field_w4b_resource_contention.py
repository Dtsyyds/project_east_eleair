#!/usr/bin/env python3
"""FIELD-W4-B-RESOURCE-CONTENTION-ISOLATION analysis helpers.

The experiment runner keeps the frozen visual chain and the existing Livox
driver black-boxed.  It only observes processes, topics, bags, and timestamps.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import re
import statistics
import subprocess
import time
from typing import Any

import field_w4_multisensor_e2e as base


ROOT = base.ROOT
OUTPUT = ROOT / "output" / "field_w4b_resource_contention"
DATA_ROOT = Path(os.environ.get("DATA_ROOT", "/media/mcap_data"))
VISUAL_TOPICS = [
    "/camera/front/camera_info", "/vision/front/seam_geometry",
    "/vision/front/seam_state", "/diagnostics", "/tf_static",
]
REQUIRED_LABELS = ("B0", "B1", "B2", "B3", "B4")


def _run(argv: list[str], timeout: float = 30.0) -> dict[str, Any]:
    try:
        result = subprocess.run(argv, capture_output=True, text=True, timeout=timeout, check=False)
        return {"argv": argv, "returncode": result.returncode,
                "stdout": result.stdout, "stderr": result.stderr}
    except (OSError, subprocess.SubprocessError) as exc:
        return {"argv": argv, "returncode": None, "stdout": "", "stderr": str(exc)}


def _load(path: Path, default: Any = None) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return default


def _write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "p50": None, "p95": None, "max": None, "mean": None}
    ordered = sorted(values)

    def percentile(q: float) -> float:
        position = (len(ordered) - 1) * q
        low = int(position)
        high = min(low + 1, len(ordered) - 1)
        return ordered[low] + (ordered[high] - ordered[low]) * (position - low)

    return {"count": len(ordered), "p50": percentile(.5), "p95": percentile(.95),
            "max": max(ordered), "mean": statistics.fmean(ordered)}


def _proc_status(pid: int) -> dict[str, int]:
    result = {"threads": 0, "voluntary_context_switches": 0,
              "nonvoluntary_context_switches": 0}
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="utf-8", errors="replace").splitlines():
            key, _, value = line.partition(":")
            value = value.strip().split()[0] if value.strip() else "0"
            if key == "Threads":
                result["threads"] = int(value)
            elif key == "voluntary_ctxt_switches":
                result["voluntary_context_switches"] = int(value)
            elif key == "nonvoluntary_ctxt_switches":
                result["nonvoluntary_context_switches"] = int(value)
    except (OSError, ValueError):
        pass
    return result


def _cpu_counters() -> dict[str, tuple[int, int]]:
    result: dict[str, tuple[int, int]] = {}
    try:
        lines = Path("/proc/stat").read_text(encoding="utf-8").splitlines()
    except OSError:
        return result
    for line in lines:
        fields = line.split()
        if not fields or not re.fullmatch(r"cpu\d*", fields[0]):
            continue
        values = [int(value) for value in fields[1:]]
        if len(values) < 4:
            continue
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        result[fields[0]] = (sum(values), idle)
    return result


def _temperatures() -> dict[str, float]:
    result = {}
    for path in sorted(Path("/sys/class/thermal").glob("thermal_zone*/temp")):
        try:
            value = float(path.read_text().strip())
            result[path.parent.name] = value / 1000.0 if value > 200 else value
        except (OSError, ValueError):
            pass
    return result


def _frequencies() -> dict[str, float]:
    result = {}
    for path in sorted(Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq")):
        try:
            result[path.parent.parent.name] = float(path.read_text().strip()) / 1000.0
        except (OSError, ValueError):
            pass
    return result


def _role(command: str, visual_bag: str, livox_bag: str) -> str | None:
    if visual_bag and visual_bag in command:
        return "rosbag2_visual_recorder"
    if livox_bag and livox_bag in command:
        return "rosbag2_livox_recorder"
    if "mindvision_camera_node" in command:
        return "mindvision_camera_node"
    if "stable_seam_node" in command:
        return "stable_seam_node"
    if "livox_ros_driver2_node" in command:
        return "livox_ros_driver2"
    if "ros2 bag record" in command or "rosbag2_transport" in command:
        return "rosbag2_other_recorder"
    if "ros2 launch" in command or "inspection_bringup" in command:
        return "launch_aux"
    return None


def _bag_bytes(paths: list[Path]) -> dict[str, int]:
    result = {}
    for path in paths:
        total = 0
        if path.exists():
            for item in path.rglob("*"):
                if item.is_file():
                    try:
                        total += item.stat().st_size
                    except OSError:
                        pass
        result[str(path)] = total
    return result


def resource_monitor(args: argparse.Namespace) -> int:
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    visual_bag = str(Path(args.visual_bag).resolve()) if args.visual_bag else ""
    livox_bag = str(Path(args.livox_bag).resolve()) if args.livox_bag else ""
    bag_paths = [Path(value) for value in (args.visual_bag, args.livox_bag) if value]
    previous_cpu = _cpu_counters()
    previous_time = time.monotonic()
    deadline = previous_time + max(0.1, float(args.duration_sec))
    with output.open("w", encoding="utf-8") as handle:
        while True:
            now = time.monotonic()
            process_result = _run(["ps", "-eo", "pid=,pcpu=,rss=,nlwp=,args="], timeout=5.0)
            rows = []
            for line in process_result["stdout"].splitlines():
                parts = line.strip().split(None, 4)
                if len(parts) != 5:
                    continue
                try:
                    pid = int(parts[0])
                    command = parts[4]
                    role = _role(command, visual_bag, livox_bag)
                    if role is None:
                        continue
                    status = _proc_status(pid)
                    rows.append({"role": role, "pid": pid, "cpu_percent": float(parts[1]),
                                 "rss_kib": int(parts[2]), "threads_ps": int(parts[3]),
                                 **status, "command": command})
                except (ValueError, OSError):
                    continue
            current_cpu = _cpu_counters()
            core_usage = {}
            for name, (total, idle) in current_cpu.items():
                if name not in previous_cpu:
                    continue
                old_total, old_idle = previous_cpu[name]
                delta_total, delta_idle = total - old_total, idle - old_idle
                core_usage[name] = ((delta_total - delta_idle) / delta_total * 100.0
                                    if delta_total > 0 else 0.0)
            bag_sizes = _bag_bytes(bag_paths)
            record = {"host_time_ns": time.time_ns(), "elapsed_sec": now,
                      "processes": rows, "per_core_cpu_percent": core_usage,
                      "temperatures_c": _temperatures(), "cpu_frequencies_mhz": _frequencies(),
                      "bag_bytes": bag_sizes}
            handle.write(json.dumps(record, ensure_ascii=False) + "\n")
            handle.flush()
            previous_cpu = current_cpu
            previous_time = now
            if now >= deadline:
                break
            time.sleep(max(0.05, float(args.interval_sec)))
    return 0


def resource_summary(args: argparse.Namespace) -> int:
    samples = []
    for line in Path(args.input).read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            samples.append(json.loads(line))
        except json.JSONDecodeError:
            pass
    by_role: dict[str, list[dict[str, Any]]] = {}
    for sample in samples:
        for row in sample.get("processes", []):
            by_role.setdefault(row["role"], []).append(row)
    roles = {}
    for role, rows in sorted(by_role.items()):
        roles[role] = {
            "sample_count": len(rows), "pids": sorted({row["pid"] for row in rows}),
            "cpu_percent": _stats([float(row["cpu_percent"]) for row in rows]),
            "rss_kib": _stats([float(row["rss_kib"]) for row in rows]),
            "threads": _stats([float(row.get("threads", row.get("threads_ps", 0))) for row in rows]),
            "context_switches_total": {
                "voluntary": max((int(row.get("voluntary_context_switches", 0)) for row in rows), default=0),
                "nonvoluntary": max((int(row.get("nonvoluntary_context_switches", 0)) for row in rows), default=0),
            },
        }
    core_values: dict[str, list[float]] = {}
    for sample in samples:
        for core, value in sample.get("per_core_cpu_percent", {}).items():
            core_values.setdefault(core, []).append(float(value))
    bag_rate = {}
    for path in sorted({path for sample in samples for path in sample.get("bag_bytes", {})}):
        points = [(float(sample["elapsed_sec"]), int(sample.get("bag_bytes", {}).get(path, 0)))
                  for sample in samples if path in sample.get("bag_bytes", {})]
        rates = []
        for (t0, b0), (t1, b1) in zip(points, points[1:]):
            if t1 > t0 and b1 >= b0:
                rates.append((b1 - b0) / (t1 - t0) / (1024 * 1024))
        bag_rate[path] = _stats(rates)
    report = {"input": str(args.input), "sample_count": len(samples), "processes": roles,
              "per_core_cpu_percent": {core: _stats(values) for core, values in core_values.items()},
              "bag_write_mib_s": bag_rate,
              "temperature_c": _stats([float(value) for sample in samples for value in sample.get("temperatures_c", {}).values()]),
              "cpu_frequency_mhz": _stats([float(value) for sample in samples for value in sample.get("cpu_frequencies_mhz", {}).values()])}
    _write(Path(args.output), report)
    if args.csv:
        with Path(args.csv).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=["role", "sample_count", "cpu_p50", "cpu_p95", "cpu_max",
                                                        "rss_p50_kib", "rss_p95_kib", "rss_max_kib", "threads_p50",
                                                        "threads_p95", "threads_max", "voluntary_context_switches",
                                                        "nonvoluntary_context_switches"])
            writer.writeheader()
            for role, value in roles.items():
                writer.writerow({"role": role, "sample_count": value["sample_count"],
                                 "cpu_p50": value["cpu_percent"]["p50"], "cpu_p95": value["cpu_percent"]["p95"],
                                 "cpu_max": value["cpu_percent"]["max"], "rss_p50_kib": value["rss_kib"]["p50"],
                                 "rss_p95_kib": value["rss_kib"]["p95"], "rss_max_kib": value["rss_kib"]["max"],
                                 "threads_p50": value["threads"]["p50"], "threads_p95": value["threads"]["p95"],
                                 "threads_max": value["threads"]["max"],
                                 "voluntary_context_switches": value["context_switches_total"]["voluntary"],
                                 "nonvoluntary_context_switches": value["context_switches_total"]["nonvoluntary"]})
    return 0


def _validate_bag(path: Path, run_id: str, label: str, audit: dict[str, Any]) -> dict[str, Any]:
    raw_path = path / "raw_validation_report.json"
    command = _run(["ros2", "run", "inspection_tools", "validate_bag", str(path),
                    "--allow-partial", "--output", str(raw_path)], timeout=180.0)
    raw = _load(raw_path, {}) or {}
    errors = list(raw.get("errors", []))
    only_diag_regression = bool(errors) and all(str(error).startswith("/diagnostics:") for error in errors)
    aggregate_interleave = audit.get("classification") == "aggregate_multi_publisher_interleaving"
    effective = bool(raw.get("valid")) or (only_diag_regression and aggregate_interleave and
                                             not audit.get("raw_validator_relevant", True))
    effective_path = path / "effective_validation_report.json"
    effective_doc = dict(raw)
    effective_doc.update({"raw_validator_valid": bool(raw.get("valid")),
                          "effective_valid": effective, "policy": "diagnostics_source_aware_v1",
                          "diagnostics_audit_classification": audit.get("classification"),
                          "accepted_as_cross_publisher_interleaving": bool(effective and not raw.get("valid")),
                          "raw_errors": errors})
    if effective and not raw.get("valid"):
        effective_doc["valid"] = True
        effective_doc["errors"] = []
    _write(effective_path, effective_doc)
    finalize = None
    if effective:
        finalize = _run(["ros2", "run", "inspection_tools", "finalize_bag", str(path),
                         "--task-id", run_id, "--asset-id", "tank_multisensor",
                         "--course-id", "field_w4b_resource_contention", "--plate-id", label,
                         "--validation-report", str(effective_path)], timeout=180.0)
    return {"bag": str(path), "validate_command": command, "raw_validation": raw,
            "effective_validation": effective_doc, "finalize_command": finalize,
            "raw_valid": bool(raw.get("valid")), "effective_valid": effective,
            "finalized": bool(finalize and finalize.get("returncode") == 0 and (path / "manifest.yaml").is_file())}


def validate(args: argparse.Namespace) -> int:
    run = Path(args.run_dir)
    probe = _load(run / "logs" / "topic_probe.json", {}) or {}
    audit = probe.get("diagnostics_timestamp_audit", {"classification": "unavailable"})
    bags = {}
    for spec in args.bag:
        label, path = spec.split("=", 1)
        bags[label] = _validate_bag(Path(path), run.name, args.phase, audit)
    _write(Path(args.output), {"phase": args.phase, "run_dir": str(run),
                               "diagnostics_timestamp_audit": audit, "bags": bags,
                               "recording_requested": bool(bags),
                               "recording_ready": all(item["effective_valid"] and item["finalized"] for item in bags.values()) if bags else False})
    return 0 if all(item["effective_valid"] for item in bags.values()) else 3


def _phase(run: Path, label: str) -> dict[str, Any]:
    probe = _load(run / "logs" / "topic_probe.json", {}) or {}
    live_files = sorted(run.glob("live_session/*/live_summary.json"))
    live = _load(live_files[-1], {}) if live_files else {}
    live = live or {}
    topics = probe.get("topics", {})
    def hz(topic: str) -> float | None:
        value = topics.get(topic, {}).get("header_rate_hz")
        return float(value) if value is not None else None
    counters = live.get("counters", {})
    timing = live.get("performance_timing", {})
    timing.update({"processing_ms": live.get("processing_ms", {})})
    return {"label": label, "run_dir": str(run),
            "camera_info_hz": hz("/camera/front/camera_info"),
            "tracker_output_hz": hz("/vision/front/seam_state"),
            "live_camera_hz": live.get("camera_fps"), "live_tracker_hz": live.get("tracker_fps"),
            "lidar_hz": max((hz(name) or 0.0 for name in topics if "livox" in name and "lidar" in name), default=None),
            "imu_hz": max((hz(name) or 0.0 for name in topics if "livox" in name and "imu" in name), default=None),
            "slot_overwritten": counters.get("preview_skipped_frames"), "counters": counters,
            "callback_timing": timing,
            "diagnostics_timestamp_audit": probe.get("diagnostics_timestamp_audit", {}),
            "topics": {name: {key: value for key, value in item.items() if key in {
                "type", "count", "header_rate_hz", "arrival_rate_hz", "non_monotonic_count",
                "arrival_non_monotonic_count", "receive_non_monotonic_count"}}
                       for name, item in topics.items()}}


def aggregate(args: argparse.Namespace) -> int:
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    phases = {}
    for spec in args.phase_run:
        label, run_text = spec.split("=", 1)
        phases[label] = _phase(Path(run_text), label)
        phases[label]["mcap_validation"] = _load(Path(run_text) / "logs" / "mcap_validation.json", {}) or {}
        phases[label]["shutdown"] = _load(Path(run_text) / "logs" / "process_shutdown_audit.json", {}) or {}
        phases[label]["resources"] = _load(Path(run_text) / "logs" / "resource_summary.json", {}) or {}
    _write(output / "experiment_matrix.json", {
        "schema_version": "field_w4b_resource_contention_matrix_v1",
        "created_at": datetime.now(timezone.utc).isoformat(), "phases": phases,
        "common_constraints": {"geometry_provenance": False, "debug_image": False,
                                "roi_recorder": False, "inspection_sync": False,
                                "fast_livo2": False, "bev": False, "d405": False,
                                "algorithm_modified": False},
    })
    rows = []
    for label, phase in phases.items():
        for role, value in phase.get("resources", {}).get("processes", {}).items():
            rows.append({"experiment": label, "role": role, "sample_count": value.get("sample_count"),
                         "cpu_p50": value.get("cpu_percent", {}).get("p50"),
                         "cpu_p95": value.get("cpu_percent", {}).get("p95"),
                         "cpu_max": value.get("cpu_percent", {}).get("max"),
                         "rss_p50_kib": value.get("rss_kib", {}).get("p50"),
                         "rss_p95_kib": value.get("rss_kib", {}).get("p95"),
                         "rss_max_kib": value.get("rss_kib", {}).get("max"),
                         "threads_p95": value.get("threads", {}).get("p95"),
                         "threads_max": value.get("threads", {}).get("max"),
                         "voluntary_context_switches": value.get("context_switches_total", {}).get("voluntary"),
                         "nonvoluntary_context_switches": value.get("context_switches_total", {}).get("nonvoluntary")})
    with (output / "per_process_resources.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = sorted({key for row in rows for key in row}) if rows else ["experiment", "role"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    _write(output / "topic_rate_comparison.json", {label: {key: phase.get(key) for key in (
        "camera_info_hz", "tracker_output_hz", "live_camera_hz", "live_tracker_hz", "lidar_hz", "imu_hz",
        "slot_overwritten")} for label, phase in phases.items()})
    _write(output / "callback_timing.json", {label: phase.get("callback_timing", {}) for label, phase in phases.items()})
    _write(output / "diagnostics_timestamp_audit.json", {label: phase.get("diagnostics_timestamp_audit", {}) for label, phase in phases.items()})
    _write(output / "mcap_comparison.json", {label: phase.get("mcap_validation", {}) for label, phase in phases.items()})
    _write(output / "process_shutdown_audit.json", {label: phase.get("shutdown", {}) for label, phase in phases.items()})

    def rate_ok(value: Any, lo: float, hi: float) -> bool:
        return value is not None and lo <= float(value) <= hi
    def visual_ok(phase: dict[str, Any]) -> bool:
        return rate_ok(phase.get("tracker_output_hz"), 6.0, float("inf")) and phase.get("slot_overwritten") is not None
    def lidar_ok(phase: dict[str, Any]) -> bool:
        return (phase.get("lidar_hz") is None or rate_ok(phase.get("lidar_hz"), 9.0, 11.0)) and \
               (phase.get("imu_hz") is None or rate_ok(phase.get("imu_hz"), 180.0, 220.0))
    b0, b1, b2, b3, b4 = (phases.get(label, {}) for label in REQUIRED_LABELS)
    b1_visual, b2_visual, b3_visual, b4_visual = (visual_ok(value) for value in (b1, b2, b3, b4))
    b1_lidar, b2_lidar, b3_lidar, b4_lidar = (lidar_ok(value) for value in (b1, b2, b3, b4))
    recording_labels = ("B0", "B2", "B3", "B4")
    recording_ready = all(all(item.get("effective_valid") and item.get("finalized")
                              for item in phases.get(label, {}).get("mcap_validation", {}).get("bags", {}).values())
                         for label in recording_labels if phases.get(label))
    if not phases:
        recording_ready = False
    decisions = {
        "B1_below_6hz": bool(b1 and not b1_visual),
        "B1_pass_B2_fail": bool(b1_visual and b1_lidar and not b2_visual),
        "B1_B2_pass_B3_fail": bool(b1_visual and b2_visual and b3 and not b3_visual),
        "B3_pass_B4_fail": bool(b3_visual and b4 and not b4_visual),
        "all_groups_pass": bool(all(visual_ok(phases[label]) and lidar_ok(phases[label]) for label in REQUIRED_LABELS if phases.get(label))),
    }
    if decisions["B1_below_6hz"]:
        next_decision = "A: driver/DDS/CPU contention is the primary suspect; evaluate CPU affinity, camera publish rate and ROI topic without changing the seam algorithm"
    elif decisions["B1_pass_B2_fail"]:
        next_decision = "B: visual MCAP recording contention"
    elif decisions["B1_B2_pass_B3_fail"]:
        next_decision = "C: Livox/IMU recording or serialization contention"
    elif decisions["B3_pass_B4_fail"]:
        next_decision = "D: unified rosbag recorder contention; prefer split visual/Livox bags"
    elif decisions["all_groups_pass"]:
        next_decision = "E: compare manifests for an environmental or configuration difference"
    else:
        next_decision = "inconclusive: complete missing controlled groups before changing architecture"
    _write(output / "diagnostic_summary.json", {
        "task": "FIELD-W4-B-RESOURCE-CONTENTION-ISOLATION", "decisions": decisions,
        "next_decision": next_decision, "algorithm_modified": False,
        "geometry_provenance": False, "temporal_sync_ready": False,
        "spatial_fusion_ready": False, "production_ready": False,
    })
    _write(output / "next_decision.json", {"decision": next_decision, "decision_flags": decisions,
                                           "notes": ["Do not tune seam algorithm from this experiment",
                                                     "CameraInfo calibration remains incomplete",
                                                     "Do not run inspection_sync or FAST-LIVO2 in this phase"]})
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("resource-monitor")
    p.add_argument("--output", required=True); p.add_argument("--duration-sec", type=float, required=True)
    p.add_argument("--interval-sec", type=float, default=1.0); p.add_argument("--visual-bag", default="")
    p.add_argument("--livox-bag", default=""); p.set_defaults(func=resource_monitor)
    p = sub.add_parser("resource-summary")
    p.add_argument("--input", required=True); p.add_argument("--output", required=True); p.add_argument("--csv", default="")
    p.set_defaults(func=resource_summary)
    p = sub.add_parser("validate")
    p.add_argument("--run-dir", required=True); p.add_argument("--phase", required=True); p.add_argument("--output", required=True)
    p.add_argument("--bag", action="append", default=[], help="label=/path/to/bag")
    p.set_defaults(func=validate)
    p = sub.add_parser("aggregate")
    p.add_argument("--output-dir", default=str(OUTPUT)); p.add_argument("--phase-run", action="append", default=[])
    p.set_defaults(func=aggregate)
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
