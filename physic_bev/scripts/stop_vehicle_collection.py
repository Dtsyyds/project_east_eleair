#!/usr/bin/env python3
"""Stop only registered FIELD-W5 process groups, seal bags, and finalize."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import (  # noqa: E402
    atomic_json, load_json, process_cmdline, process_matches, process_start_ticks, utc_now,
)


ORDER = {
    "raw_burst_recorder": 0,
    "core_recorder": 1,
    "mapping_recorder": 2,
    "mid360_recorder": 2,
    "d405_recorder": 3,
    "eddy_recorder": 4,
    "front_video": 5,
    "front_video_encoder": 6,
    "monitor": 7,
    "d405_driver": 8,
    "mid360_driver": 9,
    "eddy_driver": 10,
    "visual": 11,
    "disk_guard": 12,
}

MAPPING_EDDY_ORDER = {
    "visual": 0,
    "front_video_encoder": 1,
    "eddy_driver": 2,
    "front_video": 3,
    "core_recorder": 4,
    "mapping_recorder": 5,
    "eddy_recorder": 6,
    "monitor": 7,
    "disk_guard": 8,
}


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def _signal_item(item: dict[str, Any], signum: signal.Signals) -> str:
    pid = int(item["pid"])
    pgid = int(item.get("pgid", pid))
    # A live leader must still match its recorded /proc start time. If only
    # descendants remain, the original dedicated PGID is still safe to target.
    if process_start_ticks(pid) is not None and not process_matches(item):
        return "identity_mismatch"
    try:
        if _group_alive(pgid):
            os.killpg(pgid, signum)
            return "signaled_group"
        if process_matches(item):
            os.kill(pid, signum)
            return "signaled_pid"
        return "not_running"
    except ProcessLookupError:
        return "not_running"
    except PermissionError:
        return "permission_denied"


def _wait(items: list[dict[str, Any]], seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not any(_group_alive(int(item.get("pgid", item["pid"]))) for item in items):
            return
        time.sleep(0.25)


def _scan_group_members(groups: set[int]) -> list[dict[str, Any]]:
    values = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit():
            continue
        pid = int(proc.name)
        try:
            pgid = os.getpgid(pid)
        except (ProcessLookupError, PermissionError):
            continue
        if pgid in groups:
            values.append({"pid": pid, "pgid": pgid, "cmdline": process_cmdline(pid)})
    return values


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--skip-finalize", action="store_true")
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    registry_path = run_dir / "logs" / "process_registry.json"
    registry = load_json(registry_path, {}) or {}
    manifest = load_json(run_dir / "run_manifest.json", {}) or {}
    order = MAPPING_EDDY_ORDER if manifest.get("collection_profile") == "mapping_eddy" else ORDER
    items = sorted(registry.get("processes", []), key=lambda value: order.get(value.get("name"), 50))
    from_guard = os.environ.get("FIELD_STOP_FROM_GUARD") == "1"
    guard_items = [item for item in items if item.get("name") == "disk_guard"]
    targets = [item for item in items if item.get("name") != "disk_guard"]
    if from_guard:
        (run_dir / "logs" / "disk_guard_finalize_in_progress").write_text(
            f"{os.getpid()}\n", encoding="ascii"
        )
    events = []
    for item in targets:
        events.append({"name": item.get("name"), "signal": "SIGINT", "result": _signal_item(item, signal.SIGINT)})
    _wait(targets, 60.0)
    remaining = [item for item in targets if _group_alive(int(item.get("pgid", item["pid"])))]
    for item in remaining:
        events.append({"name": item.get("name"), "signal": "SIGTERM", "result": _signal_item(item, signal.SIGTERM)})
    _wait(remaining, 15.0)
    remaining = [item for item in remaining if _group_alive(int(item.get("pgid", item["pid"])))]
    for item in remaining:
        events.append({"name": item.get("name"), "signal": "SIGKILL", "result": _signal_item(item, signal.SIGKILL)})
    _wait(remaining, 5.0)

    if not from_guard:
        for item in guard_items:
            events.append({"name": item.get("name"), "signal": "SIGINT", "result": _signal_item(item, signal.SIGINT)})
        _wait(guard_items, 5.0)
        for item in guard_items:
            if _group_alive(int(item.get("pgid", item["pid"]))):
                events.append({"name": item.get("name"), "signal": "SIGTERM", "result": _signal_item(item, signal.SIGTERM)})
        _wait(guard_items, 5.0)

    enabled = manifest.get("enabled_writers", {})
    writers = [name for name in ("core", "mapping", "mid360", "d405", "eddy") if enabled.get(name)]
    if enabled.get("front_raw_burst") and enabled.get("raw_burst_mode") == "full":
        writers.append("raw_burst")
    seal_command = [
        sys.executable, str(SCRIPT_DIR / "vehicle_collection_manager.py"),
        "seal", "--run-dir", str(run_dir),
    ]
    for writer in writers:
        seal_command += ["--writer", writer]
    seal = subprocess.run(seal_command, text=True, capture_output=True, check=False) if writers else None

    groups = {int(item.get("pgid", item["pid"])) for item in targets}
    orphan_members = _scan_group_members(groups)
    stop_report = {
        "schema_version": "field_w5_stop_report_v1",
        "stopped_at": utc_now(), "events": events,
        "seal": None if seal is None else {
            "returncode": seal.returncode, "stdout": seal.stdout, "stderr": seal.stderr,
        },
        "registered_group_members_after_stop": orphan_members,
    }
    atomic_json(run_dir / "reports" / "stop_report.json", stop_report)
    print(f"recording processes stopped for {run_dir.name}", flush=True)
    if seal is None or seal.returncode == 0:
        print("MCAP writers sealed; starting offline validation (this may take several minutes)", flush=True)
    else:
        print("one or more MCAP writers could not be sealed", file=sys.stderr, flush=True)

    finalize_returncode = 0
    if not args.skip_finalize:
        try:
            finalize_returncode = subprocess.run([
                sys.executable, str(SCRIPT_DIR / "finalize_vehicle_collection.py"), str(run_dir),
            ], check=False).returncode
        except KeyboardInterrupt:
            stop_report["finalize"] = {
                "completed": False, "interrupted": True,
                "message": "recorders are stopped and sealed; rerun finalize_vehicle_collection.py",
            }
            atomic_json(run_dir / "reports" / "stop_report.json", stop_report)
            print(
                "offline validation interrupted; recording is already stopped and sealed. "
                "Run finalize_vehicle_collection.py later.",
                file=sys.stderr, flush=True,
            )
            return 130
    else:
        print("offline validation skipped by FIELD_SKIP_FINALIZE=1", flush=True)

    stop_report["events"] = events
    atomic_json(run_dir / "reports" / "stop_report.json", stop_report)
    if orphan_members:
        return 3
    if seal is not None and seal.returncode != 0:
        return 4
    return finalize_returncode


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("stop interrupted", file=sys.stderr)
        raise SystemExit(130)
