#!/usr/bin/env python3
"""Finalize a stopped FIELD-W5 run without deleting incomplete evidence."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
from vehicle_collection_common import (  # noqa: E402
    atomic_json, atomic_write_text, load_json, sha256_file, update_manifest, utc_now,
)


def _copy_reports(run_dir: Path, destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    names = (
        "diagnostic_summary.json", "source_inventory.json", "topic_inventory.json",
        "encoder_inventory.json", "storage_bandwidth_report.json", "performance_matrix.json",
        "timestamp_validation.json", "video_validation.json", "mcap_validation.json",
        "replay_validation.json", "snapshot_integrity.json", "orphan_process_check.json",
        "system_acceptance.json", "next_action.json",
        "mapping_topic_inventory.json", "eddy_topic_inventory.json",
        "qos_inventory.json", "path_growth_analysis.json",
        "baseline_performance.json", "recording_performance.json",
        "compact_trajectory_summary.json", "compact_trajectory.jsonl",
    )
    for name in names:
        source = run_dir / "reports" / name
        if not source.is_file() and name == "topic_inventory.json":
            source = run_dir / name
        if source.is_file():
            (destination / name).write_bytes(source.read_bytes())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--summary-dir",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--reuse-existing-validation", action="store_true",
        help="reuse an immediately preceding validation pass when only report collation changed",
    )
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    manifest_path = run_dir / "run_manifest.json"
    if not manifest_path.is_file():
        print(f"missing run manifest: {manifest_path}", file=sys.stderr)
        return 2
    existing_complete = load_json(run_dir / "complete.json", {}) or {}
    if existing_complete.get("finalized") is True:
        print(f"run already sealed; unchanged: {run_dir}")
        return 0 if existing_complete.get("validation") == "passed" else 2
    manifest = load_json(manifest_path, {}) or {}
    is_mapping_eddy = manifest.get("collection_profile") == "mapping_eddy"
    summary_dir = (args.summary_dir or (
        WORKSPACE_ROOT / "physic_bev/output/field_w6_mapping_eddy_collection"
        if is_mapping_eddy else
        WORKSPACE_ROOT / "tank_vision_inspection/output/field_w5_vehicle_collection"
    )).resolve()
    if args.reuse_existing_validation:
        validation_report = run_dir / "reports" / "system_acceptance.json"
        validation = subprocess.CompletedProcess(
            args=["existing", str(validation_report)],
            returncode=0 if validation_report.is_file() else 2,
        )
    else:
        validation = subprocess.run(
            [sys.executable, str(SCRIPT_DIR / "validate_vehicle_collection.py"), str(run_dir)],
            check=False,
        )
    acceptance = load_json(run_dir / "reports" / "system_acceptance.json", {}) or {}
    valid = validation.returncode == 0 and acceptance.get("valid") is True
    replay_valid = False
    if is_mapping_eddy:
        replay_valid = (
            load_json(run_dir / "reports" / "replay_validation.json", {}) or {}
        ).get("valid") is True
        # The deep validator runs before replay and therefore initializes S4
        # to false.  A post-replay refinalization is the authoritative place
        # to promote this stage before the new SHA manifest is generated.
        acceptance.setdefault("stage_results", {})[
            "S4_finalize_sha_replay"
        ] = bool(valid and replay_valid)
        atomic_json(run_dir / "reports" / "system_acceptance.json", acceptance)
    generator = ("generate_mapping_eddy_summary.py" if is_mapping_eddy
                 else "generate_vehicle_collection_summary.py")
    subprocess.run(
        [sys.executable, str(SCRIPT_DIR / generator), str(run_dir),
         "--summary-dir", str(summary_dir)],
        check=True,
    )
    failed_checks = [
        name for name, value in acceptance.get("checks", {}).items() if not value
    ]
    if is_mapping_eddy:
        elapsed = float(acceptance.get("elapsed_sec", 0) or 0)
        current_status = ("field_collection_ready" if valid and replay_valid and elapsed >= 1800 else
                          "replay_validated" if valid and replay_valid else
                          "30min_collection_validated" if valid and elapsed >= 1800 else
                          "10min_collection_validated" if valid and elapsed >= 600 else
                          "60s_collection_validated" if valid and elapsed >= 60 else
                          "mapping_sources_verified" if valid else "blocked")
        next_action = {
            "schema_version": "field_w6_next_action_v1",
            "current_status": current_status,
            "primary_blocker": None if valid else (failed_checks[0] if failed_checks else "validation_failed"),
            "next_action": ("field collection ready" if current_status == "field_collection_ready" else
                            "continue the next duration gate" if current_status == "replay_validated" else
                            "run replay validation" if valid else "resolve the first failed acceptance check"),
        }
    else:
        next_action = {
            "schema_version": "field_w5_next_action_v1",
            "current_status": "field_collection_ready" if valid else "blocked",
            "primary_blocker": None if valid else (
                "tracker_rate_below_6_hz_under_full_chain"
                if "tracker_at_least_6_hz" in failed_checks
                else (failed_checks[0] if failed_checks else "validation_failed")
            ),
            "next_action": "run replay validation" if valid else (
                "isolate tracker CPU/DDS contention without changing algorithm parameters"
                if "tracker_at_least_6_hz" in failed_checks
                else "resolve the first failed acceptance check"
            ),
        }
    atomic_json(run_dir / "reports" / "next_action.json", next_action)
    excluded = {"sha256sums.txt", "complete.json", "run_manifest.json"}
    file_entries: list[dict[str, Any]] = []
    for path in sorted(run_dir.rglob("*")):
        if not path.is_file() or path.name in excluded or path.name.endswith(".partial"):
            continue
        file_entries.append({
            "path": str(path.relative_to(run_dir)),
            "size": path.stat().st_size,
            "sha256": sha256_file(path),
        })
    sums = "".join(f"{item['sha256']}  {item['path']}\n" for item in file_entries)
    atomic_write_text(run_dir / "sha256sums.txt", sums)
    sha256_valid = all(
        sha256_file(run_dir / item["path"]) == item["sha256"] for item in file_entries
    )
    valid = valid and sha256_valid
    manifest["files"] = file_entries
    manifest["hash_exclusions"] = sorted(excluded)
    manifest.setdefault("status", {}).update({
        "complete": valid, "finalized": valid,
        "validation": "passed" if valid else "failed",
        "valid_for_formal_validation": valid,
        "sha256_valid": sha256_valid,
    })
    atomic_json(manifest_path, manifest)
    complete = {
        "complete": valid, "finalized": valid,
        "validation": "passed" if valid else "failed",
        "updated_at": utc_now(),
        "failed_checks": failed_checks,
        "sha256_valid": sha256_valid,
    }
    atomic_json(run_dir / "complete.json", complete)
    _copy_reports(run_dir, summary_dir)
    print(json.dumps(complete, ensure_ascii=False))
    return 0 if valid else 2


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("vehicle collection finalization interrupted", file=sys.stderr)
        raise SystemExit(130)
