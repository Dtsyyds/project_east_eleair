#!/usr/bin/env python3
"""Validate and finalize one closed visual field run without deleting evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def update_status(run_dir: Path, **values) -> None:
    path = run_dir / "run_manifest.json"
    document = json.loads(path.read_text()) if path.is_file() else {"status": {}}
    document.setdefault("status", {}).update(values)
    path.write_text(json.dumps(document, indent=2, ensure_ascii=False) + "\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--asset-id", required=True)
    parser.add_argument("--course-id", required=True)
    parser.add_argument("--plate-id", required=True)
    parser.add_argument("--visual-with-raw", action="store_true")
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    if not args.visual_with_raw:
        try:
            launch_values = json.loads((run_dir / "launch_parameters.json").read_text())
            args.visual_with_raw = str(launch_values.get("record_raw_image", "false")).lower() in {"1", "true", "yes", "on"}
        except (OSError, json.JSONDecodeError):
            pass
    bag_dir = run_dir / "mcap"
    manifest_path = run_dir / "run_manifest.json"
    live_summaries = sorted(run_dir.glob("live_session/**/live_summary.json"))
    if live_summaries and manifest_path.is_file():
        try:
            manifest = json.loads(manifest_path.read_text())
            summary = json.loads(live_summaries[-1].read_text())
            counters = summary.get("counters", {})
            manifest["camera_info_valid"] = counters.get("camera_info_valid")
            manifest["camera_info_received"] = counters.get("camera_info_received")
            metric_files = sorted(run_dir.glob("live_session/**/frame_metrics.jsonl"))
            if metric_files:
                first_line = metric_files[-1].read_text(encoding="utf-8").splitlines()[0]
                first_metrics = json.loads(first_line)
                manifest["roi"] = first_metrics.get("roi_original_px")
            manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
        except (OSError, json.JSONDecodeError, IndexError):
            pass
    if not (bag_dir / "metadata.yaml").is_file():
        update_status(run_dir, validation="failed:metadata_missing", valid_for_formal_validation=False)
        print(f"missing closed MCAP metadata: {bag_dir / 'metadata.yaml'}", file=sys.stderr)
        return 1
    info = subprocess.run(["ros2", "bag", "info", str(bag_dir)], text=True, capture_output=True, check=False)
    (run_dir / "logs" / "bag_info.txt").write_text(info.stdout + info.stderr)
    if info.returncode != 0:
        update_status(run_dir, validation="failed:bag_info", valid_for_formal_validation=False)
        return 1
    command = ["ros2", "run", "inspection_tools", "validate_bag", str(bag_dir), "--visual-profile"]
    if args.visual_with_raw:
        command.append("--visual-with-raw")
    validation = subprocess.run(command, text=True, capture_output=True, check=False)
    (run_dir / "logs" / "validation_command.txt").write_text(validation.stdout + validation.stderr)
    report = bag_dir / "validation_report.json"
    valid = False
    if report.is_file():
        try:
            valid = bool(json.loads(report.read_text()).get("valid"))
        except json.JSONDecodeError:
            valid = False
    if validation.returncode != 0 or not valid:
        update_status(run_dir, validation="failed", valid_for_formal_validation=False,
                      complete=False, finalized=False)
        print(f"validation failed; evidence retained at {run_dir}", file=sys.stderr)
        return 1
    if manifest_path.is_file() and report.is_file():
        try:
            manifest = json.loads(manifest_path.read_text())
            validation_document = json.loads(report.read_text())
            diagnostic = validation_document.get("diagnostics", {}).get("last_by_status", {}).get("mindvision_camera", {})
            values = diagnostic.get("values", {})
            if values:
                manifest.setdefault("camera", {}).update({
                    "model": values.get("camera_model", manifest.get("camera", {}).get("model")),
                    "serial_number": diagnostic.get("hardware_id", manifest.get("camera", {}).get("serial_number")),
                    "encoding": values.get("encoding", manifest.get("camera", {}).get("encoding")),
                    "width": values.get("width"), "height": values.get("height"),
                })
                manifest.setdefault("exposure", {}).update({
                    "runtime_exposure_us": values.get("exposure_us"),
                    "runtime_analog_gain": values.get("analog_gain"),
                    "runtime_analog_gain_raw": values.get("analog_gain_raw"),
                    "runtime_gain_db": values.get("gain_db", "unavailable"),
                })
                manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
        except (OSError, json.JSONDecodeError):
            pass
    finalize = subprocess.run([
        "ros2", "run", "inspection_tools", "finalize_bag", str(bag_dir),
        "--task-id", args.task_id, "--asset-id", args.asset_id,
        "--course-id", args.course_id, "--plate-id", args.plate_id,
        "--validation-report", str(report),
    ], text=True, capture_output=True, check=False)
    (run_dir / "logs" / "finalize_command.txt").write_text(finalize.stdout + finalize.stderr)
    if finalize.returncode != 0:
        update_status(run_dir, validation="passed", valid_for_formal_validation=False,
                      complete=False, finalized=False)
        return 1
    hashes = {}
    for path in sorted(run_dir.rglob("*")):
        if path.is_file() and path.name not in {"sha256sums.txt", "sha256sums.json"}:
            hashes[str(path.relative_to(run_dir))] = sha256(path)
    (run_dir / "sha256sums.json").write_text(json.dumps(hashes, indent=2) + "\n")
    update_status(run_dir, validation="passed", valid_for_formal_validation=True,
                  complete=True, finalized=True)
    print(f"finalized_run={run_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
