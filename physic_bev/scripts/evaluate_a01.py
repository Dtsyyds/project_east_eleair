#!/usr/bin/env python3
"""Build an evidence-backed A01 acceptance report from one closed live run."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


THRESHOLDS = {
    "detected_rate": 0.95,
    "tracking_rate": 0.95,
    "enter_tracking_processed_frames": 3,
    "processing_p95_ms": 100.0,
    "dropped_frames": 0,
    "coverage_min": 0.60,
    "joint_inlier_min": 0.68,
    "residual_max_px": 14.0,
    "crossing_max": 0,
}


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    run_dir = args.run_dir.resolve()
    summary_path = next(iter(sorted(run_dir.glob("live_session/**/live_summary.json"))), None)
    metrics_path = next(iter(sorted(run_dir.glob("live_session/**/frame_metrics.jsonl"))), None)
    if summary_path is None or metrics_path is None:
        raise SystemExit(f"missing live session evidence under {run_dir}")
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = [json.loads(line) for line in metrics_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    counters = summary.get("counters", {})
    total = len(rows)
    detected = sum(bool(row.get("detected")) for row in rows)
    tracking = sum(row.get("post_state") == "TRACKING" for row in rows)
    first_tracking = next((int(row["frame_index"]) for row in rows if row.get("post_state") == "TRACKING"), None)
    coverage = [float(row["coverage"]) for row in rows if row.get("coverage") is not None]
    inlier = [float(row["joint_inlier_ratio"]) for row in rows if row.get("joint_inlier_ratio") is not None]
    residual = [float(row["fit_residual_px"]) for row in rows if row.get("fit_residual_px") is not None]
    processing = [float(row["processing_ms"]) for row in rows if row.get("processing_ms") is not None]
    crossing = [int(row["crossing_count"]) for row in rows if isinstance(row.get("crossing_count"), (int, float))]
    observed = {
        "frame_count": total,
        "detected_rate": detected / total if total else 0.0,
        "tracking_rate": tracking / total if total else 0.0,
        "enter_tracking_processed_frames": first_tracking,
        "processing_ms": {"p50": percentile(processing, 0.50), "p95": percentile(processing, 0.95),
                           "max": max(processing) if processing else None},
        "dropped_frames": int(counters.get("preview_skipped_frames", 0)),
        "coverage": {"min": min(coverage) if coverage else None, "p50": percentile(coverage, 0.50)},
        "joint_inlier": {"min": min(inlier) if inlier else None, "p50": percentile(inlier, 0.50)},
        "residual_px": {"max": max(residual) if residual else None, "p50": percentile(residual, 0.50)},
        "crossing_count": max(crossing) if crossing else None,
        "state_counts": counters.get("state_counts", {}),
        "failure_reason_counts": counters.get("failure_reason_counts", {}),
        "input_fps": counters.get("input_fps"),
        "output_fps": counters.get("output_fps"),
        "common_active_duration_sec": counters.get("common_active_duration_sec"),
        "warmup_duration_sec": counters.get("warmup_duration_sec"),
        "shutdown_duration_sec": counters.get("shutdown_duration_sec"),
        "camera_info_received": counters.get("camera_info_received"),
        "camera_info_valid": counters.get("camera_info_valid"),
        "snapshot_writer_failed": counters.get("snapshot_writer_failed"),
    }
    checks = {
        "detected_rate": observed["detected_rate"] >= THRESHOLDS["detected_rate"],
        "tracking_rate": observed["tracking_rate"] >= THRESHOLDS["tracking_rate"],
        "enter_tracking_processed_frames": (first_tracking is not None and first_tracking <= THRESHOLDS["enter_tracking_processed_frames"]),
        "processing_p95_ms": (observed["processing_ms"]["p95"] is not None and observed["processing_ms"]["p95"] <= THRESHOLDS["processing_p95_ms"]),
        "dropped_frames": observed["dropped_frames"] <= THRESHOLDS["dropped_frames"],
        "coverage_min": observed["coverage"]["min"] is not None and observed["coverage"]["min"] >= THRESHOLDS["coverage_min"],
        "joint_inlier_min": observed["joint_inlier"]["min"] is not None and observed["joint_inlier"]["min"] >= THRESHOLDS["joint_inlier_min"],
        "residual_max_px": observed["residual_px"]["max"] is not None and observed["residual_px"]["max"] <= THRESHOLDS["residual_max_px"],
        "crossing_max": (observed["crossing_count"] is not None and observed["crossing_count"] <= THRESHOLDS["crossing_max"]),
    }
    primary_failures = [name for name, passed in checks.items() if not passed]
    report = {
        "schema_version": "1.0",
        "status": "passed" if not primary_failures else "failed",
        "run_dir": str(run_dir),
        "config_sha256": sha256(run_dir / "frozen_configs/tracker.yaml") if (run_dir / "frozen_configs/tracker.yaml").is_file() else None,
        "reference_sha256": sha256(run_dir / "frozen_configs/reference.json") if (run_dir / "frozen_configs/reference.json").is_file() else None,
        "thresholds": THRESHOLDS,
        "observed": observed,
        "checks": checks,
        "primary_failures": primary_failures,
        "evidence": {"summary": str(summary_path), "metrics": str(metrics_path)},
        "note": "Strict A01 remains unchanged. New runs use the tracker-provided fitted_output_crossing_count; legacy runs with absent crossing_count remain unclaimed.",
    }
    output = args.output or (run_dir / "A01_acceptance.json")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if not primary_failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
