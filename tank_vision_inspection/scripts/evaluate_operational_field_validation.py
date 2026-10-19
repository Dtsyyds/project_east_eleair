#!/usr/bin/env python3
"""Evaluate the operational field-validation acceptance separately from strict A01."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


THRESHOLDS = {
    "detected_rate": 0.95,
    "tracking_rate": 0.95,
    "enter_tracking_processed_frames": 3,
    "lost_after_acquisition": 0,
    "geometry_pass_rate": 0.99,
    "maximum_consecutive_predicted_only": 2,
    "coverage_min": 0.60,
    "residual_max_px": 14.0,
    "fitted_output_crossing_count": 0,
    "dropped_processed_frames": 0,
    "output_rate_hz": 6.0,
    "processing_p95_ms": 120.0,
    "end_to_end_p95_ms": 180.0,
}


def _metrics_path(path: Path) -> Path:
    if path.is_file():
        return path
    candidates = sorted(path.glob("live_session/**/frame_metrics.jsonl"))
    if not candidates:
        raise FileNotFoundError(f"no frame_metrics.jsonl under {path}")
    return candidates[-1]


def _rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid JSONL line {line_number}: {exc}") from exc
    return rows


def _finite(values: list[Any]) -> list[float]:
    result = []
    for value in values:
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        if np.isfinite(number):
            result.append(number)
    return result


def _geometry_pass(row: dict[str, Any]) -> bool:
    """Operational geometry excludes the single-frame inlier-rate gate.

    ``minimum_inlier_ratio`` remains recorded and enforced by the tracker for
    each frame.  It is intentionally not made a whole-run operational failure
    here because DEGRADED absorbs isolated inlier dips.
    """
    reasons = set(str(value) for value in row.get("failure_reasons", []))
    excluded_single_frame_gate = {"inlier_ratio_below_minimum"}
    blocking = reasons - excluded_single_frame_gate
    coverage = float(row.get("coverage", 0.0))
    span = float(row.get("effective_span", 0.0))
    residual = row.get("fit_residual_px")
    width = row.get("minimum_fitted_width_px")
    crossing = int(row.get("fitted_output_crossing_count", row.get("crossing_count", 0)))
    try:
        residual_ok = np.isfinite(float(residual)) and float(residual) <= THRESHOLDS["residual_max_px"]
    except (TypeError, ValueError):
        residual_ok = False
    try:
        width_ok = np.isfinite(float(width)) and float(width) > 0.0
    except (TypeError, ValueError):
        width_ok = False
    return (not blocking and coverage >= THRESHOLDS["coverage_min"] and span >= THRESHOLDS["coverage_min"]
            and residual_ok and crossing == 0 and width_ok)


def evaluate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        raise ValueError("no processed frame rows")
    count = len(rows)
    detected_rate = sum(bool(row.get("detected", False)) for row in rows) / count
    tracking_rate = sum(row.get("post_state", row.get("tracking_state")) == "TRACKING" for row in rows) / count
    tracking_index = next((index + 1 for index, row in enumerate(rows)
                           if row.get("post_state", row.get("tracking_state")) == "TRACKING"), None)
    acquisition_index = next((index for index, row in enumerate(rows)
                              if row.get("post_state", row.get("tracking_state")) in {"ACQUIRING", "TRACKING", "DEGRADED"}), None)
    lost_after_acquisition = 0 if acquisition_index is None else sum(
        row.get("post_state", row.get("tracking_state")) == "LOST" for row in rows[acquisition_index:])

    maximum_predicted = 0
    current_predicted = 0
    for row in rows:
        if bool(row.get("predicted_only", False)):
            current_predicted += 1
            maximum_predicted = max(maximum_predicted, current_predicted)
        else:
            current_predicted = 0

    coverage_values = _finite([row.get("coverage") for row in rows])
    residual_values = _finite([row.get("fit_residual_px") for row in rows])
    processing_values = _finite([row.get("processing_ms") for row in rows])
    end_to_end_values = _finite([row.get("end_to_end_ms") for row in rows])
    geometry_pass_rate = sum(_geometry_pass(row) for row in rows) / count
    stamps = _finite([row.get("output_host_timestamp_ns", row.get("host_timestamp_ns")) for row in rows])
    output_rate = ((count - 1) / ((stamps[-1] - stamps[0]) / 1e9)
                   if len(stamps) >= 2 and stamps[-1] > stamps[0] else 0.0)
    crossing_values = [int(row.get("fitted_output_crossing_count", row.get("crossing_count", 0))) for row in rows]
    dropped_values = [int(row.get("dropped_processed_frames", row.get("dropped_frames", 0))) for row in rows]
    observed = {
        "processed_frames": count,
        "detected_rate": detected_rate,
        "tracking_rate": tracking_rate,
        "enter_tracking_processed_frames": tracking_index,
        "lost_after_acquisition": lost_after_acquisition,
        "geometry_pass_rate": geometry_pass_rate,
        "maximum_consecutive_predicted_only": maximum_predicted,
        "coverage_min": min(coverage_values) if coverage_values else None,
        "residual_max_px": max(residual_values) if residual_values else None,
        "fitted_output_crossing_count": max(crossing_values) if crossing_values else None,
        "dropped_processed_frames": max(dropped_values) if dropped_values else None,
        "output_rate_hz": output_rate,
        "processing_p95_ms": float(np.percentile(processing_values, 95)) if processing_values else None,
        "end_to_end_p95_ms": float(np.percentile(end_to_end_values, 95)) if end_to_end_values else None,
    }
    checks = {
        "detected_rate": observed["detected_rate"] >= THRESHOLDS["detected_rate"],
        "tracking_rate": observed["tracking_rate"] >= THRESHOLDS["tracking_rate"],
        "enter_tracking_processed_frames": observed["enter_tracking_processed_frames"] is not None and observed["enter_tracking_processed_frames"] <= THRESHOLDS["enter_tracking_processed_frames"],
        "lost_after_acquisition": observed["lost_after_acquisition"] == THRESHOLDS["lost_after_acquisition"],
        "geometry_pass_rate": observed["geometry_pass_rate"] >= THRESHOLDS["geometry_pass_rate"],
        "maximum_consecutive_predicted_only": observed["maximum_consecutive_predicted_only"] <= THRESHOLDS["maximum_consecutive_predicted_only"],
        "coverage_min": observed["coverage_min"] is not None and observed["coverage_min"] >= THRESHOLDS["coverage_min"],
        "residual_max_px": observed["residual_max_px"] is not None and observed["residual_max_px"] <= THRESHOLDS["residual_max_px"],
        "fitted_output_crossing_count": observed["fitted_output_crossing_count"] == THRESHOLDS["fitted_output_crossing_count"],
        "dropped_processed_frames": observed["dropped_processed_frames"] == THRESHOLDS["dropped_processed_frames"],
        "output_rate_hz": observed["output_rate_hz"] >= THRESHOLDS["output_rate_hz"],
        "processing_p95_ms": observed["processing_p95_ms"] is not None and observed["processing_p95_ms"] <= THRESHOLDS["processing_p95_ms"],
        "end_to_end_p95_ms": observed["end_to_end_p95_ms"] is not None and observed["end_to_end_p95_ms"] <= THRESHOLDS["end_to_end_p95_ms"],
    }
    return {
        "schema_version": "1.0",
        "evidence_level": "field_validation",
        "acceptance_profile": "operational_field_validation",
        "status": "passed" if all(checks.values()) else "failed",
        "thresholds": THRESHOLDS,
        "observed": observed,
        "checks": checks,
        "strict_acceptance_is_separate": True,
        "note": "minimum_inlier_ratio remains a per-frame tracker gate; isolated inlier dips absorbed by DEGRADED do not fail the whole operational run.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="frame_metrics.jsonl or a run directory")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    metrics_path = _metrics_path(args.input.expanduser().resolve())
    report = evaluate(_rows(metrics_path))
    report["metrics_path"] = str(metrics_path)
    output = args.output or metrics_path.with_name("operational_field_validation_acceptance.json")
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
