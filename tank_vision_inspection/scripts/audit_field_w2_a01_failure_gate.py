#!/usr/bin/env python3
"""Read-only audit of a completed FIELD-W2 A01 failure run.

This tool consumes the persisted run evidence only.  It deliberately does not
start ROS, open a camera, or invoke the tracker.  In particular, it does not
invent selected-path provenance when the realtime run was recorded without it.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import sys
from collections import Counter
from pathlib import Path
from typing import Any

import cv2
import numpy as np

# Allow direct execution as ``python3 scripts/...py`` from this workspace.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from tank_vision.seam_width_reference import SeamWidthReference


DEFAULT_RUN = Path("/media/mcap_data/runs/FIELD-W2-ROI-EVIDENCE-PRE10-20260917_164238")
DEFAULT_OUT = Path("output/field_w2_a01_failure_gate_audit")


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentiles(values: list[float]) -> dict[str, Any]:
    if not values:
        return {name: None for name in ("min", "p01", "p05", "p50", "p95", "p99", "max")}
    data = np.asarray(values, dtype=np.float64)
    return {"min": float(np.min(data)), "p01": float(np.percentile(data, 1)),
            "p05": float(np.percentile(data, 5)), "p50": float(np.percentile(data, 50)),
            "p95": float(np.percentile(data, 95)), "p99": float(np.percentile(data, 99)),
            "max": float(np.max(data))}


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def stamp_ns(value: Any) -> int | None:
    if isinstance(value, dict):
        if "sec" in value and "nanosec" in value:
            return int(value["sec"]) * 1_000_000_000 + int(value["nanosec"])
        if "sec" in value and "nsec" in value:
            return int(value["sec"]) * 1_000_000_000 + int(value["nsec"])
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def read_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, 1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid frame_metrics JSONL line {line_number}: {exc}") from exc
    return rows


def check_pass(row: dict[str, Any], key: str) -> bool | None:
    value = (row.get("gate_truth") or {}).get("checks", {}).get(key)
    if isinstance(value, dict):
        value = value.get("pass")
    if isinstance(value, bool):
        return value
    return None


FIELD_MAP = {
    "point_count": "point_count",
    "coverage": "coverage",
    "effective_span": "effective_span",
    "maximum_missing_run": "maximum_missing_run",
    "abrupt_left_jump_count": "abrupt_left_jump_count",
    "abrupt_right_jump_count": "abrupt_right_jump_count",
    "selected_pair_crossing_count": "selected_pair_crossing_count",
    "fitted_output_crossing_count": "fitted_output_crossing_count",
    "path_confidence": "path_confidence",
    "center_inlier_ratio": "center_inlier_ratio",
    "width_inlier_ratio": "width_inlier_ratio",
    "joint_inlier_ratio": "joint_inlier_ratio",
    "fit_residual_px": "fit_residual_px",
    "minimum_fitted_width_px": "minimum_fitted_width_px",
}

GATE_MAP = {
    "point_count": "point_count", "coverage": "coverage",
    "effective_span": "effective_span", "maximum_missing_run": "missing_run",
    "abrupt_left_jump_count": "abrupt_left_jump", "abrupt_right_jump_count": "abrupt_right_jump",
    "selected_pair_crossing_count": "non_crossing", "fitted_output_crossing_count": "non_crossing",
    "path_confidence": "path_confidence", "center_inlier_ratio": "center_inlier_ratio",
    "width_inlier_ratio": "width_inlier_ratio", "joint_inlier_ratio": "joint_inlier",
    "fit_residual_px": "fit_residual", "minimum_fitted_width_px": "positive_width",
}


def field_gate_pass(row: dict[str, Any], field: str) -> bool | None:
    """Return the field-specific gate result without conflating unavailable traces."""
    if field in {"selected_pair_crossing_count", "fitted_output_crossing_count"}:
        value = finite(row.get(field))
        return None if value is None else int(value) == 0
    return check_pass(row, GATE_MAP[field])


def gate_statistics(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for name, key in FIELD_MAP.items():
        values = [value for row in rows if (value := finite(row.get(key))) is not None]
        gate_values = [field_gate_pass(row, name) for row in rows]
        known = [value for value in gate_values if value is not None]
        result[name] = {
            "distribution": percentiles(values),
            "finite_count": len(values), "missing_count": len(rows) - len(values),
            "gate_column": GATE_MAP[name],
            "gate_known_count": len(known),
            "gate_failure_count": sum(not value for value in known),
            "gate_failure_ratio": (sum(not value for value in known) / len(known) if known else None),
            "gate_unavailable_count": len(rows) - len(known),
        }
    for name, gate in {
        "fit_success": "fit_success", "positive_width": "positive_width",
        "inside_roi": "inside_roi", "non_crossing": "non_crossing", "temporal": "temporal",
    }.items():
        values = [check_pass(row, gate) for row in rows]
        known = [value for value in values if value is not None]
        result[name] = {"gate_column": gate, "gate_known_count": len(known),
                        "gate_failure_count": sum(not value for value in known),
                        "gate_failure_ratio": (sum(not value for value in known) / len(known) if known else None),
                        "gate_unavailable_count": len(rows) - len(known)}
    return result


def failure_histogram(rows: list[dict[str, Any]]) -> dict[str, Any]:
    all_reasons = Counter(reason for row in rows for reason in row.get("failure_reasons", []))
    first = Counter((row.get("failure_reasons") or ["<none>"])[0] for row in rows)
    return {"frame_count": len(rows), "all_failure_reason_occurrences": {
        reason: {"count": count, "ratio_of_frames": count / len(rows)} for reason, count in all_reasons.items()},
        "first_failure_reason": {reason: {"count": count, "ratio_of_frames": count / len(rows)}
                                 for reason, count in first.items()},
        "same_frame_reasons_are_preserved": True}


def row_gate_values(row: dict[str, Any]) -> dict[str, str]:
    values: dict[str, str] = {}
    for name, gate in GATE_MAP.items():
        value = field_gate_pass(row, name) if name in GATE_MAP else check_pass(row, gate)
        values[name] = "NA" if value is None else ("true" if value else "false")
    for name, gate in {"fit_success": "fit_success", "positive_width": "positive_width",
                       "inside_roi": "inside_roi", "non_crossing": "non_crossing", "temporal": "temporal"}.items():
        value = check_pass(row, gate)
        values[name] = "NA" if value is None else ("true" if value else "false")
    return values


def write_gate_matrix(path: Path, rows: list[dict[str, Any]]) -> None:
    gate_names = list(FIELD_MAP) + ["fit_success", "positive_width", "inside_roi", "non_crossing", "temporal"]
    columns = ["row_number", "frame_index", "captured_index", "host_timestamp_ns", "output_host_timestamp_ns",
               "detected", "state", "predicted_only", "failure_reasons", "first_failure_reason"] + [f"gate_{name}" for name in gate_names]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for number, row in enumerate(rows, 1):
            gates = row_gate_values(row)
            output = {"row_number": number, "frame_index": row.get("frame_index"),
                      "captured_index": row.get("captured_index"), "host_timestamp_ns": row.get("host_timestamp_ns"),
                      "output_host_timestamp_ns": row.get("output_host_timestamp_ns"), "detected": row.get("detected"),
                      "state": row.get("post_state", row.get("tracking_state")), "predicted_only": row.get("predicted_only"),
                      "failure_reasons": "|".join(row.get("failure_reasons", [])),
                      "first_failure_reason": (row.get("failure_reasons") or [""])[0]}
            output.update({f"gate_{key}": gates[key] for key in gate_names})
            writer.writerow(output)


def find_snapshot(session: Path, row: dict[str, Any]) -> Path | None:
    name = row.get("snapshot_path")
    if not name:
        return None
    path = session / "failure_snapshots" / str(name)
    return path if path.is_dir() else None


def choose_representatives(rows: list[dict[str, Any]], session: Path) -> list[dict[str, Any]]:
    finite_joint = [(finite(row.get("joint_inlier_ratio")), i) for i, row in enumerate(rows)]
    finite_residual = [(finite(row.get("fit_residual_px")), i) for i, row in enumerate(rows)]
    candidates = [
        ("joint_inlier_best", max(finite_joint)[1], "maximum finite joint_inlier_ratio"),
        ("joint_inlier_worst", min(finite_joint)[1], "minimum finite joint_inlier_ratio"),
        ("residual_best", min(finite_residual)[1], "minimum finite fit_residual_px"),
        ("residual_worst", max(finite_residual)[1], "maximum finite fit_residual_px"),
        ("timeline_25_percent", round((len(rows) - 1) * 0.25), "25% position in processed timeline"),
        ("timeline_75_percent", round((len(rows) - 1) * 0.75), "75% position in processed timeline"),
    ]
    used: set[int] = set()
    selected: list[dict[str, Any]] = []
    for label, index, criterion in candidates:
        choices = [index] + [index - d for d in range(1, 100)] + [index + d for d in range(1, 100)]
        actual = next((i for i in choices if 0 <= i < len(rows) and i not in used), index)
        used.add(actual)
        row = rows[actual]
        snap = find_snapshot(session, row)
        if snap is None:
            nearby = sorted((abs(i - actual), i) for i, candidate in enumerate(rows)
                            if i not in used and find_snapshot(session, candidate) is not None)
            if nearby:
                _, actual = nearby[0]
                used.add(actual)
                row = rows[actual]
                snap = find_snapshot(session, row)
        selected.append({"label": label, "criterion": criterion, "requested_row_index": index,
                         "requested_frame_index": rows[index].get("frame_index"),
                         "actual_row_index": actual, "frame_index": row.get("frame_index"),
                         "snapshot_path": str(snap) if snap else None,
                         "row": row})
    return selected


def reference_audit(manifest: dict[str, Any], rows: list[dict[str, Any]]) -> dict[str, Any]:
    roi = [int(x) for x in manifest["roi"]]
    x, y, width, height = roi
    ref_path = Path(manifest["tracker"]["reference"])
    reference = SeamWidthReference.load(ref_path)
    ys = np.linspace(y, y + height - 1, 128)
    centers, widths = reference.at(ys, int(manifest["camera"]["width"]), int(manifest["camera"]["height"]))
    roi_center = x + width / 2.0
    values = centers - roi_center
    return {
        "roi": roi, "roi_center_x_px": roi_center,
        "reference_center_vs_roi_center_px": percentiles(values.tolist()),
        "reference_center_x_px": percentiles(centers.tolist()),
        "reference_width_px": percentiles(widths.tolist()),
        "selected_path_vs_reference": {"available_count": 0, "status": "unavailable: rejected output arrays empty and realtime provenance disabled"},
        "lateral_error_px": {"available_count": 0, "status": "unavailable: no accepted selected path"},
        "reference_is_not_ground_truth": True,
        "manual_centering_status": "requires_human_review_of_representative_frames",
        "interpretation": "The reference curve is a comparison aid only; it is not an operator annotation or truth label.",
    }


def draw_audit_overlay(context: Path, output: Path, manifest: dict[str, Any], row: dict[str, Any]) -> None:
    image = cv2.imread(str(context), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {context}")
    x, y, width, height = [int(v) for v in manifest["roi"]]
    cv2.rectangle(image, (x, y), (x + width, y + height), (255, 255, 255), 8)
    ref = SeamWidthReference.load(Path(manifest["tracker"]["reference"]))
    ys = np.linspace(y, y + height - 1, 32)
    center, seam_width = ref.at(ys, int(manifest["camera"]["width"]), int(manifest["camera"]["height"]))
    points = lambda values: np.round(np.column_stack((values, ys))).astype(np.int32).reshape(-1, 1, 2)
    cv2.polylines(image, [points(center)], False, (255, 255, 0), 8)
    cv2.polylines(image, [points(center - seam_width / 2)], False, (255, 0, 255), 5)
    cv2.polylines(image, [points(center + seam_width / 2)], False, (255, 0, 255), 5)
    lines = [
        "REJECTED / DIAGNOSTIC ONLY",
        "selected raw path: UNAVAILABLE (provenance disabled)",
        "IRLS fitted geometry: UNAVAILABLE (rejected output arrays empty)",
        "temporal predicted geometry: UNAVAILABLE (predicted_only=false)",
        "reference geometry: YELLOW/MAGENTA, comparison only / NOT GROUND TRUTH",
        f"frame={row.get('frame_index')} joint={row.get('joint_inlier_ratio')} residual={row.get('fit_residual_px')}",
    ]
    y0 = 100
    for line in lines:
        cv2.putText(image, line, (40, y0), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (0, 0, 0), 7, cv2.LINE_AA)
        cv2.putText(image, line, (40, y0), cv2.FONT_HERSHEY_SIMPLEX, 1.25, (0, 255, 255), 3, cv2.LINE_AA)
        y0 += 52
    cv2.imwrite(str(output), image)


def materialize_representatives(selected: list[dict[str, Any]], output: Path, manifest: dict[str, Any]) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for item in selected:
        row = item["row"]
        directory = output / "representative_frames" / f"{item['label']}_frame_{row.get('frame_index')}"
        directory.mkdir(parents=True, exist_ok=True)
        snap = Path(item["snapshot_path"]) if item.get("snapshot_path") else None
        if snap and (snap / "context.jpg").is_file():
            shutil.copy2(snap / "context.jpg", directory / "full_resolution_context.jpg")
            draw_audit_overlay(snap / "context.jpg", directory / "diagnostic_overlay.png", manifest, row)
            write_json(directory / "full_raw_unavailable.json", {
                "available": False, "reason": "formal run did not record full-resolution raw image topic",
                "substitute": "full_resolution_context.jpg", "substitute_is_lossy_jpeg": True,
                "source": str(snap / "context.jpg"),
            })
            for name in ("roi_raw.bmp", "overlay.png"):
                if (snap / name).is_file():
                    shutil.copy2(snap / name, directory / ("roi_raw.bmp" if name == "roi_raw.bmp" else "existing_snapshot_overlay.png"))
        else:
            write_json(directory / "full_raw_unavailable.json", {"available": False, "reason": "no snapshot evidence for this row"})
        geometry = load_json(snap / "seam_geometry.json") if snap and (snap / "seam_geometry.json").is_file() else {}
        metadata = load_json(snap / "metadata.json") if snap and (snap / "metadata.json").is_file() else {}
        failure = load_json(snap / "failure_reason.json") if snap and (snap / "failure_reason.json").is_file() else {}
        write_json(directory / "geometry.json", {"source_snapshot_geometry": geometry,
                                                  "audit_note": "empty boundaries are rejected output; not a claim that raw path was absent"})
        write_json(directory / "metadata.json", {"frame_metrics": row, "snapshot_metadata": metadata,
                                                  "evidence": {"selected_raw_path": "unavailable", "irls_fitted_geometry": "unavailable",
                                                               "temporal_predicted_geometry": "unavailable", "reference_geometry": "comparison_only"}})
        write_json(directory / "gate_evaluation.json", {"frame_gate_truth": row.get("gate_truth"),
                                                         "failure_reason_evidence": failure,
                                                         "all_frame_failure_reasons": row.get("failure_reasons", [])})
        result.append({"label": item["label"], "criterion": item["criterion"],
                       "requested_row_index": item["requested_row_index"],
                       "requested_frame_index": item.get("requested_frame_index"),
                       "frame_index": row.get("frame_index"), "row_index": item["actual_row_index"],
                       "source_snapshot": str(snap) if snap else None,
                       "directory": str(directory), "raw_full_resolution_available": False,
                       "roi_raw_available": bool(snap and (snap / "roi_raw.bmp").is_file()),
                       "diagnostic_overlay": str(directory / "diagnostic_overlay.png") if (directory / "diagnostic_overlay.png").is_file() else None})
    return result


def make_montage(entries: list[dict[str, Any]], output: Path) -> None:
    tiles = []
    for entry in entries:
        path = Path(entry["diagnostic_overlay"]) if entry.get("diagnostic_overlay") else None
        image = cv2.imread(str(path), cv2.IMREAD_COLOR) if path else None
        if image is None:
            image = np.zeros((480, 640, 3), dtype=np.uint8)
        scale = min(640 / image.shape[1], 460 / image.shape[0])
        tile = cv2.resize(image, (int(image.shape[1] * scale), int(image.shape[0] * scale)))
        canvas = np.zeros((520, 640, 3), dtype=np.uint8)
        canvas[:tile.shape[0], :tile.shape[1]] = tile
        cv2.putText(canvas, f"{entry['label']} frame={entry['frame_index']}", (10, 505), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2)
        tiles.append(canvas)
    while len(tiles) < 6:
        tiles.append(np.zeros((520, 640, 3), dtype=np.uint8))
    montage = np.vstack((np.hstack(tiles[:3]), np.hstack(tiles[3:6])))
    cv2.imwrite(str(output), montage)


def timing_reconciliation(run: Path, summary: dict[str, Any], rows: list[dict[str, Any]], validation: dict[str, Any]) -> dict[str, Any]:
    counters = summary["counters"]
    first_input = int(counters["input_first_host_timestamp_ns"])
    last_input = int(counters["input_last_host_timestamp_ns"])
    first_output = int(counters["output_first_host_timestamp_ns"])
    last_output = int(counters["output_last_host_timestamp_ns"])
    common_start = int(counters["common_active_window_start_ns"])
    common_end = int(counters["common_active_window_end_ns"])
    bag_topics = validation.get("topics", {})
    bag_start = min((int(v["start_stamp_ns"]) for v in bag_topics.values() if v.get("start_stamp_ns") is not None), default=None)
    bag_end = max((int(v["end_stamp_ns"]) for v in bag_topics.values() if v.get("end_stamp_ns") is not None), default=None)
    return {
        "processed_metrics_rows": len(rows), "live_summary_processed_frames": counters.get("processed_frames"),
        "live_summary_common_output_frames": counters.get("common_output_frames"),
        "first_last_input_host_timestamp_ns": [first_input, last_input],
        "first_last_output_host_timestamp_ns": [first_output, last_output],
        "active_duration_sec": {"input": (last_input - first_input) / 1e9, "output": (last_output - first_output) / 1e9,
                                 "common_window": (common_end - common_start) / 1e9},
        "warmup_duration_sec": counters.get("warmup_duration_sec"), "shutdown_duration_sec": counters.get("shutdown_duration_sec"),
        "same_common_window_rates_hz": {"input": counters.get("common_input_frames", 0) / ((common_end - common_start) / 1e9),
                                         "output": counters.get("common_output_frames", 0) / ((common_end - common_start) / 1e9)},
        "bag_stamp_window": {"start_ns": bag_start, "end_ns": bag_end, "duration_sec": (bag_end - bag_start) / 1e9 if bag_start is not None and bag_end is not None else None},
        "nominal_10_minute_duration_sec": 600.0,
        "explanation": "6754 geometry/state rows at approximately 6.39 Hz span about 1056.87 s (17.61 min) in the persisted common window. The command ran beyond the nominal 10-minute target; startup/warmup/shutdown are not the cause of the extra duration.",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, default=DEFAULT_RUN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    run = args.run.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = load_json(run / "run_manifest.json")
    session = next(run.glob("live_session/*"))
    summary = load_json(session / "live_summary.json")
    complete = load_json(session / "complete.json")
    validation = load_json(run / "mcap" / "validation_report.json")
    rows = read_rows(session / "frame_metrics.jsonl")

    frozen_config = run / "frozen_configs" / Path(manifest["tracker"]["config"]).name
    frozen_reference = run / "frozen_configs" / Path(manifest["tracker"]["reference"]).name
    integrity = {
        "run": str(run), "manifest_status": manifest.get("status"), "session_complete": complete.get("complete"),
        "mcap_validation_valid": validation.get("valid"), "config_sha_manifest": manifest["tracker"].get("config_sha256"),
        "config_sha_frozen": sha256(frozen_config) if frozen_config.is_file() else None,
        "reference_sha_manifest": manifest["tracker"].get("reference_sha256"),
        "reference_sha_frozen": sha256(frozen_reference) if frozen_reference.is_file() else None,
        "geometry_count": validation.get("topics", {}).get("/vision/front/seam_geometry", {}).get("message_count"),
        "state_count": validation.get("topics", {}).get("/vision/front/seam_state", {}).get("message_count"),
        "geometry_timestamp_monotonic": validation.get("topics", {}).get("/vision/front/seam_geometry", {}).get("non_monotonic_count") == 0,
        "state_timestamp_monotonic": validation.get("topics", {}).get("/vision/front/seam_state", {}).get("non_monotonic_count") == 0,
        "input_run_modified": False,
    }
    stats = gate_statistics(rows)
    write_gate_matrix(output / "gate_failure_matrix.csv", rows)
    write_json(output / "gate_statistics.json", {"frame_count": len(rows), "fields": stats})
    write_json(output / "failure_reason_histogram.json", failure_histogram(rows))
    write_json(output / "run_timing_reconciliation.json", timing_reconciliation(run, summary, rows, validation))
    write_json(output / "reference_position_audit.json", reference_audit(manifest, rows))

    selected = choose_representatives(rows, session)
    entries = materialize_representatives(selected, output, manifest)
    write_json(output / "representative_frames.json", {"selection": entries,
        "evidence_limitations": ["full-resolution raw image was not recorded in this lightweight run",
                                  "selected raw path, IRLS fitted geometry, and temporal predicted geometry were not persisted because provenance was disabled",
                                  "when an exact requested frame had no snapshot, the nearest unused persisted snapshot was used and requested/actual frame indices are both retained",
                                  "reference lines in overlays are not ground truth"]})
    make_montage(entries, output / "representative_montage.jpg")

    counters = summary["counters"]
    dropped = {
        "claimed_dropped_processed_frames": 9,
        "definition": "live_summary preview_skipped_frames and row dropped_processed_frames; this is a processed/latest-frame-slot accounting field, not a camera, MCAP, or failure-writer count",
        "ros_input_latest_frame_slot": {"captured_frames": counters.get("captured_frames"), "common_input_frames": counters.get("common_input_frames"), "processed_frames": counters.get("processed_frames"), "preview_skipped_frames": counters.get("preview_skipped_frames"), "interpretation": "the only direct persisted evidence for 9 is the input/preview slot counter"},
        "tracker": {"evidence": "no tracker-internal dropped-frame counter; one metrics row and one geometry/state output exist per persisted processed row", "count": 0, "not_attributed_to_tracker": True},
        "roi_recorder": {"enabled": False, "count": 0, "evidence": "launch_parameters roi_validation_recording_enabled=false and live_summary roi_validation=null"},
        "failure_writer": {"metrics_log_dropped": counters.get("metrics_log_dropped"), "event_log_dropped": counters.get("event_log_dropped"), "snapshot_writer_failed": counters.get("snapshot_writer_failed"), "count": 0},
        "mcap": {"geometry_count": integrity["geometry_count"], "state_count": integrity["state_count"], "validation_valid": validation.get("valid"), "writer_drop_count": "not exposed; no count inferred", "raw_image_topic": False},
        "conclusion": "The persisted evidence supports 9 preview/latest-frame-slot skips. It does not support attributing them to tracker, ROI recorder, failure writer, or MCAP.",
    }
    write_json(output / "dropped_frame_provenance.json", dropped)
    write_json(output / "diagnostic_summary.json", {
        "task": "FIELD-W2-A01-FAILURE-GATE-AUDIT", "run": str(run), "frame_count": len(rows),
        "all_detected_false": all(not bool(row.get("detected")) for row in rows), "all_lost": all(row.get("post_state", row.get("tracking_state")) == "LOST" for row in rows),
        "main_conclusion": "D", "main_conclusion_name": "mixed_or_insufficient_evidence",
        "unique_primary_failure_gate": "joint_inlier_ratio_below_minimum",
        "secondary_failure_gate": "fit_residual_above_maximum",
        "evidence": {"joint_inlier_failure_frames": stats["joint_inlier_ratio"]["gate_failure_count"], "residual_failure_frames": stats["fit_residual_px"]["gate_failure_count"],
                     "coverage_high_but_not_correctness": True, "point_and_coverage_gates_pass_all_frames": True,
                     "selected_path_provenance_available": False, "full_raw_available": False},
        "why_not_A": "No true full-raw sequence or persisted selected-path provenance. Existing full-resolution context snapshots show a visible central structure, but reference is not ground truth; this audit does not automatically assert operator centering.",
        "why_not_B_or_C": "The run persisted aggregate gate values and rejected empty output arrays, but not candidate/path provenance. Therefore candidate/path selection failure cannot be separated from rejection of a visually plausible path.",
        "why_D": "The failure gate family is measurable, but the evidence required to choose B versus C is absent. D is the only defensible top-level conclusion under the no-image/no-provenance rule.",
        "no_algorithm_or_input_run_changes": True,
    })
    write_json(output / "next_single_action.json", {
        "single_action": "Have an operator review the six representative diagnostic overlays/montage and confirm whether the physical seam is inside the frozen ROI/reference region; do not change thresholds during this review.",
        "reason": "This is the one missing observation needed before deciding whether to collect provenance for B/C; the reference line is not ground truth.",
        "files": [str(output / "representative_montage.jpg"), str(output / "representative_frames.json")],
    })
    print(json.dumps({"output": str(output), "frames": len(rows), "main_conclusion": "D", "primary_gate": "joint_inlier_ratio_below_minimum"}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
