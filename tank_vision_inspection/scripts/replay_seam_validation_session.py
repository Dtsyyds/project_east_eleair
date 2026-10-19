#!/usr/bin/env python3
"""Replay every raw capture frame and export lightweight results plus keyframe trace."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from seam_validation_common import (  # noqa: E402
    compare_signatures,
    diagnostic_overlay,
    distribution,
    frame_record,
    json_safe,
    preview_frame,
    reconstruct_from_roi,
    sha256_file,
    tracker_signature,
    write_json,
)
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402
from tank_vision.types import FrameData  # noqa: E402


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--keyframe-index", action="append", type=int, default=[])
    return parser.parse_args()


def _read_capture_rows(session: Path) -> list[dict[str, Any]]:
    path = session / "capture_frames.jsonl"
    if not path.exists():
        # Compatibility with the first recorder implementation only.
        path = session / "frames.jsonl"
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if "capture_index" not in row:
                    row["capture_index"] = row.get("frame_index", len(rows))
                rows.append(row)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid capture JSONL line {line_number}: {exc}") from exc
    return rows


def _load_raw(handle: Any, row: dict[str, Any]) -> np.ndarray:
    handle.seek(int(row["raw_offset"]))
    payload = handle.read(int(row["raw_nbytes"]))
    if len(payload) != int(row["raw_nbytes"]):
        raise ValueError(f"truncated raw payload at capture_index={row['capture_index']}")
    shape = tuple(int(value) for value in row["raw_shape"])
    if len(payload) != int(np.prod(shape)):
        raise ValueError(f"raw byte count mismatch at capture_index={row['capture_index']}")
    return np.frombuffer(payload, dtype=np.uint8).reshape(shape).copy()


def _raw_integrity(rows: list[dict[str, Any]], raw_path: Path) -> dict[str, Any]:
    expected_offset = 0
    capture_continuous = True
    offsets_continuous = True
    for index, row in enumerate(rows):
        capture_continuous = capture_continuous and int(row["capture_index"]) == index
        offsets_continuous = offsets_continuous and int(row["raw_offset"]) == expected_offset
        expected_offset += int(row["raw_nbytes"])
    return {"capture_index_continuous": capture_continuous, "offsets_continuous": offsets_continuous,
            "expected_bytes": expected_offset, "actual_bytes": raw_path.stat().st_size,
            "raw_file_length_correct": raw_path.stat().st_size == expected_offset}


def _keyframe_reasons(row: dict[str, Any], previous: dict[str, Any] | None, seen: set[str], explicit: set[int], minima: dict[str, float]) -> list[str]:
    reasons: list[str] = []
    index = int(row["capture_index"])
    if index == 0: reasons.append("first_frame")
    if index in explicit: reasons.append("user_specified")
    state = row["tracking_state"]
    if f"first_{state}" not in seen: reasons.append(f"first_{state}")
    for failure in row.get("failure_reasons", []):
        if f"failure:{failure}" not in seen: reasons.append(f"first_failure:{failure}")
    if row.get("joint_inlier_ratio") is not None and float(row["joint_inlier_ratio"]) < minima["joint"]:
        minima["joint"] = float(row["joint_inlier_ratio"]); reasons.append("minimum_joint_inlier")
    if row.get("fit_residual_px") is not None and float(row["fit_residual_px"]) > minima["residual"]:
        minima["residual"] = float(row["fit_residual_px"]); reasons.append("maximum_fit_residual")
    if previous is not None:
        if previous["tracking_state"] != state:
            reasons.append(f"transition_{previous['tracking_state']}_to_{state}")
    return sorted(set(reasons))


def _failure_segments(rows: list[dict[str, Any]]) -> list[dict[str, Any]]:
    segments: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    for row in rows:
        reasons = tuple(row.get("failure_reasons", [])) or ("PASS",)
        state = row["tracking_state"]
        if current is None or current["reasons"] != list(reasons) or current["state"] != state:
            if current is not None:
                segments.append(current)
            current = {"start_capture_index": row["capture_index"], "end_capture_index": row["capture_index"],
                       "start_host_timestamp_ns": row["host_timestamp_ns"], "end_host_timestamp_ns": row["host_timestamp_ns"],
                       "frame_count": 1, "state": state, "reasons": list(reasons)}
        else:
            current["end_capture_index"] = row["capture_index"]
            current["end_host_timestamp_ns"] = row["host_timestamp_ns"]
            current["frame_count"] += 1
    if current is not None: segments.append(current)
    for segment in segments:
        segment["duration_ms"] = (segment["end_host_timestamp_ns"] - segment["start_host_timestamp_ns"]) / 1e6
    return segments


def _longest_lost(rows: list[dict[str, Any]]) -> dict[str, Any]:
    best: dict[str, Any] | None = None
    start = None
    for index, row in enumerate(rows + [{"tracking_state": "__END__"}]):
        if row.get("tracking_state") == "LOST" and start is None:
            start = index
        if row.get("tracking_state") != "LOST" and start is not None:
            values = rows[start:index]
            candidate = {"frame_count": len(values), "start_capture_index": values[0]["capture_index"],
                         "end_capture_index": values[-1]["capture_index"],
                         "duration_ms": (values[-1]["host_timestamp_ns"] - values[0]["host_timestamp_ns"]) / 1e6}
            if best is None or candidate["duration_ms"] > best["duration_ms"]:
                best = candidate
            start = None
    return best or {"frame_count": 0, "start_capture_index": None, "end_capture_index": None, "duration_ms": 0.0}


def main() -> int:
    args = _args()
    session = args.session.expanduser().resolve()
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
    rows = _read_capture_rows(session)
    if not rows:
        raise SystemExit("capture JSONL contains no frames")
    config = Path(manifest["config"]); reference = Path(manifest["reference"])
    config_sha_match = sha256_file(config) == manifest["config_sha256"]
    reference_sha_match = sha256_file(reference) == manifest["reference_sha256"]
    raw_path = session / manifest.get("raw_file", "roi_frames.raw")
    integrity = _raw_integrity(rows, raw_path)
    full_shape = tuple(int(value) for value in manifest.get("input_full_shape", [3072, 4096, 3]))
    roi = tuple(int(value) for value in manifest["roi_original_px"])
    storage_mode = manifest.get("storage_mode", "roi")
    tracker = StableSeamTracker(config, reference); tracker.reset()
    deterministic_tracker = StableSeamTracker(config, reference); deterministic_tracker.reset()
    trace_explicit = set(args.keyframe_index)
    seen: set[str] = set(); minima = {"joint": float("inf"), "residual": -float("inf")}
    replay_rows: list[dict[str, Any]] = []; mismatches: list[dict[str, Any]] = []
    history: list[str] = []; previous: dict[str, Any] | None = None
    video = cv2.VideoWriter(str(session / "replay_diagnostic.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                            max(args.preview_fps, 1.0), (args.preview_width, 720))
    if not video.isOpened(): raise SystemExit("could not open replay_diagnostic.mp4")
    keyframe_dir = session / "keyframes"; keyframe_dir.mkdir(exist_ok=True)
    replay_started = time.perf_counter_ns()
    try:
        with raw_path.open("rb") as handle:
            for capture_row in rows:
                stored = _load_raw(handle, capture_row)
                image = reconstruct_from_roi(stored, full_shape, roi) if storage_mode == "roi" else stored
                pre_state = tracker.state.name
                result = tracker.process(image, diagnostics=False, provenance=False)
                row = frame_record(type("Packet", (), {"captured_index": capture_row["capture_index"], "frame": FrameData(
                    image=image, timestamp_ns=int(capture_row["device_timestamp_ns"]), frame_id="replay",
                    sequence_id=int(capture_row["capture_index"]), source_name="seam_validation_replay",
                    exposure_us=capture_row.get("exposure_us"), gain_db=capture_row.get("gain_db"),
                    analog_gain_raw=capture_row.get("analog_gain_raw"), exposure_mode=capture_row.get("exposure_mode")),
                    "host_timestamp_ns": capture_row["host_timestamp_ns"], "telemetry_unavailable_fields": capture_row.get("telemetry_unavailable_fields", [])})(),
                                    pre_state, result, tracker)
                row["record_type"] = "offline_replay"
                row["capture_index"] = capture_row["capture_index"]
                row["processing_ms"] = float(result.processing_ms)
                row["raw_offset"] = capture_row["raw_offset"]; row["raw_nbytes"] = capture_row["raw_nbytes"]
                row["raw_shape"] = capture_row["raw_shape"]
                # Independent same-input tracker proves deterministic replay without changing the primary state.
                deterministic_result = deterministic_tracker.process(image, diagnostics=False, provenance=False)
                mismatch_fields = compare_signatures(tracker_signature(tracker, result), tracker_signature(deterministic_tracker, deterministic_result))
                if mismatch_fields:
                    mismatches.append({"capture_index": capture_row["capture_index"], "fields": mismatch_fields})
                replay_rows.append(row)
                reasons = _keyframe_reasons(row, previous, seen, trace_explicit, minima)
                if reasons:
                    trace_tracker = StableSeamTracker(config, reference); trace_tracker.reset()
                    trace_result = trace_tracker.process(image, diagnostics=True, provenance=True)
                    trace_overlay = diagnostic_overlay(image, trace_result, trace_tracker)
                    stem = f"replay_{int(capture_row['capture_index']):06d}"
                    cv2.imwrite(str(keyframe_dir / f"{stem}.roi_raw.bmp"), stored if storage_mode == "roi" else roi_crop(image, roi))
                    cv2.imwrite(str(keyframe_dir / f"{stem}.overlay.png"), trace_overlay)
                    write_json(keyframe_dir / f"{stem}.frame_metrics.json", {"diagnostic_only": True, "reasons": reasons,
                        "replay_frame": row, "trace_signature": tracker_signature(trace_tracker, trace_result)})
                    seen.update(reasons); seen.update(f"first_{row['tracking_state']}" for _ in [0])
                    seen.update(f"failure:{failure}" for failure in row.get("failure_reasons", []))
                preview = preview_frame(image, result, tracker, row, history, args.preview_width)
                video.write(preview)
                history.append(result.tracking_state.name); previous = row
    finally:
        video.release()
    replay_elapsed = (time.perf_counter_ns() - replay_started) / 1e9
    replay_path = session / "replay_frames.jsonl"
    with replay_path.open("w", encoding="utf-8") as handle:
        for row in replay_rows:
            handle.write(json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False) + "\n")
    columns = ["capture_index", "pre_state", "post_state", "detected", "predicted_only", "confidence", "coverage",
               "effective_span", "point_count", "maximum_missing_run", "abrupt_left_jump_count", "abrupt_right_jump_count",
               "crossing_count", "path_confidence", "center_inlier_ratio", "width_inlier_ratio", "joint_inlier_ratio",
               "fit_residual_px", "median_width_px", "lateral_error_px", "heading_error_deg", "processing_ms",
               "failure_reasons", "human_readable_reason"]
    with (session / "replay_frame_metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns); writer.writeheader()
        for row in replay_rows:
            output = {key: row.get(key) for key in columns}
            output["failure_reasons"] = ";".join(row.get("failure_reasons", []))
            writer.writerow(output)
    segments = _failure_segments(replay_rows); write_json(session / "failure_segments.json", {"segments": segments})
    failure_counts = {reason: sum(reason in row.get("failure_reasons", []) for row in replay_rows)
                      for reason in sorted({reason for row in replay_rows for reason in row.get("failure_reasons", [])})}
    lost = _longest_lost(replay_rows)
    camera_span = (rows[-1]["host_timestamp_ns"] - rows[0]["host_timestamp_ns"]) / 1e9 if len(rows) >= 2 else 0.0
    replay_summary = {"status": "complete", "capture_frames": len(rows), "replay_frames": len(replay_rows),
        "captured_frames": manifest.get("counters", {}).get("captured_frames"),
        "raw_stored_frames": manifest.get("counters", {}).get("raw_stored_frames"),
        "camera_active_duration_sec": camera_span, "offline_replay_wall_duration_sec": replay_elapsed,
        "camera_fps": (len(rows) - 1) / camera_span if camera_span > 0 else 0.0,
        "offline_replay_fps": len(replay_rows) / replay_elapsed if replay_elapsed > 0 else 0.0,
        "processing_ms": distribution([float(row["processing_ms"]) for row in replay_rows]),
        "detected_rate": float(np.mean([row["detected"] for row in replay_rows])),
        "state_counts": {state: sum(row["tracking_state"] == state for row in replay_rows) for state in ("TRACKING", "ACQUIRING", "DEGRADED", "LOST")},
        "failure_reason_counts": failure_counts, "longest_lost_frames": lost["frame_count"], "longest_lost_ms": lost["duration_ms"],
        "config_reference_sha_match": config_sha_match and reference_sha_match, "roi_reconstruction_match": bool(manifest.get("roi_reconstruction", {}).get("match", True)),
        "raw_integrity": integrity, "deterministic_exact_match_count": len(replay_rows) - len(mismatches),
        "mismatch_count": len(mismatches), "first_mismatch_frame": mismatches[0]["capture_index"] if mismatches else None,
        "mismatches": mismatches, "diagnostic_provenance": "keyframes only; independent tracker; diagnostic_only=true"}
    write_json(session / "replay_summary.json", replay_summary)
    write_json(session / "replay_comparison.json", {"raw_integrity": integrity, "raw_sequence_valid": bool(integrity["raw_file_length_correct"] and integrity["capture_index_continuous"] and integrity["offsets_continuous"]),
        "exact_match_count": len(replay_rows) - len(mismatches), "mismatch_count": len(mismatches),
        "first_mismatch_frame": mismatches[0]["capture_index"] if mismatches else None,
        "mismatches": mismatches, "replayable": not mismatches and integrity["raw_file_length_correct"]})
    validation_rule = dict(manifest.get("validation_rule", {}))
    validation_rule.update({"replay_frames_equals_raw_stored": len(replay_rows) == int(manifest.get("counters", {}).get("raw_stored_frames", len(rows))),
                            "config_reference_sha_match": config_sha_match and reference_sha_match,
                            "roi_reconstruction_match": bool(manifest.get("roi_reconstruction", {}).get("match", True)),
                            "replay_mismatch_zero": len(mismatches) == 0,
                            "raw_file_length_correct": integrity["raw_file_length_correct"],
                            "capture_index_continuous": integrity["capture_index_continuous"],
                            "raw_offsets_continuous": integrity["offsets_continuous"]})
    complete = all(value is True for value in validation_rule.values())
    manifest.update({"recording": False, "complete": complete, "validation_rule": validation_rule,
                     "replay_summary": "replay_summary.json", "replay_frames": len(replay_rows)})
    write_json(session / "manifest.json", manifest)
    write_json(session / "complete.json", {"recording": False, "complete": complete,
        "valid_for_formal_validation": complete, "validation_rule": validation_rule,
        "counters": manifest.get("counters", {}), "replay_summary": "replay_summary.json"})
    return 0 if complete else 2


if __name__ == "__main__":
    raise SystemExit(main())
