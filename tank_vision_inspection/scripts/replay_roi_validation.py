#!/usr/bin/env python3
"""Cold-start replay and deterministic comparison for ROI validation evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np

from seam_validation_common import compare_signatures, frame_record, reconstruct_from_roi, sha256_file, write_json
from tank_vision.stable_seam_tracker import StableSeamTracker
from tank_vision.types import FrameData


def _read_rows(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            if line.strip():
                try:
                    rows.append(json.loads(line))
                except json.JSONDecodeError as exc:
                    raise ValueError(f"invalid frames.jsonl line {line_number}: {exc}") from exc
    return rows


def _load_roi(handle: Any, row: dict[str, Any]) -> np.ndarray:
    handle.seek(int(row["offset"]))
    payload = handle.read(int(row["length"]))
    if len(payload) != int(row["length"]):
        raise ValueError(f"short ROI payload at processed_index={row.get('processed_index')}")
    shape = tuple(int(value) for value in row["shape"])
    if len(payload) != int(np.prod(shape)):
        raise ValueError(f"ROI byte count mismatch at processed_index={row.get('processed_index')}")
    return np.frombuffer(payload, dtype=np.uint8).reshape(shape).copy()


def _raw_integrity(rows: list[dict[str, Any]], raw_path: Path) -> dict[str, Any]:
    expected = 0
    offsets_continuous = True
    for row in rows:
        offsets_continuous = offsets_continuous and int(row["offset"]) == expected
        expected += int(row["length"])
    return {
        "offsets_continuous": offsets_continuous,
        "expected_bytes": expected,
        "actual_bytes": raw_path.stat().st_size,
        "raw_file_length_correct": raw_path.stat().st_size == expected,
    }


def _geometry_signature(geometry: dict[str, Any]) -> dict[str, Any]:
    return {
        "detected": bool(geometry["detected"]),
        "predicted_only": bool(geometry["predicted_only"]),
        "tracking_state": int(geometry["tracking_state"]),
        "confidence": float(geometry["confidence"]),
        "left_boundary": geometry["left_boundary"],
        "right_boundary": geometry["right_boundary"],
        "centerline": geometry["centerline"],
        "coverage": float(geometry["coverage"]),
        "effective_span": float(geometry["effective_span"]),
        "fit_residual_px": geometry["fit_residual_px"],
    }


def _replay_geometry(result: Any) -> dict[str, Any]:
    return {
        "detected": bool(result.detected),
        "predicted_only": bool(result.predicted_only),
        "tracking_state": int(result.tracking_state),
        "confidence": float(result.confidence),
        "left_boundary": [{"x": float(x), "y": float(y), "z": 0.0} for x, y in result.left_boundary_original_px],
        "right_boundary": [{"x": float(x), "y": float(y), "z": 0.0} for x, y in result.right_boundary_original_px],
        "centerline": [{"x": float(x), "y": float(y), "z": 0.0} for x, y in result.centerline_original_px],
        "coverage": float(result.coverage),
        "effective_span": float(result.effective_span),
        "fit_residual_px": (float(result.fit_residual_px) if result.fit_residual_px is not None and np.isfinite(result.fit_residual_px) else None),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--tolerance", type=float, default=1e-6)
    args = parser.parse_args()
    session = args.session.expanduser().resolve()
    manifest = json.loads((session / "manifest.json").read_text(encoding="utf-8"))
    rows = _read_rows(session / manifest.get("frames_file", "frames.jsonl"))
    raw_path = session / manifest.get("raw_file", "roi_frames.raw")
    config = Path(manifest["config"])
    reference = Path(manifest["reference"])
    config_match = sha256_file(config) == manifest.get("config_sha256")
    reference_match = sha256_file(reference) == manifest.get("reference_sha256")
    integrity = _raw_integrity(rows, raw_path)
    full_shape = tuple(int(value) for value in manifest["input_full_shape"])
    roi = tuple(int(value) for value in manifest["roi_original_px"])

    tracker = StableSeamTracker(config, reference)
    deterministic_tracker = StableSeamTracker(config, reference)
    mismatches: list[dict[str, Any]] = []
    matched = 0
    replay_rows: list[dict[str, Any]] = []
    with raw_path.open("rb") as handle:
        for expected in rows:
            roi_image = _load_roi(handle, expected)
            image = reconstruct_from_roi(roi_image, full_shape, roi)
            packet = SimpleNamespace(
                captured_index=int(expected["captured_index"]),
                host_timestamp_ns=int(expected["host_timestamp_ns"]),
                telemetry_unavailable_fields=[],
                frame=FrameData(image=image, timestamp_ns=int(expected["device_timestamp_ns"]),
                                frame_id=str(expected.get("frame_id", "roi_replay")),
                                sequence_id=int(expected["captured_index"]), source_name="roi_validation_replay"),
            )
            pre_state = tracker.state.name
            result = tracker.process(image, diagnostics=False, provenance=False)
            replay_row = frame_record(packet, pre_state, result, tracker)
            replay_rows.append(replay_row)
            actual_geometry = _replay_geometry(result)
            expected_geometry = _geometry_signature(expected["geometry"])
            fields = compare_signatures(expected_geometry, actual_geometry)
            expected_crossing = expected.get("crossing", {})
            actual_crossing = {
                "selected_pair_crossing_count": int(result.selected_pair_crossing_count),
                "fitted_output_crossing_count": int(result.fitted_output_crossing_count),
                "minimum_fitted_width_px": result.minimum_fitted_width_px,
            }
            fields.extend(compare_signatures(expected_crossing, actual_crossing))
            for key in ("state", "detected", "predicted_only", "failure_reasons"):
                expected_value = expected.get(key)
                actual_value = result.tracking_state.name if key == "state" else getattr(result, key)
                if expected_value != actual_value:
                    fields.append(key)
            deterministic_result = deterministic_tracker.process(image, diagnostics=False, provenance=False)
            fields.extend(compare_signatures(
                {"detected": bool(result.detected), "state": result.tracking_state.name,
                 "centerline": result.centerline_original_px, "left": result.left_boundary_original_px,
                 "right": result.right_boundary_original_px},
                {"detected": bool(deterministic_result.detected), "state": deterministic_result.tracking_state.name,
                 "centerline": deterministic_result.centerline_original_px, "left": deterministic_result.left_boundary_original_px,
                 "right": deterministic_result.right_boundary_original_px},
            ))
            if fields:
                mismatches.append({"processed_index": expected.get("processed_index"), "fields": sorted(set(fields))})
            matched += 1

    report = {
        "schema_version": "1.0",
        "status": "passed" if matched == len(rows) and not mismatches and config_match and reference_match and integrity["raw_file_length_correct"] and integrity["offsets_continuous"] else "failed",
        "evidence_level": "field_validation",
        "matched_frame_count": matched,
        "unmatched_frame_count": 0,
        "mismatch_count": len(mismatches),
        "first_mismatch": mismatches[0] if mismatches else None,
        "config_sha256": manifest.get("config_sha256"),
        "reference_sha256": manifest.get("reference_sha256"),
        "config_sha_match": config_match,
        "reference_sha_match": reference_match,
        "raw_integrity": integrity,
        "deterministic_replay_checked": True,
        "deterministic_claim": "same frozen ROI sequence, same config/reference, cold-start tracker; not a claim about a different temporal sequence",
        "mismatches": mismatches,
    }
    output = args.output or session / "replay_report.json"
    write_json(output, report)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["status"] == "passed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
