#!/usr/bin/env python3
"""Offline classification for a geometry-provenance session.

The script is intentionally conservative: without three human annotations it
emits ``manual_annotation_pending`` and never treats the frozen reference as
truth.  An annotation JSON may use original-image coordinates (the existing
annotation tool) or ROI-local coordinates with ``coordinate_system`` set to
``roi_pixels``.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

try:
    from tank_vision.seam_width_reference import SeamWidthReference
except ImportError:  # pragma: no cover - allows --help from a bare checkout
    SeamWidthReference = None


def load(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def finite(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if np.isfinite(value) else None


def percentile(values: list[float]) -> dict[str, Any]:
    if not values:
        return {key: None for key in ("count", "min", "p05", "p50", "p95", "max")}
    data = np.asarray(values, dtype=np.float64)
    return {"count": len(data), "min": float(data.min()), "p05": float(np.percentile(data, 5)),
            "p50": float(np.percentile(data, 50)), "p95": float(np.percentile(data, 95)), "max": float(data.max())}


def reference_center_comparison(events: list[tuple[Path, dict[str, Any], dict[str, Any]]], annotations: dict[str, dict[str, Any]]) -> dict[str, Any]:
    if SeamWidthReference is None:
        return {"status": "reference_loader_unavailable", "reference_not_ground_truth": True}
    reference_path = None
    for _, provenance, metadata in events:
        candidate = metadata.get("reference_path") or provenance.get("reference_path")
        if candidate:
            reference_path = Path(candidate)
            break
    if reference_path is None:
        reference_path = Path("configs/seam_geometry_reference_s100p_live_v1.json")
    try:
        reference = SeamWidthReference.load(reference_path)
    except (OSError, ValueError, KeyError):
        return {"status": "reference_unavailable", "path": str(reference_path), "reference_not_ground_truth": True}
    per_event = []
    all_abs: list[float] = []
    roi = [1229, 1536, 1229, 1290]
    for event_name, annotation in annotations.items():
        left = sorted([(float(item["y"]) + roi[1], float(item["x"]) + roi[0])
                       for item in annotation.get("left_boundary_manual_points", [])])
        right = sorted([(float(item["y"]) + roi[1], float(item["x"]) + roi[0])
                        for item in annotation.get("right_boundary_manual_points", [])])
        if len(left) < 2 or len(right) < 2:
            continue
        y0, y1 = max(left[0][0], right[0][0]), min(left[-1][0], right[-1][0])
        ys = np.linspace(y0, y1, 32)
        lm = np.interp(ys, [item[0] for item in left], [item[1] for item in left])
        rm = np.interp(ys, [item[0] for item in right], [item[1] for item in right])
        manual_center = (lm + rm) * 0.5
        reference_center, _ = reference.at(ys, 4096, 3072)
        delta = manual_center - reference_center
        absolute = np.abs(delta).tolist()
        all_abs.extend(absolute)
        per_event.append({"event": event_name, "reference_not_ground_truth": True,
                          "center_delta_px": percentile(delta.tolist()),
                          "absolute_center_delta_px": percentile(absolute)})
    return {"status": "complete", "reference_not_ground_truth": True, "path": str(reference_path),
            "events": per_event, "aggregate_absolute_center_delta_px": percentile(all_abs)}


def points(annotation: dict[str, Any], side: str, roi: list[int]) -> list[tuple[float, float]]:
    values = annotation.get(f"{side}_boundary_manual_points", annotation.get(side, []))
    output = [(float(item["x"]), float(item["y"])) for item in values]
    if annotation.get("coordinate_system") == "roi_pixels":
        output = [(x + roi[0], y + roi[1]) for x, y in output]
    return sorted(output, key=lambda item: item[1])


def interpolate(points_: list[tuple[float, float]], ys: np.ndarray) -> np.ndarray:
    return np.interp(ys, [item[1] for item in points_], [item[0] for item in points_])


def nearest(values: list[dict[str, Any]], x: float, key: str = "x") -> float | None:
    if not values:
        return None
    return min((abs(float(item[key]) - x) for item in values), default=None)


def event_dirs(session: Path) -> list[Path]:
    return sorted(item for item in session.glob("event_*") if (item / "complete.json").exists() is False
                  and (item / "metadata.json").is_file() and (item / "provenance.json").is_file())


def gate_margin_stats(events: list[tuple[Path, dict[str, Any], dict[str, Any]]]) -> dict[str, Any]:
    output: dict[str, Any] = {}
    names = set()
    for _, provenance, _ in events:
        names.update((provenance.get("result", {}).get("geometry_gates") or {}).keys())
    for name in sorted(names):
        values = []
        failures = 0
        for _, provenance, _ in events:
            item = (provenance.get("result", {}).get("geometry_gates") or {}).get(name, {})
            margin = finite(item.get("margin"))
            if margin is not None:
                values.append(margin)
            if item.get("passed") is False:
                failures += 1
        output[name] = {"margin": percentile(values), "failure_event_count": failures,
                        "failure_event_ratio": failures / len(events) if events else None}
    return output


def classify_band(band: dict[str, Any], selected_by_band: dict[int, dict[str, Any]], manual: dict[str, Any] | None,
                  roi: list[int], scales: dict[str, Any], fitted: dict[str, Any], gates: dict[str, Any]) -> dict[str, Any]:
    index = int(band.get("band_index", 0))
    y_processed = float(band.get("y_processed", 0.0))
    # Scan-band classification is conservative and uses the persisted trace;
    # it never uses the reference curve as a label.
    row: dict[str, Any] = {"band_index": index, "y_processed": y_processed}
    pairs = band.get("pair_stages", {}).get("pair_after_max_pairs_per_band", [])
    candidates = band.get("top_k_after", {})
    if manual is None:
        row.update({"classification": "manual_annotation_pending", "manual_available": False,
                    "candidate_layer": "unclassified", "pair_layer": "unclassified",
                    "dp_layer": "unclassified", "irls_layer": "unclassified", "gate_layer": "unclassified"})
        return row
    left_manual = points(manual, "left", roi)
    right_manual = points(manual, "right", roi)
    sx = max(float(scales.get("sx", 1.0)), 1e-9)
    sy = max(float(scales.get("sy", 1.0)), 1e-9)
    y_original = roi[1] + y_processed / sy
    lm = float(interpolate(left_manual, np.asarray([y_original]))[0])
    rm = float(interpolate(right_manual, np.asarray([y_original]))[0])
    left_candidates = candidates.get("left", [])
    right_candidates = candidates.get("right", [])
    # Candidate/pair traces are in the resized ROI coordinate system; manual
    # annotations and selected/fitted outputs are in original-image pixels.
    left_errors = [abs(roi[0] + float(item["x"]) / sx - lm) for item in left_candidates]
    right_errors = [abs(roi[0] + float(item["x"]) / sx - rm) for item in right_candidates]
    left_error = min(left_errors) if left_errors else None
    right_error = min(right_errors) if right_errors else None
    candidate_near = left_error is not None and right_error is not None and left_error <= 20 and right_error <= 20
    pair_near = [item for item in pairs if abs(roi[0] + float(item.get("left_processed", 0)) / sx - lm) <= 20
                 and abs(roi[0] + float(item.get("right_processed", 0)) / sx - rm) <= 20]
    selected = selected_by_band.get(index)
    selected_error = None if selected is None else max(abs(float(selected["left"]) - lm), abs(float(selected["right"]) - rm))
    fitted_error = None
    fitted_y = np.asarray(fitted.get("y", []), dtype=np.float64)
    fitted_left = np.asarray(fitted.get("left", []), dtype=np.float64)
    fitted_right = np.asarray(fitted.get("right", []), dtype=np.float64)
    if len(fitted_y) >= 2 and len(fitted_left) == len(fitted_y) and len(fitted_right) == len(fitted_y):
        fitted_error = max(abs(float(np.interp(y_original, fitted_y, fitted_left)) - lm),
                           abs(float(np.interp(y_original, fitted_y, fitted_right)) - rm))
    fitted_near = fitted_error is not None and fitted_error <= 20
    failed_gates = sorted(name for name, item in gates.items() if item.get("passed") is False)
    if not candidate_near:
        classification = "manual_candidate_missing"
    elif not pair_near:
        classification = "manual_near_pair_missing"
    elif selected is None:
        classification = "manual_near_pair_exists_but_lost_by_dp"
    elif selected_error is not None and selected_error > 20:
        classification = "selected_raw_pair_far_from_manual"
    elif not fitted_near:
        classification = "selected_raw_correct_but_irls_fitted_wrong"
    elif failed_gates:
        classification = "fitted_geometry_correct_but_rejected_by_gate"
    else:
        classification = "selected_raw_pair_near_manual"
    row.update({"classification": classification, "manual_available": True,
                "manual_left_x_original": lm, "manual_right_x_original": rm,
                "candidate_left_error_px": left_error, "candidate_right_error_px": right_error,
                "candidate_near": candidate_near, "manual_near_pair_count": len(pair_near),
                "selected_error_px": selected_error, "selected_raw_near_manual": selected_error is not None and selected_error <= 20,
                "fitted_error_px": fitted_error, "fitted_near_manual": fitted_near,
                "failed_gates": failed_gates})
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, default=None)
    args = parser.parse_args()
    session, output = args.session.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    events: list[tuple[Path, dict[str, Any], dict[str, Any]]] = []
    for event in event_dirs(session):
        try:
            events.append((event, load(event / "provenance.json"), load(event / "metadata.json")))
        except (OSError, json.JSONDecodeError):
            continue
    annotations: dict[str, dict[str, Any]] = {}
    if args.annotations and args.annotations.is_dir():
        # Accept both a flat annotation directory and one directory per event.
        # The latter keeps the three human-labeling sessions separate and avoids
        # accidental overwrites when the annotation tool uses a fixed basename.
        for item in args.annotations.rglob("*.json"):
            if item.name in {"manual_error_summary.json"}:
                continue
            try:
                annotation = load(item)
                event_name = item.parent.name if item.parent.name.startswith("event_") else item.stem
                annotations[event_name] = annotation
            except (OSError, json.JSONDecodeError):
                pass
    schema = load(session / "provenance_schema.json") if (session / "provenance_schema.json").is_file() else {}
    write = lambda name, value: (output / name).write_text(json.dumps(value, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write("provenance_schema.json", schema)
    index = []
    for event, provenance, metadata in events:
        index.append({"event": event.name, "path": str(event), "frame_index": metadata.get("captured_index"),
                      "labels": metadata.get("labels", []), "complete": metadata.get("complete"),
                      "config_sha256": metadata.get("config_sha256"), "reference_sha256": metadata.get("reference_sha256")})
    write("sampled_event_index.json", {"session": str(session), "event_count": len(events), "events": index})
    write("gate_margin_statistics.json", gate_margin_stats(events))

    classification_rows: list[dict[str, Any]] = []
    layer_errors: dict[str, list[float]] = {key: [] for key in ("selected_raw", "irls_fitted", "temporal", "final_output")}
    telemetry: list[dict[str, Any]] = []
    for event, provenance, metadata in events:
        roi = [int(item) for item in metadata.get("roi_original_px", [0, 0, 0, 0])]
        trace = provenance.get("result", {})
        selected = {int(item["band_index"]): item for item in provenance.get("selected_raw_path", [])}
        manual = annotations.get(event.name)
        scales = provenance.get("scales", {})
        fitted = {"y": trace.get("fitted_y", []), "left": trace.get("fitted_left", []),
                  "right": trace.get("fitted_right", [])}
        gates = trace.get("geometry_gates", {}) or {}
        for band in provenance.get("candidate_stage_trace", []):
            classification_rows.append({"event": event.name, "frame_index": metadata.get("captured_index"),
                                        **classify_band(band, selected, manual, roi, scales, fitted, gates)})
        telemetry.append(metadata.get("telemetry", {}))
        if manual is not None:
            left_manual = points(manual, "left", roi); right_manual = points(manual, "right", roi)
            ys = np.linspace(max(left_manual[0][1], right_manual[0][1]), min(left_manual[-1][1], right_manual[-1][1]), 128)
            lm, rm = interpolate(left_manual, ys), interpolate(right_manual, ys)
            for name, left_values, right_values, ys_values in (
                    ("selected_raw", [x["left"] for x in provenance.get("selected_raw_path", [])], [x["right"] for x in provenance.get("selected_raw_path", [])], [x["y"] for x in provenance.get("selected_raw_path", [])]),
                    ("irls_fitted", trace.get("fitted_left", []), trace.get("fitted_right", []), trace.get("fitted_y", [])),
                    ("temporal", trace.get("temporal_output", {}).get("left", []), trace.get("temporal_output", {}).get("right", []), trace.get("temporal_output", {}).get("y", [])),
                    ("final_output", [x[0] for x in metadata.get("geometry", {}).get("centerline", [])], [], [])):
                if name == "final_output":
                    continue
                if left_values and right_values and ys_values:
                    yv = np.asarray(ys_values, dtype=np.float64); lx = np.interp(ys, yv, left_values); rx = np.interp(ys, yv, right_values)
                    layer_errors[name].extend(np.maximum(np.abs(lx - lm), np.abs(rx - rm)).tolist())
    write("scan_band_failure_classification.json", {"rows": classification_rows, "manual_annotations_available": bool(annotations),
                                                     "reference_not_used_as_truth": True})
    if classification_rows:
        with (output / "scan_band_failure_classification.csv").open("w", newline="", encoding="utf-8") as handle:
            fields = sorted({key for row in classification_rows for key in row})
            writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader(); writer.writerows(classification_rows)
    else:
        (output / "scan_band_failure_classification.csv").write_text("event\n", encoding="utf-8")
    write("telemetry_summary.json", {"event_count": len(telemetry), "values": telemetry,
                                     "unavailable_fields": sorted({key for item in telemetry for key in item.get("telemetry_unavailable_fields", [])})})
    layer_report = {key: percentile(value) for key, value in layer_errors.items()}
    if not layer_errors["temporal"]:
        layer_report["temporal_status"] = "unavailable: all sampled frames rejected before temporal output"
    if not layer_errors["final_output"]:
        layer_report["final_output_status"] = "unavailable: no accepted geometry was published"
    write("manual_error_summary.json", {"status": "complete" if annotations else "manual_annotation_pending",
                                        "annotation_event_count": len(annotations), "layers": layer_report,
                                        "reference_comparison": reference_center_comparison(events, annotations),
                                        "reference_not_ground_truth": True})
    write("layer_error_comparison.json", layer_report)
    # A provenance session lives at <run>/geometry_provenance/<session>/.  Use
    # the run's live summary when available, but do not manufacture a rate from
    # the sparse provenance event count.
    live_summary = session.parents[1] / "live_session"
    live_candidates = sorted(live_summary.glob("*/live_summary.json")) if live_summary.is_dir() else []
    live = load(live_candidates[-1]) if live_candidates else {}
    write("performance_comparison.json", {
        "status": "live_summary_loaded" if live else "requires_run_live_summary_comparison",
        "provenance_enabled_event_count": len(events),
        "live_summary": live,
        "note": "Provenance is sampled and asynchronous; event count is not tracker frame rate."
    })

    tiles = []
    for event, _, _ in events[:6]:
        image = cv2.imread(str(event / "fitted_geometry_overlay.png"), cv2.IMREAD_COLOR)
        if image is not None:
            image = cv2.resize(image, (512, 360)); cv2.putText(image, event.name[-18:], (8, 350), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 255, 255), 1); tiles.append(image)
    while len(tiles) < 6: tiles.append(np.zeros((360, 512, 3), dtype=np.uint8))
    cv2.imwrite(str(output / "diagnostic_montage.jpg"), np.vstack((np.hstack(tiles[:3]), np.hstack(tiles[3:6]))))
    joint = sorted(events, key=lambda item: finite(item[2].get("joint_inlier_ratio")) if finite(item[2].get("joint_inlier_ratio")) is not None else float("inf"))
    selected_events = []
    if events:
        selected_events = [joint[0][0].name, sorted(events, key=lambda item: finite(item[2].get("joint_inlier_ratio")) or 0)[len(events)//2][0].name, sorted(events, key=lambda item: finite(item[2].get("joint_inlier_ratio")) or 0)[-1][0].name]
    class_counts = Counter(row.get("classification") for row in classification_rows if row.get("manual_available"))
    if not annotations:
        main_conclusion, main_name = "F", "mixed_or_insufficient_evidence"
    elif class_counts.get("manual_candidate_missing", 0) + class_counts.get("manual_near_pair_missing", 0) > max(
            class_counts.get("selected_raw_pair_far_from_manual", 0),
            class_counts.get("selected_raw_correct_but_irls_fitted_wrong", 0),
            class_counts.get("fitted_geometry_correct_but_rejected_by_gate", 0)):
        main_conclusion, main_name = "A", "candidate_localization_or_pair_formation_failure"
    else:
        main_conclusion, main_name = "F", "mixed_or_insufficient_evidence"
    write("diagnostic_summary.json", {"main_conclusion": main_conclusion,
                                      "main_conclusion_name": main_name,
                                      "event_count": len(events), "manual_annotation_event_count": len(annotations),
                                      "annotated_scan_band_class_counts": dict(class_counts),
                                      "reference_not_used_as_truth": True,
                                      "failure_classification_not_final": False if annotations else True,
                                      "selected_manual_truth_events": selected_events})
    write("next_single_action.json", {"single_action": "Audit candidate localization and guided-window/pair formation at the annotated bands; do not lower geometry gates or alter production parameters.",
                                      "main_conclusion": main_conclusion, "events": selected_events,
                                      "annotation_format": "existing manual annotation JSON; ROI-local coordinates use coordinate_system=roi_pixels"})
    print(json.dumps({"session": str(session), "events": len(events), "annotations": len(annotations),
                      "main_conclusion": main_conclusion, "main_conclusion_name": main_name}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
