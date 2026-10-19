#!/usr/bin/env python3
"""S1-P2H: ground-truth audit of three live LOST/detected decisions."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402

OUT = ROOT / "output/s1_p2h_three_frame_ground_truth_evaluation"
ANNOTATION_ROOT = ROOT / "output/s1_p2h_live_ground_truth"
SNAPSHOT_ROOT = ROOT / "output/stable_seam_failure_snapshots"
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
EXPECTED_RAW = {
    "capture_01": "b92d7fa379c987f0815e4b7e5c1c4cb7675bd182dfc2edd5e29501878b76bc32",
    "capture_03": "45834fda5bf183136bc555b176d6a7a535f1f087765e86744d4d3db1e5d9269d",
    "capture_05": "8d0f24cfcd1f3f464a8e18a2364209b0291b62e3bd5217fce81e3dbcb375877f",
}
CASES = {
    "capture_01": "20260910_160926_591_stamp_21705_511000000",
    "capture_03": "20260910_161037_929_stamp_21776_838700000",
    "capture_05": "20260910_161207_181_stamp_21866_60800000",
}


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(x) for x in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        x = float(value)
        return x if np.isfinite(x) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, TrackingState):
        return value.name
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


def write_json(name: str, value: Any) -> None:
    (OUT / name).write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None, "max": None}
    array = np.asarray(values, dtype=np.float64)
    return {"count": int(len(array)), "mean": float(np.mean(array)),
            "p50": float(np.percentile(array, 50)), "p90": float(np.percentile(array, 90)),
            "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}


def error_report(rows: list[dict[str, Any]]) -> dict[str, Any]:
    output = {}
    for field in ("left", "right", "center", "width"):
        errors = [float(row["estimate"][field] - row["manual"][field]) for row in rows]
        output[field] = {"signed": stats(errors), "absolute": stats([abs(x) for x in errors])}
    return output


def annotation_payload(name: str) -> dict[str, Any]:
    return json.loads((ANNOTATION_ROOT / name / "raw_annotation.json").read_text(encoding="utf-8"))


def points(payload: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    left = np.asarray([(float(p["y"]), float(p["x"])) for p in payload["left_boundary_manual_points"]])
    right = np.asarray([(float(p["y"]), float(p["x"])) for p in payload["right_boundary_manual_points"]])
    return left, right


def manual_at(y: float, manual: tuple[np.ndarray, np.ndarray]) -> dict[str, float]:
    left, right = manual
    ml = float(np.interp(y, left[:, 0], left[:, 1]))
    mr = float(np.interp(y, right[:, 0], right[:, 1]))
    return {"left": ml, "right": mr, "center": 0.5 * (ml + mr), "width": mr - ml}


def common_y_range(manual: tuple[np.ndarray, np.ndarray], roi: tuple[int, int, int, int]) -> tuple[float, float]:
    left, right = manual
    return max(float(left[0, 0]), float(right[0, 0]), float(roi[1])), min(
        float(left[-1, 0]), float(right[-1, 0]), float(roi[1] + roi[3] - 1))


def pair_rows(result: Any, diagnostics: Any, manual: tuple[np.ndarray, np.ndarray], y_range: tuple[float, float]) -> list[dict[str, Any]]:
    roi = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / roi[2]
    sy = diagnostics.processed_gray.shape[0] / roi[3]
    rows = []
    for pair in diagnostics.selected_pairs:
        y = float(roi[1] + pair.y / sy)
        if not y_range[0] <= y <= y_range[1]:
            continue
        left = float(roi[0] + pair.left / sx)
        right = float(roi[0] + pair.right / sx)
        rows.append({"band_index": int(pair.band_index), "y": y, "manual": manual_at(y, manual),
                     "estimate": {"left": left, "right": right, "center": 0.5 * (left + right),
                                  "width": right - left}})
    return rows


def coefficient_rows(coefficients: dict[str, Any], ys: np.ndarray, manual: tuple[np.ndarray, np.ndarray]) -> list[dict[str, Any]]:
    center = np.polyval(coefficients["center"], ys)
    width = np.polyval(coefficients["width"], ys)
    left = center - 0.5 * width
    right = center + 0.5 * width
    return [{"y": float(y), "manual": manual_at(float(y), manual),
             "estimate": {"left": float(l), "right": float(r), "center": float(c), "width": float(w)}}
            for y, l, r, c, w in zip(ys, left, right, center, width)]


def model_rows(result: Any, manual: tuple[np.ndarray, np.ndarray], y_range: tuple[float, float]) -> list[dict[str, Any]]:
    rows = []
    for left, right, center, y in zip(result.left_boundary_original_px, result.right_boundary_original_px,
                                      result.centerline_original_px, [p[1] for p in result.centerline_original_px]):
        if not y_range[0] <= y <= y_range[1]:
            continue
        rows.append({"y": float(y), "manual": manual_at(float(y), manual),
                     "estimate": {"left": float(left[0]), "right": float(right[0]),
                                  "center": float(center[0]), "width": float(right[0] - left[0])}})
    return rows


def field_geometry_rows(payload: dict[str, Any], manual: tuple[np.ndarray, np.ndarray], y_range: tuple[float, float]) -> list[dict[str, Any]]:
    rows = []
    for left, right in zip(payload.get("left_boundary", []), payload.get("right_boundary", [])):
        y = float(left["y"])
        if not y_range[0] <= y <= y_range[1]:
            continue
        l, r = float(left["x"]), float(right["x"])
        rows.append({"y": y, "manual": manual_at(y, manual),
                     "estimate": {"left": l, "right": r, "center": 0.5 * (l + r), "width": r - l}})
    return rows


def overlay_matches(raw: np.ndarray, payload: dict[str, Any], overlay: np.ndarray) -> bool:
    canvas = raw.copy()
    left, right = points(payload)
    cv2.polylines(canvas, [np.round(left[:, ::-1]).astype(np.int32).reshape(-1, 1, 2)], False, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.polylines(canvas, [np.round(right[:, ::-1]).astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 0), 2, cv2.LINE_AA)
    # The historical annotation tool also draws four-pixel endpoint circles
    # and the yellow interpolated midpoint guidance line.
    for values, color in ((left, (255, 0, 0)), (right, (0, 255, 0))):
        for y, x in values:
            cv2.circle(canvas, (int(round(x)), int(round(y))), 4, color, -1, cv2.LINE_AA)
    ys = sorted(set(left[:, 0].tolist() + right[:, 0].tolist()))
    lx = np.interp(ys, left[:, 0], left[:, 1]); rx = np.interp(ys, right[:, 0], right[:, 1])
    midpoint = np.asarray([(int(round((l + r) * 0.5)), int(round(y))) for y, l, r in zip(ys, lx, rx) if l < r], dtype=np.int32)
    if len(midpoint) >= 2:
        cv2.polylines(canvas, [midpoint.reshape(-1, 1, 2)], False, (0, 255, 255), 1, cv2.LINE_AA)
    return overlay.shape == canvas.shape and bool(np.array_equal(overlay, canvas))


def gate_truth(result: Any, diagnostics: Any, provenance: dict[str, Any], config: dict[str, Any], manual_rows: dict[str, Any]) -> dict[str, Any]:
    metrics = provenance.get("result", {}).get("geometry_metrics") or {}
    geometry = config["geometry"]
    selected = diagnostics.selected_pairs
    crossing = sum(int(pair.left >= pair.right) for pair in selected)
    inside = True
    if provenance.get("result", {}).get("fit_center_coefficients") is not None:
        center = np.asarray(provenance["result"]["fit_center_coefficients"])
        width = np.asarray(provenance["result"]["fit_width_coefficients"])
        y0 = float(min(pair.y for pair in selected)) if selected else 0.0
        y1 = float(max(pair.y for pair in selected)) if selected else 0.0
        ys = np.linspace(y0, y1, 32)
        sy = diagnostics.processed_gray.shape[0] / result.roi_original_px[3]
        ys_original = result.roi_original_px[1] + ys / sy
        centers = np.polyval(center, ys_original); widths = np.polyval(width, ys_original)
        left = centers - 0.5 * widths; right = centers + 0.5 * widths
        x0, _y0, rw, _rh = result.roi_original_px
        inside = bool(np.isfinite(left + right).all() and (widths > 0).all() and
                      (left >= x0).all() and (right < x0 + rw).all())
    return {"thresholds": {"minimum_point_count": int(geometry["minimum_point_count"]),
                            "minimum_coverage": float(geometry["minimum_coverage"]),
                            "minimum_effective_span": float(geometry["minimum_effective_span"]),
                            "maximum_missing_run": int(geometry["maximum_missing_run"]),
                            "maximum_abrupt_jump_count": int(geometry["maximum_abrupt_jump_count"]),
                            "minimum_inlier_ratio": float(geometry["minimum_inlier_ratio"]),
                            "maximum_fit_residual_px": float(geometry["maximum_fit_residual_px"])},
            "observed": {"minimum_point_count": len(selected), "coverage": float(result.coverage),
                         "effective_span": float(result.effective_span), "missing_run": int(result.maximum_missing_run),
                         "abrupt_left_jumps": int(result.abrupt_left_jump_count), "abrupt_right_jumps": int(result.abrupt_right_jump_count),
                         "path_confidence": metrics.get("path_confidence"), "joint_inlier_ratio": float(result.fit_inlier_ratio),
                         "fit_residual_px": result.fit_residual_px, "crossing": crossing,
                         "boundary_inside_roi": inside, "temporal_rejection": [x for x in result.failure_reasons if x.startswith("temporal_")]},
            "failed_items": list(result.failure_reasons), "manual_accuracy_pass": {
                "center_p95_le_25": manual_rows.get("center", {}).get("absolute", {}).get("p95") is not None and manual_rows["center"]["absolute"]["p95"] <= 25,
                "left_p95_le_30": manual_rows.get("left", {}).get("absolute", {}).get("p95") is not None and manual_rows["left"]["absolute"]["p95"] <= 30,
                "right_p95_le_30": manual_rows.get("right", {}).get("absolute", {}).get("p95") is not None and manual_rows["right"]["absolute"]["p95"] <= 30,
                "width_p95_le_30": manual_rows.get("width", {}).get("absolute", {}).get("p95") is not None and manual_rows["width"]["absolute"]["p95"] <= 30}}


def side_counts(rows: list[dict[str, Any]], layer: str) -> dict[str, Any]:
    left_ok = [abs(r["estimate"]["left"] - r["manual"]["left"]) <= 30 for r in rows]
    right_ok = [abs(r["estimate"]["right"] - r["manual"]["right"]) <= 30 for r in rows]
    return {"layer": layer, "bands": len(rows), "left_correct_right_wrong": int(sum(l and not rr for l, rr in zip(left_ok, right_ok))),
            "right_correct_left_wrong": int(sum(rr and not l for l, rr in zip(left_ok, right_ok))),
            "both_correct": int(sum(l and rr for l, rr in zip(left_ok, right_ok))),
            "both_wrong": int(sum(not l and not rr for l, rr in zip(left_ok, right_ok)))}


def one_side_oracle(rows: list[dict[str, Any]], layer: str) -> dict[str, Any]:
    right_from_left = []
    left_from_right = []
    for row in rows:
        m, e = row["manual"], row["estimate"]
        right_from_left.append((e["left"] + m["width"]) - m["right"])
        left_from_right.append((e["right"] - m["width"]) - m["left"])
    return {"layer": layer, "right_from_left_manual_width": {"signed": stats(right_from_left), "absolute": stats([abs(x) for x in right_from_left])},
            "left_from_right_manual_width": {"signed": stats(left_from_right), "absolute": stats([abs(x) for x in left_from_right])},
            "interpretation": "oracle only; not tracker output"}


def draw_comparison(case_rows: list[dict[str, Any]]) -> None:
    panels = []
    for item in case_rows:
        canvas = item["raw"].copy()
        def line(values: list[tuple[float, float]], color: tuple[int, int, int], thickness: int = 6) -> None:
            if len(values) > 1:
                cv2.polylines(canvas, [np.round(np.asarray(values)).astype(np.int32).reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA)
        left, right = item["manual_points"]
        line(left, (255, 0, 0)); line(right, (0, 255, 0))
        fit = item["fit_rows"]
        line([(r["estimate"]["left"], r["y"]) for r in fit], (0, 0, 255))
        line([(r["estimate"]["right"], r["y"]) for r in fit], (0, 0, 255))
        field = item.get("field_rows", [])
        line([(r["estimate"]["left"], r["y"]) for r in field], (0, 180, 255), 4)
        line([(r["estimate"]["right"], r["y"]) for r in field], (0, 180, 255), 4)
        text = f"{item['capture']} reset={item['result']['tracking_state']} detected={item['result']['detected']} fit diagnostic only"
        cv2.putText(canvas, text, (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(canvas, text, (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        panels.append(cv2.resize(canvas, (1024, 768), interpolation=cv2.INTER_AREA))
    cv2.imwrite(str(OUT / "comparison_montage.jpg"), np.vstack(panels))


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    if config.get("guided_search", {}).get("reference_anchor_response", "signed_sobel") != "signed_sobel":
        raise RuntimeError("P2H requires normalized-step anchor disabled")
    manifest = {"config_sha256": sha256(CONFIG), "reference_sha256": sha256(REFERENCE), "cases": {}, "all_valid": True}
    analyses = []
    for capture, snapshot in CASES.items():
        annotation_path = ANNOTATION_ROOT / capture / "raw_annotation.json"
        overlay_path = ANNOTATION_ROOT / capture / "raw_annotation_overlay.png"
        raw_path = SNAPSHOT_ROOT / snapshot / "raw.bmp"
        metadata_path = SNAPSHOT_ROOT / snapshot / "metadata.json"
        geometry_path = SNAPSHOT_ROOT / snapshot / "seam_geometry.json"
        annotation = annotation_payload(capture)
        raw = cv2.imread(str(raw_path), cv2.IMREAD_COLOR)
        overlay = cv2.imread(str(overlay_path), cv2.IMREAD_COLOR)
        valid_points = True
        left, right = points(annotation)
        if len(left) < 6 or len(right) < 6 or np.any(np.diff(left[:, 0]) < 0) or np.any(np.diff(right[:, 0]) < 0):
            valid_points = False
        valid_points = valid_points and bool(np.all(left[:, 0] >= 0) and np.all(left[:, 0] < 3072) and
                                             np.all(right[:, 0] >= 0) and np.all(right[:, 0] < 3072) and
                                             np.all(left[:, 1] >= 0) and np.all(left[:, 1] < 4096) and
                                             np.all(right[:, 1] >= 0) and np.all(right[:, 1] < 4096) and
                                             np.all(np.interp(sorted(set(left[:, 0].tolist() + right[:, 0].tolist())), left[:, 0], left[:, 1]) <
                                                    np.interp(sorted(set(left[:, 0].tolist() + right[:, 0].tolist())), right[:, 0], right[:, 1])))
        checks = {"annotation_parse": True, "raw_exists": raw is not None, "overlay_exists": overlay is not None,
                  "raw_sha_matches_annotation": raw is not None and sha256(raw_path) == annotation.get("input_sha256") == EXPECTED_RAW[capture],
                  "point_legality": valid_points, "overlay_matches_json": raw is not None and overlay is not None and overlay_matches(raw, annotation, overlay),
                  "raw_shape": raw is not None and tuple(raw.shape) == (3072, 4096, 3),
                  "snapshot_metadata_complete": json.loads(metadata_path.read_text()).get("complete") is True}
        manifest["cases"][capture] = {"annotation": str(annotation_path), "overlay": str(overlay_path), "raw": str(raw_path),
                                       "snapshot": snapshot, "files_sha256": {"annotation": sha256(annotation_path), "overlay": sha256(overlay_path), "raw": sha256(raw_path)},
                                       "point_count": annotation.get("point_count"), "checks": checks, "valid": all(checks.values())}
        manifest["all_valid"] = manifest["all_valid"] and all(checks.values())
        analyses.append({"capture": capture, "raw": raw, "annotation": annotation, "manual": (left, right),
                         "metadata": json.loads(metadata_path.read_text()), "field_geometry": json.loads(geometry_path.read_text())})
    write_json("annotation_manifest.json", manifest)
    if not manifest["all_valid"]:
        return 2

    all_accuracy = {}; all_stages = {}; gate_table = {}; side_table = {}; oracle_table = {}; montage = []
    for item in analyses:
        tracker = StableSeamTracker(config, REFERENCE)
        first = tracker.process(item["raw"], diagnostics=True, provenance=True)
        diagnostics = tracker.last_diagnostics; provenance = tracker.last_provenance or {}
        repeats = [first]
        for _ in range(2):
            repeats.append(tracker.process(item["raw"]))
        y_range = common_y_range(item["manual"], first.roi_original_px)
        raw_rows = pair_rows(first, diagnostics, item["manual"], y_range)
        fit_coefficients = {"center": np.asarray(provenance["result"]["fit_center_coefficients"], dtype=float),
                            "width": np.asarray(provenance["result"]["fit_width_coefficients"], dtype=float)}
        fit_ys = np.asarray([row["y"] for row in raw_rows], dtype=float)
        fit_rows = coefficient_rows(fit_coefficients, fit_ys, item["manual"])
        temporal_rows = model_rows(repeats[-1], item["manual"], y_range)
        final_rows = temporal_rows
        field_rows = field_geometry_rows(item["field_geometry"], item["manual"], y_range)
        stage_data = {"raw_selected_pair": {"accuracy": error_report(raw_rows), "common_y_range": y_range, "count": len(raw_rows)},
                      "irls_fitted": {"accuracy": error_report(fit_rows), "common_y_range": y_range, "count": len(fit_rows)},
                      "temporal": {"accuracy": error_report(temporal_rows), "common_y_range": y_range, "count": len(temporal_rows)},
                      "final_sampled_output": {"accuracy": error_report(final_rows), "common_y_range": y_range, "count": len(final_rows)},
                      "field_recorded_geometry_extra": {"accuracy": error_report(field_rows), "common_y_range": y_range, "count": len(field_rows)}}
        all_accuracy[item["capture"]] = {name: data["accuracy"] for name, data in stage_data.items()}
        all_stages[item["capture"]] = stage_data
        gate_table[item["capture"]] = gate_truth(first, diagnostics, provenance, config, stage_data["irls_fitted"]["accuracy"])
        side_table[item["capture"]] = [side_counts(raw_rows, "raw_selected_pair"), side_counts(fit_rows, "irls_fitted"), side_counts(field_rows, "field_recorded_geometry")]
        oracle_table[item["capture"]] = [one_side_oracle(raw_rows, "raw_selected_pair"), one_side_oracle(fit_rows, "irls_fitted")]
        montage.append({"capture": item["capture"], "raw": item["raw"], "manual_points": [(x, y) for y, x in item["manual"][0]] + [],
                        "result": first.to_dict(), "fit_rows": fit_rows, "field_rows": field_rows})
        item["gate"] = gate_table[item["capture"]]; item["stages"] = stage_data; item["first"] = first; item["fit_rows"] = fit_rows

    write_json("three_frame_accuracy.json", all_accuracy)
    write_json("stagewise_accuracy.json", all_stages)
    write_json("gate_truth_table.json", gate_table)
    write_json("side_failure_classification.json", side_table)
    write_json("one_side_width_oracle.json", oracle_table)

    # Build a compact visual directly from the stored frame records.
    panels = []
    for item in analyses:
        canvas = item["raw"].copy()
        left, right = item["manual"]
        cv2.polylines(canvas, [np.round(np.column_stack((left[:, 1], left[:, 0]))).astype(np.int32).reshape(-1, 1, 2)], False, (255, 0, 0), 6, cv2.LINE_AA)
        cv2.polylines(canvas, [np.round(np.column_stack((right[:, 1], right[:, 0]))).astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 0), 6, cv2.LINE_AA)
        for row in item["fit_rows"]:
            cv2.circle(canvas, (int(round(row["estimate"]["left"])), int(round(row["y"]))), 3, (0, 0, 255), -1)
            cv2.circle(canvas, (int(round(row["estimate"]["right"])), int(round(row["y"]))), 3, (0, 0, 255), -1)
        text = f"{item['capture']} reset={item['first'].tracking_state.name} detected={item['first'].detected} fit=diagnostic"
        cv2.putText(canvas, text, (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(canvas, text, (24, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        panels.append(cv2.resize(canvas, (1024, 768), interpolation=cv2.INTER_AREA))
    cv2.imwrite(str(OUT / "comparison_montage.jpg"), np.vstack(panels))

    choices = {"capture_01": "bad_geometry_correctly_rejected", "capture_03": "one_side_failure", "capture_05": "bad_geometry_correctly_rejected"}
    summary = {"capture_classification": choices, "all_annotation_valid": manifest["all_valid"],
               "normalized_step_disabled": config.get("guided_search", {}).get("reference_anchor_response", "signed_sobel") == "signed_sobel",
               "center_transition_weight": config["path"]["center_transition_weight"],
               "field_detected_only_capture_03": next(x["metadata"].get("detected") for x in analyses if x["capture"] == "capture_03"),
               "decision": "B", "decision_reason": "capture_01 and capture_05 fitted curves are far outside manual boundaries; capture_03 is closer but still fails left/center precision and is not validated by detected=true",
               "production_changes": {"algorithm": False, "gate": False, "reference": False, "configuration": False, "ROS": False}}
    write_json("diagnostic_summary.json", summary)
    write_json("next_single_action.json", {"decision": "B", "implemented": False,
        "unique_next_action": "Do not lower gates; inspect and correct boundary selection/geometry on the three labeled field frames before any gate recalibration",
        "do_not_change": ["tracker behavior", "gate", "reference", "normalized step", "center transition", "ROS"]})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
