#!/usr/bin/env python3
"""S1-P2E5: diagnose the remaining width-error tail after P2E4-B.

This script is deliberately an offline audit.  It uses the production tracker
with the P2E4 option supplied in memory, calls the production ``_fit`` method
for stage separation, and never changes the tracker configuration on disk.
"""

from __future__ import annotations

import copy
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_reference_selection_error import (  # noqa: E402
    THRESHOLDS,
    manual_at,
    manual_points,
    pstats,
    reference_curve,
)
from scripts.experiment_reference_anchor_candidate import (  # noqa: E402
    ANCHOR_RADIUS,
    ANNOTATION,
    CONFIG,
    RAW,
    REFERENCE,
    STABLE_CONFIG,
    TRACKER,
    EXPECTED_RAW,
    config_for,
)
from tank_vision.stable_seam_tracker import (  # noqa: E402
    StableSeamTracker,
    robust_polynomial_fit,
)
from tank_vision.seam_tracking_types import TrackingState  # noqa: E402

OUT = ROOT / "output/s1_p2e5_width_tail_diagnostic"
WIDTH_LIMIT = 30.0
FOCUS_Y = (2400.0, 2700.0)
KEY_Y = (2442.0, 2453.0, 2672.0)


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(x) for x in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        x = float(value)
        return x if math.isfinite(x) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, TrackingState):
        return value.name
    return value


def write_json(name: str, value: Any) -> None:
    (OUT / name).write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pair_dict(pair: Any, result: Any, diagnostics: Any) -> dict[str, Any]:
    x0, _y0, rw, _rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    return {
        "pair_id": f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}",
        "band_index": int(pair.band_index),
        "left": float(x0 + pair.left / sx),
        "right": float(x0 + pair.right / sx),
        "center": float(x0 + pair.center / sx),
        "width": float(pair.width / sx),
        "unary_score": float(pair.score),
        "score_parts": {k: float(v) for k, v in pair.score_parts.items()},
        "left_processed": float(pair.left),
        "right_processed": float(pair.right),
        "width_processed": float(pair.width),
    }


def curve_from_model(center: np.ndarray, width: np.ndarray, ys: np.ndarray) -> list[dict[str, float]]:
    c = np.polyval(center, ys)
    w = np.polyval(width, ys)
    return [{"left": float(x - z / 2.0), "right": float(x + z / 2.0),
             "center": float(x), "width": float(z)} for x, z in zip(c, w)]


def error_stats(rows: list[dict[str, Any]], field: str) -> dict[str, Any]:
    values = [float(row[field]) for row in rows if row.get(field) is not None]
    return {"signed": pstats(values), "absolute": pstats([abs(x) for x in values])}


def stage_stats(curves: list[dict[str, Any]], manuals: list[dict[str, float]]) -> dict[str, Any]:
    rows = []
    for curve, manual in zip(curves, manuals):
        rows.append({field: curve[field] - manual[field] for field in THRESHOLDS})
    return {field: error_stats(rows, field) for field in THRESHOLDS} | {"count": len(rows)}


def pair_score_rank(pairs: list[dict[str, Any]], pair_id: str | None) -> int | None:
    if pair_id is None:
        return None
    return next((i + 1 for i, item in enumerate(pairs) if item["pair_id"] == pair_id), None)


def find_pair(pairs: list[dict[str, Any]], predicate: Any) -> dict[str, Any] | None:
    candidates = [item for item in pairs if predicate(item)]
    return candidates[0] if candidates else None


def anchor_pair_for_band(anchor: dict[str, Any] | None, pairs: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not anchor or not anchor.get("enabled"):
        return None
    left = anchor.get("left", {})
    right = anchor.get("right", {})
    if not left.get("valid") or not right.get("valid"):
        return None
    left_candidates = anchor.get("final_left_candidates_processed", [])
    right_candidates = anchor.get("final_right_candidates_processed", [])
    if not left_candidates or not right_candidates:
        return None
    # A merged anchor may have been replaced by the stronger existing sample;
    # the nearest final candidate is the actual candidate retained by the
    # production merge, not an oracle coordinate.
    l = min(left_candidates, key=lambda x: abs(int(x) - int(left["position_processed"])))
    r = min(right_candidates, key=lambda x: abs(int(x) - int(right["position_processed"])))
    return find_pair(pairs, lambda p: int(round(p["left_processed"])) == int(l)
                     and int(round(p["right_processed"])) == int(r))


def nearest_pair(pairs: list[dict[str, Any]], target: dict[str, float]) -> dict[str, Any] | None:
    if not pairs:
        return None
    return min(pairs, key=lambda p: abs(p["left"] - target["left"]) + abs(p["right"] - target["right"]))


def curve_errors(curves: list[dict[str, float]], manuals: list[dict[str, float]]) -> list[dict[str, float]]:
    return [{field: curve[field] - manual[field] for field in THRESHOLDS}
            for curve, manual in zip(curves, manuals)]


def fit_width_audit(ys: np.ndarray, widths: np.ndarray, config: dict[str, Any]) -> dict[str, Any]:
    g = config["geometry"]
    delta = float(g["width_fit_huber_delta_scale"]) * float(g["robust_fit_huber_delta_px"])
    fit = robust_polynomial_fit(ys, widths, int(g["width_fit_degree"]),
                                int(g["robust_fit_iterations"]), delta)
    if fit is None:
        return {"status": "fit_failed", "huber_delta_px": delta}
    coeff, inliers, rms = fit
    residuals = widths - np.polyval(coeff, ys)
    weights = np.where(np.abs(residuals) <= delta, 1.0,
                       delta / np.maximum(np.abs(residuals), 1e-12))
    return {"status": "ok", "coefficients": coeff, "inliers": inliers,
            "huber_delta_px": delta, "rms_px": rms, "residuals_px": residuals,
            "final_huber_weights": weights}


def run_b(image: np.ndarray, points: tuple[np.ndarray, np.ndarray]) -> dict[str, Any]:
    config = config_for(True)
    tracker = StableSeamTracker(config, REFERENCE)
    results = []
    first_diag = None
    first_provenance = None
    first_result = None
    for index in range(3):
        result = tracker.process(image, diagnostics=index == 0, provenance=index == 0)
        results.append(result)
        if index == 0:
            first_result = result
            first_diag = tracker.last_diagnostics
            first_provenance = copy.deepcopy(tracker.last_provenance)
    assert first_result is not None and first_diag is not None and first_provenance is not None
    roi = first_result.roi_original_px
    sx = first_diag.processed_gray.shape[1] / roi[2]
    sy = first_diag.processed_gray.shape[0] / roi[3]
    ys = roi[1] + first_diag.scan_ys.astype(np.float64) / sy
    manuals = [manual_at(float(y), points) for y in ys]
    references = [reference_curve(tracker.reference, float(y), (image.shape[1], image.shape[0])) for y in ys]
    selected_by_band = {p.band_index: p for p in first_diag.selected_pairs}
    rows = []
    for band, y in enumerate(ys):
        pairs = [pair_dict(p, first_result, first_diag) for p in first_diag.all_pairs[band]]
        selected = pair_dict(selected_by_band[band], first_result, first_diag) if band in selected_by_band else None
        anchor_trace = first_provenance["candidate_stage_trace"][band].get("reference_anchor")
        anchor_pair = anchor_pair_for_band(anchor_trace, pairs)
        manual_near = nearest_pair(pairs, manuals[band])
        reference_near = nearest_pair(pairs, references[band])
        rows.append({"band_index": band, "y_original": float(y), "manual": manuals[band],
                     "reference": references[band], "selected": selected,
                     "anchor": anchor_trace, "anchor_pair": anchor_pair,
                     "manual_nearest": manual_near, "reference_nearest": reference_near,
                     "pairs": pairs, "pair_count": len(pairs),
                     "selected_pair_rank": pair_score_rank(pairs, selected and selected["pair_id"]),
                     "anchor_pair_rank": pair_score_rank(pairs, anchor_pair and anchor_pair["pair_id"]),
                     "manual_nearest_rank": pair_score_rank(pairs, manual_near and manual_near["pair_id"]),
                     "reference_nearest_rank": pair_score_rank(pairs, reference_near and reference_near["pair_id"])})

    selected_path = [selected_by_band[b] for b in sorted(selected_by_band)]
    fit = tracker._fit(selected_path, roi, sx, sy, image.shape[1], image.shape[0])
    assert fit is not None
    fit_curves = curve_from_model(fit["center"], fit["width"], ys)
    temporal_curves = curve_from_model(tracker._center_coefficients, tracker._width_coefficients, ys)
    final_result = results[-1]
    ros_curves = []
    for left, right, center, width in zip(final_result.left_boundary_original_px,
                                          final_result.right_boundary_original_px,
                                          final_result.centerline_original_px,
                                          final_result.width_profile_px):
        ros_curves.append({"left": float(left[0]), "right": float(right[0]),
                           "center": float(center[0]), "width": float(width),
                           "y": float(center[1])})
    ros_manuals = [manual_at(item["y"], points) for item in ros_curves]
    raw_curves = [row["selected"] for row in rows]
    raw_curves = [{k: float(item[k]) for k in THRESHOLDS} for item in raw_curves]
    stagewise = {"raw_selected_pair": stage_stats(raw_curves, manuals),
                 "irls_fitted_model": stage_stats(fit_curves, manuals),
                 "temporal_model_after_three_repeats": stage_stats(temporal_curves, manuals),
                 "final_ros_message_sampling": stage_stats([{k: x[k] for k in THRESHOLDS} for x in ros_curves], ros_manuals),
                 "ros_sample_count": len(ros_curves),
                 "p2e4_width_p95_layer": "raw_selected_pair"}
    dp_states = {item["pair_id"]: item for item in first_provenance.get("dp_state_trace", [])}
    final_path_ids = set(first_provenance.get("dp_final_path", []))
    for row in rows:
        for item in row["pairs"] + [row[label] for label in ("selected", "anchor_pair", "manual_nearest", "reference_nearest") if row[label] is not None]:
            if item is not None:
                state = dp_states.get(item["pair_id"], {})
                item["dp_entered"] = True
                item["dp_on_final_path"] = item["pair_id"] in final_path_ids
                item["dp_cumulative_score"] = state.get("cumulative_score")
                item["dp_best_predecessor"] = state.get("predecessor_pair_id")
    return {"config": config, "tracker": tracker, "results": results, "first_result": first_result,
            "first_diag": first_diag, "provenance": first_provenance, "fit": fit, "rows": rows,
            "ys": ys, "manuals": manuals, "references": references, "fit_curves": fit_curves,
            "temporal_curves": temporal_curves, "ros_curves": ros_curves, "ros_manuals": ros_manuals,
            "stagewise": stagewise, "dp_states": dp_states, "final_path_ids": final_path_ids,
            "sx": sx, "sy": sy, "image_shape": (image.shape[1], image.shape[0])}


def width_tail(rows: list[dict[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    entries = []
    for row in rows:
        selected = row["selected"]
        if selected is None:
            continue
        error = float(selected["width"] - row["manual"]["width"])
        entries.append({"band_index": row["band_index"], "y_original": row["y_original"],
                        "width_error": error, "abs_width_error": abs(error)})
    ordered = sorted(entries, key=lambda x: x["abs_width_error"], reverse=True)
    return {f"over_{int(t)}_px": {"count": sum(x["abs_width_error"] > t for x in entries),
                                  "bands": [x["band_index"] for x in entries if x["abs_width_error"] > t]}
            for t in (20, 25, 30, 40, 50)}, ordered


def enrich_worst(rows: list[dict[str, Any]], fit_curves: list[dict[str, float]], temporal_curves: list[dict[str, float]],
                 fit: dict[str, Any]) -> list[dict[str, Any]]:
    ordered = sorted([r for r in rows if r["selected"] is not None],
                     key=lambda r: abs(r["selected"]["width"] - r["manual"]["width"]), reverse=True)
    output = []
    for row in ordered[:12]:
        b = row["band_index"]
        selected = row["selected"]
        fitted = fit_curves[b]
        item = {"band_index": b, "y_original": row["y_original"], "manual": row["manual"],
                "reference": row["reference"], "anchor_pair": row["anchor_pair"], "selected": selected,
                "fitted": fitted,
                "errors": {"left": selected["left"] - row["manual"]["left"],
                           "right": selected["right"] - row["manual"]["right"],
                           "center": selected["center"] - row["manual"]["center"],
                           "width": selected["width"] - row["manual"]["width"],
                           "fitted_width": fitted["width"] - row["manual"]["width"]},
                "joint_inlier": bool(fit["inliers"][b]),
                "fit_downweighted": bool(abs(float(fit["widths"][b] - np.polyval(fit["width"], fit["ys"][b]))) > 40.0),
                "in_y_2400_2700": bool(FOCUS_Y[0] <= row["y_original"] <= FOCUS_Y[1]),
                "temporal_width": temporal_curves[b]["width"]}
        output.append(item)
    return output


def classify_failures(data: dict[str, Any]) -> dict[str, Any]:
    rows = data["rows"]
    fit_curves = data["fit_curves"]
    temporal = data["temporal_curves"]
    counts: dict[str, int] = {}
    classified = []
    for row, fitted, temp in zip(rows, fit_curves, temporal):
        selected = row["selected"]
        if selected is None:
            continue
        raw_error = selected["width"] - row["manual"]["width"]
        fit_error = fitted["width"] - row["manual"]["width"]
        temp_error = temp["width"] - row["manual"]["width"]
        category = None
        reason = ""
        if abs(raw_error) <= WIDTH_LIMIT and abs(fit_error) > WIDTH_LIMIT:
            category, reason = "selected_pair_correct_but_fit_shifted", "raw width passes, fitted width fails"
        elif abs(fit_error) <= WIDTH_LIMIT and abs(temp_error) > WIDTH_LIMIT:
            category, reason = "fit_correct_but_temporal_shifted", "fit width passes, temporal width fails"
        elif abs(raw_error) > WIDTH_LIMIT:
            anchor = row["anchor"] or {}
            ap = row["anchor_pair"]
            if not anchor.get("left", {}).get("valid") or not anchor.get("right", {}).get("valid"):
                category, reason = "anchor_candidate_missing", "one or both signed anchor responses did not pass validity"
            elif ap is None:
                category, reason = "anchor_pair_not_formed", "both anchor candidates were retained but no legal final pair used both"
            elif ap["pair_id"] != selected["pair_id"] and ap["unary_score"] < selected["unary_score"]:
                category, reason = "anchor_pair_lower_unary", "anchor pair entered DP but had lower unary score"
            elif ap["pair_id"] != selected["pair_id"]:
                category, reason = "anchor_pair_lost_by_dp", "anchor pair entered DP and was not on the production path"
            else:
                category, reason = "other", "selected anchor pair itself remains more than 30 px from manual width"
        if category is not None:
            counts[category] = counts.get(category, 0) + 1
            classified.append({"band_index": row["band_index"], "y_original": row["y_original"],
                               "category": category, "reason": reason,
                               "selected": selected, "anchor_pair": row["anchor_pair"],
                               "selected_width_error": raw_error, "fitted_width_error": fit_error,
                               "temporal_width_error": temp_error,
                               "selected_rank": row["selected_pair_rank"], "anchor_rank": row["anchor_pair_rank"],
                               "anchor_pair_unary": None if row["anchor_pair"] is None else row["anchor_pair"]["unary_score"],
                               "selected_unary": selected["unary_score"]})
    # Separate the stagewise audit so the mutually exclusive tail categories
    # above remain about the requested >30 px raw tail.
    return {"threshold_px": WIDTH_LIMIT, "counts": counts, "rows": classified,
            "stagewise_events": [{"band_index": row["band_index"], "y_original": row["y_original"],
                                  "raw_width_error": row["selected"]["width"] - row["manual"]["width"],
                                  "fit_width_error": f["width"] - row["manual"]["width"],
                                  "temporal_width_error": t["width"] - row["manual"]["width"]}
                                 for row, f, t in zip(rows, fit_curves, temporal)
                                 if abs(row["selected"]["width"] - row["manual"]["width"]) <= WIDTH_LIMIT
                                 and (abs(f["width"] - row["manual"]["width"]) > WIDTH_LIMIT
                                      or abs(t["width"] - row["manual"]["width"]) > WIDTH_LIMIT)]}


def forced_oracle(data: dict[str, Any], name: str) -> dict[str, Any]:
    rows = data["rows"]
    key = {"anchor_anchor": "anchor_pair", "manual_nearest": "manual_nearest",
           "reference_nearest": "reference_nearest", "selected": "selected"}[name]
    chosen = [row[key] for row in rows]
    present = sum(item is not None for item in chosen)
    raw_curves = [{k: item[k] for k in THRESHOLDS} for item in chosen if item is not None]
    manual_values = [row["manual"] for row, item in zip(rows, chosen) if item is not None]
    return {"definition": "diagnostic-only pair replacement using actual returned pairs; no detected/TRACKING result",
            "pair_present_count": present, "scan_band_count": len(rows),
            "raw_errors": stage_stats(raw_curves, manual_values)}


def forced_fit_oracle(data: dict[str, Any], name: str) -> dict[str, Any]:
    key = {"anchor_anchor": "anchor_pair", "manual_nearest": "manual_nearest", "reference_nearest": "reference_nearest"}[name]
    pairs = [row[key] for row in data["rows"]]
    if not all(pairs):
        return {"fit_status": "incomplete_pair_path", "pair_present_count": sum(x is not None for x in pairs)}
    # Convert actual returned pair dictionaries back into the production type
    # only for the read-only production _fit call.
    diag = data["first_diag"]
    by_id = {p_id: p for band in diag.all_pairs for p in band for p_id in [f"b{p.band_index}:l{int(round(p.left))}:r{int(round(p.right))}"]}
    path = [by_id[p["pair_id"]] for p in pairs]
    tracker = data["tracker"]
    fit = tracker._fit(path, data["first_result"].roi_original_px, data["sx"], data["sy"],
                       data["image_shape"][0], data["image_shape"][1])
    if fit is None:
        return {"fit_status": "fit_failed", "pair_present_count": len(path)}
    curves = curve_from_model(fit["center"], fit["width"], data["ys"])
    return {"fit_status": "ok", "pair_present_count": len(path),
            "errors": stage_stats(curves, data["manuals"]),
            "coefficients": {"center": fit["center"], "width": fit["width"]}}


def build_key_trace(data: dict[str, Any]) -> list[dict[str, Any]]:
    out = []
    for target in KEY_Y:
        row = min(data["rows"], key=lambda x: abs(x["y_original"] - target))
        diag = data["first_diag"]
        band = row["band_index"]
        response = diag.edge_response[band]
        threshold = max(float(data["config"]["processing"]["edge_min_response"]),
                        float(np.percentile(np.abs(response), float(data["config"]["processing"]["edge_percentile"]))))
        def response_at(x: float) -> float:
            processed_x = int(round((x - row["manual"]["left"] + 0)))
            # Use the production ROI transform, not the manual point, below.
            return float(response[int(np.clip(round((x - data["first_result"].roi_original_px[0]) * data["sx"]), 0, len(response) - 1))])
        anchor = row["anchor"] or {}
        out.append({"target_y": target, "band_index": band, "y_original": row["y_original"],
                    "manual": row["manual"], "reference": row["reference"], "selected": row["selected"],
                    "anchor": anchor, "anchor_pair": row["anchor_pair"],
                    "manual_nearest": row["manual_nearest"], "reference_nearest": row["reference_nearest"],
                    "pair_rankings": [{"pair_id": p["pair_id"], "unary_rank": i + 1,
                                       "unary_score": p["unary_score"], "width": p["width"],
                                       "dp_on_final_path": p.get("dp_on_final_path", False),
                                       "dp_cumulative_score": p.get("dp_cumulative_score")}
                                      for i, p in enumerate(row["pairs"])],
                    "manual_left_signed_sobel": response_at(row["manual"]["left"]),
                    "manual_right_signed_sobel": response_at(row["manual"]["right"]),
                    "reference_left_signed_sobel": response_at(row["reference"]["left"]),
                    "reference_right_signed_sobel": response_at(row["reference"]["right"]),
                    "selected_left_signed_sobel": response_at(row["selected"]["left"]),
                    "selected_right_signed_sobel": response_at(row["selected"]["right"]),
                    "edge_min_response": float(data["config"]["processing"]["edge_min_response"]),
                    "profile_threshold": threshold,
                    "all_candidates_processed": {"left": [int(x) for x in diag.left_candidates[band]],
                                                  "right": [int(x) for x in diag.right_candidates[band]]},
                    "dp_note": "all final returned pairs are DP input; selected membership is read from production provenance"})
    return out


def draw_width_plot(data: dict[str, Any]) -> None:
    w, h = 1400, 700
    canvas = np.full((h, w, 3), 250, np.uint8)
    ys = data["ys"]
    manual = np.asarray([x["width"] for x in data["manuals"]])
    reference = np.asarray([x["width"] for x in data["references"]])
    selected = np.asarray([x["width"] for x in [r["selected"] for r in data["rows"]]])
    fitted = np.asarray([x["width"] for x in data["fit_curves"]])
    xmin, xmax = float(ys.min()), float(ys.max())
    ymin = float(min(manual.min(), reference.min(), selected.min(), fitted.min()) - 30)
    ymax = float(max(manual.max(), reference.max(), selected.max(), fitted.max()) + 30)
    def pt(y: float, x: float) -> tuple[int, int]:
        return int(70 + (y - xmin) * (w - 110) / max(xmax - xmin, 1)), int(50 + (ymax - x) * (h - 100) / max(ymax - ymin, 1))
    for value, color, name in ((manual, (0, 0, 0), "manual"), (reference, (0, 140, 255), "reference"),
                               (selected, (0, 0, 220), "selected raw"), (fitted, (150, 0, 150), "fitted")):
        points = np.asarray([pt(float(y), float(x)) for y, x in zip(ys, value)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [points], False, color, 3, cv2.LINE_AA)
        cv2.putText(canvas, name, (850, 55 + 28 * (("manual", "reference", "selected raw", "fitted").index(name))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
    for delta, color, label in ((30, (0, 0, 220), "+30"), (-30, (0, 0, 220), "-30")):
        y = pt(xmin, float(np.median(manual) + delta))[1]
        cv2.line(canvas, (70, y), (w - 40, y), color, 1, cv2.LINE_AA)
        cv2.putText(canvas, label, (75, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
    cv2.putText(canvas, "P2E5 width: raw selected vs production fit", (70, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (20, 20, 20), 2)
    cv2.imwrite(str(OUT / "width_tail_plot.png"), canvas)


def draw_key_profiles(data: dict[str, Any]) -> None:
    panels = []
    diag = data["first_diag"]
    gray = diag.processed_gray
    for key in build_key_trace(data):
        band = key["band_index"]
        profile = diag.edge_response[band]
        half = int(data["config"]["processing"]["scan_band_half_height_px"])
        smoothed_gray = cv2.blur(diag.processed_gray.astype(np.float32), (1, 2 * half + 1))[int(diag.scan_ys[band])]
        center = int(round((key["manual"]["center"] - data["first_result"].roi_original_px[0]) * data["sx"]))
        x0, x1 = max(0, center - 150), min(len(profile) - 1, center + 150)
        canvas = np.full((430, 760, 3), 248, np.uint8)
        scale = max(float(np.max(np.abs(profile[x0:x1 + 1]))), 1.0)
        def xp(x: float) -> int: return int(25 + (x - x0) * 710 / max(x1 - x0, 1))
        def yp(v: float) -> int: return int(210 - v * 150 / scale)
        pts = np.asarray([(xp(x), yp(float(profile[x]))) for x in range(x0, x1 + 1)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], False, (160, 0, 160), 2, cv2.LINE_AA)
        gray_values = smoothed_gray[x0:x1 + 1]
        gray_min, gray_max = float(np.min(gray_values)), float(np.max(gray_values))
        gray_pts = np.asarray([(xp(x), int(300 - (float(smoothed_gray[x]) - gray_min) * 90 /
                                             max(gray_max - gray_min, 1.0)))
                               for x in range(x0, x1 + 1)], np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [gray_pts], False, (0, 150, 0), 2, cv2.LINE_AA)
        cv2.line(canvas, (xp(x0), yp(key["profile_threshold"])),
                 (xp(x1), yp(key["profile_threshold"])), (80, 80, 80), 1)
        positions = [("M-L", key["manual"]["left"], (0, 0, 0)), ("M-R", key["manual"]["right"], (0, 0, 0)),
                     ("R-L", key["reference"]["left"], (0, 140, 255)), ("R-R", key["reference"]["right"], (0, 140, 255)),
                     ("S-L", key["selected"]["left"], (0, 0, 220)), ("S-R", key["selected"]["right"], (0, 0, 220))]
        for label, original_x, color in positions:
            px = (original_x - data["first_result"].roi_original_px[0]) * data["sx"]
            if x0 <= px <= x1:
                cv2.line(canvas, (xp(px), 40), (xp(px), 350), color, 2)
                cv2.putText(canvas, label, (xp(px) + 2, 30 + 18 * (label.endswith("R"))), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1)
        for pair in data["rows"][band]["pairs"]:
            for x in (pair["left_processed"], pair["right_processed"]):
                if x0 <= x <= x1:
                    cv2.circle(canvas, (xp(x), yp(float(profile[int(x)]))), 4, (70, 70, 70), -1)
        cv2.putText(canvas, "green=smoothed gray; purple=signed Sobel; gray=threshold", (20, 365), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"band {band}, y={key['y_original']:.1f}; black=manual, orange=reference, red=selected", (20, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1)
        panels.append(canvas)
    cv2.imwrite(str(OUT / "key_band_profile_montage.png"), np.hstack(panels))


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    fixed = {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "stable_config": STABLE_CONFIG,
             "reference": REFERENCE, "tracker": TRACKER}
    before = {name: sha256(path) for name, path in fixed.items()}
    if before["raw"] != EXPECTED_RAW:
        raise RuntimeError(f"raw SHA mismatch: {before['raw']}")
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read fixed raw image")
    data = run_b(image, manual_points())
    b = data["results"]
    p95 = data["stagewise"]["raw_selected_pair"]
    baseline_checks = {
        "path_117": b[0].accepted_scan_bands == 117,
        "coverage_1": abs(b[0].coverage - 1.0) < 1e-12,
        "joint_inlier": abs(b[0].fit_inlier_ratio - 0.9401709401709402) < 1e-9,
        "states": [x.tracking_state.name for x in b] == ["ACQUIRING", "ACQUIRING", "TRACKING"],
        "center_p95": abs(p95["center"]["absolute"]["p95"] - 23.333557018983623) < 1e-6,
        "left_p95": abs(p95["left"]["absolute"]["p95"] - 29.219146219938185) < 1e-6,
        "right_p95": abs(p95["right"]["absolute"]["p95"] - 22.74976211194553) < 1e-6,
        "width_p95": abs(p95["width"]["absolute"]["p95"] - 32.532251387164905) < 1e-6,
        "crossing_zero": b[0].abrupt_left_jump_count == 0 and b[0].abrupt_right_jump_count == 0,
    }
    baseline = {"status": "matched" if all(baseline_checks.values()) else "baseline_mismatch",
                "checks": baseline_checks, "actual": {"path": b[0].accepted_scan_bands,
                "coverage": b[0].coverage, "joint_inlier": b[0].fit_inlier_ratio,
                "states": [x.tracking_state.name for x in b], "processing_ms": [x.processing_ms for x in b],
                "p95": p95}}
    write_json("baseline_reproduction.json", baseline)
    if baseline["status"] != "matched":
        return 2
    write_json("stagewise_accuracy.json", data["stagewise"])
    tail, ordered = width_tail(data["rows"])
    write_json("width_tail_distribution.json", {"p95_sample": {"n": 117, "numpy_zero_based_index": 110.2,
        "one_based_interpolation": "between sorted samples 111 and 112"}, "thresholds": tail,
        "ordered_width_error": ordered})
    write_json("worst_width_bands.json", {"count": 12, "bands": enrich_worst(data["rows"], data["fit_curves"], data["temporal_curves"], data["fit"])})
    classification = classify_failures(data)
    write_json("width_failure_classification.json", classification)
    write_json("key_band_trace.json", {"bands": build_key_trace(data), "focus": "2442/2453/2672 nearest production scan bands"})
    fit_width = fit_width_audit(data["fit"]["ys"], data["fit"]["widths"], data["config"])
    fit_audit = {"production_fit": {"center_coefficients": data["fit"]["center"], "width_coefficients": data["fit"]["width"],
                                     "joint_inliers": data["fit"]["inliers"], "residual_px": data["fit"]["residual_px"]},
                 "width_irls_reconstruction": fit_width,
                 "fit_vs_raw_abs_width": {"raw": data["stagewise"]["raw_selected_pair"]["width"],
                                            "fitted": data["stagewise"]["irls_fitted_model"]["width"]},
                 "fit_improved_band_count": sum(abs(f["width"] - m["width"]) < abs(r["selected"]["width"] - m["width"])
                                                 for f, r, m in zip(data["fit_curves"], data["rows"], data["manuals"])),
                 "fit_worsened_band_count": sum(abs(f["width"] - m["width"]) > abs(r["selected"]["width"] - m["width"])
                                                 for f, r, m in zip(data["fit_curves"], data["rows"], data["manuals"])),
                 "fit_downweighted_band_count": int(np.count_nonzero(fit_width.get("final_huber_weights", np.ones(117)) < 1.0))}
    write_json("fit_residual_audit.json", fit_audit)
    oracles = {name: forced_oracle(data, name) for name in ("anchor_anchor", "manual_nearest", "reference_nearest", "selected")}
    # Production _fit is separately evaluated for the complete actual paths;
    # raw oracle metrics above remain the primary diagnostic because missing
    # anchor pairs cannot be silently filled.
    fitted_oracles = {name: forced_fit_oracle(data, name)
                      for name in ("anchor_anchor", "manual_nearest", "reference_nearest")}
    write_json("oracle_counterfactual.json", {"raw": oracles, "forced_pair_production_fit": fitted_oracles,
                                               "selected_same_production_fit": data["stagewise"]["irls_fitted_model"]})
    draw_width_plot(data)
    draw_key_profiles(data)
    after = {name: sha256(path) for name, path in fixed.items()}
    # Sensitivity replaces only selected width errors in the k worst bands by
    # the manual width; this is explicitly not a tracker result.
    raw_abs = np.asarray([abs(r["selected"]["width"] - r["manual"]["width"]) for r in data["rows"]])
    order = np.argsort(raw_abs)[::-1]
    sensitivity = {str(k): float(np.percentile(np.delete(raw_abs, order[:k]), 95)) if k else float(np.percentile(raw_abs, 95))
                   for k in range(1, 6)}
    focus = [r for r in data["rows"] if FOCUS_Y[0] <= r["y_original"] <= FOCUS_Y[1]]
    summary = {"task": "S1-P2E5", "baseline_status": baseline["status"],
               "p2e4_width_p95_layer": "raw_selected_pair", "fixed_b_scheme": {"anchor_enabled": True, "anchor_radius_width_scale": ANCHOR_RADIUS, "center_transition_weight": 0.42},
               "width_over_30_count": tail["over_30_px"]["count"], "worst_band_count": 12,
               "focus_y_2400_2700": {"count": len(focus), "width_abs_p95": pstats([abs(r["selected"]["width"] - r["manual"]["width"]) for r in focus])["p95"]},
               "p95_sensitivity_correct_worst_k": sensitivity,
               "joint_inlier_high_width_tail_explanation": "joint inlier is a combined center/half-width fit residual gate; it does not impose a 30 px per-band width absolute error limit, and a small number of pair/fit tail bands can remain.",
               "classification": classification["counts"],
               "anchor_anchor_raw_width_p95": oracles["anchor_anchor"]["raw_errors"]["width"]["absolute"]["p95"],
               "fixed_input_sha256_before": before, "fixed_input_sha256_after": after,
               "fixed_input_hashes_unchanged": before == after,
               "choice": "D" if oracles["anchor_anchor"]["raw_errors"]["width"]["absolute"]["p95"] > WIDTH_LIMIT else ("A" if classification["counts"].get("anchor_pair_lost_by_dp", 0) else "D"),
               "production_changes": {"algorithm": False, "parameters": False, "reference": False, "gate": False, "ros": False}}
    # Ensure the choice is deterministic and explainable from the observed
    # tail: if anchor pairs exist but are not selected, the only next object is
    # pair selection; otherwise candidate response is the remaining object.
    if summary["choice"] == "A":
        next_issue = "one pair-selection item (unary or DP) for existing anchor pairs; do not change anchor radius"
    else:
        next_issue = "anchor response/candidate quality at the remaining tail bands; do not change radius or DP"
    write_json("diagnostic_summary.json", summary)
    write_json("next_single_change.json", {"choice": summary["choice"], "implemented": False,
        "unique_next_modification_object": next_issue,
        "do_not_change": ["anchor radius", "candidate set policy beyond the one selected object", "unary other than one selected item", "DP other than one selected item", "fit", "temporal", "gate", "YAML", "reference", "ROS"]})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
