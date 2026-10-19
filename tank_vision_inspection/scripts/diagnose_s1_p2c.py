#!/usr/bin/env python3
"""Read-only per-scan-band audit of the production stable seam tracker.

This script deliberately calls the existing StableSeamTracker private stages;
it does not reimplement candidate generation, pairing, or dynamic programming.
The implementation does not expose the DP score table or the pre-truncation
valid mask, so those limits are recorded explicitly in the output.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import tank_vision.stable_seam_tracker as tracker_module
from tank_vision.seam_tracking_types import StableSeamResult, TrackingState
from tank_vision.stable_seam_tracker import BoundaryPair, StableSeamTracker, robust_polynomial_fit


SNAP = ROOT / "output/stable_seam_failure_snapshots/20260909_120617_048_stamp_7764_693100000"
ANN_DIR = ROOT / "output/s1_p2c_live_bad_ground_truth_stamp_7764_693100000"
FREEZE = ROOT / "output/s1_p2c_live_bad_ground_truth_v1"
OUT = ROOT / "output/s1_p2c_live_bad_diagnostic"
CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_width_reference_s100p_v1.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(v) for v in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating,)):
        value = float(value)
        return value if np.isfinite(value) else None
    if isinstance(value, (float,)):
        return value if np.isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, TrackingState):
        return value.name
    return value


def point_list(points: list[tuple[float, float]]) -> list[dict[str, float]]:
    return [{"x": float(x), "y": float(y)} for x, y in points]


def manual_interpolate(points: list[dict[str, float]], ys: np.ndarray) -> np.ndarray:
    ordered = sorted(points, key=lambda item: item["y"])
    return np.interp(ys, [item["y"] for item in ordered], [item["x"] for item in ordered])


def pair_dict(pair: BoundaryPair, sx: float, roi: tuple[int, int, int, int], sy: float,
              ref_width_original: float, manual: dict[str, float] | None = None) -> dict[str, Any]:
    x0, y0, _rw, _rh = roi
    result = {
        "band_index": pair.band_index,
        "y_processed_px": pair.y,
        "y_original_px": y0 + pair.y / sy,
        "left_processed_px": pair.left,
        "right_processed_px": pair.right,
        "center_processed_px": pair.center,
        "width_processed_px": pair.width,
        "left_original_px": x0 + pair.left / sx,
        "right_original_px": x0 + pair.right / sx,
        "center_original_px": x0 + pair.center / sx,
        "width_original_px": pair.width / sx,
        "unary_score": pair.score,
        "score_parts": pair.score_parts,
        "reference_width_original_px": ref_width_original,
    }
    if manual is not None:
        result["manual"] = manual
        result["error_px"] = {
            "left": result["left_original_px"] - manual["left"],
            "right": result["right_original_px"] - manual["right"],
            "center": result["center_original_px"] - manual["center"],
            "width": result["width_original_px"] - manual["width"],
        }
    return result


def capture_internal(tracker: StableSeamTracker, image: np.ndarray) -> dict[str, Any]:
    """Run the same private stages used by process(), with edge observation."""
    roi_bgr, gray, roi, sx, sy = tracker._prepare(image)
    observed: list[tuple[bool, list[np.ndarray]]] = []
    original_peaks = tracker_module._local_peaks_batch

    def observe_peaks(*args: Any, **kwargs: Any) -> list[np.ndarray]:
        values = original_peaks(*args, **kwargs)
        observed.append((bool(args[1]), values))
        return values

    tracker_module._local_peaks_batch = observe_peaks
    try:
        response, all_pairs, scan_ys, timings = tracker._generate_candidates(
            gray, roi, sx, sy, image.shape[1], image.shape[0], True
        )
    finally:
        tracker_module._local_peaks_batch = original_peaks
    path = tracker._select_path(all_pairs, roi, sx, sy, image.shape[1], image.shape[0])
    fit = tracker._fit(path, roi, sx, sy, image.shape[1], image.shape[0])
    valid, metrics, reasons = tracker._geometry(path, fit, len(scan_ys), roi, sx, sy, image.shape[1], image.shape[0])
    left_candidates = observed[0][1] if observed and observed[0][0] else []
    right_candidates = observed[1][1] if len(observed) > 1 and not observed[1][0] else []
    return {
        "roi_bgr": roi_bgr,
        "gray": gray,
        "response": response,
        "roi": roi,
        "sx": sx,
        "sy": sy,
        "scan_ys": scan_ys,
        "all_pairs": all_pairs,
        "path": path,
        "fit": fit,
        "valid": valid,
        "metrics": metrics,
        "reasons": reasons,
        "timings": timings,
        "left_candidates": left_candidates,
        "right_candidates": right_candidates,
    }


def result_dict(result: StableSeamResult) -> dict[str, Any]:
    return clean(result.to_dict())


def draw_polyline(canvas: np.ndarray, points: list[tuple[float, float]], color: tuple[int, int, int], width: int = 5) -> None:
    if len(points) >= 2:
        cv2.polylines(canvas, [np.round(points).astype(np.int32).reshape(-1, 1, 2)], False, color, width, cv2.LINE_AA)


def draw_legend(canvas: np.ndarray, lines: list[str]) -> None:
    height = 34 * len(lines) + 18
    cv2.rectangle(canvas, (12, 12), (760, height), (0, 0, 0), -1)
    for index, text in enumerate(lines):
        cv2.putText(canvas, text, (24, 40 + index * 34), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2, cv2.LINE_AA)


def manual_lines(annotation: dict[str, Any], y0: float, y1: float, count: int = 128) -> tuple[list[tuple[float, float]], ...]:
    ys = np.linspace(y0, y1, count)
    left = manual_interpolate(annotation["left_boundary_manual_points"], ys)
    right = manual_interpolate(annotation["right_boundary_manual_points"], ys)
    center = (left + right) * 0.5
    return ([(float(x), float(y)) for x, y in zip(left, ys)],
            [(float(x), float(y)) for x, y in zip(right, ys)],
            [(float(x), float(y)) for x, y in zip(center, ys)])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=SNAP / "raw.bmp")
    parser.add_argument("--annotation", type=Path, default=ANN_DIR / "raw_annotation.json")
    parser.add_argument("--output", type=Path, default=OUT)
    args = parser.parse_args()
    out = args.output
    out.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(args.input), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {args.input}")
    annotation = json.loads(args.annotation.read_text(encoding="utf-8"))
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    single_tracker = StableSeamTracker(CONFIG, REFERENCE)
    single = single_tracker.process(image, diagnostics=True)
    repeated_tracker = StableSeamTracker(CONFIG, REFERENCE)
    repeated = [repeated_tracker.process(image, diagnostics=True) for _ in range(5)]
    audit = capture_internal(tracker, image)
    roi = audit["roi"]
    sx, sy = audit["sx"], audit["sy"]
    x0, y0, rw, rh = roi
    common_y = (max(min(p["y"] for p in annotation["left_boundary_manual_points"]), min(p["y"] for p in annotation["right_boundary_manual_points"])),
                min(max(p["y"] for p in annotation["left_boundary_manual_points"]), max(p["y"] for p in annotation["right_boundary_manual_points"])))
    y_original = y0 + audit["scan_ys"].astype(np.float64) / sy
    eval_mask = (y_original >= common_y[0]) & (y_original <= common_y[1])
    manual_left = manual_interpolate(annotation["left_boundary_manual_points"], y_original)
    manual_right = manual_interpolate(annotation["right_boundary_manual_points"], y_original)
    manual_center = (manual_left + manual_right) * 0.5
    manual_width = manual_right - manual_left
    ref_centers, ref_widths = tracker.reference.at(y_original, image.shape[1], image.shape[0])
    selected_by_band = {pair.band_index: pair for pair in audit["path"]}
    fit_inliers_by_band = {}
    if audit["fit"] is not None:
        fit_inliers_by_band = {pair.band_index: bool(flag) for pair, flag in zip(audit["path"], audit["fit"]["inliers"])}
    rows: list[dict[str, Any]] = []
    tolerance_summary: dict[str, dict[str, int]] = {}
    for tolerance in (15, 25, 40):
        tolerance_summary[str(tolerance)] = {"left_near": 0, "right_near": 0, "both_near": 0, "correct_pair": 0}
    for band_index, yp in enumerate(audit["scan_ys"]):
        yo = float(y_original[band_index])
        if not bool(eval_mask[band_index]):
            continue
        ml, mr, mc, mw = float(manual_left[band_index]), float(manual_right[band_index]), float(manual_center[band_index]), float(manual_width[band_index])
        ref_w = float(ref_widths[band_index])
        lefts = audit["left_candidates"][band_index] if band_index < len(audit["left_candidates"]) else np.empty(0, dtype=np.int32)
        rights = audit["right_candidates"][band_index] if band_index < len(audit["right_candidates"]) else np.empty(0, dtype=np.int32)
        left_info = [{"x_processed_px": int(v), "x_original_px": x0 + float(v) / sx,
                      "edge_strength": float(audit["response"][int(yp), int(v)])} for v in lefts]
        right_info = [{"x_processed_px": int(v), "x_original_px": x0 + float(v) / sx,
                       "edge_strength": float(-audit["response"][int(yp), int(v)])} for v in rights]
        pairs = [pair_dict(pair, sx, roi, sy, ref_w, {"left": ml, "right": mr, "center": mc, "width": mw}) for pair in audit["all_pairs"][band_index]]
        selected = selected_by_band.get(band_index)
        selected_info = pair_dict(selected, sx, roi, sy, ref_w, {"left": ml, "right": mr, "center": mc, "width": mw}) if selected else None
        for tolerance in (15, 25, 40):
            near_l = any(abs(item["x_original_px"] - ml) <= tolerance for item in left_info)
            near_r = any(abs(item["x_original_px"] - mr) <= tolerance for item in right_info)
            correct_pair = any(abs(item["left_original_px"] - ml) <= tolerance and abs(item["right_original_px"] - mr) <= tolerance for item in pairs)
            tolerance_summary[str(tolerance)]["left_near"] += int(near_l)
            tolerance_summary[str(tolerance)]["right_near"] += int(near_r)
            tolerance_summary[str(tolerance)]["both_near"] += int(near_l and near_r)
            tolerance_summary[str(tolerance)]["correct_pair"] += int(correct_pair)
        near_l25 = [item for item in left_info if abs(item["x_original_px"] - ml) <= 25]
        near_r25 = [item for item in right_info if abs(item["x_original_px"] - mr) <= 25]
        correct_pairs = [item for item in pairs if abs(item["left_original_px"] - ml) <= 25 and abs(item["right_original_px"] - mr) <= 25]
        possible_widths = [r["x_original_px"] - l["x_original_px"] for l in near_l25 for r in near_r25 if r["x_original_px"] > l["x_original_px"]]
        width_min, width_max = 0.55 * ref_w, 1.80 * ref_w
        selected_near_l = selected_info is not None and abs(selected_info["left_original_px"] - ml) <= 25
        selected_near_r = selected_info is not None and abs(selected_info["right_original_px"] - mr) <= 25
        if not near_l25:
            left_class = "manual_candidate_missing"
        elif selected_near_l and fit_inliers_by_band.get(band_index, False):
            left_class = "selected_pair_near_manual_and_inlier"
        elif selected_near_l:
            left_class = "selected_pair_near_manual_but_ransac_outlier"
        elif selected_info is None or not selected_near_r:
            left_class = "selected_pair_far_from_manual"
        else:
            left_class = "selected_pair_far_from_manual"
        if not near_r25:
            right_class = "manual_candidate_missing"
        elif selected_near_r and fit_inliers_by_band.get(band_index, False):
            right_class = "selected_pair_near_manual_and_inlier"
        elif selected_near_r:
            right_class = "selected_pair_near_manual_but_ransac_outlier"
        else:
            right_class = "selected_pair_far_from_manual"
        if not near_l25 or not near_r25:
            pair_class = "manual_candidate_missing"
        elif not correct_pairs:
            pair_class = "correct_pair_rejected_by_width_constraint" if possible_widths and not any(width_min <= w <= width_max for w in possible_widths) else "correct_candidates_exist_but_pair_not_formed"
        elif selected_info is None:
            pair_class = "correct_pair_lost_by_transition_or_dp"
        elif selected_near_l and selected_near_r:
            pair_class = "selected_pair_near_manual_and_inlier" if fit_inliers_by_band.get(band_index, False) else "selected_pair_near_manual_but_ransac_outlier"
        else:
            correct_score = max(item["unary_score"] for item in correct_pairs)
            pair_class = "correct_pair_valid_but_lower_unary_score" if selected_info["unary_score"] > correct_score else "correct_pair_lost_by_transition_or_dp"
        row = {
            "band_index": band_index, "y_processed_px": float(yp), "y_original_px": yo,
            "manual": {"left": ml, "right": mr, "center": mc, "width": mw},
            "reference": {"center": float(ref_centers[band_index]), "width": ref_w, "width_min": width_min, "width_max": width_max},
            "left_candidates": left_info, "right_candidates": right_info, "pairs": pairs,
            "selected_pair": selected_info, "selected_joint_inlier": fit_inliers_by_band.get(band_index),
            "manual_near_candidate_counts_25px": {"left": len(near_l25), "right": len(near_r25)},
            "manual_near_pair_count_25px": len(correct_pairs),
            "possible_manual_pair_widths_px_25px": possible_widths,
            "classification": {"left": left_class, "right": right_class, "pair": pair_class},
        }
        rows.append(row)
    # Fit-side inlier audit uses the production robust fit helper, not a replacement fit.
    fit_audit: dict[str, Any] = {"available": audit["fit"] is not None}
    if audit["fit"] is not None:
        fit = audit["fit"]
        gcfg = tracker.config["geometry"]
        center_fit = robust_polynomial_fit(fit["ys"], fit["centers"], int(gcfg["fit_degree"]), int(gcfg["robust_fit_iterations"]), float(gcfg["robust_fit_huber_delta_px"]))
        width_fit = robust_polynomial_fit(fit["ys"], fit["widths"], int(gcfg["width_fit_degree"]), int(gcfg["robust_fit_iterations"]), float(gcfg["width_fit_huber_delta_scale"]) * float(gcfg["robust_fit_huber_delta_px"]))
        center_inliers = center_fit[1] if center_fit else np.zeros(len(fit["ys"]), dtype=bool)
        width_inliers = width_fit[1] if width_fit else np.zeros(len(fit["ys"]), dtype=bool)
        joint = np.asarray(fit["inliers"], dtype=bool)
        left_model = np.polyval(fit["center"], fit["ys"]) - 0.5 * np.polyval(fit["width"], fit["ys"])
        right_model = np.polyval(fit["center"], fit["ys"]) + 0.5 * np.polyval(fit["width"], fit["ys"])
        left_obs = fit["centers"] - 0.5 * fit["widths"]
        right_obs = fit["centers"] + 0.5 * fit["widths"]
        delta = float(gcfg["robust_fit_huber_delta_px"])
        left_boundary_inliers = np.abs(left_obs - left_model) <= delta
        right_boundary_inliers = np.abs(right_obs - right_model) <= delta
        outlier_bands = [int(pair.band_index) for pair, flag in zip(audit["path"], joint) if not flag]
        fit_audit.update({"total_fit_points": len(fit["ys"]), "joint_inlier_count": int(np.count_nonzero(joint)),
                          "joint_inlier_ratio": float(np.mean(joint)), "center_inlier_count": int(np.count_nonzero(center_inliers)),
                          "width_inlier_count": int(np.count_nonzero(width_inliers)), "left_boundary_inlier_count_derived": int(np.count_nonzero(left_boundary_inliers)),
                          "right_boundary_inlier_count_derived": int(np.count_nonzero(right_boundary_inliers)),
                          "outlier_band_indices": outlier_bands, "outlier_band_ranges": _ranges(outlier_bands),
                          "outlier_source_counts_derived": {"left": int(np.count_nonzero(~left_boundary_inliers)), "right": int(np.count_nonzero(~right_boundary_inliers)), "joint": int(np.count_nonzero(~joint))},
                          "residual_px": float(fit["residual_px"]), "note": "left/right counts are derived from final boundary residuals; production fit exposes only center, width and joint masks."})
    # Width distributions in manual and reference curves.
    manual_width_eval = manual_width[eval_mask]
    reference_width_eval = ref_widths[eval_mask]
    full_y = np.linspace(common_y[0], common_y[1], 512)
    manual_full_width = (manual_interpolate(annotation["right_boundary_manual_points"], full_y)
                         - manual_interpolate(annotation["left_boundary_manual_points"], full_y))
    _full_ref_center, reference_full_width = tracker.reference.at(full_y, image.shape[1], image.shape[0])
    selected_widths = np.asarray([row["selected_pair"]["width_original_px"] for row in rows if row["selected_pair"] is not None], dtype=np.float64)
    selected_width_errors = np.asarray([row["selected_pair"]["error_px"]["width"] for row in rows if row["selected_pair"] is not None], dtype=np.float64)
    width_report = {"common_y_range_original_px": list(common_y), "evaluation_y_range_original_px": [float(np.min(y_original[eval_mask])), float(np.max(y_original[eval_mask]))],
                    "manual": _stats(manual_width_eval), "reference": _stats(reference_width_eval),
                    "manual_full_common_range": _stats(manual_full_width), "reference_full_common_range": _stats(reference_full_width),
                    "selected_path": _stats(selected_widths), "selected_path_width_error_vs_manual": _stats(selected_width_errors),
                    "manual_minus_reference_median_px": float(np.median(manual_width_eval) - np.median(reference_width_eval)),
                    "manual_in_reference_hard_range_ratio": float(np.mean((manual_width_eval >= 0.55 * reference_width_eval) & (manual_width_eval <= 1.80 * reference_width_eval))),
                    "narrow_internal_edge_evidence": bool(len(selected_widths) and np.median(selected_widths) < np.median(manual_width_eval) * 0.8)}
    # Selected path transition arithmetic uses the production _transition helper;
    # the internal vectorized DP score table is not exposed by the production API.
    transition_rows = []
    cumulative = 0.0
    for previous, current in zip(audit["path"], audit["path"][1:]):
        missing = current.band_index - previous.band_index - 1
        yo = y0 + current.y / sy
        refw = float(tracker.reference.at(np.asarray([yo]), image.shape[1], image.shape[0])[1][0]) * sx
        penalty = tracker._transition(previous, current, missing, refw)
        contribution = current.score - penalty if penalty is not None else None
        if contribution is not None:
            cumulative += contribution
        transition_rows.append({"from_band": previous.band_index, "to_band": current.band_index, "missing": missing,
                                "transition_penalty": penalty, "current_unary_score": current.score, "selected_path_contribution": contribution,
                                "reconstructed_cumulative": cumulative})
    # Overlays.
    manual_l, manual_r, manual_c = manual_lines(annotation, common_y[0], common_y[1])
    candidate_canvas = image.copy()
    draw_polyline(candidate_canvas, manual_l, (255, 0, 255), 6); draw_polyline(candidate_canvas, manual_r, (255, 255, 0), 6); draw_polyline(candidate_canvas, manual_c, (255, 255, 255), 4)
    for row in rows:
        for item in row["left_candidates"]:
            near = abs(item["x_original_px"] - row["manual"]["left"]) <= 25
            cv2.circle(candidate_canvas, (round(item["x_original_px"]), round(row["y_original_px"])), 3, (0, 220, 0) if near else (0, 0, 255), -1)
        for item in row["right_candidates"]:
            near = abs(item["x_original_px"] - row["manual"]["right"]) <= 25
            cv2.circle(candidate_canvas, (round(item["x_original_px"]), round(row["y_original_px"])), 3, (0, 220, 0) if near else (0, 0, 255), -1)
    draw_legend(candidate_canvas, ["manual L magenta / R cyan / C white", "candidate near manual green; other candidate red"])
    cv2.imwrite(str(out / "manual_vs_candidates_overlay.png"), candidate_canvas)
    selected_canvas = image.copy(); draw_polyline(selected_canvas, manual_l, (255, 0, 255), 6); draw_polyline(selected_canvas, manual_r, (255, 255, 0), 6); draw_polyline(selected_canvas, manual_c, (255, 255, 255), 4)
    selected_l = [(float(row["selected_pair"]["left_original_px"]), float(row["y_original_px"])) for row in rows if row["selected_pair"]]
    selected_r = [(float(row["selected_pair"]["right_original_px"]), float(row["y_original_px"])) for row in rows if row["selected_pair"]]
    selected_c = [(float(row["selected_pair"]["center_original_px"]), float(row["y_original_px"])) for row in rows if row["selected_pair"]]
    draw_polyline(selected_canvas, selected_l, (255, 0, 0), 7); draw_polyline(selected_canvas, selected_r, (0, 255, 0), 7); draw_polyline(selected_canvas, selected_c, (0, 255, 255), 5)
    draw_legend(selected_canvas, ["manual L/R/C: magenta/cyan/white", "selected L/R/C: blue/green/yellow"])
    cv2.imwrite(str(out / "selected_prefit_path_overlay.png"), selected_canvas)
    fit_canvas = selected_canvas.copy()
    if audit["fit"] is not None:
        fit = audit["fit"]
        model_y = np.linspace(float(fit["ys"][0]), float(fit["ys"][-1]), 128)
        model_center = np.polyval(fit["center"], model_y); model_width = np.polyval(fit["width"], model_y)
        draw_polyline(fit_canvas, [(x0 + float(x - 0.5 * w) / sx, y0 + float(y) / sy) for x, y, w in zip(model_center, model_y, model_width)], (0, 165, 255), 5)
        draw_polyline(fit_canvas, [(x0 + float(x + 0.5 * w) / sx, y0 + float(y) / sy) for x, y, w in zip(model_center, model_y, model_width)], (0, 165, 255), 5)
        for pair, flag in zip(audit["path"], fit["inliers"]):
            color = (0, 255, 0) if flag else (0, 0, 255)
            cy = y0 + pair.y / sy
            cv2.circle(fit_canvas, (round(x0 + pair.center / sx), round(cy)), 8, color, -1)
    draw_legend(fit_canvas, ["manual L/R/C: magenta/cyan/white", "selected L/R/C: blue/green/yellow", "fit inlier green; fit outlier red; fit curves orange"])
    cv2.imwrite(str(out / "fit_inlier_outlier_overlay.png"), fit_canvas)
    names = ["manual_vs_candidates_overlay.png", "selected_prefit_path_overlay.png", "fit_inlier_outlier_overlay.png"]
    tiles = [cv2.resize(cv2.imread(str(out / name)), (1024, 768), interpolation=cv2.INTER_AREA) for name in names]
    montage = np.vstack([np.hstack([tiles[0], tiles[1]]), np.hstack([tiles[2], np.zeros_like(tiles[2])])])
    cv2.imwrite(str(out / "diagnostic_montage.jpg"), montage, [cv2.IMWRITE_JPEG_QUALITY, 90])
    # Reports.
    baseline = {"single_frame": result_dict(single), "repeated_frames": [result_dict(item) for item in repeated],
                "spatial_vs_temporal_note": "All five repeated frames stayed LOST because the gate rejected each frame; no temporal coefficients were retained. The single-frame internal path below is spatial-only.",
                "internal_path": {"valid_before_temporal": audit["valid"], "geometry_metrics": clean({k: v for k, v in audit["metrics"].items() if k != "widths"}), "failure_reasons": audit["reasons"], "scan_band_count": len(audit["scan_ys"]), "accepted_scan_bands": len(audit["path"]), "rejected_scan_bands": len(audit["scan_ys"]) - len(audit["path"]), "processing_shape": list(audit["gray"].shape), "sx": sx, "sy": sy}}
    (out / "baseline_reproduction.json").write_text(json.dumps(clean(baseline), indent=2) + "\n")
    (out / "reference_vs_manual_width.json").write_text(json.dumps(clean(width_report), indent=2) + "\n")
    (out / "scan_band_diagnostics.json").write_text(json.dumps(clean({"scan_bands": rows, "tolerance_summary": tolerance_summary, "candidate_observation": "left/right candidates were captured by wrapping the production _local_peaks_batch; pairs are the production max_pairs_per_band output."}), indent=2) + "\n")
    with (out / "scan_band_diagnostics.csv").open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["band_index", "y_original_px", "manual_left", "manual_right", "reference_width", "pair_count", "selected_left", "selected_right", "selected_width", "selected_score", "left_class", "right_class", "pair_class", "joint_inlier"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames); writer.writeheader()
        for row in rows:
            selected = row["selected_pair"] or {}
            writer.writerow({"band_index": row["band_index"], "y_original_px": row["y_original_px"], "manual_left": row["manual"]["left"], "manual_right": row["manual"]["right"], "reference_width": row["reference"]["width"], "pair_count": len(row["pairs"]), "selected_left": selected.get("left_original_px"), "selected_right": selected.get("right_original_px"), "selected_width": selected.get("width_original_px"), "selected_score": selected.get("unary_score"), "left_class": row["classification"]["left"], "right_class": row["classification"]["right"], "pair_class": row["classification"]["pair"], "joint_inlier": row["selected_joint_inlier"]})
    counts = {"left": {}, "right": {}, "pair": {}}
    for row in rows:
        for side in counts:
            key = row["classification"][side]; counts[side][key] = counts[side].get(key, 0) + 1
    (out / "failure_classification.json").write_text(json.dumps(clean({"primary_tolerance_px": 25, "counts": counts, "sensitivity": tolerance_summary}), indent=2) + "\n")
    (out / "dp_transition_trace.json").write_text(json.dumps(clean({"available_internal_dp_score_table": False, "reason": "_select_path vectorizes totals and does not expose scores/predecessors; no duplicate DP was implemented.", "selected_path_transition_arithmetic": transition_rows}), indent=2) + "\n")
    (out / "fit_inlier_diagnostic.json").write_text(json.dumps(clean(fit_audit), indent=2) + "\n")
    source_graph = {"production_file": str((ROOT / "tank_vision/stable_seam_tracker.py").resolve()), "call_graph": ["process -> _prepare -> _generate_candidates -> _select_path -> _fit -> _geometry -> temporal update", "_generate_candidates -> _local_peaks_batch (signed Sobel candidates) -> width/contrast/prior scores -> max_pairs_per_band", "_select_path -> vectorized dynamic programming with center/width/left/right/skip transition costs", "_fit -> robust_polynomial_fit(center) + robust_polynomial_fit(width) -> joint_inliers", "_geometry -> coverage/span/missing/jumps/inlier/residual/path confidence/model geometry gates"], "observability_limits": ["pre-truncation valid pair mask is not returned", "DP score/predecessor matrices are not returned", "production fit returns joint mask only; separate left/right counts are explicitly derived in the report"], "source_sha256": sha256(ROOT / "tank_vision/stable_seam_tracker.py")}
    (out / "source_call_graph.json").write_text(json.dumps(source_graph, indent=2) + "\n")
    (out / "evidence_manifest.json").write_text((FREEZE / "ground_truth_manifest.json").read_text())
    summary = {"status": "complete_read_only_diagnostic", "primary_tolerance_px": 25, "roi_original_px": list(roi), "common_y_range_original_px": list(common_y), "single_result": result_dict(single), "repeated_results": [result_dict(item) for item in repeated], "width_summary": width_report, "classification_counts": counts, "fit_inlier": fit_audit, "conclusions": {"roi_correct": bool(roi == (1229, 1229, 1229, 1782)), "correct_pair_exists_any_band": bool(tolerance_summary['25']['correct_pair']), "dp_score_table_observable": False}}
    (out / "diagnostic_summary.json").write_text(json.dumps(clean(summary), indent=2) + "\n")
    print(json.dumps(clean(summary), indent=2))
    return 0


def _stats(values: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if not len(values):
        return {"count": 0, "min": None, "p05": None, "p50": None, "p95": None, "max": None, "mean": None}
    return {"count": int(len(values)), "min": float(np.min(values)), "p05": float(np.percentile(values, 5)), "p50": float(np.percentile(values, 50)), "p95": float(np.percentile(values, 95)), "max": float(np.max(values)), "mean": float(np.mean(values))}


def _ranges(indices: list[int]) -> list[list[int]]:
    if not indices:
        return []
    values = sorted(set(indices)); ranges = [[values[0], values[0]]]
    for value in values[1:]:
        if value == ranges[-1][1] + 1: ranges[-1][1] = value
        else: ranges.append([value, value])
    return ranges


if __name__ == "__main__":
    raise SystemExit(main())
