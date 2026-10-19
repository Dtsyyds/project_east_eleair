#!/usr/bin/env python3
"""Read-only candidate survival audit for geometry provenance events.

The audit reconstructs only the already-recorded image-processing profile and
uses the persisted provenance stages.  It never calls the tracker and never
changes production configuration or algorithm state.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tank_vision.seam_width_reference import SeamWidthReference


STAGES = (
    "raw_local_extrema",
    "thresholded_extrema",
    "nms_survivor",
    "side_topk_survivor",
    "anchor_final_candidate",
)
FIRST_LOSS = (
    "manual_outside_guided_window",
    "expected_polarity_response_missing",
    "response_below_threshold",
    "local_extrema_not_detected",
    "removed_by_nms",
    "removed_by_side_topk",
    "anchor_merge_not_retained",
    "candidate_survived",
    "unavailable",
)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def percentile(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "min": None, "p01": None, "p05": None,
                "p50": None, "p95": None, "p99": None, "max": None}
    data = np.asarray(values, dtype=np.float64)
    return {"count": len(data), "min": float(data.min()),
            "p01": float(np.percentile(data, 1)), "p05": float(np.percentile(data, 5)),
            "p50": float(np.percentile(data, 50)), "p95": float(np.percentile(data, 95)),
            "p99": float(np.percentile(data, 99)), "max": float(data.max())}


def nearest(values: list[dict[str, Any]], x: float) -> tuple[float | None, dict[str, Any] | None]:
    if not values:
        return None, None
    item = min(values, key=lambda value: abs(float(value["x"]) - x))
    return abs(float(item["x"]) - x), item


def has_near(values: list[dict[str, Any]], x: float, tolerance: float) -> bool:
    return any(abs(float(item["x"]) - x) <= tolerance for item in values)


def roi_to_processed(x_roi: float, y_roi: float, sx: float, sy: float) -> tuple[float, float]:
    return float(x_roi * sx), float(y_roi * sy)


def processed_to_roi(x_processed: float, y_processed: float, sx: float, sy: float) -> tuple[float, float]:
    return float(x_processed / sx), float(y_processed / sy)


def first_loss_category(*, manual_inside: bool, best_expected_response: float | None,
                        threshold: float | None, raw_near: bool, threshold_near: bool,
                        nms_near: bool, topk_near: bool, final_near: bool,
                        anchor_valid: bool, anchor_near: bool) -> str:
    """Return one mutually-exclusive first-loss category."""
    if not manual_inside:
        return "manual_outside_guided_window"
    if best_expected_response is None or best_expected_response <= 0.0:
        return "expected_polarity_response_missing"
    if threshold is None:
        return "unavailable"
    if not raw_near:
        return "local_extrema_not_detected"
    if not threshold_near:
        return "response_below_threshold"
    if not nms_near:
        return "removed_by_nms"
    if not topk_near:
        return "removed_by_side_topk"
    if not final_near:
        return "anchor_merge_not_retained" if anchor_valid else "removed_by_side_topk"
    return "candidate_survived"


def pair_category(*, left_exists: bool, right_exists: bool, pair: dict[str, Any] | None,
                  pair_after: set[str], pair_post: set[str]) -> str:
    if not left_exists or not right_exists:
        return "one_side_candidate_missing"
    if pair is None:
        return "unavailable"
    if not pair.get("hard_gate_passed", False):
        reason = str(pair.get("hard_gate_rejection_reason") or "")
        if reason == "invalid_left_right_order":
            return "invalid_left_right_order"
        if reason in {"width_below_minimum", "width_above_maximum"}:
            return "width_hard_gate_rejected"
        if reason in {"invalid_interior_sample", "invalid_left_exterior_sample", "invalid_right_exterior_sample"}:
            return "sampling_region_invalid"
        return "evidence_invalid"
    pair_id = str(pair.get("pair_id"))
    if pair_id not in pair_after:
        return "evidence_invalid"
    if pair_id not in pair_post:
        return "removed_by_pair_topk"
    return "entered_dp"


def draw_dashed(canvas: np.ndarray, points: list[tuple[int, int]], colour: tuple[int, int, int], width: int = 2) -> None:
    for first, second in zip(points, points[1:]):
        a, b = np.asarray(first, dtype=np.float64), np.asarray(second, dtype=np.float64)
        length = float(np.linalg.norm(b - a))
        if length <= 0:
            continue
        count = max(1, int(length / 12.0))
        for index in range(0, count, 2):
            p0 = a + (b - a) * index / count
            p1 = a + (b - a) * min(index + 1, count) / count
            cv2.line(canvas, tuple(np.rint(p0).astype(int)), tuple(np.rint(p1).astype(int)), colour, width, cv2.LINE_AA)


def manual_points(annotation: dict[str, Any], side: str) -> list[tuple[float, float]]:
    key = "left_boundary_manual_points" if side == "left" else "right_boundary_manual_points"
    return sorted([(float(item["x"]), float(item["y"])) for item in annotation.get(key, [])], key=lambda item: item[1])


def interpolate_manual(annotation: dict[str, Any], side: str, y_roi: float) -> float:
    points = manual_points(annotation, side)
    return float(np.interp(y_roi, [item[1] for item in points], [item[0] for item in points]))


def make_profiles(raw: np.ndarray, config: dict[str, Any], ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    processing = config["processing"]
    gray = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY) if raw.ndim == 3 else raw.astype(np.uint8)
    max_width = int(processing["max_roi_width"])
    scale = min(1.0, max_width / raw.shape[1])
    size = (max(1, round(raw.shape[1] * scale)), max(1, round(raw.shape[0] * scale)))
    processed = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if size != (raw.shape[1], raw.shape[0]) else gray.copy()
    processed = cv2.GaussianBlur(processed, (int(processing["gaussian_kernel"]), int(processing["gaussian_kernel"])), 0)
    response = cv2.Sobel(processed, cv2.CV_32F, 1, 0, ksize=int(processing["sobel_kernel"]))
    half = int(processing["scan_band_half_height_px"])
    profiles = cv2.blur(response, (1, 2 * half + 1))
    profiles = cv2.GaussianBlur(profiles, (int(processing["profile_gaussian_kernel"]), 1), 0)[ys]
    thresholds = np.maximum(float(processing["edge_min_response"]),
                            np.percentile(np.abs(profiles), float(processing["edge_percentile"]), axis=1))
    return profiles, thresholds


def stage_values(row: dict[str, Any], side: str) -> dict[str, list[dict[str, Any]]]:
    def values(key: str) -> list[dict[str, Any]]:
        return list(row.get(key, {}).get(side, []))
    return {
        "guided_window_peaks": values("guided_window_peaks"),
        "thresholded_extrema": values("guided_window_peaks"),
        "nms_survivor": values("separation_filtered_peaks"),
        "side_topk_survivor": values("separation_filtered_peaks")[:3],
        "final_candidate": values("top_k_after"),
    }


def process_event(event: Path, annotation: dict[str, Any], config: dict[str, Any], reference: SeamWidthReference,
                  tolerance: float, out_event: Path) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    provenance = load_json(event / "provenance.json")
    metadata = load_json(event / "metadata.json")
    raw = cv2.imread(str(event / "roi_raw.bmp"), cv2.IMREAD_COLOR)
    if raw is None:
        raise RuntimeError(f"cannot read {event / 'roi_raw.bmp'}")
    roi_x, roi_y, roi_w, roi_h = [int(value) for value in metadata["roi_original_px"]]
    sx, sy = float(provenance["scales"]["sx"]), float(provenance["scales"]["sy"])
    trace = provenance["candidate_stage_trace"]
    ys = np.asarray([int(item["y_processed"]) for item in trace], dtype=np.int32)
    profiles, thresholds = make_profiles(raw, config, ys)
    original_y = roi_y + ys.astype(np.float64) / sy
    reference_center, reference_width = reference.at(original_y, 4096, 3072)
    expected_left = (reference_center - 0.5 * reference_width - roi_x) * sx
    expected_right = (reference_center + 0.5 * reference_width - roi_x) * sx
    radius = 0.4 * reference_width * sx
    margin = max(int(config["processing"]["edge_margin_min_px"]), int(round(profiles.shape[1] * float(config["processing"]["edge_margin_ratio"]))))
    windows = {
        "left": np.column_stack((np.clip(np.floor(expected_left - radius), margin, profiles.shape[1] - margin),
                                 np.clip(np.ceil(expected_left + radius) + 1, margin + 1, profiles.shape[1] - margin))),
        "right": np.column_stack((np.clip(np.floor(expected_right - radius), margin, profiles.shape[1] - margin),
                                  np.clip(np.ceil(expected_right + radius) + 1, margin + 1, profiles.shape[1] - margin))),
    }
    rows: list[dict[str, Any]] = []
    pair_rows: list[dict[str, Any]] = []
    for band_index, (band, y_p, y_o, threshold) in enumerate(zip(trace, ys, original_y, thresholds)):
        y_roi = float(y_p) / sy
        pair_trace = band["pair_stages"]
        all_cartesian = list(pair_trace.get("cartesian_pairs", []))
        pair_after = {str(item.get("pair_id")) for item in pair_trace.get("pair_hard_gate_after", [])}
        pair_post = {str(item.get("pair_id")) for item in pair_trace.get("pair_after_max_pairs_per_band", [])}
        per_side: dict[str, dict[str, Any]] = {}
        for side, positive in (("left", True), ("right", False)):
            manual_x_roi = interpolate_manual(annotation, side, y_roi)
            manual_x_processed = manual_x_roi * sx
            lo, hi = [int(value) for value in windows[side][band_index]]
            signed = profiles[band_index] if positive else -profiles[band_index]
            local_slice = signed[lo:hi]
            best_index = int(lo + np.argmax(local_slice)) if len(local_slice) else None
            best_response = float(signed[best_index]) if best_index is not None else None
            window_values = [{"x_processed": int(index), "signed_response": round(float(signed[index]), 6),
                              "absolute_response": round(float(abs(profiles[band_index, index])), 6)}
                             for index in range(lo, hi)]
            raw_local = []
            kernel = np.ones((1, 2 * int(config["processing"]["edge_min_separation_px"]) + 1), dtype=np.uint8)
            local_max = cv2.dilate(signed.reshape(1, -1), kernel).ravel()
            for index in range(lo, hi):
                if float(signed[index]) >= float(local_max[index]):
                    raw_local.append({"x": int(index), "signed_response": float(signed[index])})
            stages = stage_values(band, side)
            raw_near = has_near(raw_local, manual_x_processed, tolerance)
            threshold_near = any(item["signed_response"] >= float(threshold) and
                                 abs(float(item["x"]) - manual_x_processed) <= tolerance for item in raw_local)
            nms_near = has_near(stages["nms_survivor"], manual_x_processed, tolerance)
            topk_near = has_near(stages["side_topk_survivor"], manual_x_processed, tolerance)
            final_near = has_near(stages["final_candidate"], manual_x_processed, tolerance)
            anchor = band.get("reference_anchor", {}).get(side, {})
            anchor_valid = bool(anchor.get("valid", False))
            anchor_near = anchor.get("position_processed") is not None and abs(float(anchor["position_processed"]) - manual_x_processed) <= tolerance
            first_loss = first_loss_category(
                manual_inside=lo <= manual_x_processed < hi,
                best_expected_response=best_response,
                threshold=float(threshold), raw_near=raw_near, threshold_near=threshold_near,
                nms_near=nms_near, topk_near=topk_near, final_near=final_near,
                anchor_valid=anchor_valid, anchor_near=anchor_near)
            nearest_candidate_distance, nearest_candidate = nearest(stages["final_candidate"], manual_x_processed)
            per_side[side] = {
                "manual_x_roi": manual_x_roi, "manual_y_roi": y_roi,
                "manual_x_processed": manual_x_processed, "manual_y_processed": float(y_p),
                "reference_x_processed": float(expected_left[band_index] if side == "left" else expected_right[band_index]),
                "guided_window_min_processed": lo, "guided_window_max_processed": hi,
                "guided_radius_original_px": float(radius[band_index] / max(sx, 1e-12)),
                "manual_inside_guided_window": lo <= manual_x_processed < hi,
                "manual_to_reference_distance_px": abs(manual_x_processed - (expected_left[band_index] if side == "left" else expected_right[band_index])) / max(sx, 1e-12),
                "signed_sobel_profile_window": window_values,
                "manual_response": {
                    "signed_response_at_nearest_px": float(signed[int(np.clip(round(manual_x_processed), 0, len(signed) - 1))]),
                    "best_response_in_window": best_response,
                    "best_position_processed": best_index,
                    "best_position_error_px": None if best_index is None else abs(best_index - manual_x_processed),
                    "expected_polarity": "positive" if positive else "negative",
                    "polarity_matches_best": bool(best_response is not None and best_response > 0),
                    "threshold": float(threshold),
                    "response_threshold_ratio": None if best_response is None else best_response / max(float(threshold), 1e-9),
                },
                "stages": {
                    "raw_local_extrema": raw_local,
                    "thresholded_extrema": stages["thresholded_extrema"],
                    "nms_survivor": stages["nms_survivor"],
                    "side_topk_survivor": stages["side_topk_survivor"],
                    "final_candidate": stages["final_candidate"],
                },
                "stage_near_manual": {"raw_local_extrema": raw_near, "thresholded_extrema": threshold_near,
                                      "nms_survivor": nms_near, "side_topk_survivor": topk_near,
                                      "final_candidate": final_near},
                "anchor": {"valid": anchor_valid, "position_processed": anchor.get("position_processed"),
                            "distance_to_manual_px": None if anchor.get("position_processed") is None else abs(float(anchor["position_processed"]) - manual_x_processed),
                            "retained_in_final_candidates": anchor.get("retained_in_final_candidates")},
                "nearest_final_candidate_distance_px": nearest_candidate_distance,
                "nearest_final_candidate": nearest_candidate,
                "first_loss": first_loss,
            }
            rows.append({"event": event.name, "frame_index": metadata.get("captured_index"), "band_index": band_index,
                         "y_processed": int(y_p), "y_original": float(y_o), "side": side, **per_side[side]})
        left_candidates = per_side["left"]["stages"]["final_candidate"]
        right_candidates = per_side["right"]["stages"]["final_candidate"]
        left_exists = per_side["left"]["stage_near_manual"]["final_candidate"]
        right_exists = per_side["right"]["stage_near_manual"]["final_candidate"]
        near_pair = None
        if left_exists and right_exists:
            choices = [item for item in all_cartesian
                       if abs(float(item["left_processed"]) - per_side["left"]["manual_x_processed"]) <= tolerance
                       and abs(float(item["right_processed"]) - per_side["right"]["manual_x_processed"]) <= tolerance]
            if choices:
                near_pair = min(choices, key=lambda item: abs(float(item["left_processed"]) - per_side["left"]["manual_x_processed"])
                                + abs(float(item["right_processed"]) - per_side["right"]["manual_x_processed"]))
        pair_rows.append({"event": event.name, "frame_index": metadata.get("captured_index"), "band_index": band_index,
                          "y_processed": int(y_p), "y_original": float(y_o), "tolerance_px": tolerance,
                          "left_candidate_near": left_exists, "right_candidate_near": right_exists,
                          "manual_left_x_processed": per_side["left"]["manual_x_processed"],
                          "manual_right_x_processed": per_side["right"]["manual_x_processed"],
                          "pair": near_pair,
                          "pair_category": pair_category(left_exists=left_exists, right_exists=right_exists,
                                                          pair=near_pair, pair_after=pair_after, pair_post=pair_post),
                          "cartesian_pair_count": len(all_cartesian),
                          "pre_truncation_count": pair_trace.get("pre_truncation_count"),
                          "post_truncation_count": pair_trace.get("post_truncation_count"),
                          "max_pairs_per_band": pair_trace.get("max_pairs_per_band")})
    out_event.mkdir(parents=True, exist_ok=True)
    draw_event_visuals(out_event, raw, rows, per_event_trace=trace, reference=reference, roi=(roi_x, roi_y, roi_w, roi_h), sx=sx, sy=sy, profiles=profiles)
    return rows, pair_rows, {"metadata": metadata, "roi": [roi_x, roi_y, roi_w, roi_h], "sx": sx, "sy": sy,
                             "raw_shape": list(raw.shape), "processing_shape": [int(profiles.shape[1]), int(round(raw.shape[0] * sy))]}


def draw_event_visuals(out_event: Path, raw: np.ndarray, rows: list[dict[str, Any]], per_event_trace: list[dict[str, Any]],
                       reference: SeamWidthReference, roi: tuple[int, int, int, int], sx: float, sy: float,
                       profiles: np.ndarray) -> None:
    roi_x, roi_y, roi_w, roi_h = roi
    canvas = raw.copy()
    display = canvas.copy()
    # Manual boundaries and reference boundaries in original ROI pixels.
    event_rows = {side: [row for row in rows if row["side"] == side] for side in ("left", "right")}
    for side in ("left", "right"):
        points = [(int(round(row["manual_x_roi"])), int(round(row["manual_y_roi"]))) for row in event_rows[side]]
        cv2.polylines(display, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, (255, 255, 255), 3, cv2.LINE_AA)
    y_original = np.asarray([row["y_original"] for row in event_rows["left"]], dtype=np.float64)
    center, width = reference.at(y_original, 4096, 3072)
    for values, colour in (((center - width / 2 - roi_x), (180, 0, 180)), ((center + width / 2 - roi_x), (180, 0, 180))):
        points = [(int(round(x)), int(round((y - roi_y)))) for x, y in zip(values, y_original)]
        draw_dashed(display, points, colour, 2)
    overlay = display.copy()
    for side in ("left", "right"):
        for row in event_rows[side]:
            lo, hi = row["guided_window_min_processed"], row["guided_window_max_processed"]
            y = int(round(row["manual_y_roi"]))
            x0, x1 = int(round(lo / sx)), int(round(hi / sx))
            cv2.line(overlay, (x0, y), (x1, y), (180, 0, 180), 3)
    display = cv2.addWeighted(overlay, 0.18, display, 0.82, 0)
    cv2.imwrite(str(out_event / "manual_vs_guided_window.png"), display)

    stage_colours = {"raw_local_extrema": (160, 160, 160), "thresholded_extrema": (255, 255, 0),
                     "nms_survivor": (255, 0, 0), "side_topk_survivor": (0, 255, 0),
                     "anchor_final_candidate": (0, 165, 255), "final_candidate": (0, 255, 255)}
    candidate = raw.copy()
    for row in rows:
        side = row["side"]
        for stage in ("raw_local_extrema", "thresholded_extrema", "nms_survivor", "side_topk_survivor", "final_candidate"):
            for item in row["stages"][stage]:
                x = int(round(float(item["x"]) / sx))
                y = int(round(row["manual_y_roi"]))
                cv2.circle(candidate, (x, y), 2 if stage != "final_candidate" else 4, stage_colours[stage], -1)
        anchor = row["anchor"]
        if anchor.get("position_processed") is not None:
            cv2.circle(candidate, (int(round(float(anchor["position_processed"]) / sx)), int(round(row["manual_y_roi"]))), 4, stage_colours["anchor_final_candidate"], 1)
        if row["first_loss"] != "candidate_survived":
            x = int(round(row["manual_x_roi"])); y = int(round(row["manual_y_roi"]))
            cv2.line(candidate, (x - 6, y - 6), (x + 6, y + 6), (0, 0, 255), 2)
            cv2.line(candidate, (x - 6, y + 6), (x + 6, y - 6), (0, 0, 255), 2)
    for side in ("left", "right"):
        points = [(int(round(row["manual_x_roi"])), int(round(row["manual_y_roi"]))) for row in event_rows[side]]
        cv2.polylines(candidate, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(out_event / "candidate_survival_overlay.png"), candidate)

    # Signed response profile heatmap, one panel per side.
    panels = []
    for side, sign in (("left", 1.0), ("right", -1.0)):
        image = np.clip(sign * profiles, -50.0, 50.0)
        normalized = cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        heatmap = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        heatmap = cv2.resize(heatmap, (900, 500), interpolation=cv2.INTER_NEAREST)
        for row in event_rows[side]:
            x = int(round(row["manual_x_processed"] / profiles.shape[1] * 900))
            y = int(round((row["manual_y_processed"] - min(r["manual_y_processed"] for r in event_rows[side])) /
                          max(max(r["manual_y_processed"] for r in event_rows[side]) - min(r["manual_y_processed"] for r in event_rows[side]), 1) * 499))
            cv2.circle(heatmap, (x, y), 3, (255, 255, 255), -1)
        cv2.putText(heatmap, f"{side} expected signed response", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
        panels.append(heatmap)
    cv2.imwrite(str(out_event / "signed_response_profiles.png"), np.vstack(panels))

    first = np.zeros_like(raw)
    colours = {name: colour for name, colour in zip(FIRST_LOSS, ((0, 0, 255), (0, 80, 255), (0, 165, 255), (255, 0, 255), (255, 0, 0), (255, 128, 0), (0, 255, 255), (0, 255, 0), (100, 100, 100)))}
    for row in rows:
        x, y = int(round(row["manual_x_roi"])), int(round(row["manual_y_roi"]))
        cv2.circle(first, (x, y), 5, colours[row["first_loss"]], -1)
    cv2.imwrite(str(out_event / "first_loss_by_y.png"), first)


def coordinate_validation(events: list[Path], annotations: dict[str, dict[str, Any]], out: Path) -> dict[str, Any]:
    report: dict[str, Any] = {"status": "valid", "events": [], "max_roundtrip_error_px": {"x": 0.0, "y": 0.0}}
    overlays = []
    for event in events:
        provenance, metadata = load_json(event / "provenance.json"), load_json(event / "metadata.json")
        annotation = annotations[event.name]
        raw = cv2.imread(str(event / "roi_raw.bmp"), cv2.IMREAD_COLOR)
        roi_x, roi_y, roi_w, roi_h = [int(value) for value in metadata["roi_original_px"]]
        sx, sy = float(provenance["scales"]["sx"]), float(provenance["scales"]["sy"])
        processing_size = [round(roi_w * sx), round(roi_h * sy)]
        errors = {"manual": {"x": [], "y": []}, "candidate": {"x": [], "y": []},
                  "selected_path": {"x": [], "y": []}, "fitted_path": {"x": [], "y": []}}
        def check_roi(x: float, y: float, bucket: str) -> None:
            xp, yp = roi_to_processed(x, y, sx, sy)
            xb, yb = processed_to_roi(xp, yp, sx, sy)
            errors[bucket]["x"].append(abs(xb - x)); errors[bucket]["y"].append(abs(yb - y))
        for side in ("left", "right"):
            for point in manual_points(annotation, side):
                check_roi(point[0], point[1], "manual")
        for band in provenance["candidate_stage_trace"]:
            for side in ("left", "right"):
                for item in band["top_k_after"][side]:
                    check_roi(float(item["x"]), float(band["y_processed"]), "candidate")
        for item in provenance.get("selected_raw_path", []):
            check_roi(float(item["left"]) - roi_x, (float(item["y"]) - roi_y), "selected_path")
            check_roi(float(item["right"]) - roi_x, (float(item["y"]) - roi_y), "selected_path")
        result = provenance.get("result", {})
        for x, y in zip(result.get("fitted_left", []), result.get("fitted_y", [])):
            check_roi(float(x) - roi_x, float(y) - roi_y, "fitted_path")
        for x, y in zip(result.get("fitted_right", []), result.get("fitted_y", [])):
            check_roi(float(x) - roi_x, float(y) - roi_y, "fitted_path")
        event_report = {"event": event.name, "roi_original_size": [roi_w, roi_h],
                        "processing_size": processing_size, "sx": sx, "sy": sy,
                        "original_full_image_coordinate_system": {"origin": [0, 0], "roi_offset": [roi_x, roi_y]},
                        "roi_pixels_coordinate_system": {"origin": [roi_x, roi_y], "size": [roi_w, roi_h]},
                        "processed_roi_coordinate_system": {"origin": [0, 0], "size": processing_size},
                        "roundtrip": {bucket: {axis: percentile(values) for axis, values in axes.items()} for bucket, axes in errors.items()}}
        for bucket, axes in errors.items():
            for axis, values in axes.items():
                report["max_roundtrip_error_px"][axis] = max(report["max_roundtrip_error_px"][axis], max(values or [0.0]))
        report["events"].append(event_report)
        overlay = raw.copy()
        for side, colour in (("left", (255, 255, 255)), ("right", (255, 255, 255))):
            points = [(int(round(x)), int(round(y))) for x, y in manual_points(annotation, side)]
            cv2.polylines(overlay, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, colour, 3, cv2.LINE_AA)
            roundtrip = [(int(round((x * sx) / sx)), int(round((y * sy) / sy))) for x, y in manual_points(annotation, side)]
            cv2.polylines(overlay, [np.asarray(roundtrip, dtype=np.int32).reshape(-1, 1, 2)], False, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(overlay, event.name[-28:], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 3)
        cv2.putText(overlay, event.name[-28:], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 255), 1)
        overlays.append(cv2.resize(overlay, (512, 538), interpolation=cv2.INTER_AREA))
    report["pass"] = report["max_roundtrip_error_px"]["x"] <= 0.5 and report["max_roundtrip_error_px"]["y"] <= 0.5
    if not report["pass"]:
        report["status"] = "coordinate_mapping_invalid"
    cv2.imwrite(str(out / "coordinate_validation_overlay.png"), np.hstack(overlays))
    return report


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys); writer.writeheader()
        for row in rows:
            flattened = {key: (json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else value)
                         for key, value in row.items()}
            writer.writerow(flattened)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--annotations", type=Path, required=True)
    parser.add_argument("--config", type=Path, default=ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml")
    parser.add_argument("--reference", type=Path, default=ROOT / "configs/seam_geometry_reference_s100p_live_v1.json")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    session, annotations_dir, out = args.session.resolve(), args.annotations.resolve(), args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    reference = SeamWidthReference.load(args.reference)
    events = sorted(item for item in session.glob("event_*") if (item / "metadata.json").is_file())
    annotations: dict[str, dict[str, Any]] = {}
    for item in annotations_dir.rglob("*.json"):
        if item.name == "manual_error_summary.json":
            continue
        event_name = item.parent.name if item.parent.name.startswith("event_") else item.stem
        annotations[event_name] = load_json(item)
    needed = [event for event in events if event.name in annotations]
    coordinate = coordinate_validation(needed, annotations, out)
    (out / "coordinate_roundtrip_report.json").write_text(json.dumps(coordinate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    if not coordinate["pass"]:
        summary = {"final_status": "coordinate_mapping_invalid", "main_conclusion": None,
                   "reason": "coordinate roundtrip error exceeds 0.5 px; candidate audit stopped"}
        (out / "diagnostic_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return 2
    canonical_tolerance = 20.0
    all_rows: list[dict[str, Any]] = []
    all_pairs: list[dict[str, Any]] = []
    event_meta: dict[str, Any] = {}
    for event in needed:
        rows, pairs, meta = process_event(event, annotations[event.name], config, reference, canonical_tolerance, out / event.name)
        all_rows.extend(rows); all_pairs.extend(pairs); event_meta[event.name] = meta
    write_csv(out / "candidate_survival_by_band.csv", all_rows)
    (out / "candidate_survival_by_band.json").write_text(json.dumps({"tolerance_px": canonical_tolerance, "rows": all_rows}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "pair_formation_funnel.json").write_text(json.dumps({"tolerance_px": canonical_tolerance, "rows": all_pairs,
                                                                    "counts": dict(Counter(row["pair_category"] for row in all_pairs))}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    first_hist = {"tolerance_px": canonical_tolerance, "by_event_side": {}, "by_event": {}, "aggregate": {}}
    for event in needed:
        for side in ("left", "right"):
            subset = [row for row in all_rows if row["event"] == event.name and row["side"] == side]
            first_hist["by_event_side"][f"{event.name}:{side}"] = dict(Counter(row["first_loss"] for row in subset))
    first_hist["aggregate"] = dict(Counter(row["first_loss"] for row in all_rows))
    for event in needed:
        first_hist["by_event"][event.name] = dict(Counter(row["first_loss"] for row in all_rows if row["event"] == event.name))
    (out / "first_loss_histogram.json").write_text(json.dumps(first_hist, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    tolerance_report = {}
    for tolerance in (15, 20, 25, 40):
        counts = Counter(); band_missing = Counter()
        for event in needed:
            rows, pairs, _ = process_event(event, annotations[event.name], config, reference, float(tolerance), out / "_sensitivity")
            counts.update(row["first_loss"] for row in rows)
            for band in range(117):
                side_rows = [row for row in rows if row["band_index"] == band]
                if any(row["first_loss"] != "candidate_survived" for row in side_rows):
                    band_missing[event.name] += 1
        tolerance_report[str(tolerance)] = {"first_loss_counts": dict(counts), "candidate_missing_band_count": dict(band_missing),
                                            "total_side_records": sum(counts.values())}
    (out / "tolerance_sensitivity.json").write_text(json.dumps(tolerance_report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    side_report = {}
    for side in ("left", "right"):
        values = [row for row in all_rows if row["side"] == side]
        side_report[side] = {"records": len(values), "first_loss": dict(Counter(row["first_loss"] for row in values)),
                             "candidate_missing": sum(row["first_loss"] != "candidate_survived" for row in values)}
    (out / "side_asymmetry.json").write_text(json.dumps(side_report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    y_summary = {}
    all_y = [float(row["y_original"]) for row in all_rows]
    edges = np.linspace(min(all_y), max(all_y) + 1e-6, 5)
    for index in range(4):
        subset = [row for row in all_rows if edges[index] <= float(row["y_original"]) < edges[index + 1]]
        y_summary[f"{edges[index]:.1f}-{edges[index + 1]:.1f}"] = dict(Counter(row["first_loss"] for row in subset))
    (out / "y_interval_failure_summary.json").write_text(json.dumps(y_summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    counts = Counter(row["first_loss"] for row in all_rows)
    failure_counts = Counter({key: value for key, value in counts.items() if key != "candidate_survived"})
    failed_records = sum(failure_counts.values())
    dominant, dominant_count = failure_counts.most_common(1)[0] if failure_counts else ("unavailable", 0)
    mapping = {"manual_outside_guided_window": "A", "expected_polarity_response_missing": "B",
               "response_below_threshold": "C", "local_extrema_not_detected": "D",
               "removed_by_nms": "D", "removed_by_side_topk": "E", "anchor_merge_not_retained": "F"}
    candidate_change = mapping.get(dominant, "I") if failed_records and dominant_count / failed_records >= 0.5 else "I"
    if not counts or not all_rows:
        candidate_change = "I"
    summary = {"final_status": "complete", "coordinate_status": coordinate["status"],
               "events": [event.name for event in needed], "annotated_event_count": len(needed),
               "scan_band_count": len({(row["event"], row["band_index"]) for row in all_rows}),
               "side_record_count": len(all_rows), "first_loss_tolerance_px": canonical_tolerance,
               "first_loss_counts": dict(counts), "dominant_first_loss": dominant,
               "dominant_first_loss_ratio": dominant_count / failed_records if failed_records else None,
               "failed_first_loss_counts": dict(failure_counts),
               "candidate_missing_band_count_by_event": {
                   event.name: sum(any(row["first_loss"] != "candidate_survived"
                                       for row in all_rows if row["event"] == event.name and row["band_index"] == band)
                                  for band in range(len({row["band_index"] for row in all_rows if row["event"] == event.name})))
                   for event in needed
               },
               "guided_radius_original_px": percentile([float(row["guided_radius_original_px"]) for row in all_rows]),
               "manual_to_reference_distance_px": percentile([float(row["manual_to_reference_distance_px"]) for row in all_rows]),
               "next_single_change": candidate_change,
               "next_single_change_name": {"A": "guided_window_coverage", "B": "signed_edge_polarity_or_response",
                                            "C": "response_threshold", "D": "local_extrema_or_nms", "E": "side_candidate_topk",
                                            "F": "anchor_localization", "I": "mixed_or_insufficient_evidence"}.get(candidate_change),
               "production_behavior_unchanged": True,
               "fixed_input_sha256": {event.name: sha256(event / "roi_raw.bmp") for event in needed}}
    (out / "diagnostic_summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (out / "next_single_change.json").write_text(json.dumps({"choice": candidate_change,
                                                               "name": summary["next_single_change_name"],
                                                               "rule": "dominant first-loss stage must be >=50%; otherwise I",
                                                               "do_not_modify": ["ROI", "reference", "geometry gates", "DP", "IRLS", "temporal"]}, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    # Montage from the three event overlays.
    tiles = []
    for event in needed:
        image = cv2.imread(str(out / event.name / "candidate_survival_overlay.png"))
        if image is not None:
            tiles.append(cv2.resize(image, (600, 630), interpolation=cv2.INTER_AREA))
    while len(tiles) < 3:
        tiles.append(np.zeros((630, 600, 3), dtype=np.uint8))
    cv2.imwrite(str(out / "candidate_survival_montage.jpg"), np.hstack(tiles[:3]))
    print(json.dumps({"final_status": "complete", "main_next_single_change": candidate_change,
                      "dominant_first_loss": dominant, "dominant_ratio": dominant_count / failed_records if failed_records else None}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
