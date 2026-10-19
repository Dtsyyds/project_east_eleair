#!/usr/bin/env python3
"""S1-P2E3: decompose reference-model and returned-edge selection error."""

from __future__ import annotations

import csv
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

from tank_vision.seam_width_reference import SeamWidthReference  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402

OUT = ROOT / "output/s1_p2e3_reference_selection_decomposition"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
TRACKER = ROOT / "tank_vision/stable_seam_tracker.py"
WEIGHTS = {"center_transition_weight": 0.42}
EXPECTED_RAW = "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02"
EXPECTED_ANNOTATION = "c43ab7ffc01b076f7de5671ea7e72af60113e03d01e72e5c6d6a26b0d4010a15"
EXPECTED_CONFIG = "15e93c6f311445c7d986b0492a4dfb43179fc2092e54c81bcc0090d69cd298fc"
EXPECTED_REFERENCE = "0324547cf7d7d304b16deca1e06550c3c212fd1615c21caab5f6c4a49bde7935"
THRESHOLDS = {"left": 30.0, "right": 30.0, "center": 25.0, "width": 30.0}


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
    return value


def write_json(name: str, value: Any) -> None:
    (OUT / name).write_text(json.dumps(clean(value), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pstats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"count": int(len(a)), "mean": float(np.mean(a)), "p50": float(np.percentile(a, 50)),
            "p90": float(np.percentile(a, 90)), "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def error_stats(values: list[float]) -> dict[str, Any]:
    return {"signed": pstats(values), "absolute": pstats([abs(x) for x in values])}


def rankdata(values: list[float]) -> np.ndarray:
    a = np.asarray(values, dtype=np.float64)
    order = np.argsort(a, kind="mergesort")
    ranks = np.empty(len(a), dtype=np.float64)
    i = 0
    while i < len(a):
        j = i + 1
        while j < len(a) and a[order[j]] == a[order[i]]:
            j += 1
        ranks[order[i:j]] = 0.5 * (i + 1 + j)
        i = j
    return ranks


def spearman(x: list[float], y: list[float]) -> float | None:
    if len(x) < 2 or len(x) != len(y):
        return None
    rx, ry = rankdata(x), rankdata(y)
    if np.std(rx) == 0 or np.std(ry) == 0:
        return 0.0
    return float(np.corrcoef(rx, ry)[0, 1])


def manual_points() -> tuple[np.ndarray, np.ndarray]:
    payload = json.loads(ANNOTATION.read_text(encoding="utf-8"))
    left = np.asarray([(p["y"], p["x"]) for p in payload["left_boundary_manual_points"]], dtype=np.float64)
    right = np.asarray([(p["y"], p["x"]) for p in payload["right_boundary_manual_points"]], dtype=np.float64)
    return left, right


def manual_at(y: float, points: tuple[np.ndarray, np.ndarray]) -> dict[str, float]:
    left, right = points
    ml = float(np.interp(y, left[:, 0], left[:, 1]))
    mr = float(np.interp(y, right[:, 0], right[:, 1]))
    return {"left": ml, "right": mr, "center": 0.5 * (ml + mr), "width": mr - ml}


def curve_from_pair(left: float, right: float, center: float | None = None) -> dict[str, float]:
    c = 0.5 * (left + right) if center is None else center
    return {"left": float(left), "right": float(right), "center": float(c), "width": float(right - left)}


def original_pair(pair: Any, result: Any, diagnostics: Any) -> dict[str, float | int | str]:
    x0, _y0, rw, _rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    return {"pair_id": f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}",
            "band_index": int(pair.band_index), "left": float(x0 + pair.left / sx),
            "right": float(x0 + pair.right / sx), "center": float(x0 + pair.center / sx),
            "width": float(pair.width / sx), "unary_score": float(pair.score),
            "score_parts": {k: float(v) for k, v in pair.score_parts.items()}}


def verify_inputs() -> dict[str, Any]:
    paths = {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": TRACKER}
    if any(not path.is_file() for path in paths.values()):
        raise RuntimeError("one or more fixed inputs are missing")
    before = {name: sha256(path) for name, path in paths.items()}
    expected = {"raw": EXPECTED_RAW, "annotation": EXPECTED_ANNOTATION,
                "config": EXPECTED_CONFIG, "reference": EXPECTED_REFERENCE}
    mismatch = {k: {"expected": v, "actual": before[k]} for k, v in expected.items() if before[k] != v}
    if mismatch:
        raise RuntimeError(f"fixed input hash mismatch: {mismatch}")
    ref_payload = json.loads(REFERENCE.read_text(encoding="utf-8"))
    return {"sha256_before": before, "raw_expected_sha256": EXPECTED_RAW,
            "raw_sha256_matches": before["raw"] == EXPECTED_RAW,
            "reference_status": ref_payload.get("status"),
            "reference_generated_from_manual_annotation": ref_payload.get("generated_from_manual_annotation"),
            "reference_annotation_sha256": ref_payload.get("annotation_sha256"),
            "reference_used_roi": ref_payload.get("used_roi")}


def baseline(image: np.ndarray, points: tuple[np.ndarray, np.ndarray]) -> tuple[dict[str, Any], Any, Any, StableSeamTracker]:
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    results = []
    first = first_diag = None
    for index in range(3):
        result = tracker.process(image, diagnostics=index == 0)
        results.append(result)
        if index == 0:
            first, first_diag = result, tracker.last_diagnostics
    assert first is not None and first_diag is not None
    rows = make_rows(first, first_diag, points, tracker.reference)
    selected = [row for row in rows if row["selected"] is not None]
    errors = {name: [row["total_error"][name] for row in selected] for name in THRESHOLDS}
    actual = {"selected_path_count": len(selected), "scan_band_count": len(rows),
              "coverage": first.coverage, "effective_span": first.effective_span,
              "states": [r.tracking_state.name for r in results],
              "center_abs_p95": pstats([abs(x) for x in errors["center"]])["p95"],
              "left_abs_p95": pstats([abs(x) for x in errors["left"]])["p95"],
              "right_abs_p95": pstats([abs(x) for x in errors["right"]])["p95"],
              "width_abs_p95": pstats([abs(x) for x in errors["width"]])["p95"],
              "joint_inlier_ratio": first.fit_inlier_ratio}
    expected = {"selected_path_count": 116, "scan_band_count": 117, "coverage": 116 / 117,
                "states": ["ACQUIRING", "ACQUIRING", "TRACKING"], "center_abs_p95": 44.14758855034813,
                "left_abs_p95": 59.276740922960016, "right_abs_p95": 47.91756560334744,
                "width_abs_p95": 70.42111032039196}
    checks = {"selected_path_count": actual["selected_path_count"] == expected["selected_path_count"],
              "scan_band_count": actual["scan_band_count"] == expected["scan_band_count"],
              "coverage": abs(actual["coverage"] - expected["coverage"]) < 1e-9,
              "states": actual["states"] == expected["states"]}
    for key in ("center_abs_p95", "left_abs_p95", "right_abs_p95", "width_abs_p95"):
        checks[key] = abs(actual[key] - expected[key]) < 1e-6
    return {"status": "matched" if all(checks.values()) else "baseline_mismatch", "expected": expected,
            "actual": actual, "checks": checks, "reference_load": "StableSeamTracker(CONFIG, REFERENCE)"}, first, first_diag, tracker


def reference_curve(reference: SeamWidthReference, y: float, image_size: tuple[int, int]) -> dict[str, float]:
    width_px, height_px = image_size
    center, width = reference.at(y, width_px, height_px)
    center, width = float(center), float(width)
    return curve_from_pair(center - width / 2, center + width / 2, center)


def make_rows(result: Any, diagnostics: Any, points: tuple[np.ndarray, np.ndarray], reference: SeamWidthReference) -> list[dict[str, Any]]:
    roi = result.roi_original_px
    sy = diagnostics.processed_gray.shape[0] / roi[3]
    selected_by_band = {p.band_index: p for p in diagnostics.selected_pairs}
    rows: list[dict[str, Any]] = []
    # The reference is evaluated at every production scan-band y, including
    # the one band absent from the selected path.
    for band, y_processed in enumerate(diagnostics.scan_ys):
        y = float(roi[1] + y_processed / sy)
        manual = manual_at(y, points)
        ref = reference_curve(reference, y, result_image_shape(result))
        selected_pair = selected_by_band.get(band)
        selected = original_pair(selected_pair, result, diagnostics) if selected_pair else None
        pairs = diagnostics.all_pairs[band]
        ref_near = None
        manual_near = None
        if pairs:
            ref_near_pair = min(pairs, key=lambda p: abs(original_pair(p, result, diagnostics)["left"] - ref["left"]) + abs(original_pair(p, result, diagnostics)["right"] - ref["right"]))
            manual_near_pair = min(pairs, key=lambda p: abs(original_pair(p, result, diagnostics)["left"] - manual["left"]) + abs(original_pair(p, result, diagnostics)["right"] - manual["right"]))
            ref_near = original_pair(ref_near_pair, result, diagnostics)
            manual_near = original_pair(manual_near_pair, result, diagnostics)
        selected_curve = selected if selected else None
        reference_error = {k: ref[k] - manual[k] for k in THRESHOLDS}
        selection_error = {k: (selected_curve[k] - ref[k]) if selected_curve else None for k in THRESHOLDS}
        total_error = {k: (selected_curve[k] - manual[k]) if selected_curve else None for k in THRESHOLDS}
        identity_residual = {k: (total_error[k] - reference_error[k] - selection_error[k]) if selected_curve else None for k in THRESHOLDS}
        pair_summaries = []
        for rank, candidate in enumerate(pairs, start=1):
            item = original_pair(candidate, result, diagnostics)
            item.update({"unary_rank": rank, "distance_to_manual_l1": abs(item["left"] - manual["left"]) + abs(item["right"] - manual["right"]),
                         "distance_to_reference_l1": abs(item["left"] - ref["left"]) + abs(item["right"] - ref["right"])})
            pair_summaries.append(item)
        left_candidates_original = [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in diagnostics.left_candidates[band]]
        right_candidates_original = [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in diagnostics.right_candidates[band]]
        rows.append({"band_index": int(band), "y_processed": int(y_processed), "y_original": y,
                     "manual": manual, "reference": ref, "selected": selected,
                     "reference_nearest_pair": ref_near, "manual_nearest_pair": manual_near,
                     "left_candidates_processed": [int(x) for x in diagnostics.left_candidates[band]],
                     "right_candidates_processed": [int(x) for x in diagnostics.right_candidates[band]],
                     "left_candidates_original": left_candidates_original, "right_candidates_original": right_candidates_original,
                     "pair_rankings": pair_summaries,
                     "reference_error": reference_error, "selection_error": selection_error,
                     "total_error": total_error, "decomposition_residual": identity_residual,
                     "pair_count": len(pairs), "selected_pair_rank": pair_rank(selected, pairs, result, diagnostics),
                     "manual_nearest_pair_rank": pair_rank(manual_near, pairs, result, diagnostics),
                     "reference_nearest_pair_rank": pair_rank(ref_near, pairs, result, diagnostics),
                     "selected_reference_distance_l1": None if selected is None else abs(selected["left"] - ref["left"]) + abs(selected["right"] - ref["right"])})
    return rows


def result_image_shape(result: Any) -> tuple[int, int]:
    # StableSeamTracker only accepts the frozen 4096x3072 image in this audit.
    return (4096, 3072)


def aggregate(rows: list[dict[str, Any]], key: str, fields: tuple[str, ...] = ("left", "right", "center", "width")) -> dict[str, Any]:
    usable = [row for row in rows if row.get(key) and all(row[key][field] is not None for field in fields)]
    return {field: error_stats([float(row[key][field]) for row in usable]) for field in fields} | {"count": len(usable)}


def error_correlations(rows: list[dict[str, Any]]) -> dict[str, Any]:
    usable = [row for row in rows if row["selected"] is not None]
    out: dict[str, Any] = {"evaluated_bands": len(usable), "by_metric": {}}
    for field in THRESHOLDS:
        ref = [row["reference_error"][field] for row in usable]
        sel = [row["selection_error"][field] for row in usable]
        total = [row["total_error"][field] for row in usable]
        out["by_metric"][field] = {"reference_vs_total_spearman": spearman(ref, total),
            "selection_vs_total_spearman": spearman(sel, total),
            "reference_total_same_sign_fraction": float(np.mean([a * b >= 0 for a, b in zip(ref, total)])),
            "selection_total_same_sign_fraction": float(np.mean([a * b >= 0 for a, b in zip(sel, total)])),
            "reference_selection_same_sign_fraction": float(np.mean([a * b >= 0 for a, b in zip(ref, sel)])),
            "reference_error": error_stats(ref), "selection_error": error_stats(sel), "total_error": error_stats(total)}
    return out


def classify(rows: list[dict[str, Any]]) -> dict[str, Any]:
    counts = {name: 0 for name in ("reference_dominant", "selection_dominant", "both_same_direction", "partial_cancellation", "near_manual", "not_evaluable")}
    output = []
    for row in rows:
        if row["selected"] is None:
            category = "not_evaluable"
        else:
            ref_norm = max(abs(row["reference_error"][f]) / THRESHOLDS[f] for f in THRESHOLDS)
            sel_norm = max(abs(row["selection_error"][f]) / THRESHOLDS[f] for f in THRESHOLDS)
            total_norm = max(abs(row["total_error"][f]) / THRESHOLDS[f] for f in THRESHOLDS)
            opposite = any(row["reference_error"][f] * row["selection_error"][f] < 0 for f in THRESHOLDS)
            same = all(row["reference_error"][f] * row["selection_error"][f] >= 0 for f in THRESHOLDS if abs(row["reference_error"][f]) > 1e-9 and abs(row["selection_error"][f]) > 1e-9)
            if total_norm <= 1.0:
                category = "near_manual"
            elif opposite and total_norm < max(ref_norm, sel_norm):
                category = "partial_cancellation"
            elif same and ref_norm > 0 and sel_norm > 0:
                category = "both_same_direction"
            elif ref_norm >= sel_norm:
                category = "reference_dominant"
            else:
                category = "selection_dominant"
            row["normalized_max"] = {"reference": ref_norm, "selection": sel_norm, "total": total_norm}
            row["larger_absolute_contribution_by_metric"] = {f: ("reference" if abs(row["reference_error"][f]) >= abs(row["selection_error"][f]) else "selection") for f in THRESHOLDS}
        counts[category] += 1
        output.append({"band_index": row["band_index"], "y_original": row["y_original"], "category": category,
                       "normalized_max": row.get("normalized_max"), "larger_absolute_contribution_by_metric": row.get("larger_absolute_contribution_by_metric")})
    return {"main_tolerances_px": THRESHOLDS, "counts": counts, "counts_sum": sum(counts.values()), "rows": output}


def pair_rank(pair: dict[str, Any] | None, pairs: list[Any], result: Any, diagnostics: Any) -> int | None:
    if pair is None:
        return None
    for index, candidate in enumerate(pairs):
        item = original_pair(candidate, result, diagnostics)
        if item["pair_id"] == pair["pair_id"]:
            return index + 1
    return None


def profile_for_band(result: Any, diagnostics: Any, config: dict[str, Any], row: dict[str, Any]) -> dict[str, Any]:
    band = row["band_index"]
    gray = diagnostics.processed_gray
    half = int(config["processing"]["scan_band_half_height_px"])
    sobel = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(config["processing"]["sobel_kernel"]))
    signed_profiles = cv2.GaussianBlur(cv2.blur(sobel, (1, 2 * half + 1)),
                                       (int(config["processing"]["profile_gaussian_kernel"]), 1), 0)
    gray_profiles = cv2.GaussianBlur(cv2.blur(gray.astype(np.float32), (1, 2 * half + 1)), (9, 1), 0)
    scan_y = int(diagnostics.scan_ys[band])
    signed = signed_profiles[scan_y]
    gray_line = gray_profiles[scan_y]
    threshold = max(float(config["processing"]["edge_min_response"]),
                    float(np.percentile(np.abs(signed), float(config["processing"]["edge_percentile"]))))
    roi = result.roi_original_px
    sx = gray.shape[1] / roi[2]
    def proc(original: float) -> int:
        return int(round((original - roi[0]) * sx))
    manual = row["manual"]
    ref = row["reference"]
    selected = row["selected"]
    positions = {"manual_left": proc(manual["left"]), "manual_right": proc(manual["right"]),
                 "reference_left": proc(ref["left"]), "reference_right": proc(ref["right"]),
                 "selected_left": None if selected is None else proc(selected["left"]),
                 "selected_right": None if selected is None else proc(selected["right"])}
    left_candidates = [int(x) for x in diagnostics.left_candidates[band]]
    right_candidates = [int(x) for x in diagnostics.right_candidates[band]]
    return {"band_index": band, "y_original": row["y_original"], "gray_profile_smoothed": gray_line.tolist(),
            "signed_sobel_profile": signed.tolist(), "threshold": threshold, "positions_processed": positions,
            "left_candidates_processed": left_candidates, "right_candidates_processed": right_candidates,
            "manual_signed_sobel": {"left": float(signed[np.clip(positions["manual_left"], 0, len(signed) - 1)]),
                                    "right": float(-signed[np.clip(positions["manual_right"], 0, len(signed) - 1)])},
            "reference_signed_sobel": {"left": float(signed[np.clip(positions["reference_left"], 0, len(signed) - 1)]),
                                       "right": float(-signed[np.clip(positions["reference_right"], 0, len(signed) - 1)])},
            "selected_signed_sobel": None if selected is None else {"left": float(signed[np.clip(positions["selected_left"], 0, len(signed) - 1)]),
                                                                       "right": float(-signed[np.clip(positions["selected_right"], 0, len(signed) - 1)])}}


def draw_profile(profile: dict[str, Any], path: Path) -> np.ndarray:
    width, height = 900, 480
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    gray = np.asarray(profile["gray_profile_smoothed"], dtype=np.float64)
    signed = np.asarray(profile["signed_sobel_profile"], dtype=np.float64)
    pos = profile["positions_processed"]
    all_x = [v for v in pos.values() if v is not None] + profile["left_candidates_processed"] + profile["right_candidates_processed"]
    x0, x1 = max(0, min(all_x) - 55), min(len(gray) - 1, max(all_x) + 55)
    px0, px1 = 45, width - 25
    top, mid, bottom = 35, 220, 445
    def xp(x: int) -> int: return int(px0 + (x - x0) * (px1 - px0) / max(x1 - x0, 1))
    def gy(v: float) -> int: return int(mid - (v - np.min(gray)) / max(np.max(gray) - np.min(gray), 1) * (mid - top - 10))
    scale = max(np.max(np.abs(signed)), profile["threshold"], 1e-6)
    def sy(v: float) -> int: return int(mid + (profile["threshold"] - v) / (2 * scale) * (bottom - mid - 15))
    for values, fn, color in ((gray, gy, (80, 80, 80)), (signed, sy, (180, 80, 180))):
        points = [(xp(x), fn(float(values[x]))) for x in range(x0, x1 + 1)]
        cv2.polylines(canvas, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, color, 2, cv2.LINE_AA)
    for yval in (profile["threshold"], -profile["threshold"]):
        cv2.line(canvas, (px0, sy(yval)), (px1, sy(yval)), (0, 150, 150), 1, cv2.LINE_AA)
    colors = {"manual_left": (0, 180, 180), "manual_right": (0, 180, 180),
              "reference_left": (255, 120, 0), "reference_right": (255, 120, 0),
              "selected_left": (0, 0, 230), "selected_right": (0, 0, 230)}
    for name, value in pos.items():
        if value is not None:
            cv2.line(canvas, (xp(value), top), (xp(value), bottom), colors[name], 2)
            cv2.putText(canvas, name.replace("_", " "), (min(width - 190, xp(value) + 3), 22 + 18 * (list(pos).index(name) % 3)), cv2.FONT_HERSHEY_SIMPLEX, 0.38, colors[name], 1, cv2.LINE_AA)
    for value in profile["left_candidates_processed"]:
        cv2.circle(canvas, (xp(value), sy(float(signed[value]))), 4, (255, 0, 0), -1)
    for value in profile["right_candidates_processed"]:
        cv2.circle(canvas, (xp(value), sy(float(-signed[value]))), 4, (0, 140, 0), -1)
    cv2.putText(canvas, f"y={profile['y_original']:.1f}  gray=gray profile  magenta=signed Sobel  cyan=threshold", (15, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def draw_overlays(image: np.ndarray, rows: list[dict[str, Any]]) -> None:
    canvas = image.copy()
    colors = {"manual": (0, 255, 255), "reference": (255, 150, 0), "selected": (0, 0, 255)}
    for name in colors:
        points = [(int(round(row[name]["left"])), int(round(row["y_original"]))) for row in rows if row[name] is not None]
        right = [(int(round(row[name]["right"])), int(round(row["y_original"]))) for row in rows if row[name] is not None]
        if len(points) > 1:
            cv2.polylines(canvas, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, colors[name], 6, cv2.LINE_AA)
            cv2.polylines(canvas, [np.asarray(right, dtype=np.int32).reshape(-1, 1, 2)], False, colors[name], 6, cv2.LINE_AA)
    cv2.putText(canvas, "yellow=manual orange=reference red=selected", (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(canvas, "yellow=manual orange=reference red=selected", (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "reference_manual_selected_overlay.png"), canvas)


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    fixed = verify_inputs()
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read frozen raw.bmp")
    points = manual_points()
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    if float(config["path"]["center_transition_weight"]) != WEIGHTS["center_transition_weight"]:
        raise RuntimeError("center transition baseline is not 0.42")
    base, result, diagnostics, tracker = baseline(image, points)
    write_json("baseline_reproduction.json", {"fixed_inputs": fixed, "center_transition_weight": 0.42, "baseline": base})
    if base["status"] != "matched":
        write_json("diagnostic_summary.json", {"status": "baseline_mismatch", "stopped": True})
        return 2
    reference = tracker.reference
    rows = make_rows(result, diagnostics, points, reference)
    reference_only = aggregate(rows, "reference_error")
    selected_only = aggregate(rows, "total_error")
    manual_near = aggregate([{**row, "manual_near_error": None if row["manual_nearest_pair"] is None else {k: row["manual_nearest_pair"][k] - row["manual"][k] for k in THRESHOLDS}}
                             for row in rows], "manual_near_error")
    reference_near = aggregate([{**row, "reference_near_error": None if row["reference_nearest_pair"] is None else {k: row["reference_nearest_pair"][k] - row["manual"][k] for k in THRESHOLDS}}
                               for row in rows], "reference_near_error")
    ref_payload = json.loads(REFERENCE.read_text(encoding="utf-8"))
    ref_meta = {"representation": "normalized polynomial evaluated by SeamWidthReference.at", "center_degree": ref_payload["center_ref"]["degree"],
                "width_degree": ref_payload["width_ref"]["degree"], "center_coefficients": ref_payload["center_ref"]["coefficients"],
                "width_coefficients": ref_payload["width_ref"]["coefficients"], "fit_source": ref_payload.get("fit_source"),
                "generated_from_manual_annotation": ref_payload.get("generated_from_manual_annotation"),
                "used_roi": ref_payload.get("used_roi"), "annotation_sha256": ref_payload.get("annotation_sha256"),
                "reference_sample_count": len(ref_payload.get("reference_samples", []))}
    ref_curve_fit = {"reference_error_stats": reference_only, "model_residual_rms": {f: float(np.sqrt(np.mean(np.square([r["reference_error"][f] for r in rows])))) for f in THRESHOLDS},
                      "representation": ref_meta, "can_meet_gate": {f: reference_only[f]["absolute"]["p95"] <= THRESHOLDS[f] for f in THRESHOLDS}}
    write_json("reference_only_accuracy.json", ref_curve_fit)
    write_json("per_band_error_decomposition.json", {"record_count": len(rows), "records": rows, "identity_check": {"max_abs_residual": max(abs(v) for row in rows for v in row["decomposition_residual"].values() if v is not None)}})
    fields = ["band_index", "y_original", "reference_error", "selection_error", "total_error", "decomposition_residual"]
    with (OUT / "per_band_error_decomposition.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields); writer.writeheader()
        for row in rows: writer.writerow({f: json.dumps(row[f], ensure_ascii=False) if isinstance(row[f], dict) else row[f] for f in fields})
    correlations = error_correlations(rows)
    write_json("reference_vs_selection_correlation.json", correlations)
    classification = classify(rows)
    write_json("failure_classification.json", classification)
    oracle = {"definition": "actual final returned pairs only; no candidate generation, DP, detected or tracking rerun",
              "reference_nearest": reference_near, "manual_nearest": manual_near, "selected": selected_only,
              "counts": {"reference_nearest": sum(row["reference_nearest_pair"] is not None for row in rows),
                         "manual_nearest": sum(row["manual_nearest_pair"] is not None for row in rows),
                         "selected": sum(row["selected"] is not None for row in rows)}}
    write_json("oracle_pair_comparison.json", oracle)
    focus_rows = [row for row in rows if 2400 <= row["y_original"] <= 2700]
    profiles = [profile_for_band(result, diagnostics, config, row) for row in focus_rows]
    write_json("y_2400_2700_diagnostic.json", {"rows": focus_rows, "profile_bands": [p["band_index"] for p in profiles]})
    key = sorted(profiles, key=lambda p: min(abs(p["y_original"] - target) for target in (2442, 2453, 2661, 2672)))[:4]
    # Ensure the requested key regions are represented when available.
    selected_profiles = []
    for target in (2442, 2453, 2661, 2672):
        selected_profiles.append(min(profiles, key=lambda p: abs(p["y_original"] - target)))
    panels = [draw_profile(p, OUT / "candidate_profile_key_bands.json") for p in selected_profiles]
    montage = np.hstack(panels)
    cv2.imwrite(str(OUT / "y_2400_2700_profile_montage.png"), montage)
    write_json("candidate_profile_key_bands.json", {"profiles": selected_profiles, "requested_y_targets": [2442, 2453, 2661, 2672]})
    draw_overlays(image, rows)
    def oracle_meets_gate(report: dict[str, Any]) -> bool:
        return all(report[field]["absolute"]["p95"] <= THRESHOLDS[field] for field in THRESHOLDS)
    reference_model_good = all(ref_curve_fit["can_meet_gate"].values())
    manual_nearest_good = oracle_meets_gate(manual_near)
    reference_nearest_good = oracle_meets_gate(reference_near)
    selected_good = oracle_meets_gate(selected_only)
    if not reference_model_good:
        choice = "A"
        unique_object = "reference representation: sparse y→center/y→width linear interpolation"
        why = "The reference curve itself exceeds the requested precision gate."
    elif not manual_nearest_good:
        choice = "C"
        unique_object = "candidate localization near the reference/manual boundary"
        why = "The production reference is accurate, but even the nearest available returned pair cannot meet the gate."
    elif not selected_good and reference_nearest_good:
        choice = "B"
        unique_object = "one boundary-identity feature or reference-side proximity feature"
        why = "Accurate returned pairs exist, while current selected pairing fails to choose them."
    else:
        choice = "D"
        unique_object = "insufficient evidence for a single modification"
        why = "No single dominance rule was satisfied."
    next_change = {"choice": choice, "implemented": False,
                   "unique_modification_object": unique_object,
                   "why": why,
                   "oracle_gate_checks": {"reference_model": reference_model_good, "reference_nearest": reference_nearest_good,
                                           "manual_nearest": manual_nearest_good, "selected": selected_good},
                   "do_not_modify": ["tracker algorithm", "YAML", "reference JSON", "ROI", "candidate search", "unary", "DP", "fitting", "gate", "temporal", "ROS"],
                   "implementation_status": "design_only"}
    write_json("next_single_change.json", next_change)
    after = {name: sha256(path) for name, path in {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": TRACKER}.items()}
    summary = {"task": "S1-P2E3", "baseline_status": base["status"], "choice": choice,
               "reference_only": reference_only, "selected": selected_only, "manual_nearest": manual_near,
               "reference_nearest": reference_near, "classification_counts": classification["counts"],
               "y_2400_2700_selected_count": len(focus_rows), "y_2400_2700_reference_error": aggregate(focus_rows, "reference_error"),
               "y_2400_2700_selection_error": aggregate(focus_rows, "selection_error"),
               "y_2400_2700_total_error": aggregate(focus_rows, "total_error"),
               "correlations": correlations, "reference_metadata": ref_meta,
               "fixed_input_sha256_before": fixed["sha256_before"], "fixed_input_sha256_after": after,
               "fixed_input_hashes_unchanged": fixed["sha256_before"] == after,
               "production_changes": {"algorithm": False, "reference": False, "yaml": False, "dp": False, "gate": False, "ros": False}}
    write_json("diagnostic_summary.json", summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
