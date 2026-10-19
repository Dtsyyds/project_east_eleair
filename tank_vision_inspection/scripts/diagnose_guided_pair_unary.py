#!/usr/bin/env python3
"""Read-only guided pair/unary diagnosis for S1-P2E0.

This script deliberately uses StableSeamTracker's public diagnostics.  It does
not generate candidates, pairs, or paths itself, and it never changes the
production tracker or its inputs.  The only writes are the new S1-P2E0 output
directory.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
from pathlib import Path
from typing import Any, Iterable

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
OUT = ROOT / "output/s1_p2e0_guided_unary_diagnostic"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
MANIFEST = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/ground_truth_manifest.json"
CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
TRACKER = ROOT / "tank_vision/stable_seam_tracker.py"
TIMING_ARTIFACT = ROOT / "output/s1_p2d_simple_guided_search/timing_comparison.json"

EXPECTED = {
    "raw": "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02",
    "annotation": "c43ab7ffc01b076f7de5671ea7e72af60113e03d01e72e5c6d6a26b0d4010a15",
    "config": "15e93c6f311445c7d986b0492a4dfb43179fc2092e54c81bcc0090d69cd298fc",
    "reference": "0324547cf7d7d304b16deca1e06550c3c212fd1615c21caab5f6c4a49bde7935",
    "tracker": "088efdde35c8f5cbff1323384aae3b7468f3f6141acb18907288c5d759f60466",
}
FEATURES = (
    "left_edge_strength", "right_edge_strength", "edge_balance", "contrast",
    "width_reference", "center_prior", "temporal_center", "temporal_width",
)
LOFO_FEATURES = FEATURES[:6]
WEIGHTS = {
    "left_edge_strength": 0.10, "right_edge_strength": 0.10,
    "edge_balance": 0.12, "contrast": 0.18, "width_reference": 0.24,
    "center_prior": 0.10, "temporal_center": 0.09, "temporal_width": 0.07,
}
NEAR_PX = 25.0
PRESENCE_PX = 40.0


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def dump(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def finite(value: Any) -> float | None:
    value = float(value)
    return value if math.isfinite(value) else None


def pstats(values: Iterable[float]) -> dict[str, Any]:
    array = np.asarray(list(values), dtype=np.float64)
    if not len(array):
        return {"count": 0, "mean": None, "p50": None, "p90": None, "p95": None, "max": None}
    return {"count": int(len(array)), "mean": float(np.mean(array)),
            "p50": float(np.percentile(array, 50)), "p90": float(np.percentile(array, 90)),
            "p95": float(np.percentile(array, 95)), "max": float(np.max(array))}


def signed_stats(values: Iterable[float]) -> dict[str, Any]:
    vals = list(values)
    return {"signed": pstats(vals), "absolute": pstats(abs(value) for value in vals)}


def spearman(x: Iterable[float], y: Iterable[float]) -> float | None:
    xa, ya = np.asarray(list(x), dtype=np.float64), np.asarray(list(y), dtype=np.float64)
    if len(xa) < 2 or len(ya) != len(xa) or np.std(xa) == 0 or np.std(ya) == 0:
        return 0.0
    return float(np.corrcoef(np.argsort(np.argsort(xa)), np.argsort(np.argsort(ya)))[0, 1])


def manual_points() -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    raw = json.loads(ANNOTATION.read_text(encoding="utf-8"))
    left = np.asarray([(p["x"], p["y"]) for p in raw["left_boundary_manual_points"]], dtype=np.float64)
    right = np.asarray([(p["x"], p["y"]) for p in raw["right_boundary_manual_points"]], dtype=np.float64)
    return left[:, 1], left[:, 0], right[:, 1], right[:, 0]


def manual_at(y: float, points: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]) -> tuple[float, float, float, float]:
    ly, lx, ry, rx = points
    left = float(np.interp(y, ly, lx))
    right = float(np.interp(y, ry, rx))
    return left, right, 0.5 * (left + right), right - left


def verify_inputs() -> dict[str, Any]:
    paths = {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": TRACKER, "manifest": MANIFEST}
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        raise RuntimeError(f"missing frozen input(s): {missing}")
    hashes = {name: sha256(path) for name, path in paths.items() if name in EXPECTED}
    mismatches = {name: {"expected": EXPECTED[name], "actual": value} for name, value in hashes.items() if value != EXPECTED[name]}
    annotation = json.loads(ANNOTATION.read_text(encoding="utf-8"))
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    reference = json.loads(REFERENCE.read_text(encoding="utf-8"))
    frozen_ok = (
        manifest.get("status") == "frozen"
        and bool(manifest.get("input_sha256_matches"))
        and bool(manifest.get("human_overlay_approved"))
        and manifest.get("point_count", {}).get("total") == 18
        and annotation.get("input_sha256") == EXPECTED["raw"]
        and reference.get("input_image_sha256") == EXPECTED["raw"]
    )
    if mismatches or not frozen_ok:
        raise RuntimeError(f"frozen input verification failed: mismatches={mismatches}, frozen_ok={frozen_ok}")
    return {"paths": {key: str(value.relative_to(ROOT)) for key, value in paths.items()},
            "sha256": hashes, "manifest": {"status": manifest.get("status"),
            "input_sha256_matches": manifest.get("input_sha256_matches"),
            "human_overlay_approved": manifest.get("human_overlay_approved"),
            "point_count": manifest.get("point_count")},
            "annotation_point_count": annotation.get("point_count"),
            "reference_input_sha256": reference.get("input_image_sha256")}


def result_summary(result: Any) -> dict[str, Any]:
    return {"detected": bool(result.detected), "state": result.tracking_state.name,
            "accepted_scan_bands": int(result.accepted_scan_bands),
            "rejected_scan_bands": int(result.rejected_scan_bands),
            "coverage": float(result.coverage), "effective_span": float(result.effective_span),
            "joint_inlier_ratio": float(result.fit_inlier_ratio),
            "fit_residual_px": finite(result.fit_residual_px),
            "processing_ms": float(result.processing_ms), "failure_reasons": list(result.failure_reasons)}


def reproduce(image: np.ndarray) -> tuple[Any, Any, list[dict[str, Any]]]:
    from tank_vision.stable_seam_tracker import StableSeamTracker
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    results: list[dict[str, Any]] = []
    first_result = None
    first_diag = None
    for index in range(3):
        result = tracker.process(image, diagnostics=(index == 0))
        if index == 0:
            first_result, first_diag = result, tracker.last_diagnostics
        results.append(result_summary(result))
    if first_result is None or first_diag is None:
        raise RuntimeError("guided diagnostics were not returned")
    return first_result, first_diag, results


def baseline_report(image: np.ndarray, evidence: dict[str, Any]) -> tuple[dict[str, Any], Any, Any]:
    result, diagnostics, sequence = reproduce(image)
    selected = diagnostics.selected_pairs
    points = manual_points()
    errors = []
    for pair in selected:
        y = float(diagnostics.scan_ys[pair.band_index])
        y_original = float(result.roi_original_px[1] + y / (diagnostics.processed_gray.shape[0] / result.roi_original_px[3]))
        ml, mr, mc, mw = manual_at(y_original, points)
        sx = diagnostics.processed_gray.shape[1] / result.roi_original_px[2]
        left = result.roi_original_px[0] + pair.left / sx
        right = result.roi_original_px[0] + pair.right / sx
        errors.append((left - ml, right - mr, (left + right) / 2 - mc, right - left - mw))
    timing = json.loads(TIMING_ARTIFACT.read_text(encoding="utf-8")) if TIMING_ARTIFACT.is_file() else {}
    timing_p95 = timing.get("guided", {}).get("total_ms", {}).get("p95")
    expected = {"selected_path_count": 116, "scan_band_count": 117, "coverage": 116 / 117,
                "first_joint_inlier_ratio": 0.7931034482758621, "states": ["ACQUIRING", "ACQUIRING", "TRACKING"],
                "center_abs_p95": 44.14758855034813, "left_abs_p95": 59.276740922960016,
                "right_abs_p95": 47.91756560334744, "width_abs_p95": 70.42111032039196,
                "processing_p95": 74.64755025000001}
    actual = {"selected_path_count": len(selected), "scan_band_count": len(diagnostics.scan_ys),
              "coverage": sequence[0]["coverage"], "first_joint_inlier_ratio": sequence[0]["joint_inlier_ratio"],
              "states": [item["state"] for item in sequence],
              "center_abs_p95": pstats(abs(e[2]) for e in errors)["p95"],
              "left_abs_p95": pstats(abs(e[0]) for e in errors)["p95"],
              "right_abs_p95": pstats(abs(e[1]) for e in errors)["p95"],
              "width_abs_p95": pstats(abs(e[3]) for e in errors)["p95"],
              "processing_p95": timing_p95}
    tolerances = {"selected_path_count": 1, "scan_band_count": 0, "coverage": 0.002,
                  "first_joint_inlier_ratio": 0.002, "center_abs_p95": 0.25, "left_abs_p95": 0.25,
                  "right_abs_p95": 0.25, "width_abs_p95": 0.25, "processing_p95": 0.5}
    matches = {key: (actual[key] is not None and abs(actual[key] - value) <= tolerances[key]) if isinstance(value, (float, int)) else actual[key] == value
               for key, value in expected.items()}
    report = {"status": "matched" if all(matches.values()) else "baseline_mismatch", "evidence": evidence,
              "input_sha256": sha256(RAW), "config": str(CONFIG.relative_to(ROOT)), "reference": str(REFERENCE.relative_to(ROOT)),
              "three_identical_input_runs": sequence, "expected": expected, "actual": actual, "tolerances": tolerances,
              "matches": matches, "processing_p95_source": str(TIMING_ARTIFACT.relative_to(ROOT)) if timing_p95 is not None else "missing"}
    return report, result, diagnostics


def integral_mean(integrals: np.ndarray, row: int, a: int, b: int) -> float:
    a, b = int(np.clip(a, 0, integrals.shape[1] - 1)), int(np.clip(b, 0, integrals.shape[1] - 1))
    if b <= a:
        return float("nan")
    return float((integrals[row, b] - integrals[row, a]) / (b - a))


def raw_pair_features(pair: Any, band: int, diagnostics: Any, config: dict[str, Any], roi: tuple[int, int, int, int], sx: float) -> dict[str, Any]:
    gray = diagnostics.processed_gray
    response = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(config["processing"]["sobel_kernel"]))
    half = int(config["processing"]["scan_band_half_height_px"])
    profiles = cv2.blur(response, (1, 2 * half + 1))
    profiles = cv2.GaussianBlur(profiles, (int(config["processing"]["profile_gaussian_kernel"]), 1), 0)[diagnostics.scan_ys]
    intensity = cv2.blur(gray.astype(np.float32), (1, 2 * half + 1))[diagnostics.scan_ys]
    integrals = np.pad(np.cumsum(intensity, axis=1, dtype=np.float64), ((0, 0), (1, 0)))
    row, left, right = band, int(round(pair.left)), int(round(pair.right))
    width = right - left
    qcfg = config["pairing"]
    outside = max(int(qcfg["exterior_sample_min_px"]), int(round(width * float(qcfg["exterior_sample_width_scale"]))))
    margin = int(qcfg["interior_margin_px"])
    inside = integral_mean(integrals, row, left + margin, right - margin)
    left_exterior = integral_mean(integrals, row, left - outside, left)
    right_exterior = integral_mean(integrals, row, right, right + outside)
    outside_mean = 0.5 * (left_exterior + right_exterior)
    contrast_actual = float(np.clip(float(qcfg["contrast_score_offset"]) + (inside - outside_mean) / float(qcfg["contrast_normalization_gray"]), 0, 1))
    search = diagnostics.candidate_search[band] if band < len(diagnostics.candidate_search) else {}
    score_parts = {name: float(pair.score_parts.get(name, float("nan"))) for name in FEATURES}
    return {"processed_left": left, "processed_right": right, "left_original": float(roi[0] + left / sx),
            "right_original": float(roi[0] + right / sx), "center_original": float(roi[0] + pair.center / sx),
            "width_original": float(pair.width / sx), "left_signed_sobel_response": float(profiles[row, left]),
            "right_signed_sobel_response": float(-profiles[row, right]),
            "left_right_response_ratio": float(min(profiles[row, left], -profiles[row, right]) / max(max(profiles[row, left], -profiles[row, right]), 1e-6)),
            "pair_width_processed": float(width), "pair_width_original": float(pair.width / sx),
            "interior_mean": inside, "left_exterior_mean": left_exterior, "right_exterior_mean": right_exterior,
            "contrast_actual": contrast_actual, "reference_width_component": score_parts["width_reference"],
            "reference_center_component": score_parts["center_prior"], "score_parts": score_parts,
            "total_unary_score": float(pair.score), "from_guided_local_search": search.get("left_mode") == "local" and search.get("right_mode") == "local",
            "from_fallback": search.get("left_mode") == "global_fallback" or search.get("right_mode") == "global_fallback",
            "max_pairs_per_band": int(qcfg["max_pairs_per_band"]), "pair_count_after_max": len(diagnostics.all_pairs[band]),
            "pre_truncation_pair_count": None, "truncation_observable": False,
            "truncation_status": "unavailable_pre_truncation_not_exported"}


def pair_dict(pair: Any, band: int, tag: str, diagnostics: Any, config: dict[str, Any], result: Any, points: Any) -> dict[str, Any]:
    roi = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / roi[2]
    y_processed = float(pair.y)
    y_original = float(roi[1] + y_processed / (diagnostics.processed_gray.shape[0] / roi[3]))
    manual = manual_at(y_original, points)
    raw = raw_pair_features(pair, band, diagnostics, config, roi, sx)
    raw.update({"tag": tag, "band_index": int(band), "y_processed": y_processed, "y_original": y_original,
                "manual_left": manual[0], "manual_right": manual[1], "manual_center": manual[2], "manual_width": manual[3],
                "boundary_error_l1": abs(raw["left_original"] - manual[0]) + abs(raw["right_original"] - manual[1]),
                "left_signed_error": raw["left_original"] - manual[0], "right_signed_error": raw["right_original"] - manual[1],
                "center_signed_error": raw["center_original"] - manual[2], "width_signed_error": raw["width_original"] - manual[3]})
    return raw


def audit_bands(result: Any, diagnostics: Any, config: dict[str, Any]) -> list[dict[str, Any]]:
    points, records = manual_points(), []
    selected_by_band = {p.band_index: p for p in diagnostics.selected_pairs}
    roi = result.roi_original_px
    sy = diagnostics.processed_gray.shape[0] / roi[3]
    for band, (y, lefts, rights, pairs) in enumerate(zip(diagnostics.scan_ys, diagnostics.left_candidates, diagnostics.right_candidates, diagnostics.all_pairs)):
        y_original = float(roi[1] + float(y) / sy)
        ml, mr, mc, mw = manual_at(y_original, points)
        selected = selected_by_band.get(band)
        manual_near = min(pairs, key=lambda p: abs(roi[0] + p.left / (diagnostics.processed_gray.shape[1] / roi[2]) - ml) + abs(roi[0] + p.right / (diagnostics.processed_gray.shape[1] / roi[2]) - mr)) if pairs else None
        selected_record = pair_dict(selected, band, "selected", diagnostics, config, result, points) if selected else None
        near_record = pair_dict(manual_near, band, "manual_near", diagnostics, config, result, points) if manual_near else None
        l_original = [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in lefts]
        r_original = [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in rights]
        records.append({"band_index": band, "y_processed": float(y), "y_original": y_original,
                        "manual": {"left": ml, "right": mr, "center": mc, "width": mw},
                        "guided_reference": reference_at(config, y_original),
                        "left_candidates_processed": [int(x) for x in lefts], "right_candidates_processed": [int(x) for x in rights],
                        "left_candidates_original": l_original, "right_candidates_original": r_original,
                        "candidate_search": diagnostics.candidate_search[band], "pair_count": len(pairs),
                        "pairs": [pair_dict(p, band, "pair", diagnostics, config, result, points) for p in pairs],
                        "selected_pair": selected_record, "manual_near_pair": near_record,
                        "manual_near_both_sides_within_px": {
                            str(tolerance): bool(near_record and abs(near_record["left_signed_error"]) <= tolerance
                                                 and abs(near_record["right_signed_error"]) <= tolerance)
                            for tolerance in (15, 25, 40)
                        }})
    return records


def reference_at(config: dict[str, Any], y: float) -> dict[str, float]:
    from tank_vision.seam_width_reference import SeamWidthReference
    ref = SeamWidthReference.load(REFERENCE)
    center, width = ref.at(y, 4096, 3072)
    return {"center": float(center), "width": float(width), "left": float(center - width / 2), "right": float(center + width / 2)}


def classify(records: list[dict[str, Any]]) -> dict[str, Any]:
    categories = {name: 0 for name in ("left_candidate_missing", "right_candidate_missing", "both_candidates_present_but_pair_missing", "manual_near_pair_truncated", "manual_near_pair_lower_unary", "manual_near_pair_unary_not_lower_but_lost_by_dp", "selected_near_manual", "not_evaluable")}
    category_rows = []
    side = {name: 0 for name in ("only_left_wrong", "only_right_wrong", "both_wrong", "pair_width_too_narrow", "pair_width_too_wide", "center_left_biased", "center_right_biased")}
    lower_rows = []
    for row in records:
        selected, near = row["selected_pair"], row["manual_near_pair"]
        manual = row["manual"]
        left_near = any(abs(x - manual["left"]) <= PRESENCE_PX for x in row["left_candidates_original"])
        right_near = any(abs(x - manual["right"]) <= PRESENCE_PX for x in row["right_candidates_original"])
        if not left_near:
            category = "left_candidate_missing"
        elif not right_near:
            category = "right_candidate_missing"
        elif not near:
            category = "both_candidates_present_but_pair_missing"
        elif selected and abs(selected["left_original"] - manual["left"]) <= NEAR_PX and abs(selected["right_original"] - manual["right"]) <= NEAR_PX:
            category = "selected_near_manual"
        elif selected["total_unary_score"] > near["total_unary_score"] + 1e-12:
            category = "manual_near_pair_lower_unary"
            margin = {feature: WEIGHTS[feature] * (selected["score_parts"][feature] - near["score_parts"][feature]) for feature in FEATURES}
            lower_rows.append({"band_index": row["band_index"], "y_original": row["y_original"], "selected_score": selected["total_unary_score"], "manual_near_score": near["total_unary_score"], "total_score_margin": selected["total_unary_score"] - near["total_unary_score"], "weighted_feature_margin": margin, "ranked_contributors": sorted(margin, key=margin.get, reverse=True)})
        else:
            category = "manual_near_pair_unary_not_lower_but_lost_by_dp"
        categories[category] += 1
        category_rows.append({"band_index": row["band_index"], "y_original": row["y_original"], "category": category})
        if selected:
            le, re = selected["left_signed_error"], selected["right_signed_error"]
            wrong_l, wrong_r = abs(le) > NEAR_PX, abs(re) > NEAR_PX
            if wrong_l and not wrong_r: side["only_left_wrong"] += 1
            if wrong_r and not wrong_l: side["only_right_wrong"] += 1
            if wrong_l and wrong_r: side["both_wrong"] += 1
            if selected["width_signed_error"] < -NEAR_PX: side["pair_width_too_narrow"] += 1
            if selected["width_signed_error"] > NEAR_PX: side["pair_width_too_wide"] += 1
            if selected["center_signed_error"] < -NEAR_PX: side["center_left_biased"] += 1
            if selected["center_signed_error"] > NEAR_PX: side["center_right_biased"] += 1
    categories["manual_near_pair_truncated"] = None
    return {"thresholds_px": {"near": NEAR_PX, "candidate_presence": PRESENCE_PX}, "category_counts": categories,
            "category_rows": category_rows, "side_error_pattern_counts": side, "manual_near_pair_truncation": {"status": "unavailable", "reason": "StableSeamDiagnostics exports only the final max_pairs_per_band list"}, "lower_unary_rows": lower_rows}


def margin_report(classification: dict[str, Any]) -> dict[str, Any]:
    rows = classification["lower_unary_rows"]
    first = {feature: 0 for feature in FEATURES}
    second = {feature: 0 for feature in FEATURES}
    sums = {feature: 0.0 for feature in FEATURES}
    for row in rows:
        ranked = row["ranked_contributors"]
        if ranked:
            first[ranked[0]] += 1
        if len(ranked) > 1:
            second[ranked[1]] += 1
        for feature, value in row["weighted_feature_margin"].items(): sums[feature] += value
    return {"definition": "selected unary score minus manual-near unary score; positive means selected wins",
            "lower_unary_band_count": len(rows), "rows": rows, "first_contributor_counts": first,
            "second_contributor_counts": second, "sum_weighted_margin_by_feature": sums,
            "mean_weighted_margin_by_feature": {key: value / len(rows) if rows else None for key, value in sums.items()},
            "largest_sum_contributor": max(sums, key=sums.get) if rows else None}


def lofo(records: list[dict[str, Any]]) -> dict[str, Any]:
    output = {"definition": "actual returned pairs only; subtract one existing weighted unary contribution; no candidate generation or DP rerun", "features": {}}
    all_pairs = [p for row in records for p in row["pairs"]]
    for feature in LOFO_FEATURES:
        improved = top1 = 0
        original_ranks, new_ranks, errors, correct_values, wrong_values = [], [], [], [], []
        correct_components, wrong_components = [], []
        for row in records:
            near, selected = row["manual_near_pair"], row["selected_pair"]
            if not near or not selected: continue
            pairs = row["pairs"]
            def score(pair: dict[str, Any]) -> float:
                return pair["total_unary_score"] - WEIGHTS[feature] * pair["score_parts"][feature]
            ordered = sorted(pairs, key=score, reverse=True)
            near_key = (near["processed_left"], near["processed_right"])
            original_order = sorted(pairs, key=lambda p: p["total_unary_score"], reverse=True)
            original_rank = 1 + next((i for i, pair in enumerate(original_order) if (pair["processed_left"], pair["processed_right"]) == near_key), len(original_order))
            new_rank = 1 + next((i for i, pair in enumerate(ordered) if (pair["processed_left"], pair["processed_right"]) == near_key), len(ordered))
            original_ranks.append(original_rank); new_ranks.append(new_rank)
            improved += new_rank < original_rank; top1 += new_rank == 1
            error = near["boundary_error_l1"]
            errors.append(error)
            correct = abs(near["left_signed_error"]) <= NEAR_PX and abs(near["right_signed_error"]) <= NEAR_PX
            (correct_values if correct else wrong_values).append(score(near) - score(selected))
        for pair in all_pairs:
            is_correct = abs(pair["left_signed_error"]) <= NEAR_PX and abs(pair["right_signed_error"]) <= NEAR_PX
            (correct_components if is_correct else wrong_components).append(pair["score_parts"][feature])
        pair_feature_values = [p["score_parts"][feature] for p in all_pairs]
        pair_errors = [p["boundary_error_l1"] for p in all_pairs]
        output["features"][feature] = {"evaluated_bands": len(original_ranks), "manual_near_rank_improved": improved,
            "manual_near_top1_after_lofo": top1, "original_rank_stats": pstats(original_ranks), "lofo_rank_stats": pstats(new_ranks),
            "manual_near_error_stats": pstats(errors), "correct_vs_wrong_score_difference": {"correct_mean": float(np.mean(correct_values)) if correct_values else None, "wrong_mean": float(np.mean(wrong_values)) if wrong_values else None},
            "correct_pair_component_stats": pstats(correct_components), "wrong_pair_component_stats": pstats(wrong_components),
            "spearman_component_vs_boundary_l1_error": spearman(pair_feature_values, pair_errors),
            "all_returned_pairs_component_stats": pstats(pair_feature_values), "all_returned_pairs_error_stats": pstats(pair_errors)}
    return output


def signed_error_report(records: list[dict[str, Any]], roi_bottom: int) -> dict[str, Any]:
    fields = ("left_signed_error", "right_signed_error", "center_signed_error", "width_signed_error")
    def report(rows: list[dict[str, Any]]) -> dict[str, Any]:
        return {field: signed_stats(row["selected_pair"][field] for row in rows if row["selected_pair"]) for field in fields}
    bins = [(1536, 1800), (1800, 2100), (2100, 2400), (2400, 2700), (2700, roi_bottom)]
    return {"definition": "selected returned pair in original-image pixels; signed boundary/center/width error = selected - manual", "all": report(records),
            "by_y_range": [{"y_min": lo, "y_max": hi, "count": sum(1 for row in records if lo <= row["y_original"] < hi), "metrics": report([row for row in records if lo <= row["y_original"] < hi])} for lo, hi in bins]}


def draw_outputs(image: np.ndarray, records: list[dict[str, Any]], result: Any, classification: dict[str, Any]) -> None:
    canvas = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    colors = {"manual": (0, 255, 255), "selected": (0, 0, 255), "near": (255, 0, 255), "reference": (255, 255, 255)}
    category_by_band = {row["band_index"]: row["category"] for row in classification["category_rows"]}
    for row in records:
        y = int(round(row["y_original"])); m, s, n, ref = row["manual"], row["selected_pair"], row["manual_near_pair"], row["guided_reference"]
        pts = [(m["left"], m["right"], "manual"), (ref["left"], ref["right"], "reference")]
        if s: pts.append((s["left_original"], s["right_original"], "selected"))
        if n: pts.append((n["left_original"], n["right_original"], "near"))
        for left, right, label in pts:
            cv2.circle(canvas, (int(round(left)), y), 5, colors[label], -1); cv2.circle(canvas, (int(round(right)), y), 5, colors[label], -1)
        category = category_by_band.get(row["band_index"], "unknown")
        if category != "selected_near_manual":
            cv2.putText(canvas, category, (2500, max(20, y)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, category, (2500, max(20, y)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (result.roi_original_px[0], result.roi_original_px[1]), (result.roi_original_px[0] + result.roi_original_px[2], result.roi_original_px[1] + result.roi_original_px[3]), (128, 128, 128), 3)
    cv2.putText(canvas, "yellow manual / red selected / magenta manual-near / white reference", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(canvas, "yellow manual / red selected / magenta manual-near / white reference", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "selected_vs_manual_pair_overlay.png"), canvas)
    failures = [row for row in records if category_by_band.get(row["band_index"]) != "selected_near_manual"]
    if failures:
        indices = np.linspace(0, len(failures) - 1, min(6, len(failures))).astype(int)
        panels = []
        for index in indices:
            row = failures[int(index)]
            y = int(round(row["y_original"]))
            y0, y1 = max(result.roi_original_px[1], y - 55), min(result.roi_original_px[1] + result.roi_original_px[3], y + 55)
            crop = canvas[y0:y1, result.roi_original_px[0]:result.roi_original_px[0] + result.roi_original_px[2]].copy()
            crop = cv2.resize(crop, (640, max(60, round(crop.shape[0] * 640 / crop.shape[1]))), interpolation=cv2.INTER_AREA)
            cv2.putText(crop, f"y={y} {category_by_band[row['band_index']]}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
            panels.append(crop)
        while len(panels) < 6: panels.append(np.zeros_like(panels[0]))
        montage = np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:6])])
    else:
        montage = cv2.resize(canvas, (1280, 960), interpolation=cv2.INTER_AREA)
    cv2.putText(montage, "guided unary failure bands; yellow manual / red selected / magenta manual-near / white reference", (10, montage.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "unary_failure_montage.jpg"), montage, [int(cv2.IMWRITE_JPEG_QUALITY), 92])


def decision(records: list[dict[str, Any]], classification: dict[str, Any], margins: dict[str, Any], lofo_report: dict[str, Any]) -> tuple[str, dict[str, Any]]:
    lower = classification["category_counts"]["manual_near_pair_lower_unary"]
    dp_lost = classification["category_counts"]["manual_near_pair_unary_not_lower_but_lost_by_dp"]
    missing = classification["category_counts"]["left_candidate_missing"] + classification["category_counts"]["right_candidate_missing"] + classification["category_counts"]["both_candidates_present_but_pair_missing"]
    top = margins.get("largest_sum_contributor")
    top_lofo = max(((f, v["manual_near_rank_improved"]) for f, v in lofo_report["features"].items()), key=lambda item: item[1], default=(None, 0))
    if lower >= 8 and top and margins["first_contributor_counts"].get(top, 0) >= max(3, math.ceil(lower * 0.35)) and top_lofo[1] >= max(3, math.ceil(lower * 0.10)):
        choice = "A"
        next_change = {"choice": choice, "unique_modification_object": f"existing unary component: {top}", "why": "It is the largest aggregate positive selected-minus-manual-near margin and its removal most often improves local rank in the frozen returned-pair set.", "do_not_modify": ["candidate generation", "guided radius/Top-K", "hard width gate", "DP", "fit", "geometry gate", "temporal", "ROS"], "experiment": "One offline A/B on the same frozen raw.bmp: current score versus this component removed, using only returned pairs; no gate change.", "success_standard": "manual-near local rank and boundary identity improve without reducing strict gate criteria.", "stop_standard": "no repeatable rank improvement or any need to lower a gate.", "performance_budget_ms": "no production runtime change; later implementation must remain within current guided processing P95 budget."}
    elif missing + dp_lost > lower:
        choice = "C"
        next_change = {"choice": choice, "unique_modification_object": "none in production; first add read-only pre-truncation/DP provenance export", "why": "The dominant loss is before unary victory or after unary ranking, not a repeatable unary margin.", "do_not_modify": ["all production algorithm/configuration/thresholds/ROS"], "experiment": "Read-only provenance export on the same frozen frame.", "success_standard": "identify whether pair truncation or DP causes the loss.", "stop_standard": "do not change scores or gates before provenance is available.", "performance_budget_ms": "diagnostic-only; zero production impact."}
    else:
        choice = "B"
        next_change = {"choice": choice, "unique_modification_object": "one new identity feature: symmetric outside-to-inside step consistency", "mathematical_definition": "left_step=interior_mean-left_exterior_mean; right_step=interior_mean-right_exterior_mean; identity_score=min(max(left_step,0),max(right_step,0))/(max(abs(left_step),abs(right_step))+epsilon)", "why": "Current returned-pair unary components do not provide a reliable single-feature separation of correct and wrong boundary identities.", "do_not_modify": ["existing weights", "candidate generation", "guided radius/Top-K", "hard width gate", "DP", "fit", "geometry gate", "temporal", "ROS"], "experiment": "Offline A/B feature-only ranking on the frozen returned pairs; do not implement or change gates until this diagnostic is reviewed.", "success_standard": "correct pairs become more separable while strict geometry gates remain unchanged.", "stop_standard": "no clear separation or any benefit requiring gate reduction.", "performance_budget_ms": "feature must fit inside current guided processing P95 budget (~74.65 ms total baseline)."}
    return choice, next_change


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{field: json.dumps(row.get(field), ensure_ascii=False) if isinstance(row.get(field), (dict, list)) else row.get(field) for field in fields} for row in rows])


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite existing output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    evidence = verify_inputs()
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None: raise RuntimeError("cannot read frozen raw.bmp")
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    base, result, diagnostics = baseline_report(image, evidence)
    dump(OUT / "baseline_reproduction.json", base)
    if base["status"] != "matched":
        return 2
    records = audit_bands(result, diagnostics, config)
    classification = classify(records)
    margins = margin_report(classification)
    lofo_report = lofo(records)
    signed = signed_error_report(records, result.roi_original_px[1] + result.roi_original_px[3])
    choice, next_change = decision(records, classification, margins, lofo_report)
    dump(OUT / "scan_band_pair_audit.json", {"baseline_status": base["status"], "record_count": len(records), "records": records})
    write_csv(OUT / "scan_band_pair_audit.csv", records, ["band_index", "y_processed", "y_original", "manual", "guided_reference", "left_candidates_original", "right_candidates_original", "candidate_search", "pair_count", "pairs", "selected_pair", "manual_near_pair"])
    dump(OUT / "unary_margin_breakdown.json", margins)
    write_csv(OUT / "unary_margin_breakdown.csv", margins["rows"], ["band_index", "y_original", "selected_score", "manual_near_score", "total_score_margin", "weighted_feature_margin", "ranked_contributors"])
    dump(OUT / "leave_one_feature_out.json", lofo_report)
    dump(OUT / "signed_error_by_y.json", signed)
    dump(OUT / "failure_classification.json", classification)
    draw_outputs(image, records, result, classification)
    summary = {"baseline_status": base["status"], "record_count": len(records), "decision": choice,
               "category_counts": classification["category_counts"], "side_error_pattern_counts": classification["side_error_pattern_counts"],
               "manual_near_pair_available_count": sum(bool(r["manual_near_pair"]) for r in records),
               "manual_near_pair_available_rate": sum(bool(r["manual_near_pair"]) for r in records) / max(len(records), 1),
               "largest_unary_margin_contributor": margins["largest_sum_contributor"], "first_contributor_counts": margins["first_contributor_counts"],
               "lofo_rank_improvement": {f: v["manual_near_rank_improved"] for f, v in lofo_report["features"].items()},
               "signed_error_all": signed["all"], "input_sha256_before": evidence["sha256"],
               "input_sha256_after": {key: sha256(path) for key, path in {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": TRACKER}.items()}}
    summary["input_hashes_unchanged"] = summary["input_sha256_before"] == summary["input_sha256_after"]
    dump(OUT / "diagnostic_summary.json", summary)
    dump(OUT / "next_single_change.json", next_change)
    if not summary["input_hashes_unchanged"]: raise RuntimeError("input hash changed during diagnostic")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"diagnostic failed: {exc}", file=sys.stderr)
        raise
