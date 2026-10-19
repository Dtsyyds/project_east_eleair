#!/usr/bin/env python3
"""S1-P2E1: provenance audit for the real guided candidate and DP path.

The production tracker owns candidate generation, scoring, and DP.  This
script only enables its default-off provenance hook and consumes the resulting
trace; it does not create a second candidate generator or DP implementation.
"""

from __future__ import annotations

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

from scripts.diagnose_guided_pair_unary import (  # noqa: E402
    ANNOTATION, CONFIG, EXPECTED, MANIFEST, RAW, REFERENCE, TIMING_ARTIFACT,
    manual_at, manual_points, pstats, sha256,
)
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402

OUT = ROOT / "output/s1_p2e1_dp_provenance"
ROI_BOTTOM = 2826
TOLERANCES = (15, 25, 40)
TRACKER_SHA256_BEFORE_TRACE_HOOK = "088efdde35c8f5cbff1323384aae3b7468f3f6141acb18907288c5d759f60466"


def dump(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def verify_inputs() -> dict[str, Any]:
    paths = {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": ROOT / "tank_vision/stable_seam_tracker.py", "manifest": MANIFEST}
    if any(not path.is_file() for path in paths.values()):
        raise RuntimeError("one or more fixed inputs are missing")
    hashes = {key: sha256(path) for key, path in paths.items() if key in EXPECTED}
    mismatch = {key: {"expected": EXPECTED[key], "actual": value} for key, value in hashes.items() if value != EXPECTED[key]}
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    annotation = json.loads(ANNOTATION.read_text(encoding="utf-8"))
    fixed_mismatch = {key: value for key, value in mismatch.items() if key != "tracker"}
    if fixed_mismatch or manifest.get("status") != "frozen" or not manifest.get("input_sha256_matches") or not manifest.get("human_overlay_approved") or annotation.get("input_sha256") != EXPECTED["raw"]:
        raise RuntimeError(f"fixed input verification failed: {mismatch}")
    return {"paths": {key: str(path.relative_to(ROOT)) for key, path in paths.items()}, "sha256": hashes,
            "manifest_status": manifest.get("status"), "manifest_input_sha256_matches": manifest.get("input_sha256_matches"),
            "manifest_human_overlay_approved": manifest.get("human_overlay_approved"), "annotation_point_count": annotation.get("point_count"),
            "tracker_sha256_before_trace_hook": TRACKER_SHA256_BEFORE_TRACE_HOOK, "tracker_sha256_current": hashes["tracker"]}


def core_result(result: Any) -> dict[str, Any]:
    value = result.to_dict()
    value.pop("processing_ms", None)
    value.pop("stage_timings_ms", None)
    return value


def pair_signature(diag: Any) -> list[list[dict[str, Any]]]:
    return [[{"band_index": p.band_index, "y": p.y, "left": p.left, "right": p.right, "score": p.score, "score_parts": dict(p.score_parts)} for p in row] for row in diag.all_pairs]


def path_signature(diag: Any) -> list[dict[str, Any]]:
    return [{"band_index": p.band_index, "left": p.left, "right": p.right, "score": p.score} for p in diag.selected_pairs]


def fit_from_diagnostics(tracker: StableSeamTracker, result: Any, diag: Any) -> dict[str, Any] | None:
    sx = diag.processed_gray.shape[1] / result.roi_original_px[2]
    sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    fit = tracker._fit(diag.selected_pairs, result.roi_original_px, sx, sy, 4096, 3072)
    if fit is None:
        return None
    return {"center": [float(value) for value in fit["center"]], "width": [float(value) for value in fit["width"]],
            "inliers": [bool(value) for value in fit["inliers"]], "residual_px": float(fit["residual_px"])}


def run_baseline_and_trace(image: np.ndarray, evidence: dict[str, Any]) -> tuple[dict[str, Any], StableSeamTracker, Any, Any, Any]:
    standard = StableSeamTracker(CONFIG, REFERENCE)
    standard_results = []
    standard_diag = None
    for index in range(3):
        value = standard.process(image, diagnostics=(index == 0), provenance=False)
        if index == 0:
            standard_diag = standard.last_diagnostics
        standard_results.append(core_result(value))
    trace_false = StableSeamTracker(CONFIG, REFERENCE)
    trace_true = StableSeamTracker(CONFIG, REFERENCE)
    false_result = trace_false.process(image, diagnostics=True, provenance=False)
    true_result = trace_true.process(image, diagnostics=True, provenance=True)
    false_diag, true_diag = trace_false.last_diagnostics, trace_true.last_diagnostics
    if false_diag is None or true_diag is None or trace_true.last_provenance is None:
        raise RuntimeError("tracker provenance or diagnostics was not produced")
    fit_false = fit_from_diagnostics(trace_false, false_result, false_diag)
    fit_true = trace_true.last_provenance["result"]
    trace_equal = {
        "candidate_positions_and_strengths": np.array_equal(false_diag.edge_response, true_diag.edge_response) and np.array_equal(false_diag.scan_ys, true_diag.scan_ys) and all(np.array_equal(a, b) for a, b in zip(false_diag.left_candidates, true_diag.left_candidates)) and all(np.array_equal(a, b) for a, b in zip(false_diag.right_candidates, true_diag.right_candidates)),
        "pair_order_and_scores": pair_signature(false_diag) == pair_signature(true_diag),
        "selected_path": path_signature(false_diag) == path_signature(true_diag),
        "fit_coefficients": fit_false is not None and np.allclose(fit_false["center"], fit_true["fit_center_coefficients"], rtol=0, atol=0) and np.allclose(fit_false["width"], fit_true["fit_width_coefficients"], rtol=0, atol=0),
        "geometry_metrics": false_result.coverage == true_result.coverage and false_result.effective_span == true_result.effective_span and false_result.fit_inlier_ratio == true_result.fit_inlier_ratio and false_result.fit_residual_px == true_result.fit_residual_px,
        "detected": false_result.detected == true_result.detected,
        "tracking_state": false_result.tracking_state.name == true_result.tracking_state.name,
        "failure_reasons": false_result.failure_reasons == true_result.failure_reasons,
    }
    expected = {"selected_path": 116, "scan_bands": 117, "coverage": 116 / 117, "first_inlier": 0.7931034482758621,
                "states": ["ACQUIRING", "ACQUIRING", "TRACKING"], "center_p95": 44.14758855034813,
                "left_p95": 59.276740922960016, "right_p95": 47.91756560334744, "width_p95": 70.42111032039196,
                "processing_p95": 74.64755025000001}
    timing = json.loads(TIMING_ARTIFACT.read_text(encoding="utf-8"))
    points = manual_points()
    sx = false_diag.processed_gray.shape[1] / false_result.roi_original_px[2]
    sy = false_diag.processed_gray.shape[0] / false_result.roi_original_px[3]
    measured_errors = []
    for pair in false_diag.selected_pairs:
        y = false_result.roi_original_px[1] + pair.y / sy
        manual = manual_at(y, points)
        left = false_result.roi_original_px[0] + pair.left / sx
        right = false_result.roi_original_px[0] + pair.right / sx
        measured_errors.append(((left - manual[0]), (right - manual[1]), ((left + right) / 2 - manual[2]), (right - left - manual[3])))
    actual = {"selected_path": len(standard_diag.selected_pairs), "scan_bands": len(standard_diag.scan_ys),
              "coverage": standard_results[0]["coverage"], "first_inlier": standard_results[0]["fit_inlier_ratio"],
              "states": [item["tracking_state"] for item in standard_results],
              "center_p95": pstats(abs(item[2]) for item in measured_errors)["p95"], "left_p95": pstats(abs(item[0]) for item in measured_errors)["p95"],
              "right_p95": pstats(abs(item[1]) for item in measured_errors)["p95"], "width_p95": pstats(abs(item[3]) for item in measured_errors)["p95"],
              "processing_p95": timing["guided"]["total_ms"]["p95"]}
    matches = {key: actual[key] == value if isinstance(value, list) else abs(actual[key] - value) <= (0.002 if key in ("coverage", "first_inlier") else 0.5) for key, value in expected.items()}
    report = {"status": "matched" if all(matches.values()) and all(trace_equal.values()) else "baseline_mismatch", "evidence": evidence,
              "three_standard_runs": standard_results, "expected": expected, "actual": actual, "matches": matches,
              "trace_false_true_field_equality": trace_equal, "trace_false_result": core_result(false_result), "trace_true_result": core_result(true_result),
              "fit_false_recomputed_from_real_fit": fit_false, "processing_p95_source": str(TIMING_ARTIFACT.relative_to(ROOT))}
    return report, trace_true, true_result, true_diag, trace_true.last_provenance


def pair_records(diag: Any) -> dict[tuple[int, int, int], Any]:
    return {(p.band_index, int(round(p.left)), int(round(p.right))): p for row in diag.all_pairs for p in row}


def record_pair(pair: Any, diag: Any, result: Any, manual: tuple[float, float, float, float]) -> dict[str, Any]:
    sx = diag.processed_gray.shape[1] / result.roi_original_px[2]
    sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    y = result.roi_original_px[1] + pair.y / sy
    left = result.roi_original_px[0] + pair.left / sx
    right = result.roi_original_px[0] + pair.right / sx
    return {"pair_id": f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}", "band_index": int(pair.band_index), "y_original": float(y),
            "left": float(left), "right": float(right), "center": float((left + right) / 2), "width": float(right - left),
            "unary_score": float(pair.score), "score_parts": dict(pair.score_parts),
            "manual": {"left": manual[0], "right": manual[1], "center": manual[2], "width": manual[3]},
            "left_error": float(left - manual[0]), "right_error": float(right - manual[1]),
            "center_error": float((left + right) / 2 - manual[2]), "width_error": float(right - left - manual[3])}


def tolerance_reconciliation(diag: Any, result: Any, provenance: dict[str, Any]) -> dict[str, Any]:
    points = manual_points()
    selected_by_band = {p.band_index: p for p in diag.selected_pairs}
    stage_by_band = {row["band_index"]: row for row in provenance["candidate_stage_trace"]}
    final_pairs = pair_records(diag)
    outputs = {str(tolerance): {key: 0 for key in ("left_candidate_missing", "right_candidate_missing", "both_candidates_present_but_pair_missing", "manual_near_pair_truncated", "selected_near_manual", "manual_near_pair_lower_unary", "manual_near_pair_unary_not_lower_but_lost_by_dp", "not_evaluable")} for tolerance in TOLERANCES}
    rows = []
    sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    sx = diag.processed_gray.shape[1] / result.roi_original_px[2]
    for band, y_processed in enumerate(diag.scan_ys):
        y = result.roi_original_px[1] + float(y_processed) / sy
        manual = manual_at(y, points)
        lefts = [result.roi_original_px[0] + int(x) / sx for x in diag.left_candidates[band]]
        rights = [result.roi_original_px[0] + int(x) / sx for x in diag.right_candidates[band]]
        post = [record_pair(p, diag, result, manual) for p in diag.all_pairs[band]]
        pre = stage_by_band[band]["pair_stages"]["pair_hard_gate_after"]
        pre_near = [item for item in pre if abs(result.roi_original_px[0] + item["left_processed"] / sx - manual[0]) <= 25 and abs(result.roi_original_px[0] + item["right_processed"] / sx - manual[1]) <= 25]
        row = {"band_index": int(band), "y_original": float(y), "manual": {"left": manual[0], "right": manual[1]}, "tolerances": {}}
        for tolerance in TOLERANCES:
            left_present = any(abs(value - manual[0]) <= tolerance for value in lefts)
            right_present = any(abs(value - manual[1]) <= tolerance for value in rights)
            near = [item for item in post if abs(item["left_error"]) <= tolerance and abs(item["right_error"]) <= tolerance]
            selected = selected_by_band.get(band)
            selected_near = selected is not None and abs(result.roi_original_px[0] + selected.left / sx - manual[0]) <= tolerance and abs(result.roi_original_px[0] + selected.right / sx - manual[1]) <= tolerance
            if not left_present: category = "left_candidate_missing"
            elif not right_present: category = "right_candidate_missing"
            elif not near and pre_near and tolerance == 25: category = "manual_near_pair_truncated"
            elif not near: category = "both_candidates_present_but_pair_missing"
            elif selected_near: category = "selected_near_manual"
            else:
                best = min(near, key=lambda item: abs(item["left_error"]) + abs(item["right_error"]))
                selected_record = next((item for item in post if selected is not None and item["pair_id"] == f"b{band}:l{int(round(selected.left))}:r{int(round(selected.right))}"), None)
                category = "manual_near_pair_lower_unary" if selected_record and selected_record["unary_score"] > best["unary_score"] else "manual_near_pair_unary_not_lower_but_lost_by_dp"
            outputs[str(tolerance)][category] += 1
            row["tolerances"][str(tolerance)] = {"left_candidate_present": left_present, "right_candidate_present": right_present, "manual_near_pair_in_final_list": bool(near), "selected_near_manual": selected_near, "category": category}
        rows.append(row)
    return {"definitions": {"candidate_presence": "each side has a retained side candidate within the stated tolerance", "manual_near_pair": "a final retained pair has both boundary errors within the stated tolerance", "main_conclusion_tolerance": 25}, "category_counts": outputs, "counts_sum": {key: sum(value.values()) for key, value in outputs.items()}, "rows": rows, "p2e0_reconciliation": {"manual_near_pair_final_list": 116, "bands": 117, "manual_near_pair_definition": "minimum L1 left+right error over final returned pairs (not bilateral tolerance)", "left_candidate_missing_at_40px": 7, "right_candidate_missing_at_40px": 5, "explanation": "These are different predicates: 116/117 is pair-level L1 existence, while 7/5 are independent side-level 40px presence counts; they are not intended to sum or be mutually exclusive."}}


def predecessor_traces(provenance: dict[str, Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    states = provenance["dp_state_trace"]
    flat = []
    for state in states:
        for candidate in state.get("predecessor_candidates", []):
            flat.append({"current_pair_id": state["pair_id"], "current_band": state["band_index"], **candidate})
    return states, flat


def backward_scores(tracker: StableSeamTracker, result: Any, diag: Any) -> tuple[dict[tuple[int, int, int], float], dict[tuple[int, int, int], dict[str, Any] | None]]:
    all_pairs = pair_records(diag)
    by_band: dict[int, list[Any]] = {}
    for pair in all_pairs.values(): by_band.setdefault(pair.band_index, []).append(pair)
    suffix, next_choice = {}, {}
    max_gap = int(tracker.config["path"]["maximum_missing_run"])
    sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    sx = diag.processed_gray.shape[1] / result.roi_original_px[2]
    for band in sorted(by_band, reverse=True):
        for current in by_band[band]:
            best = 0.0
            choice = None
            for next_band in range(band + 1, min(max(by_band) + 1, band + max_gap + 2)):
                for nxt in by_band.get(next_band, []):
                    y_original = result.roi_original_px[1] + nxt.y / sy
                    _, ref_width = tracker.reference.at(y_original, 4096, 3072)
                    detail = tracker._transition_detail(current, nxt, next_band - band - 1, max(float(ref_width) * sx, 1.0))
                    if detail["passed"]:
                        candidate = suffix[(next_band, int(round(nxt.left)), int(round(nxt.right)))] - detail["total"]
                        if choice is None or candidate > best:
                            best, choice = candidate, {"next_pair_id": tracker._pair_id(nxt), "transition": detail, "candidate_after_transition": float(candidate)}
            key = (band, int(round(current.left)), int(round(current.right)))
            suffix[key] = float(current.score + best)
            next_choice[key] = choice
    return suffix, next_choice


def forced_counterfactual(tracker: StableSeamTracker, result: Any, diag: Any, provenance: dict[str, Any]) -> dict[str, Any]:
    points = manual_points()
    states, _ = predecessor_traces(provenance)
    state_map = {(s["band_index"], int(round(s["left"])), int(round(s["right"]))): s for s in states}
    suffix, next_choice = backward_scores(tracker, result, diag)
    selected_by_band = {p.band_index: p for p in diag.selected_pairs}
    production_total = max(float(s["cumulative_score"]) for s in states)
    rows = []
    for band, pairs in enumerate(diag.all_pairs):
        y = result.roi_original_px[1] + pairs[0].y / (diag.processed_gray.shape[0] / result.roi_original_px[3]) if pairs else None
        if y is None or not (1536 <= y < ROI_BOTTOM): continue
        manual = manual_at(float(y), points)
        left_present_40 = any(abs(result.roi_original_px[0] + x / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[0]) <= 40 for x in diag.left_candidates[band])
        right_present_40 = any(abs(result.roi_original_px[0] + x / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[1]) <= 40 for x in diag.right_candidates[band])
        if not left_present_40 or not right_present_40 or not pairs: continue
        target = min(pairs, key=lambda p: abs(result.roi_original_px[0] + p.left / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[0]) + abs(result.roi_original_px[0] + p.right / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[1]))
        selected_error = None if selected_by_band.get(band) is None else (abs(result.roi_original_px[0] + selected_by_band[band].left / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[0]), abs(result.roi_original_px[0] + selected_by_band[band].right / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[1]))
        if selected_error is not None and selected_error[0] <= 25 and selected_error[1] <= 25: continue
        target_key = (band, int(round(target.left)), int(round(target.right)))
        target_state = state_map[target_key]
        selected = selected_by_band.get(band)
        selected_key = None if selected is None else (band, int(round(selected.left)), int(round(selected.right)))
        target_left_error = abs(result.roi_original_px[0] + target.left / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[0])
        target_right_error = abs(result.roi_original_px[0] + target.right / (diag.processed_gray.shape[1] / result.roi_original_px[2]) - manual[1])
        p2e0_category = "manual_near_pair_lower_unary" if selected is not None and float(selected.score) > float(target.score) else "manual_near_pair_unary_not_lower_but_lost_by_dp"
        if selected_key == target_key:
            rows.append({"band_index": band, "y_original": float(y), "target_manual_near_pair_id": tracker._pair_id(target), "selected_pair_id": tracker._pair_id(selected), "target_within_main_25px": target_left_error <= 25 and target_right_error <= 25, "production_best_total_score": production_total, "forced_total_score": production_total, "total_score_loss": 0.0, "current_unary_loss": 0.0, "forward_transition_loss": 0.0, "backward_transition_loss": 0.0, "skip_missing_loss": 0.0, "cumulative_history_loss": 0.0, "transition_component_margin": {"center_cost": 0.0, "width_cost": 0.0, "left_cost": 0.0, "right_cost": 0.0, "skip_cost": 0.0}, "hard_transition_unreachable": False, "target_best_predecessor": None, "selected_best_predecessor": None, "target_best_successor": None, "selected_best_successor": None, "classification": "selected_already_l1_nearest", "p2e0_category": p2e0_category, "forced_evaluation": "not_applicable_selected_is_target"})
            continue
        suffix_after = 0.0 if next_choice.get(target_key) is None else float(next_choice[target_key]["candidate_after_transition"])
        forced_total = float(target_state["cumulative_score"] + suffix_after)
        target_best = target_state.get("best_predecessor")
        selected_state = state_map.get(selected_key) if selected_key else None
        selected_best = selected_state.get("best_predecessor") if selected_state else None
        target_next = next_choice.get(target_key)
        selected_next = next_choice.get(selected_key) if selected_key else None
        current_unary_loss = 0.0 if selected is None else float(selected.score - target.score)
        target_forward_transition = 0.0 if target_best is None else float(target_best["transition"]["total"])
        selected_forward_transition = 0.0 if selected_best is None else float(selected_best["transition"]["total"])
        target_backward_transition = 0.0 if target_next is None else float(target_next["transition"]["total"])
        selected_backward_transition = 0.0 if selected_next is None else float(selected_next["transition"]["total"])
        history_loss = 0.0 if target_best is None or selected_best is None else float(selected_best["previous_cumulative_score"] - target_best["previous_cumulative_score"])
        forward_transition_loss = max(0.0, target_forward_transition - selected_forward_transition)
        backward_transition_loss = max(0.0, target_backward_transition - selected_backward_transition)
        skip_loss = 0.0
        for item in (target_best, target_next):
            if item is not None: skip_loss += float(item["transition"].get("skip_cost", 0.0))
        for item in (selected_best, selected_next):
            if item is not None: skip_loss -= float(item["transition"].get("skip_cost", 0.0))
        component_margin = {}
        for component in ("center_cost", "width_cost", "left_cost", "right_cost", "skip_cost"):
            target_component = sum(float(item["transition"].get(component, 0.0)) for item in (target_best, target_next) if item is not None)
            selected_component = sum(float(item["transition"].get(component, 0.0)) for item in (selected_best, selected_next) if item is not None)
            component_margin[component] = target_component - selected_component
        positive = {"current_unary": current_unary_loss, "forward_transition": forward_transition_loss, "backward_transition": backward_transition_loss, "skip_missing": max(0.0, skip_loss), "cumulative_history": max(0.0, history_loss)}
        if target_best is not None and target_best.get("predecessor_candidates") is not None: pass
        if target_best is None and band > 0: category = "unreachable_from_previous"
        elif target_next is None and band < max(s["band_index"] for s in states): category = "unreachable_to_next"
        elif positive["current_unary"] > max(positive["forward_transition"], positive["backward_transition"], positive["cumulative_history"], 1e-9): category = "lost_by_unary"
        elif positive["cumulative_history"] > max(positive["forward_transition"], positive["backward_transition"], 1e-9): category = "cumulative_history_dominant"
        elif positive["forward_transition"] + positive["backward_transition"] > max(positive["current_unary"], positive["cumulative_history"], 1e-9): category = "transition_cost_dominant"
        else: category = "ambiguous"
        rows.append({"band_index": band, "y_original": float(y), "target_manual_near_pair_id": tracker._pair_id(target), "selected_pair_id": None if selected is None else tracker._pair_id(selected), "target_within_main_25px": target_left_error <= 25 and target_right_error <= 25, "production_best_total_score": production_total, "forced_total_score": forced_total, "total_score_loss": production_total - forced_total, "current_unary_loss": current_unary_loss, "forward_transition_loss": forward_transition_loss, "backward_transition_loss": backward_transition_loss, "skip_missing_loss": max(0.0, skip_loss), "cumulative_history_loss": max(0.0, history_loss), "transition_component_margin": component_margin, "hard_transition_unreachable": target_best is None or (target_next is None and band < max(s["band_index"] for s in states)), "target_best_predecessor": target_best, "selected_best_predecessor": selected_best, "target_best_successor": target_next, "selected_best_successor": selected_next, "classification": category, "p2e0_category": p2e0_category})
    counts = {key: sum(row["classification"] == key for row in rows) for key in ("lost_by_unary", "unreachable_from_previous", "unreachable_to_next", "transition_cost_dominant", "cumulative_history_dominant", "pair_top_k_truncated", "candidate_stage_missing", "ambiguous", "selected_already_l1_nearest")}
    p2e0_dp_rows = [row for row in rows if row["p2e0_category"] == "manual_near_pair_unary_not_lower_but_lost_by_dp"]
    main_25_rows = [row for row in rows if row["target_within_main_25px"]]
    p2e0_counts = {key: sum(row["classification"] == key for row in p2e0_dp_rows) for key in ("lost_by_unary", "unreachable_from_previous", "unreachable_to_next", "transition_cost_dominant", "cumulative_history_dominant", "pair_top_k_truncated", "candidate_stage_missing", "ambiguous", "selected_already_l1_nearest")}
    main_25_counts = {key: sum(row["classification"] == key for row in main_25_rows) for key in ("lost_by_unary", "unreachable_from_previous", "unreachable_to_next", "transition_cost_dominant", "cumulative_history_dominant", "pair_top_k_truncated", "candidate_stage_missing", "ambiguous", "selected_already_l1_nearest")}
    return {"main_tolerance_px": 25, "production_global_best_score": production_total, "rows": rows, "main_25_rows": main_25_rows, "classification_counts": counts, "main_25_classification_counts": main_25_counts, "p2e0_14_dp_rows": p2e0_dp_rows, "p2e0_14_dp_classification_counts": p2e0_counts, "note": "Forced paths are offline counterfactuals over the actual returned pair set; they are never fed back to tracker output. The 14-band legacy P2E0 count uses its stated nearest-pair L1 definition; main 25px bilateral rows are reported separately."}


def signed_record(diag: Any, result: Any) -> list[dict[str, Any]]:
    points = manual_points(); sx = diag.processed_gray.shape[1] / result.roi_original_px[2]; sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    out = []
    for pair in diag.selected_pairs:
        y = result.roi_original_px[1] + pair.y / sy; manual = manual_at(y, points)
        left = result.roi_original_px[0] + pair.left / sx; right = result.roi_original_px[0] + pair.right / sx
        out.append({"band_index": pair.band_index, "y_original": float(y), "left_error": float(left - manual[0]), "right_error": float(right - manual[1]), "center_error": float((left + right) / 2 - manual[2]), "width_error": float(right - left - manual[3])})
    return out


def trusted_intervals(diag: Any, result: Any) -> dict[str, Any]:
    rows = signed_record(diag, result)
    intervals = {"roi_common_1536_2826": (1536, 2826), "upper_trusted_candidate_1536_2400": (1536, 2400), "unstable_2400_2700": (2400, 2700), "lower_trusted_candidate_2700_2826": (2700, 2826), "roi_excluding_2400_2700": (1536, 2826)}
    output = {}
    for name, (lo, hi) in intervals.items():
        selected = [r for r in rows if lo <= r["y_original"] < hi and not (name == "roi_excluding_2400_2700" and 2400 <= r["y_original"] < 2700)]
        eligible = [y for y in diag.scan_ys if lo <= result.roi_original_px[1] + y / (diag.processed_gray.shape[0] / result.roi_original_px[3]) < hi and not (name == "roi_excluding_2400_2700" and 2400 <= result.roi_original_px[1] + y / (diag.processed_gray.shape[0] / result.roi_original_px[3]) < 2700)]
        metrics = {}
        for key in ("left_error", "right_error", "center_error", "width_error"):
            metrics[key] = {"absolute_p95": float(np.percentile(np.abs([r[key] for r in selected]), 95)) if selected else None, "signed_mean": float(np.mean([r[key] for r in selected])) if selected else None}
        output[name] = {"y_range": [lo, hi], "selected_count": len(selected), "eligible_scan_bands": len(eligible), "coverage": len(selected) / max(len(eligible), 1), "metrics": metrics, "meets_requested_gate": bool(selected and metrics["center_error"]["absolute_p95"] <= 25 and metrics["left_error"]["absolute_p95"] <= 30 and metrics["right_error"]["absolute_p95"] <= 30 and metrics["width_error"]["absolute_p95"] <= 30 and len(selected) / max(len(eligible), 1) >= 0.85)}
    return {"criteria": {"center_absolute_p95_px": 25, "left_right_absolute_p95_px": 30, "width_absolute_p95_px": 30, "coverage": 0.85}, "intervals": output, "conclusion": "No interval is adopted; this is read-only evaluation."}


def candidate_retention(diag: Any, result: Any, provenance: dict[str, Any]) -> dict[str, Any]:
    points = manual_points(); sx = diag.processed_gray.shape[1] / result.roi_original_px[2]; sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    stage_names = ("all_local_signed_edge_peaks", "guided_window_peaks", "separation_filtered_peaks", "top_k_after")
    output = {}
    for tolerance in (15, 25, 40):
        side = {"left": {}, "right": {}}
        pair = {"valid_before_pair_top_k": 0, "retained_after_pair_top_k": 0, "retention_rate": None}
        for stage in stage_names:
            side["left"][stage] = 0; side["right"][stage] = 0
        for band, row in enumerate(provenance["candidate_stage_trace"]):
            y = result.roi_original_px[1] + row["y_processed"] / sy; manual = manual_at(y, points)
            for side_name, manual_x in (("left", manual[0]), ("right", manual[1])):
                for stage in stage_names:
                    items = row.get(stage, {}).get(side_name, [])
                    if any(abs(result.roi_original_px[0] + item["x"] / sx - manual_x) <= tolerance for item in items): side[side_name][stage] += 1
            pre = row["pair_stages"]["pair_hard_gate_after"]
            post = row["pair_stages"]["pair_after_max_pairs_per_band"]
            def correct(items: list[dict[str, Any]]) -> bool:
                return any(abs(result.roi_original_px[0] + item["left_processed"] / sx - manual[0]) <= tolerance and abs(result.roi_original_px[0] + item["right_processed"] / sx - manual[1]) <= tolerance for item in items)
            pair["valid_before_pair_top_k"] += correct(pre); pair["retained_after_pair_top_k"] += correct(post)
        pair["retention_rate"] = pair["retained_after_pair_top_k"] / max(pair["valid_before_pair_top_k"], 1)
        output[str(tolerance)] = {"side_candidate_band_counts": side, "correct_pair_band_counts": pair, "band_denominator": len(provenance["candidate_stage_trace"])}
    return {"definition": "band-level presence of a candidate/pair within bilateral tolerance; actual production lists only, with pre/post pair-top-k trace", "by_tolerance": output}


def draw_images(image: np.ndarray, diag: Any, result: Any, provenance: dict[str, Any], counterfactual: dict[str, Any]) -> None:
    points = manual_points(); canvas = image.copy()
    if canvas.ndim == 2: canvas = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    sx = diag.processed_gray.shape[1] / result.roi_original_px[2]; sy = diag.processed_gray.shape[0] / result.roi_original_px[3]
    cf_by_band = {row["band_index"]: row for row in counterfactual["rows"]}
    for pair in diag.selected_pairs:
        y = result.roi_original_px[1] + pair.y / sy
        if not 2400 <= y <= 2700: continue
        manual = manual_at(y, points); ref = None
        left = result.roi_original_px[0] + pair.left / sx; right = result.roi_original_px[0] + pair.right / sx
        for x, color in ((manual[0], (0, 255, 255)), (manual[1], (0, 255, 255)), (left, (0, 0, 255)), (right, (0, 0, 255))): cv2.circle(canvas, (int(round(x)), int(round(y))), 6, color, -1)
        cf = cf_by_band.get(pair.band_index)
        if cf:
            target_id = cf["target_manual_near_pair_id"]
            target = next((p for p in diag.all_pairs[pair.band_index] if f"b{pair.band_index}:l{int(round(p.left))}:r{int(round(p.right))}" == target_id), None)
            if target:
                tl = result.roi_original_px[0] + target.left / sx; tr = result.roi_original_px[0] + target.right / sx
                cv2.circle(canvas, (int(round(tl)), int(round(y))), 5, (255, 0, 255), -1); cv2.circle(canvas, (int(round(tr)), int(round(y))), 5, (255, 0, 255), -1)
        state = next((s for s in provenance["dp_state_trace"] if s["pair_id"] == f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}"), None)
        if state and state.get("predecessor_pair_id"):
            prev = next((p for row in diag.all_pairs for p in row if f"b{p.band_index}:l{int(round(p.left))}:r{int(round(p.right))}" == state["predecessor_pair_id"]), None)
            if prev:
                py = result.roi_original_px[1] + prev.y / sy; cv2.line(canvas, (int(round(result.roi_original_px[0] + prev.center / sx)), int(round(py))), (int(round(result.roi_original_px[0] + pair.center / sx)), int(round(y))), (255, 128, 0), 2)
        cv2.putText(canvas, f"b{pair.band_index} {cf_by_band.get(pair.band_index, {}).get('classification', '')}", (2500, int(round(y))), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imwrite(str(OUT / "dp_trace_overlay.png"), canvas)
    failures = [row for row in counterfactual["rows"] if 2400 <= row["y_original"] < 2700]
    panels = []
    for row in failures[::max(1, len(failures) // 6)][:6]:
        y = int(round(row["y_original"])); crop = canvas[max(2400, y - 70):min(2700, y + 70), result.roi_original_px[0]:result.roi_original_px[0] + result.roi_original_px[2]]
        crop = cv2.resize(crop, (640, 120), interpolation=cv2.INTER_AREA)
        cv2.putText(crop, f"y={y} {row['classification']} loss={row['total_score_loss']:.3f}", (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA); panels.append(crop)
    while len(panels) < 6:
        panels.append(np.zeros_like(panels[0]) if panels else np.zeros((120, 640, 3), dtype=np.uint8))
    cv2.imwrite(str(OUT / "y_2400_2700_montage.jpg"), np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:6])]), [int(cv2.IMWRITE_JPEG_QUALITY), 92])


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()): raise RuntimeError(f"refusing to overwrite {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    evidence = verify_inputs(); image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None: raise RuntimeError("cannot read frozen raw image")
    baseline, tracker, result, diag, provenance = run_baseline_and_trace(image, evidence)
    dump(OUT / "baseline_reproduction.json", baseline)
    if baseline["status"] != "matched": return 2
    reconciliation = tolerance_reconciliation(diag, result, provenance)
    states, predecessor = predecessor_traces(provenance)
    counterfactual = forced_counterfactual(tracker, result, diag, provenance)
    trusted = trusted_intervals(diag, result)
    retention = candidate_retention(diag, result, provenance)
    y_trace = {"y_range": [2400, 2700], "bands": [row for row in counterfactual["rows"] if 2400 <= row["y_original"] < 2700], "candidate_trace": [row for row in provenance["candidate_stage_trace"] if 2400 <= result.roi_original_px[1] + row["y_processed"] / (diag.processed_gray.shape[0] / result.roi_original_px[3]) < 2700]}
    dump(OUT / "tolerance_reconciliation.json", reconciliation)
    dump(OUT / "candidate_stage_trace.json", {"baseline_status": baseline["status"], "trace_source": "StableSeamTracker._generate_candidates", "bands": provenance["candidate_stage_trace"]})
    dump(OUT / "pair_pre_post_truncation.json", {"bands": [row["pair_stages"] for row in provenance["candidate_stage_trace"]], "pre_truncation_observable": True})
    dump(OUT / "dp_state_trace.json", {"states": states})
    dump(OUT / "dp_predecessor_trace.json", {"predecessors": predecessor})
    dump(OUT / "forced_pair_counterfactual.json", counterfactual)
    dump(OUT / "failure_classification.json", {"main_tolerance_px": 25, "dp_lost_14_legacy_p2e0_classification": counterfactual["p2e0_14_dp_classification_counts"], "main_25_forced_classification": counterfactual["main_25_classification_counts"], "tolerance_category_counts": reconciliation["category_counts"], "candidate_stage_missing_count": sum(1 for row in counterfactual["rows"] if row["classification"] == "candidate_stage_missing"), "legacy_note": "P2E0's 14-band label includes bands where the selected pair was already the nearest L1 pair but outside bilateral 25px; those are reported as selected_already_l1_nearest, not falsely attributed to DP."})
    dump(OUT / "y_2400_2700_trace.json", y_trace)
    dump(OUT / "trusted_y_interval_evaluation.json", trusted)
    dump(OUT / "candidate_retention_summary.json", retention)
    draw_images(image, diag, result, provenance, counterfactual)
    after = {key: sha256(path) for key, path in {"raw": RAW, "annotation": ANNOTATION, "config": CONFIG, "reference": REFERENCE, "tracker": ROOT / "tank_vision/stable_seam_tracker.py"}.items()}
    choice = "E" if not all(baseline["trace_false_true_field_equality"].values()) else ("B" if any(counterfactual["classification_counts"].get(k, 0) for k in ("unreachable_from_previous", "unreachable_to_next")) else ("C" if counterfactual["classification_counts"].get("transition_cost_dominant", 0) else "D"))
    fixed_keys = ("raw", "annotation", "config", "reference")
    summary = {"baseline_status": baseline["status"], "trace_strict_reproduction": all(baseline["trace_false_true_field_equality"].values()), "tolerance_counts_sum": reconciliation["counts_sum"], "manual_near_pair_final_count": reconciliation["p2e0_reconciliation"]["manual_near_pair_final_list"], "dp_lost_14_legacy_classification": counterfactual["p2e0_14_dp_classification_counts"], "main_25_forced_classification": counterfactual["main_25_classification_counts"], "candidate_retention": retention["by_tolerance"], "trusted_interval_conclusion": trusted["intervals"], "decision": choice, "fixed_input_sha256_before": {key: evidence["sha256"][key] for key in fixed_keys}, "fixed_input_sha256_after": {key: after[key] for key in fixed_keys}, "fixed_input_hashes_unchanged": all(evidence["sha256"][key] == after[key] for key in fixed_keys), "tracker_sha256_before_trace_hook": TRACKER_SHA256_BEFORE_TRACE_HOOK, "tracker_sha256_after_trace_hook": after["tracker"]}
    transition_component_sums = {key: sum(row["transition_component_margin"][key] for row in counterfactual["main_25_rows"]) for key in ("center_cost", "width_cost", "left_cost", "right_cost", "skip_cost")}
    summary["main_25_transition_component_margin_sums"] = transition_component_sums
    summary["largest_main_25_transition_component"] = max(transition_component_sums, key=transition_component_sums.get)
    dump(OUT / "diagnostic_summary.json", summary)
    next_change = {"choice": choice, "implemented": False, "production_algorithm_change": "none", "candidate": "inspect only one provenance-supported object in the next phase", "details": counterfactual["classification_counts"], "do_not_change": ["ROI", "candidate thresholds", "guided radius/Top-K", "pair gate", "unary", "DP", "fitting", "geometry gate", "temporal", "ROS"]}
    if choice == "A": next_change.update({"unique_next_object": "the dominant candidate/pair truncation stage"})
    elif choice == "B": next_change.update({"unique_next_object": "the single most frequent hard transition constraint"})
    elif choice == "C": next_change.update({"unique_next_object": f"the {summary['largest_main_25_transition_component']} term only", "observed_component_margin_sums": transition_component_sums})
    elif choice == "D": next_change.update({"unique_next_object": "one identity feature only; return to unary diagnostic"})
    else: next_change.update({"unique_next_object": "missing observation only"})
    dump(OUT / "next_single_change.json", next_change)
    if not summary["fixed_input_hashes_unchanged"]: raise RuntimeError("fixed input hash changed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
