#!/usr/bin/env python3
"""Run the S1-P2D-SIMPLE ROI/guided-search comparison and regression report."""

from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import statistics
import sys
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay
from tank_vision.seam_tracking_types import TrackingState

OUT = ROOT / "output/s1_p2d_simple_guided_search"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
BASELINE_CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
GUIDED_CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
GUIDED_REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
V2_IMAGE = ROOT / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
V2_ANNOTATION = ROOT / "output/s1_d7_manual_ground_truth_v2/seam_center_lit_quality_01_annotation_v2.json"


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(item) for item in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if np.isfinite(value) else None
    if isinstance(value, dict):
        return {str(key): clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(item) for item in value]
    if isinstance(value, TrackingState):
        return value.name
    return value


def stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {name: None for name in ("p50", "p90", "p95", "max")}
    absolute = np.abs(np.asarray(values, dtype=np.float64))
    return {"p50": float(np.percentile(absolute, 50)), "p90": float(np.percentile(absolute, 90)),
            "p95": float(np.percentile(absolute, 95)), "max": float(np.max(absolute))}


def manual_points(path: Path) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    return (payload["left_boundary_manual_points"], payload["right_boundary_manual_points"])


def manual_at(points: list[dict[str, float]], y: float) -> float:
    ordered = sorted(points, key=lambda item: item["y"])
    return float(np.interp(y, [p["y"] for p in ordered], [p["x"] for p in ordered]))


def variant_config(roi_only: bool = False) -> dict[str, Any]:
    config = yaml.safe_load(GUIDED_CONFIG.read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["guided_search"]["enabled"] = not roi_only
    return config


def _manual_metrics(tracker: StableSeamTracker, result: Any, diagnostics: Any,
                    left_manual: list[dict[str, float]], right_manual: list[dict[str, float]]) -> dict[str, Any]:
    x0, y0, _rw, _rh = result.roi_original_px
    scan_ys = diagnostics.scan_ys
    sx = diagnostics.processed_gray.shape[1] / result.roi_original_px[2]
    sy = diagnostics.processed_gray.shape[0] / result.roi_original_px[3]
    common_y0 = max(left_manual[0]["y"], right_manual[0]["y"])
    common_y1 = min(left_manual[-1]["y"], right_manual[-1]["y"])
    errors = {name: [] for name in ("left", "right", "center", "width")}
    selected_near = selected_far = 0
    evaluable = 0
    for pair in diagnostics.selected_pairs:
        y = y0 + pair.y / sy
        if not (common_y0 <= y <= common_y1):
            continue
        ml, mr = manual_at(left_manual, y), manual_at(right_manual, y)
        pl, pr = x0 + pair.left / sx, x0 + pair.right / sx
        errors["left"].append(pl - ml)
        errors["right"].append(pr - mr)
        errors["center"].append(0.5 * (pl + pr) - 0.5 * (ml + mr))
        errors["width"].append((pr - pl) - (mr - ml))
        evaluable += 1
        if max(abs(pl - ml), abs(pr - mr)) <= 25.0:
            selected_near += 1
        else:
            selected_far += 1
    return {"evaluated_selected_pairs": evaluable,
            "selected_near_manual": selected_near, "selected_far_manual": selected_far,
            "errors_px": {key: stats(value) for key, value in errors.items()}}


def _candidate_recall(result: Any, diagnostics: Any, left_manual: list[dict[str, float]],
                      right_manual: list[dict[str, float]], tolerance: float = 25.0) -> dict[str, Any]:
    x0, y0, rw, rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    sy = diagnostics.processed_gray.shape[0] / rh
    left_hits = right_hits = pair_hits = evaluated = 0
    left_count = right_count = 0
    per_band = []
    for index, (y_processed, lefts, rights, pairs) in enumerate(zip(
            diagnostics.scan_ys, diagnostics.left_candidates, diagnostics.right_candidates, diagnostics.all_pairs)):
        y = y0 + float(y_processed) / sy
        if not (max(left_manual[0]["y"], right_manual[0]["y"]) <= y <=
                min(left_manual[-1]["y"], right_manual[-1]["y"])):
            continue
        ml, mr = manual_at(left_manual, y), manual_at(right_manual, y)
        lx = [x0 + float(value) / sx for value in lefts]
        rx = [x0 + float(value) / sx for value in rights]
        left_near = any(abs(value - ml) <= tolerance for value in lx)
        right_near = any(abs(value - mr) <= tolerance for value in rx)
        pair_near = any(abs(x0 + pair.left / sx - ml) <= tolerance
                        and abs(x0 + pair.right / sx - mr) <= tolerance for pair in pairs)
        left_hits += int(left_near)
        right_hits += int(right_near)
        pair_hits += int(pair_near)
        left_count += len(lx)
        right_count += len(rx)
        evaluated += 1
        per_band.append({"band_index": index, "y_original_px": y, "left_near": left_near,
                         "right_near": right_near, "pair_near": pair_near,
                         "left_candidate_count": len(lx), "right_candidate_count": len(rx)})
    return {"tolerance_px": tolerance, "evaluated_scan_bands": evaluated,
            "left_candidate_recall": left_hits / max(evaluated, 1),
            "right_candidate_recall": right_hits / max(evaluated, 1),
            "correct_pair_recall": pair_hits / max(evaluated, 1),
            "left_candidate_total": left_count, "right_candidate_total": right_count,
            "per_band": per_band}


def run_variant(name: str, config: str | Path | dict[str, Any], image: np.ndarray,
                left_manual: list[dict[str, float]], right_manual: list[dict[str, float]]) -> dict[str, Any]:
    tracker = StableSeamTracker(config, GUIDED_REFERENCE if name != "baseline" else None)
    first = tracker.process(image, diagnostics=True)
    first_diagnostics = tracker.last_diagnostics
    assert first_diagnostics is not None
    sequence = [first]
    for _ in range(2):
        sequence.append(tracker.process(image, diagnostics=False))
    final = sequence[-1]
    return {
        "variant": name,
        "roi_original_px": list(first.roi_original_px),
        "scan_band_count": len(first_diagnostics.scan_ys),
        "first_frame": clean(first.to_dict()),
        "final_frame_after_three_repeats": clean(final.to_dict()),
        "state_sequence": [item.tracking_state.name for item in sequence],
        "three_repeat_tracking": sequence[-1].tracking_state == TrackingState.TRACKING,
        "manual": _manual_metrics(tracker, first, first_diagnostics, left_manual, right_manual),
        "candidate_recall": _candidate_recall(first, first_diagnostics, left_manual, right_manual),
        "candidate_search_summary": {
            "left_modes": {mode: sum(item.get("left_mode") == mode for item in first_diagnostics.candidate_search)
                           for mode in ("local", "global_fallback", "global", "empty")},
            "right_modes": {mode: sum(item.get("right_mode") == mode for item in first_diagnostics.candidate_search)
                            for mode in ("local", "global_fallback", "global", "empty")},
            "max_left_candidates_per_band": max((len(item) for item in first_diagnostics.left_candidates), default=0),
            "max_right_candidates_per_band": max((len(item) for item in first_diagnostics.right_candidates), default=0),
        },
    }


def benchmark(config: str | Path | dict[str, Any], image: np.ndarray, iterations: int = 300) -> dict[str, Any]:
    tracker = StableSeamTracker(config, GUIDED_REFERENCE)
    for _ in range(30):
        tracker.process(image)
    total: list[float] = []
    stages: dict[str, list[float]] = {}
    for _ in range(iterations):
        result = tracker.process(image)
        total.append(result.processing_ms)
        for key, value in result.stage_timings_ms.items():
            stages.setdefault(key, []).append(float(value))
    def summary(values: list[float]) -> dict[str, float]:
        return {"mean": statistics.fmean(values), "p50": float(np.percentile(values, 50)),
                "p95": float(np.percentile(values, 95)), "max": max(values)}
    return {"warmup_frames": 30, "measured_frames": iterations, "total_ms": summary(total),
            "stages_ms": {key: summary(values) for key, values in stages.items()},
            "processing_p95_below_100ms": float(np.percentile(total, 95)) < 100.0}


def regression(config: str | Path | dict[str, Any], image: np.ndarray) -> dict[str, Any]:
    entries: dict[str, Any] = {}
    entries["current_p2c_manual_frame"] = {
        "tested_by": "guided_variant_and_manual_error_comparison",
        "precision_claimed": True,
    }
    v2 = cv2.imread(str(V2_IMAGE), cv2.IMREAD_COLOR)
    if v2 is not None:
        # V2 is a separate frozen reference scene.  It is checked with the
        # legacy frozen production reference and again with the new candidate,
        # while no accuracy claim is made for the unlabeled second run.
        t = StableSeamTracker(BASELINE_CONFIG)
        legacy = t.process(v2)
        t = StableSeamTracker(config, GUIDED_REFERENCE)
        guided_v2 = t.process(v2)
        entries["frozen_v2_reference_frame"] = {
            "legacy_baseline": {"detected": legacy.detected, "state": legacy.tracking_state.name,
                                 "crossing": _crosses(legacy)},
            "guided_candidate": {"detected": guided_v2.detected, "state": guided_v2.tracking_state.name,
                                  "crossing": _crosses(guided_v2)},
            "precision_claimed": False,
        }
    flat = np.full_like(image, int(np.mean(image)))
    t = StableSeamTracker(config, GUIDED_REFERENCE)
    r = t.process(flat)
    entries["no_seam_negative"] = {"detected": r.detected, "state": r.tracking_state.name,
                                    "crossing": _crosses(r), "precision_claimed": False}
    blurred = cv2.GaussianBlur(image, (251, 251), 0)
    t = StableSeamTracker(config, GUIDED_REFERENCE)
    r = t.process(blurred)
    entries["severe_blur"] = {"detected": r.detected, "state": r.tracking_state.name,
                              "crossing": _crosses(r), "precision_claimed": False}
    snapshots = []
    for raw_path in sorted((ROOT / "output/stable_seam_failure_snapshots").glob("*/raw.bmp")):
        frame = cv2.imread(str(raw_path), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        t = StableSeamTracker(config, GUIDED_REFERENCE)
        r = t.process(frame)
        snapshots.append({"input": str(raw_path.relative_to(ROOT)), "detected": r.detected,
                          "state": r.tracking_state.name, "crossing": _crosses(r),
                          "precision_claimed": False})
    entries["saved_failure_snapshots"] = snapshots
    rng = np.random.default_rng(12345)
    synthetic_records = []
    for index in range(12):
        shift = int(index - 6) * 2
        matrix = np.float32([[1.0, 0.0, shift], [0.0, 1.0, 0.0]])
        frame = cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT)
        frame = np.clip(frame.astype(np.float32) + rng.normal(0.0, 1.5, frame.shape), 0, 255).astype(np.uint8)
        if index in (6, 7):
            frame = cv2.GaussianBlur(frame, (251, 251), 0)
        synthetic_records.append(frame)
    t = StableSeamTracker(config, GUIDED_REFERENCE)
    for index, frame in enumerate(synthetic_records):
        r = t.process(frame)
        synthetic_records[index] = {"frame": index, "detected": r.detected, "state": r.tracking_state.name,
                                    "crossing": _crosses(r), "abrupt_jump": bool(
                                        r.abrupt_left_jump_count or r.abrupt_right_jump_count)}
    entries["synthetic_sequence"] = {"frame_count": len(synthetic_records), "records": synthetic_records,
                                      "precision_claimed": False}
    return entries


def _crosses(result: Any) -> bool:
    left_points = result["left_boundary_original_px"] if isinstance(result, dict) else result.left_boundary_original_px
    right_points = result["right_boundary_original_px"] if isinstance(result, dict) else result.right_boundary_original_px
    return any(left[0] >= right[0] for left, right in zip(left_points, right_points))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {RAW}")
    left_manual, right_manual = manual_points(ANNOTATION)
    reference_payload = json.loads(GUIDED_REFERENCE.read_text(encoding="utf-8"))
    (OUT / "generated_reference_validation.json").write_text(json.dumps(clean({
        "reference_path": str(GUIDED_REFERENCE.relative_to(ROOT)),
        "status": reference_payload.get("status"),
        "generated_from_manual_annotation": reference_payload.get("generated_from_manual_annotation"),
        "input_image_sha256_matches": reference_payload.get("input_image_sha256") == hashlib.sha256(RAW.read_bytes()).hexdigest(),
        "annotation_sha256_matches": reference_payload.get("annotation_sha256") == hashlib.sha256(ANNOTATION.read_bytes()).hexdigest(),
        "reference_image_size": reference_payload.get("reference_image_size"),
        "used_roi": reference_payload.get("used_roi"),
        "manual_common_y_range_original_px": reference_payload.get("manual_common_y_range_original_px"),
        "center_ref_degree": reference_payload.get("center_ref", {}).get("degree"),
        "width_ref_degree": reference_payload.get("width_ref", {}).get("degree"),
        "fit_source": reference_payload.get("fit_source"),
    }), indent=2) + "\n")
    baseline = run_variant("baseline", BASELINE_CONFIG, image, left_manual, right_manual)
    roi_only = run_variant("roi_only", variant_config(True), image, left_manual, right_manual)
    guided = run_variant("guided", variant_config(False), image, left_manual, right_manual)
    variants = {"baseline": baseline, "roi_only": roi_only, "guided": guided}
    for name, value in variants.items():
        (OUT / {"baseline": "baseline_reproduction.json", "roi_only": "roi_only_result.json",
                "guided": "guided_result.json"}[name]).write_text(json.dumps(clean(value), indent=2) + "\n")
    (OUT / "manual_error_comparison.json").write_text(json.dumps(clean({
        "manual_annotation": str(ANNOTATION.relative_to(ROOT)),
        "variants": {name: value["manual"] for name, value in variants.items()},
        "metric_definition": "absolute error percentiles over selected pairs in manual common y range",
    }), indent=2) + "\n")
    (OUT / "candidate_recall_comparison.json").write_text(json.dumps(clean({
        "variants": {name: value["candidate_recall"] for name, value in variants.items()},
        "candidate_search": {name: value["candidate_search_summary"] for name, value in variants.items()},
    }), indent=2) + "\n")
    timing = {"baseline": benchmark(BASELINE_CONFIG, image), "roi_only": benchmark(variant_config(True), image),
              "guided": benchmark(variant_config(False), image)}
    (OUT / "timing_comparison.json").write_text(json.dumps(clean(timing), indent=2) + "\n")
    for name, config in (("baseline", BASELINE_CONFIG), ("roi_only", variant_config(True)), ("guided", variant_config(False))):
        tracker = StableSeamTracker(config, GUIDED_REFERENCE if name != "baseline" else None)
        result = tracker.process(image)
        cv2.imwrite(str(OUT / {"baseline": "baseline_overlay.png", "roi_only": "roi_only_overlay.png",
                              "guided": "guided_overlay.png"}[name]), draw_stable_seam_overlay(image, result, (1280, 960)))
    panels = [cv2.imread(str(OUT / file)) for file in ("baseline_overlay.png", "roi_only_overlay.png", "guided_overlay.png")]
    panels = [panel for panel in panels if panel is not None]
    cv2.imwrite(str(OUT / "comparison_montage.jpg"), np.hstack(panels))
    regression_result = regression(variant_config(False), image)
    (OUT / "regression_summary.json").write_text(json.dumps(clean(regression_result), indent=2) + "\n")
    guided_pass = guided["first_frame"]["detected"] and guided["three_repeat_tracking"]
    gate = {"detected": guided["first_frame"]["detected"], "tracking_after_three_repeats": guided["three_repeat_tracking"],
            "joint_inlier_ratio_ge_075": guided["first_frame"]["fit_inlier_ratio"] >= 0.75,
            "coverage_ge_085": guided["first_frame"]["coverage"] >= 0.85,
            "center_p95_le_25": guided["manual"]["errors_px"]["center"]["p95"] <= 25,
            "left_right_p95_le_30": guided["manual"]["errors_px"]["left"]["p95"] <= 30 and guided["manual"]["errors_px"]["right"]["p95"] <= 30,
            "width_p95_le_30": guided["manual"]["errors_px"]["width"]["p95"] <= 30,
            "boundaries_do_not_cross": not _crosses(guided["final_frame_after_three_repeats"]),
            "abrupt_jumps_le_2": (guided["first_frame"]["abrupt_left_jump_count"] + guided["first_frame"]["abrupt_right_jump_count"]) <= 2,
            "processing_p95_lt_100ms": timing["guided"]["processing_p95_below_100ms"]}
    diagnostic = {
        "scope": "S1-P2D-SIMPLE; only ROI and candidate search changed",
        "baseline_reproduced": {"coverage": baseline["first_frame"]["coverage"],
                                 "joint_inlier_ratio": baseline["first_frame"]["fit_inlier_ratio"],
                                 "residual_px": baseline["first_frame"]["fit_residual_px"],
                                 "detected": baseline["first_frame"]["detected"],
                                 "failure": baseline["first_frame"]["failure_reasons"]},
        "roi_only_vs_guided": {"roi_only": roi_only["manual"], "guided": guided["manual"],
                               "guided_search_modes": guided["candidate_search_summary"]},
        "gate": gate,
        "first_failed_gate": next((name for name, passed in gate.items() if not passed), None),
        "production_config_decision": {
            "production_config_unchanged": True,
            "production_config": str(BASELINE_CONFIG.relative_to(ROOT)),
            "guided_config_status": "experimental_only_gate_not_passed",
        },
        "guided_beyond_roi_improvement": {
            "joint_inlier_ratio_delta": guided["first_frame"]["fit_inlier_ratio"] - roi_only["first_frame"]["fit_inlier_ratio"],
            "center_p95_delta_px": guided["manual"]["errors_px"]["center"]["p95"] - roi_only["manual"]["errors_px"]["center"]["p95"],
            "correct_pair_recall_delta": guided["candidate_recall"]["correct_pair_recall"] - roi_only["candidate_recall"]["correct_pair_recall"],
        },
        "decision": "retain_guided_only_if_all_gates_and_negative_regression_pass",
        "regression": regression_result,
        "negative_false_positive": bool(regression_result["no_seam_negative"]["detected"]),
        "precision_claim_warning": "unlabelled regression images are checked only for crossing, jumps and false positive",
    }
    (OUT / "diagnostic_summary.json").write_text(json.dumps(clean(diagnostic), indent=2) + "\n")
    return 0 if guided_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
