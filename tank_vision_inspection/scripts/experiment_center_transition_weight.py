#!/usr/bin/env python3
"""S1-P2E2: in-memory causal ablation of the center DP transition weight.

The only algorithm value changed by this script is
``config['path']['center_transition_weight']`` in a deep-copied YAML mapping.
All measurements use the production StableSeamTracker implementation.
"""

from __future__ import annotations

import copy
import csv
import hashlib
import json
import math
import shutil
import statistics
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402

OUT = ROOT / "output/s1_p2e2_center_transition_ablation"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
GUIDED_CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
STABLE_CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
P2D_REGRESSION = ROOT / "output/s1_p2d_simple_guided_search/regression_summary.json"
P2E1_FORCED = ROOT / "output/s1_p2e1_dp_provenance/forced_pair_counterfactual.json"
WEIGHTS = (0.42, 0.32, 0.24, 0.16, 0.08)
TOLERANCE = 25.0


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(v) for v in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        v = float(value)
        return v if math.isfinite(v) else None
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


def manual_points() -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    payload = json.loads(ANNOTATION.read_text(encoding="utf-8"))
    return payload["left_boundary_manual_points"], payload["right_boundary_manual_points"]


def manual_at(points: list[dict[str, float]], y: float) -> float:
    ordered = sorted(points, key=lambda p: p["y"])
    return float(np.interp(y, [p["y"] for p in ordered], [p["x"] for p in ordered]))


def percentile(values: list[float], q: float) -> float | None:
    return None if not values else float(np.percentile(np.asarray(values, dtype=np.float64), q))


def signed_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"mean": None, "p50": None, "p90": None, "p95": None, "max": None}
    a = np.asarray(values, dtype=np.float64)
    return {"mean": float(np.mean(a)), "p50": float(np.percentile(a, 50)),
            "p90": float(np.percentile(a, 90)), "p95": float(np.percentile(a, 95)),
            "max": float(np.max(a))}


def absolute_stats(values: list[float]) -> dict[str, float | None]:
    if not values:
        return {"p50": None, "p90": None, "p95": None, "max": None}
    a = np.abs(np.asarray(values, dtype=np.float64))
    return {"p50": float(np.percentile(a, 50)), "p90": float(np.percentile(a, 90)),
            "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def config_for(weight: float) -> dict[str, Any]:
    config = copy.deepcopy(yaml.safe_load(GUIDED_CONFIG.read_text(encoding="utf-8")))
    original = config["path"]["center_transition_weight"]
    if not math.isclose(float(original), 0.42, abs_tol=1e-12):
        raise RuntimeError(f"guided YAML center weight is not 0.42: {original}")
    config["path"]["center_transition_weight"] = float(weight)
    return config


def pair_id(pair: Any) -> str:
    return f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}"


def crosses(result: Any) -> int:
    return sum(int(l[0] >= r[0]) for l, r in zip(result.left_boundary_original_px, result.right_boundary_original_px))


def selected_rows(result: Any, diagnostics: Any) -> list[dict[str, Any]]:
    x0, y0, rw, rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    sy = diagnostics.processed_gray.shape[0] / rh
    return [{"band_index": int(p.band_index), "y_original": float(y0 + p.y / sy),
             "pair_id": pair_id(p), "left": float(x0 + p.left / sx),
             "right": float(x0 + p.right / sx), "center": float(x0 + p.center / sx),
             "width": float(p.width / sx), "score": float(p.score)}
            for p in diagnostics.selected_pairs]


def error_rows(result: Any, diagnostics: Any, left: list[dict[str, float]], right: list[dict[str, float]]) -> list[dict[str, Any]]:
    rows = selected_rows(result, diagnostics)
    common0 = max(left[0]["y"], right[0]["y"])
    common1 = min(left[-1]["y"], right[-1]["y"])
    for row in rows:
        if common0 <= row["y_original"] <= common1:
            ml, mr = manual_at(left, row["y_original"]), manual_at(right, row["y_original"])
            row.update({"manual_left": ml, "manual_right": mr,
                        "manual_center": 0.5 * (ml + mr), "manual_width": mr - ml,
                        "left_error": row["left"] - ml, "right_error": row["right"] - mr,
                        "center_error": row["center"] - 0.5 * (ml + mr),
                        "width_error": row["width"] - (mr - ml),
                        "bilateral_near_25": abs(row["left"] - ml) <= TOLERANCE and abs(row["right"] - mr) <= TOLERANCE})
        else:
            row["bilateral_near_25"] = False
    return rows


def pair_rows(result: Any, diagnostics: Any, left: list[dict[str, float]], right: list[dict[str, float]]) -> list[dict[str, Any]]:
    x0, y0, rw, rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    sy = diagnostics.processed_gray.shape[0] / rh
    common0 = max(left[0]["y"], right[0]["y"])
    common1 = min(left[-1]["y"], right[-1]["y"])
    selected = {p.band_index: p for p in diagnostics.selected_pairs}
    output: list[dict[str, Any]] = []
    for band, pairs in enumerate(diagnostics.all_pairs):
        if not pairs:
            continue
        y = y0 + pairs[0].y / sy
        if not common0 <= y <= common1:
            continue
        ml, mr = manual_at(left, y), manual_at(right, y)
        def distance(p: Any) -> float:
            return abs(x0 + p.left / sx - ml) + abs(x0 + p.right / sx - mr)
        near = min(pairs, key=distance)
        sel = selected.get(band)
        output.append({"band_index": int(band), "y_original": float(y),
                       "manual_left": ml, "manual_right": mr,
                       "manual_near_pair_id": pair_id(near),
                       "manual_near_left_error": float(x0 + near.left / sx - ml),
                       "manual_near_right_error": float(x0 + near.right / sx - mr),
                       "manual_near_score": float(near.score),
                       "selected_pair_id": None if sel is None else pair_id(sel),
                       "selected_score": None if sel is None else float(sel.score),
                       "manual_near_present": abs(x0 + near.left / sx - ml) <= TOLERANCE and abs(x0 + near.right / sx - mr) <= TOLERANCE,
                       "selected_near": sel is not None and abs(x0 + sel.left / sx - ml) <= TOLERANCE and abs(x0 + sel.right / sx - mr) <= TOLERANCE,
                       "pair_count": len(pairs)})
    return output


def path_shape(rows: list[dict[str, Any]], result: Any, reference: Any) -> dict[str, Any]:
    def deltas(key: str) -> dict[str, float | None]:
        return absolute_stats([b[key] - a[key] for a, b in zip(rows, rows[1:])])
    centers = [r["center"] for r in rows]
    second = [centers[i + 1] - 2 * centers[i] + centers[i - 1] for i in range(1, len(centers) - 1)]
    width_rate = [(b["width"] - a["width"]) / max(abs(a["width"]), 1e-6) for a, b in zip(rows, rows[1:])]
    return {"adjacent_center_delta": deltas("center"),
            "center_second_difference": absolute_stats(second),
            "adjacent_left_delta": deltas("left"), "adjacent_right_delta": deltas("right"),
            "width_relative_change": absolute_stats(width_rate),
            "crossing_count": crosses(result),
            "abrupt_left_jump_count": int(result.abrupt_left_jump_count),
            "abrupt_right_jump_count": int(result.abrupt_right_jump_count),
            "selected_path_count": len(rows)}


def run_positive(weight: float, image: np.ndarray, left: list[dict[str, float]], right: list[dict[str, float]]) -> dict[str, Any]:
    tracker = StableSeamTracker(config_for(weight), REFERENCE)
    first = tracker.process(image, diagnostics=True)
    diag = tracker.last_diagnostics
    assert diag is not None
    rows = error_rows(first, diag, left, right)
    pair_audit = pair_rows(first, diag, left, right)
    repeats = [first]
    for _ in range(2):
        repeats.append(tracker.process(image, diagnostics=False))
    errors = {name: [r[f"{name}_error"] for r in rows if f"{name}_error" in r]
              for name in ("left", "right", "center", "width")}
    strict_near = [r for r in pair_audit if r["manual_near_present"]]
    selected_near = [r for r in pair_audit if r["selected_near"]]
    unary_lost = [r for r in strict_near if not r["selected_near"] and (r["selected_score"] or -np.inf) > r["manual_near_score"]]
    dp_lost = [r for r in strict_near if not r["selected_near"] and (r["selected_score"] or -np.inf) <= r["manual_near_score"]]
    return {"weight": weight,
            "first_frame": first.to_dict(),
            "final_frame_after_three_repeats": repeats[-1].to_dict(),
            "state_sequence": [r.tracking_state.name for r in repeats],
            "three_repeat_tracking": repeats[-1].tracking_state == TrackingState.TRACKING,
            "temporal_rejection": any(any(x.startswith("temporal_") for x in r.failure_reasons) for r in repeats),
            "selected_rows": rows, "pair_audit": pair_audit,
            "errors": {name: {"signed": signed_stats(values), "absolute": absolute_stats(values)} for name, values in errors.items()},
            "selected_path_count": len(rows), "coverage": first.coverage, "effective_span": first.effective_span,
            "maximum_missing_run": first.maximum_missing_run, "abrupt_left_jump": first.abrupt_left_jump_count,
            "abrupt_right_jump": first.abrupt_right_jump_count, "crossing": crosses(first),
            "path_confidence": float(np.mean([p.score for p in diag.selected_pairs])) if diag.selected_pairs else 0.0,
            "joint_inlier_ratio": first.fit_inlier_ratio, "fit_residual_px": first.fit_residual_px,
            "detected": first.detected, "near_25": {"exist": len(strict_near), "selected": len(selected_near),
                "unary_lost": len(unary_lost), "dp_lost": len(dp_lost)},
            "path_stability": path_shape(rows, first, REFERENCE)}


def summary(values: list[float]) -> dict[str, float]:
    a = np.asarray(values, dtype=np.float64)
    return {"mean": float(np.mean(a)), "p50": float(np.percentile(a, 50)),
            "p95": float(np.percentile(a, 95)), "max": float(np.max(a))}


def benchmark(weight: float, image: np.ndarray, warmup: int = 30, iterations: int = 300) -> dict[str, Any]:
    tracker = StableSeamTracker(config_for(weight), REFERENCE)
    for _ in range(warmup):
        tracker.process(image)
    total: list[float] = []
    stages: dict[str, list[float]] = {}
    for _ in range(iterations):
        result = tracker.process(image)
        total.append(float(result.processing_ms))
        for key, value in result.stage_timings_ms.items():
            stages.setdefault(key, []).append(float(value))
    return {"warmup_frames": warmup, "measured_frames": iterations, "total_ms": summary(total),
            "stages_ms": {key: summary(v) for key, v in stages.items()},
            "processing_p95_lt_100ms": float(np.percentile(total, 95)) < 100.0}


def exact_negative_suite(weight: float, image: np.ndarray) -> dict[str, Any]:
    """Reproduce the P2D regression inputs, retaining their original semantics."""
    entries: dict[str, Any] = {}
    v2 = ROOT / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
    v2_image = cv2.imread(str(v2), cv2.IMREAD_COLOR)
    if v2_image is not None:
        tracker = StableSeamTracker(config_for(weight), REFERENCE)
        result = tracker.process(v2_image)
        entries["frozen_v2_reference_frame"] = {"detected": result.detected, "state": result.tracking_state.name,
                                                  "crossing": crosses(result), "precision_claimed": False}
    flat = np.full_like(image, int(np.mean(image)))
    tracker = StableSeamTracker(config_for(weight), REFERENCE)
    result = tracker.process(flat)
    entries["no_seam_negative"] = {"detected": result.detected, "state": result.tracking_state.name,
                                    "crossing": crosses(result), "precision_claimed": False}
    blurred = cv2.GaussianBlur(image, (251, 251), 0)
    tracker = StableSeamTracker(config_for(weight), REFERENCE)
    result = tracker.process(blurred)
    entries["severe_blur"] = {"detected": result.detected, "state": result.tracking_state.name,
                              "crossing": crosses(result), "precision_claimed": False}
    snapshots = []
    for raw_path in sorted((ROOT / "output/stable_seam_failure_snapshots").glob("*/raw.bmp")):
        frame = cv2.imread(str(raw_path), cv2.IMREAD_COLOR)
        if frame is None:
            continue
        tracker = StableSeamTracker(config_for(weight), REFERENCE)
        result = tracker.process(frame)
        snapshots.append({"input": str(raw_path.relative_to(ROOT)), "detected": result.detected,
                          "state": result.tracking_state.name, "crossing": crosses(result), "precision_claimed": False})
    entries["saved_failure_snapshots"] = snapshots
    rng = np.random.default_rng(12345)
    frames = []
    for index in range(12):
        shift = int(index - 6) * 2
        matrix = np.float32([[1.0, 0.0, shift], [0.0, 1.0, 0.0]])
        frame = cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT)
        frame = np.clip(frame.astype(np.float32) + rng.normal(0.0, 1.5, frame.shape), 0, 255).astype(np.uint8)
        if index in (6, 7):
            frame = cv2.GaussianBlur(frame, (251, 251), 0)
        frames.append(frame)
    tracker = StableSeamTracker(config_for(weight), REFERENCE)
    sequence = []
    for index, frame in enumerate(frames):
        result = tracker.process(frame)
        sequence.append({"frame": index, "detected": result.detected, "state": result.tracking_state.name,
                         "crossing": crosses(result), "abrupt_jump": bool(result.abrupt_left_jump_count or result.abrupt_right_jump_count)})
    entries["synthetic_sequence"] = {"frame_count": len(sequence), "records": sequence, "precision_claimed": False}
    hard = entries["no_seam_negative"]
    entries["hard_no_seam_negative_pass"] = not hard["detected"] and hard["state"] == "LOST" and hard["crossing"] == 0
    entries["suite_semantics"] = "P2D exact suite; unlabelled/reference/stress entries are not hard-negative acceptance cases"
    return entries


def read_p2e1_dp_lost() -> list[dict[str, Any]]:
    payload = json.loads(P2E1_FORCED.read_text(encoding="utf-8"))
    # P2E1 contains a legacy 14-band compatibility table as well as the
    # authoritative main-25px table.  Use only the six true DP-lost rows from
    # the latter (five transition-dominant plus one cumulative-history row).
    return [row for row in payload["main_25_rows"]
            if row.get("classification") in ("transition_cost_dominant", "cumulative_history_dominant")]


def make_montage(image: np.ndarray, results: list[dict[str, Any]]) -> None:
    panels = []
    colors = ((255, 0, 0), (0, 255, 0), (0, 255, 255))
    for item in results:
        panel = image.copy()
        for key, color in zip(("left", "right"), colors[:2]):
            points = [(int(round(r[key])), int(round(r["y_original"]))) for r in item["selected_rows"]]
            if len(points) > 1:
                cv2.polylines(panel, [np.asarray(points, dtype=np.int32).reshape(-1, 1, 2)], False, color, 7, cv2.LINE_AA)
        cv2.putText(panel, f"center_weight={item['weight']:.2f}  detected={item['detected']}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(panel, f"center_weight={item['weight']:.2f}  detected={item['detected']}", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)
        scale = min(1.0, 1200 / panel.shape[1], 700 / panel.shape[0])
        panels.append(cv2.resize(panel, (round(panel.shape[1] * scale), round(panel.shape[0] * scale)), interpolation=cv2.INTER_AREA))
    height = max(p.shape[0] for p in panels)
    normalized = [cv2.copyMakeBorder(p, 0, height - p.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0)) for p in panels]
    cv2.imwrite(str(OUT / "comparison_montage.jpg"), np.hstack(normalized), [cv2.IMWRITE_JPEG_QUALITY, 92])


def verify_inputs() -> dict[str, Any]:
    paths = [RAW, ANNOTATION, GUIDED_CONFIG, STABLE_CONFIG, REFERENCE, ROOT / "tank_vision/stable_seam_tracker.py"]
    missing = [str(p.relative_to(ROOT)) for p in paths if not p.exists()]
    if missing:
        raise RuntimeError(f"missing fixed input(s): {missing}")
    manifest = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/ground_truth_manifest.json"
    return {"exists": not missing, "missing": missing,
            "sha256_before": {str(p.relative_to(ROOT)): sha256(p) for p in paths},
            "raw_expected_sha256": "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02",
            "raw_sha256_matches": sha256(RAW) == "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02",
            "ground_truth_manifest": {"path": str(manifest.relative_to(ROOT)), "exists": manifest.exists(),
                                      "sha256": sha256(manifest) if manifest.exists() else None}}


def baseline_gate(result: dict[str, Any]) -> dict[str, Any]:
    expected = {"selected_path_count": 116, "coverage": 0.991453, "center_p95": 44.15,
                "left_p95": 59.28, "right_p95": 47.92, "width_p95": 70.42,
                "states": ["ACQUIRING", "ACQUIRING", "TRACKING"]}
    actual = {"selected_path_count": result["selected_path_count"], "coverage": result["coverage"],
              "center_p95": result["errors"]["center"]["absolute"]["p95"],
              "left_p95": result["errors"]["left"]["absolute"]["p95"],
              "right_p95": result["errors"]["right"]["absolute"]["p95"],
              "width_p95": result["errors"]["width"]["absolute"]["p95"], "states": result["state_sequence"]}
    checks = {"path": actual["selected_path_count"] == 116,
              "coverage": abs(actual["coverage"] - expected["coverage"]) < 1e-5,
              "center_p95": abs(actual["center_p95"] - expected["center_p95"]) < 0.1,
              "left_p95": abs(actual["left_p95"] - expected["left_p95"]) < 0.1,
              "right_p95": abs(actual["right_p95"] - expected["right_p95"]) < 0.1,
              "width_p95": abs(actual["width_p95"] - expected["width_p95"]) < 0.1,
              "state_sequence": actual["states"] == expected["states"]}
    return {"status": "matched" if all(checks.values()) else "baseline_mismatch", "expected": expected,
            "actual": actual, "checks": checks}


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output directory: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    fixed = verify_inputs()
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {RAW}")
    left, right = manual_points()
    baseline = run_positive(0.42, image, left, right)
    gate = baseline_gate(baseline)
    write_json("baseline_reproduction.json", {"fixed_inputs": fixed, "baseline": baseline, "gate": gate,
                                                "processing_p95_source": "measured by benchmark.json after gate"})
    if gate["status"] != "matched":
        write_json("diagnostic_summary.json", {"status": "baseline_mismatch", "stopped_before_other_weights": True,
                                                "gate": gate})
        return 2

    results = [baseline]
    for weight in WEIGHTS[1:]:
        results.append(run_positive(weight, image, left, right))
    benchmarks = {f"{weight:.2f}": benchmark(weight, image) for weight in WEIGHTS}
    negatives = {f"{weight:.2f}": exact_negative_suite(weight, image) for weight in WEIGHTS}
    write_json("benchmark.json", benchmarks)
    write_json("negative_regression.json", negatives)

    baseline_rows = {r["band_index"]: r for r in baseline["selected_rows"]}
    path_changes = {}
    comparisons = []
    for item in results:
        changes = []
        improved = []
        worsened = []
        for row in item["selected_rows"]:
            old = baseline_rows.get(row["band_index"])
            if old is None or row["pair_id"] == old["pair_id"]:
                continue
            changes.append(row["band_index"])
            center_delta = abs(row.get("center_error", 0)) - abs(old.get("center_error", 0))
            side_delta = max(abs(row.get("left_error", 0)) - abs(old.get("left_error", 0)),
                             abs(row.get("right_error", 0)) - abs(old.get("right_error", 0)))
            if center_delta <= -10.0 and side_delta <= 10.0:
                improved.append(row["band_index"])
            if center_delta > 10.0 or (item["crossing"] > baseline["crossing"]):
                worsened.append(row["band_index"])
        path_changes[f"{item['weight']:.2f}"] = {"changed_band_count": len(changes), "changed_bands": changes,
                                                  "improvement_bands": improved, "worsening_bands": worsened}
        comparisons.append({"weight": item["weight"], "selected_path": item["selected_path_count"],
                            "coverage": item["coverage"], "effective_span": item["effective_span"],
                            "maximum_missing_run": item["maximum_missing_run"], "abrupt_left_jump": item["abrupt_left_jump"],
                            "abrupt_right_jump": item["abrupt_right_jump"], "crossing": item["crossing"],
                            "path_confidence": item["path_confidence"], "joint_inlier_ratio": item["joint_inlier_ratio"],
                            "fit_residual_px": item["fit_residual_px"], "detected": item["detected"],
                            "states": ">".join(item["state_sequence"]), "center_p95": item["errors"]["center"]["absolute"]["p95"],
                            "left_p95": item["errors"]["left"]["absolute"]["p95"], "right_p95": item["errors"]["right"]["absolute"]["p95"],
                            "width_p95": item["errors"]["width"]["absolute"]["p95"], "near_exist": item["near_25"]["exist"],
                            "near_selected": item["near_25"]["selected"], "unary_lost": item["near_25"]["unary_lost"],
                            "dp_lost": item["near_25"]["dp_lost"], "processing_p95": benchmarks[f"{item['weight']:.2f}"]["total_ms"]["p95"],
                            "hard_no_seam_negative_pass": negatives[f"{item['weight']:.2f}"]["hard_no_seam_negative_pass"]})
    write_json("weight_comparison.json", {"rows": comparisons, "fixed_values": {"weights": list(WEIGHTS),
        "only_override": "path.center_transition_weight", "manual_tolerance_px": TOLERANCE}})
    with (OUT / "weight_comparison.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(comparisons[0]))
        writer.writeheader(); writer.writerows(clean(comparisons))
    write_json("path_identity_changes.json", path_changes)

    forced = read_p2e1_dp_lost()
    recovery = []
    for item in results:
        by_band = {r["band_index"]: r for r in item["pair_audit"]}
        rows = []
        for target in forced:
            actual = by_band.get(target["band_index"], {})
            rows.append({"band_index": target["band_index"], "y_original": target["y_original"],
                         "target_pair_id": target["target_manual_near_pair_id"], "selected_pair_id": actual.get("selected_pair_id"),
                         "recovered": actual.get("selected_pair_id") == target["target_manual_near_pair_id"],
                         "selected_near_25": actual.get("selected_near"), "target_classification": target["classification"]})
        recovery.append({"weight": item["weight"], "rows": rows, "recovered_count": sum(r["recovered"] for r in rows)})
    write_json("dp_lost_band_recovery.json", {"source": str(P2E1_FORCED.relative_to(ROOT)), "bands": recovery})
    write_json("signed_error_comparison.json", {f"{x['weight']:.2f}": x["errors"] for x in results})
    focus = []
    for x in results:
        focus_rows = [r for r in x["selected_rows"] if 2400 <= r["y_original"] <= 2700]
        focus_errors = {n: [r[f"{n}_error"] for r in focus_rows if f"{n}_error" in r]
                        for n in ("left", "right", "center", "width")}
        focus.append({"weight": x["weight"], "rows": focus_rows,
                      "errors": {n: {"signed": signed_stats(v), "absolute": absolute_stats(v)}
                                 for n, v in focus_errors.items()}})
    write_json("y_2400_2700_comparison.json", focus)
    write_json("path_stability_comparison.json", {f"{x['weight']:.2f}": x["path_stability"] for x in results})
    make_montage(image, results)

    hard_rows = []
    for row in comparisons:
        hard_rows.append({"weight": row["weight"], "all_thresholds": all((row["center_p95"] <= 25,
            row["left_p95"] <= 30, row["right_p95"] <= 30, row["width_p95"] <= 30,
            row["joint_inlier_ratio"] >= 0.75, row["coverage"] >= 0.85, row["effective_span"] >= 0.85,
            row["maximum_missing_run"] <= 3, row["abrupt_left_jump"] <= 2, row["abrupt_right_jump"] <= 2,
            row["crossing"] == 0, row["states"].endswith("TRACKING"), row["hard_no_seam_negative_pass"],
            row["processing_p95"] < 100))})
    passes = [r["weight"] for r in hard_rows if r["all_thresholds"]]
    recovered_total = {r["weight"]: r["recovered_count"] for r in recovery}
    any_recovery = max(recovered_total.values(), default=0) > recovered_total.get(0.42, 0)
    stable_improvement = any(bool(path_changes[f"{w:.2f}"]["improvement_bands"]) for w in WEIGHTS[1:])
    if passes:
        decision = "A"
        causal = "supported"
        next_step = {"status": "design_only", "unique_change": "none; do not modify production until review", "selected_weight": max(passes, key=lambda w: -abs(w - 0.42))}
    elif any_recovery and stable_improvement:
        decision = "B"
        causal = "partially_supported"
        next_step = {"status": "design_only", "unique_problem": "global geometric precision remains above gate after center-transition recovery; return to candidate/identity evidence in y=2400-2700", "do_not_change": ["second DP parameter", "geometry gate", "production YAML"]}
    else:
        decision = "C"
        causal = "rejected"
        next_step = {"status": "design_only", "unique_problem": "center-transition ablation did not produce a stable causal recovery; do not tune another parameter, return to candidate/identity diagnosis", "do_not_change": ["second DP parameter", "geometry gate", "production YAML"]}
    write_json("next_decision.json", {"choice": decision, "causal_judgment": causal, "threshold_passing_weights": passes,
                                       "hard_gate_rows": hard_rows, "selected_production_weight": None,
                                       "next_step": next_step})
    write_json("diagnostic_summary.json", {"task": "S1-P2E2", "baseline_gate": gate, "decision": decision,
        "causal_judgment": causal, "all_thresholds_passed": bool(passes), "threshold_passing_weights": passes,
        "recovered_dp_lost_bands": recovered_total, "fixed_inputs_before": fixed,
        "negative_suite_note": "The exact P2D suite is retained; only no_seam_negative is a hard negative acceptance case.",
        "production_changes": {"yaml": False, "algorithm_behavior": False, "ros": False,
                                "only_in_memory_override": "path.center_transition_weight"}})
    after = {str(p.relative_to(ROOT)): sha256(p) for p in [RAW, ANNOTATION, GUIDED_CONFIG, STABLE_CONFIG, REFERENCE, ROOT / "tank_vision/stable_seam_tracker.py"]}
    write_json("input_sha256_after.json", {"before": fixed["sha256_before"], "after": after,
                                            "unchanged": fixed["sha256_before"] == after})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
