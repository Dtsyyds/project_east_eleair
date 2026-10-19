#!/usr/bin/env python3
"""S1-P2E4: one reference-anchored signed-edge candidate A/B experiment."""

from __future__ import annotations

import copy
import hashlib
import json
import statistics
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tank_vision.stable_seam_tracker import StableSeamTracker  # noqa: E402
from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from scripts.diagnose_reference_selection_error import (  # noqa: E402
    THRESHOLDS, manual_at, manual_points, original_pair, pstats, reference_curve,
)

OUT = ROOT / "output/s1_p2e4_reference_anchor_candidate"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
CONFIG = ROOT / "configs/s100p_guided_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
STABLE_CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
TRACKER = ROOT / "tank_vision/stable_seam_tracker.py"
ANCHOR_RADIUS = 0.12
EXPECTED_RAW = "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02"


def clean(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [clean(x) for x in value.tolist()]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        value = float(value)
        return value if np.isfinite(value) else None
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


def config_for(anchor: bool) -> dict[str, Any]:
    config = copy.deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
    if anchor:
        config.setdefault("guided_search", {})["reference_anchor_enabled"] = True
        config["guided_search"]["reference_anchor_radius_width_scale"] = ANCHOR_RADIUS
    return config


def result_pair_rows(result: Any, diagnostics: Any, points: tuple[np.ndarray, np.ndarray], reference: Any,
                     image: np.ndarray) -> list[dict[str, Any]]:
    roi = result.roi_original_px
    sy = diagnostics.processed_gray.shape[0] / roi[3]
    selected = {p.band_index: p for p in diagnostics.selected_pairs}
    rows = []
    for band, y_processed in enumerate(diagnostics.scan_ys):
        y = float(roi[1] + y_processed / sy)
        manual = manual_at(y, points)
        ref = reference_curve(reference, y, (image.shape[1], image.shape[0]))
        sel_pair = selected.get(band)
        sel = original_pair(sel_pair, result, diagnostics) if sel_pair else None
        pairs = [original_pair(p, result, diagnostics) for p in diagnostics.all_pairs[band]]
        near = min(pairs, key=lambda p: abs(p["left"] - manual["left"]) + abs(p["right"] - manual["right"])) if pairs else None
        rows.append({"band_index": band, "y_original": y, "manual": manual, "reference": ref,
                     "selected": sel, "manual_nearest": near, "pairs": pairs,
                     "left_candidates_processed": [int(x) for x in diagnostics.left_candidates[band]],
                     "right_candidates_processed": [int(x) for x in diagnostics.right_candidates[band]],
                     "left_candidates_original": [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in diagnostics.left_candidates[band]],
                     "right_candidates_original": [float(roi[0] + int(x) / (diagnostics.processed_gray.shape[1] / roi[2])) for x in diagnostics.right_candidates[band]],
                     "pair_count": len(pairs), "selected_pair_rank": next((i + 1 for i, p in enumerate(pairs) if sel and p["pair_id"] == sel["pair_id"]), None),
                     "manual_nearest_pair_rank": next((i + 1 for i, p in enumerate(pairs) if near and p["pair_id"] == near["pair_id"]), None),
                     "selected_reference_distance_l1": None if sel is None else abs(sel["left"] - ref["left"]) + abs(sel["right"] - ref["right"])})
    return rows


def error_report(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    output = {}
    for field in THRESHOLDS:
        values = [row[key][field] for row in rows if row.get(key) is not None]
        output[field] = {"signed": pstats(values), "absolute": pstats([abs(v) for v in values])}
    output["count"] = sum(row.get(key) is not None for row in rows)
    return output


def path_result(weight_name: str, config: dict[str, Any], image: np.ndarray, points: tuple[np.ndarray, np.ndarray]) -> dict[str, Any]:
    tracker = StableSeamTracker(config, REFERENCE)
    results = []
    first_diag = None
    for index in range(3):
        result = tracker.process(image, diagnostics=index == 0, provenance=index == 0)
        results.append(result)
        if index == 0:
            first = result
            first_diag = tracker.last_diagnostics
            first_provenance = tracker.last_provenance
    assert first_diag is not None
    rows = result_pair_rows(first, first_diag, points, tracker.reference, image)
    for row in rows:
        row["selected_error"] = None if row["selected"] is None else {field: row["selected"][field] - row["manual"][field] for field in THRESHOLDS}
        row["manual_nearest_error"] = None if row["manual_nearest"] is None else {field: row["manual_nearest"][field] - row["manual"][field] for field in THRESHOLDS}
    errors = {field: [row["selected_error"][field] for row in rows if row["selected_error"] is not None] for field in THRESHOLDS}
    path_confidence = float(np.mean([p.score for p in first_diag.selected_pairs])) if first_diag.selected_pairs else 0.0
    anchor = [item.get("reference_anchor") for item in first_diag.candidate_search]
    anchor_stats = summarize_anchor(anchor, rows, first_diag)
    return {"variant": weight_name, "config_center_transition_weight": config["path"]["center_transition_weight"],
            "anchor_enabled": bool(config.get("guided_search", {}).get("reference_anchor_enabled", False)),
            "first_frame": first.to_dict(), "final_frame_after_three_repeats": results[-1].to_dict(),
            "state_sequence": [r.tracking_state.name for r in results],
            "temporal_rejection": any(any(x.startswith("temporal_") for x in r.failure_reasons) for r in results),
            "rows": rows, "selected_path_count": len(first_diag.selected_pairs), "coverage": first.coverage,
            "effective_span": first.effective_span, "maximum_missing_run": first.maximum_missing_run,
            "abrupt_left_jump": first.abrupt_left_jump_count, "abrupt_right_jump": first.abrupt_right_jump_count,
            "crossing": sum(int(p.left >= p.right) for p in first_diag.selected_pairs), "path_confidence": path_confidence,
            "joint_inlier_ratio": first.fit_inlier_ratio, "fit_residual_px": first.fit_residual_px,
            "detected": first.detected, "three_repeat_tracking": results[-1].tracking_state == TrackingState.TRACKING,
            "errors": error_report(rows, "selected_error"), "manual_nearest_errors": error_report(rows, "manual_nearest_error"),
            "anchor_statistics": anchor_stats, "provenance_enabled": first_provenance is not None,
            "provenance_candidate_stage_count": len(first_provenance.get("candidate_stage_trace", [])) if first_provenance else 0}


def summarize_anchor(anchor_rows: list[dict[str, Any] | None], rows: list[dict[str, Any]], diagnostics: Any) -> dict[str, Any]:
    if not anchor_rows or not any(anchor_rows):
        return {"enabled": False}
    stats = {"left": {"valid": 0, "invalid": 0, "merged": 0, "replaced": 0, "new_retained": 0, "formed_pair": 0, "selected": 0},
             "right": {"valid": 0, "invalid": 0, "merged": 0, "replaced": 0, "new_retained": 0, "formed_pair": 0, "selected": 0}}
    for band, entry in enumerate(anchor_rows):
        if entry is None:
            continue
        for side in ("left", "right"):
            item = entry[side]
            destination = stats[side]
            destination["valid" if item.get("valid") else "invalid"] += 1
            destination["merged"] += int(item.get("merged_with_existing", False))
            destination["replaced"] += int(item.get("replaced_existing", False))
            destination["new_retained"] += int(item.get("valid") and not item.get("merged_with_existing", False) and item.get("retained_in_final_candidates", False))
            final_candidates = set(entry[f"final_{side}_candidates_processed"])
            pair_has = any(int(round(p.left if side == "left" else p.right)) == int(item.get("position_processed", -999)) for p in diagnostics.all_pairs[band])
            selected_has = any(int(round(p.left if side == "left" else p.right)) == int(item.get("position_processed", -999)) for p in diagnostics.selected_pairs if p.band_index == band)
            destination["formed_pair"] += int(pair_has)
            destination["selected"] += int(selected_has)
            item["final_candidate_count"] = len(final_candidates)
    return {"enabled": True, "left": stats["left"], "right": stats["right"],
            "total_valid_generated": stats["left"]["valid"] + stats["right"]["valid"],
            "total_merged": stats["left"]["merged"] + stats["right"]["merged"],
            "total_replaced": stats["left"]["replaced"] + stats["right"]["replaced"]}


def candidate_recall(result: dict[str, Any], tolerance: float) -> dict[str, Any]:
    bands = [row for row in result["rows"] if row["manual"]]
    left = right = both = pair = 0
    for row in bands:
        ml, mr = row["manual"]["left"], row["manual"]["right"]
        left_hit = any(abs(x - ml) <= tolerance for x in row["left_candidates_original"])
        right_hit = any(abs(x - mr) <= tolerance for x in row["right_candidates_original"])
        # Side candidate presence is derived from pair endpoints in the final
        # returned pair list; the separate raw side arrays are in diagnostics.
        pair_hit = row["manual_nearest"] is not None and abs(row["manual_nearest"]["left"] - ml) <= tolerance and abs(row["manual_nearest"]["right"] - mr) <= tolerance
        left += int(left_hit); right += int(right_hit); both += int(left_hit and right_hit); pair += int(pair_hit)
    return {"tolerance_px": tolerance, "bands": len(bands), "left_candidate_presence": left / max(len(bands), 1),
            "right_candidate_presence": right / max(len(bands), 1), "both_side_candidate_presence": both / max(len(bands), 1),
            "manual_near_pair_presence": pair / max(len(bands), 1), "counts": {"left": left, "right": right, "both": both, "pair": pair}}


def benchmark(config: dict[str, Any], image: np.ndarray, iterations: int = 300) -> dict[str, Any]:
    tracker = StableSeamTracker(config, REFERENCE)
    for _ in range(30):
        tracker.process(image)
    values = []
    for _ in range(iterations):
        values.append(float(tracker.process(image).processing_ms))
    return {"warmup_frames": 30, "measured_frames": iterations,
            "processing_ms": {"mean": statistics.fmean(values), "p50": float(np.percentile(values, 50)),
                               "p95": float(np.percentile(values, 95)), "max": max(values)},
            "p95_lt_100ms": float(np.percentile(values, 95)) < 100.0}


def hard_negative(config: dict[str, Any], image: np.ndarray) -> dict[str, Any]:
    negative = np.full_like(image, int(np.mean(image)))
    tracker = StableSeamTracker(config, REFERENCE)
    result = tracker.process(negative)
    crossing = sum(int(l[0] >= r[0]) for l, r in zip(result.left_boundary_original_px, result.right_boundary_original_px))
    return {"input": "P2D/P2E2 hard no-seam negative", "detected": result.detected,
            "state": result.tracking_state.name, "crossing": crossing,
            "hard_negative_pass": not result.detected and result.tracking_state == TrackingState.LOST and crossing == 0,
            "other_p2d_stress_samples": "not included as hard-negative acceptance cases"}


def focus_rows(a: dict[str, Any], b: dict[str, Any]) -> list[dict[str, Any]]:
    by_a = {r["band_index"]: r for r in a["rows"]}
    by_b = {r["band_index"]: r for r in b["rows"]}
    output = []
    for band in sorted(set(by_a) | set(by_b)):
        if not 2400 <= by_a[band]["y_original"] <= 2700:
            continue
        output.append({"band_index": band, "y_original": by_a[band]["y_original"], "manual": by_a[band]["manual"],
                       "reference": by_a[band]["reference"], "A": {"pairs": by_a[band]["pairs"], "selected": by_a[band]["selected"]},
                       "B": {"pairs": by_b[band]["pairs"], "selected": by_b[band]["selected"]},
                       "anchor": next((x for x in b["anchor_statistics"].get("trace", []) if x["band_index"] == band), None),
                       "A_selected_reference_distance_l1": by_a[band]["selected_reference_distance_l1"],
                       "B_selected_reference_distance_l1": by_b[band]["selected_reference_distance_l1"]})
    return output


def draw_comparison(image: np.ndarray, a: dict[str, Any], b: dict[str, Any]) -> None:
    panels = []
    for item, color, title in ((a, (0, 0, 255), "A anchor=false"), (b, (255, 0, 0), "B anchor=true")):
        canvas = image.copy()
        for name, c in (("manual", (0, 255, 255)), ("reference", (0, 165, 255)), ("selected", color)):
            left = [(int(round(r[name]["left"])), int(round(r["y_original"]))) for r in item["rows"] if r.get(name)]
            right = [(int(round(r[name]["right"])), int(round(r["y_original"]))) for r in item["rows"] if r.get(name)]
            if len(left) > 1:
                cv2.polylines(canvas, [np.asarray(left, dtype=np.int32).reshape(-1, 1, 2)], False, c, 6, cv2.LINE_AA)
                cv2.polylines(canvas, [np.asarray(right, dtype=np.int32).reshape(-1, 1, 2)], False, c, 6, cv2.LINE_AA)
        cv2.putText(canvas, title, (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(canvas, title, (25, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2, cv2.LINE_AA)
        canvas = cv2.resize(canvas, (1024, 768), interpolation=cv2.INTER_AREA)
        panels.append(canvas)
    cv2.imwrite(str(OUT / "comparison_overlay.png"), np.hstack(panels))


def profile_panel(image: np.ndarray, row_a: dict[str, Any], row_b: dict[str, Any], config: dict[str, Any], diagnostics_a: Any) -> np.ndarray:
    gray = diagnostics_a.processed_gray
    half = int(config["processing"]["scan_band_half_height_px"])
    sobel = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(config["processing"]["sobel_kernel"]))
    profile = cv2.GaussianBlur(cv2.blur(sobel, (1, 2 * half + 1)), (int(config["processing"]["profile_gaussian_kernel"]), 1), 0)[int(diagnostics_a.scan_ys[row_a["band_index"]])]
    width, height = 720, 360
    canvas = np.full((height, width, 3), 245, dtype=np.uint8)
    roi = diagnostics_a.roi_bgr.shape[1], diagnostics_a.roi_bgr.shape[0]
    x_values = []
    for row in (row_a, row_b):
        for name in ("manual", "reference", "selected"):
            if row.get(name):
                x_values.extend([int(round((row[name]["left"] - 1229) * gray.shape[1] / 1229)), int(round((row[name]["right"] - 1229) * gray.shape[1] / 1229))])
    x_values.extend([p["left"] for p in row_a["pairs"]]); x_values.extend([p["right"] for p in row_a["pairs"]])
    x0, x1 = max(0, min(x_values) - 45), min(len(profile) - 1, max(x_values) + 45)
    def xp(x): return int(20 + (x - x0) * 680 / max(x1 - x0, 1))
    scale = max(np.max(np.abs(profile)), 1.0)
    pts = [(xp(x), int(180 - profile[x] / scale * 130)) for x in range(x0, x1 + 1)]
    cv2.polylines(canvas, [np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)], False, (170, 0, 170), 2, cv2.LINE_AA)
    colors = {"manual": (0, 180, 180), "reference": (0, 140, 255), "selected": (0, 0, 230)}
    for name in ("manual", "reference", "selected"):
        for row, variant in ((row_a, "A"), (row_b, "B")):
            if row.get(name):
                for side in ("left", "right"):
                    x = int(round((row[name][side] - 1229) * gray.shape[1] / 1229))
                    cv2.line(canvas, (xp(x), 20), (xp(x), 330), colors[name], 2)
                    cv2.putText(canvas, f"{variant}{name[0]}{side[0]}", (xp(x) + 2, 18 + (0 if side == "left" else 16)), cv2.FONT_HERSHEY_SIMPLEX, 0.35, colors[name], 1, cv2.LINE_AA)
    for p in row_a["pairs"]:
        for side in ("left", "right"):
            x = int(round((p[side] - 1229) * gray.shape[1] / 1229)); cv2.circle(canvas, (xp(x), int(180 - profile[x] / scale * 130)), 3, (80, 80, 80), -1)
    cv2.putText(canvas, f"y={row_a['y_original']:.1f}; purple=signed Sobel; A/B lines", (10, 350), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    return canvas


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    before = {"raw": sha256(RAW), "annotation": sha256(ANNOTATION), "config": sha256(CONFIG),
              "stable_config": sha256(STABLE_CONFIG), "reference": sha256(REFERENCE), "tracker": sha256(TRACKER)}
    if before["raw"] != EXPECTED_RAW:
        raise RuntimeError("raw SHA mismatch")
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read raw.bmp")
    points = manual_points()
    a = path_result("A", config_for(False), image, points)
    baseline = {"status": "matched" if (a["selected_path_count"] == 116 and abs(a["coverage"] - 116 / 117) < 1e-9 and
                  a["state_sequence"] == ["ACQUIRING", "ACQUIRING", "TRACKING"] and
                  abs(a["errors"]["center"]["absolute"]["p95"] - 44.14758855034813) < 1e-6 and
                  abs(a["errors"]["left"]["absolute"]["p95"] - 59.276740922960016) < 1e-6 and
                  abs(a["errors"]["right"]["absolute"]["p95"] - 47.91756560334744) < 1e-6 and
                  abs(a["errors"]["width"]["absolute"]["p95"] - 70.421110320392) < 1e-6) else "baseline_mismatch", "A": a}
    write_json("baseline_reproduction.json", baseline)
    if baseline["status"] != "matched":
        return 2
    b = path_result("B", config_for(True), image, points)
    recalls = {"A": {str(t): candidate_recall(a, t) for t in (15, 25, 40)},
               "B": {str(t): candidate_recall(b, t) for t in (15, 25, 40)}}
    write_json("candidate_recall_comparison.json", recalls)
    write_json("manual_nearest_oracle_comparison.json", {"A": a["manual_nearest_errors"], "B": b["manual_nearest_errors"],
                                                           "definition": "nearest pair from actual final returned pair list; no oracle pair construction"})
    write_json("final_path_comparison.json", {"A": a, "B": b})
    anchor_trace = {"A": [{"band_index": i, "enabled": False} for i in range(117)],
                    "B": [{"band_index": i, "reference_anchor": x, "final_pairs": b["rows"][i]["pairs"],
                           "selected_pair": b["rows"][i]["selected"]} for i, x in enumerate([None] * 117)]}
    # The detailed trace is taken directly from the enabled tracker diagnostics
    # through a fresh provenance run, avoiding any reconstructed candidate list.
    traced = StableSeamTracker(config_for(True), REFERENCE)
    traced.process(image, diagnostics=True, provenance=True)
    trace = traced.last_provenance or {}
    anchor_trace["B"] = [{"band_index": i, "candidate_search": trace.get("candidate_stage_trace", [])[i].get("reference_anchor"),
                          "final_pairs": b["rows"][i]["pairs"], "selected_pair": b["rows"][i]["selected"]}
                         for i in range(117)]
    # Keep a direct per-band trace also in the result object for counts/audit.
    b["anchor_statistics"]["trace"] = anchor_trace["B"]
    write_json("anchor_candidate_trace.json", anchor_trace)
    focus = focus_rows(a, b)
    write_json("y_2400_2700_comparison.json", {"rows": focus})
    write_json("negative_regression.json", {"A": hard_negative(config_for(False), image), "B": hard_negative(config_for(True), image)})
    benchmarks = {"A": benchmark(config_for(False), image), "B": benchmark(config_for(True), image)}
    write_json("benchmark.json", benchmarks)
    draw_comparison(image, a, b)
    # Reuse the first diagnostics object only for the exact production profile
    # coordinates; B/A candidate positions are overlaid from returned pairs.
    ta = StableSeamTracker(config_for(False), REFERENCE); ta.process(image, diagnostics=True)
    da = ta.last_diagnostics
    key_rows = [min([r for r in a["rows"] if 2400 <= r["y_original"] <= 2700], key=lambda r: abs(r["y_original"] - target)) for target in (2442, 2453, 2661, 2672)]
    panels = [profile_panel(image, row, b["rows"][row["band_index"]], config_for(False), da) for row in key_rows]
    cv2.imwrite(str(OUT / "key_band_profile_montage.png"), np.hstack(panels))
    after = {"raw": sha256(RAW), "annotation": sha256(ANNOTATION), "config": sha256(CONFIG),
             "stable_config": sha256(STABLE_CONFIG), "reference": sha256(REFERENCE), "tracker": sha256(TRACKER)}
    a_by_band = {row["band_index"]: row for row in a["rows"]}
    b_by_band = {row["band_index"]: row for row in b["rows"]}
    changed_bands = []
    improved_bands = []
    worsened_over_10px = []
    for band in sorted(a_by_band):
        old, new = a_by_band[band], b_by_band[band]
        old_id = None if old["selected"] is None else old["selected"]["pair_id"]
        new_id = None if new["selected"] is None else new["selected"]["pair_id"]
        if old_id == new_id:
            continue
        changed_bands.append({"band_index": band, "y_original": old["y_original"], "A_pair_id": old_id, "B_pair_id": new_id})
        if old["selected_error"] is not None and new["selected_error"] is not None:
            delta = max(abs(new["selected_error"][field]) - abs(old["selected_error"][field]) for field in THRESHOLDS)
            if delta <= -10:
                improved_bands.append({"band_index": band, "y_original": old["y_original"], "max_error_delta": delta})
            if delta > 10:
                worsened_over_10px.append({"band_index": band, "y_original": old["y_original"], "max_error_delta": delta})
    gates = {}
    for name, item in (("A", a), ("B", b)):
        gates[name] = {"center_p95": item["errors"]["center"]["absolute"]["p95"] <= 25,
                       "left_p95": item["errors"]["left"]["absolute"]["p95"] <= 30,
                       "right_p95": item["errors"]["right"]["absolute"]["p95"] <= 30,
                       "width_p95": item["errors"]["width"]["absolute"]["p95"] <= 30,
                       "joint_inlier": item["joint_inlier_ratio"] >= 0.75, "coverage": item["coverage"] >= 0.85,
                       "effective_span": item["effective_span"] >= 0.85, "missing_run": item["maximum_missing_run"] <= 3,
                       "abrupt_left": item["abrupt_left_jump"] <= 2, "abrupt_right": item["abrupt_right_jump"] <= 2,
                       "crossing": item["crossing"] == 0, "tracking": item["three_repeat_tracking"],
                       "negative": json.loads((OUT / "negative_regression.json").read_text())[name]["hard_negative_pass"],
                       "processing": benchmarks[name]["p95_lt_100ms"]}
    b_improved_manual = b["manual_nearest_errors"]["center"]["absolute"]["p95"] < a["manual_nearest_errors"]["center"]["absolute"]["p95"]
    all_b = all(gates["B"].values()) and b_improved_manual
    if all_b:
        choice = "A"
    elif b_improved_manual or b["anchor_statistics"].get("total_valid_generated", 0) > 0:
        choice = "B"
    else:
        choice = "C"
    write_json("diagnostic_summary.json", {"task": "S1-P2E4", "choice": choice, "baseline_status": baseline["status"],
        "gates": gates, "manual_nearest_center_improved": b_improved_manual,
        "anchor_statistics": b["anchor_statistics"], "fixed_input_sha256_before": before,
        "fixed_input_sha256_after": after, "fixed_input_hashes_unchanged": before == after,
        "path_identity_changes": {"changed_band_count": len(changed_bands), "changed_bands": changed_bands,
                                  "improved_over_10px": improved_bands, "worsened_over_10px": worsened_over_10px},
        "production_changes": {"yaml": False, "reference": False, "dp": False, "unary": False, "gate": False, "ros": False},
        "stress_semantics": "only no-seam negative is a hard-negative acceptance case"})
    write_json("next_decision.json", {"choice": choice, "implemented": False,
        "unique_next_issue": "candidate localization remains insufficient near reference/manual boundaries" if choice in ("B", "C") else "none",
        "do_not_change": ["radius", "second parameter", "DP", "unary", "gate", "production YAML", "ROS"]})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
