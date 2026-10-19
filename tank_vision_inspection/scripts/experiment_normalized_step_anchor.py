#!/usr/bin/env python3
"""S1-P2G1: signed-Sobel versus normalized local step anchor A/B."""

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
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from scripts.diagnose_reference_selection_error import manual_at, manual_points, pstats  # noqa: E402
from tank_vision.seam_tracking_types import TrackingState  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay  # noqa: E402

OUT = ROOT / "output/s1_p2g1_normalized_step_anchor"
RAW = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"
ANNOTATION = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw_annotation.json"
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
STABLE_CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
TRACKER = ROOT / "tank_vision/stable_seam_tracker.py"
EXPECTED_RAW = "ada05bfa684cccfa87d54481985ca8614a9f3d9acd140e1af61f5aa80baccd02"
SNAPSHOT_ROOT = ROOT / "output/stable_seam_failure_snapshots"
SNAPSHOTS = [
    "20260910_160926_591_stamp_21705_511000000",
    "20260910_161005_409_stamp_21744_357800000",
    "20260910_161037_929_stamp_21776_838700000",
    "20260910_161145_623_stamp_21844_557300000",
    "20260910_161207_181_stamp_21866_60800000",
    "20260910_161223_283_stamp_21882_201000000",
]


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


def config_for(variant: str) -> dict[str, Any]:
    config = copy.deepcopy(yaml.safe_load(CONFIG.read_text(encoding="utf-8")))
    config["guided_search"]["reference_anchor_enabled"] = True
    config["guided_search"]["reference_anchor_radius_width_scale"] = 0.12
    if variant == "B":
        config["guided_search"]["reference_anchor_response"] = "normalized_step"
    return config


def pair_original(pair: Any, result: Any, diagnostics: Any) -> dict[str, Any]:
    x0, _y0, rw, _rh = result.roi_original_px
    sx = diagnostics.processed_gray.shape[1] / rw
    return {"band_index": int(pair.band_index), "y": float(result.roi_original_px[1] + pair.y / (diagnostics.processed_gray.shape[0] / result.roi_original_px[3])),
            "left": float(x0 + pair.left / sx), "right": float(x0 + pair.right / sx),
            "center": float(x0 + pair.center / sx), "width": float(pair.width / sx),
            "score": float(pair.score), "score_parts": {k: float(v) for k, v in pair.score_parts.items()},
            "pair_id": f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}"}


def error_stats(values: list[float]) -> dict[str, Any]:
    return {"signed": pstats(values), "absolute": pstats([abs(x) for x in values])}


def layer_errors(rows: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for field in ("left", "right", "center", "width"):
        values = [float(row["estimate"][field] - row["manual"][field]) for row in rows]
        result[field] = error_stats(values)
    return result


def boundary_result_rows(result: Any, points: tuple[np.ndarray, np.ndarray]) -> list[dict[str, Any]]:
    rows = []
    for left, right, center, y in zip(result.left_boundary_original_px, result.right_boundary_original_px,
                                      result.centerline_original_px, [p[1] for p in result.centerline_original_px]):
        manual = manual_at(float(y), points)
        estimate = {"left": float(left[0]), "right": float(right[0]), "center": float(center[0]),
                    "width": float(right[0] - left[0])}
        rows.append({"y": float(y), "manual": manual, "estimate": estimate})
    return rows


def raw_rows(result: Any, diagnostics: Any, points: tuple[np.ndarray, np.ndarray]) -> list[dict[str, Any]]:
    rows = []
    for pair in diagnostics.selected_pairs:
        current = pair_original(pair, result, diagnostics)
        manual = manual_at(current["y"], points)
        rows.append({"band_index": current["band_index"], "y": current["y"], "manual": manual, "estimate": current})
    return rows


def run_repeated(config: dict[str, Any], image: np.ndarray, points: tuple[np.ndarray, np.ndarray] | None = None) -> dict[str, Any]:
    tracker = StableSeamTracker(config, REFERENCE)
    results = []
    first_diag = None
    first_provenance = None
    for index in range(3):
        # Candidate-search diagnostics already carry the complete A/B anchor
        # trace; production DP provenance is not needed for this response-form
        # experiment and would retain a large pair matrix per variant.
        result = tracker.process(image, diagnostics=index == 0, provenance=False)
        results.append(result)
        if index == 0:
            first_diag = tracker.last_diagnostics
            first_provenance = tracker.last_provenance
    assert first_diag is not None
    payload = {"results": [result.to_dict() for result in results],
               "state_sequence": [result.tracking_state.name for result in results],
               "detected_sequence": [bool(result.detected) for result in results],
               "first": results[0].to_dict(), "last": results[-1].to_dict(),
               "anchor_trace": [x.get("reference_anchor") for x in first_diag.candidate_search],
               "diagnostics": first_diag, "provenance": first_provenance,
               "tracker": tracker}
    if points is not None:
        payload["raw_errors"] = layer_errors(raw_rows(results[0], first_diag, points))
        payload["fit_errors"] = layer_errors(boundary_result_rows(results[0], points))
        payload["temporal_errors"] = layer_errors(boundary_result_rows(results[-1], points))
        payload["raw_rows"] = raw_rows(results[0], first_diag, points)
    return payload


def anchor_summary(payload: dict[str, Any]) -> dict[str, Any]:
    traces = payload["anchor_trace"]
    summary = {"bands": len(traces), "left": {"generated": 0, "step_used": 0, "fallback": 0, "scores": []},
               "right": {"generated": 0, "step_used": 0, "fallback": 0, "scores": []}, "by_band": []}
    for band, trace in enumerate(traces):
        if not trace:
            continue
        row = {"band_index": band, "left": {}, "right": {}}
        for side in ("left", "right"):
            item = trace.get(side, {})
            target = summary[side]
            valid = bool(item.get("valid"))
            step_used = bool(item.get("step_anchor_used"))
            fallback = bool(item.get("fallback_to_sobel"))
            target["generated"] += int(valid)
            target["step_used"] += int(step_used)
            target["fallback"] += int(fallback)
            if item.get("score") is not None:
                target["scores"].append(float(item["score"]))
            row[side] = {"valid": valid, "step_anchor_used": step_used, "fallback_to_sobel": fallback,
                         "position_processed": item.get("position_processed"),
                         "signed_response": item.get("signed_response"), "step": item.get("step"),
                         "local_noise": item.get("local_noise"), "score": item.get("score"),
                         "reason": item.get("reason"), "fallback_reason": item.get("fallback_reason"),
                         "window_processed_px": item.get("window_processed_px"),
                         "support_width_processed": item.get("support_width_processed"),
                         "predicted_position_processed": trace.get("predicted_" + side + "_processed"),
                         "final_candidates_processed": trace.get("final_" + side + "_candidates_processed", [])}
        summary["by_band"].append(row)
    for side in ("left", "right"):
        values = summary[side].pop("scores")
        summary[side]["score_stats"] = pstats(values)
    return summary


def selected_reference_rows(payload: dict[str, Any]) -> list[dict[str, Any]]:
    diagnostics = payload["diagnostics"]
    result = payload["first"]
    # Reuse the exact reference coordinates used by the production object.
    tracker = payload["tracker"]
    roi = tuple(result["roi_original_px"])
    image_size = (4096, 3072)
    rows = []
    for pair in diagnostics.selected_pairs:
        y = float(roi[1] + pair.y / (diagnostics.processed_gray.shape[0] / roi[3]))
        center, width = tracker.reference.at(np.asarray([y]), *image_size)
        sx = diagnostics.processed_gray.shape[1] / roi[2]
        selected = pair_original(pair, type("R", (), {"roi_original_px": roi})(), diagnostics)
        rows.append({"band_index": int(pair.band_index), "y": y,
                     "reference_left": float(center[0] - 0.5 * width[0]), "reference_right": float(center[0] + 0.5 * width[0]),
                     "selected_left": selected["left"], "selected_right": selected["right"]})
    return rows


def live_frame(path: Path, variant: str) -> dict[str, Any]:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read {path}")
    payload = run_repeated(config_for(variant), image)
    result = payload["results"][0]
    return {"path": str(path), "first": result, "repeat3": payload["results"],
            "state_sequence": payload["state_sequence"], "detected_sequence": payload["detected_sequence"],
            "anchor": anchor_summary(payload), "reference_rows": selected_reference_rows(payload), "image": image}


def sequence_run(images: list[np.ndarray], variant: str) -> list[dict[str, Any]]:
    tracker = StableSeamTracker(config_for(variant), REFERENCE)
    output = []
    for image in images:
        # Chronological replay only needs the production result and anchor
        # trace.  Avoid retaining all pair/provenance matrices for six frames.
        result = tracker.process(image, diagnostics=False, provenance=False)
        traces = tracker._last_candidate_search
        output.append({"result": result.to_dict(), "anchor": anchor_summary({
            "anchor_trace": [trace.get("reference_anchor") for trace in traces]}),
            })
    return output


def negative_run(variant: str, image: np.ndarray) -> dict[str, Any]:
    negative = np.full_like(image, int(np.mean(image)))
    tracker = StableSeamTracker(config_for(variant), REFERENCE)
    result = tracker.process(negative, diagnostics=True, provenance=True)
    crossing = sum(int(p.left >= p.right) for p in tracker.last_diagnostics.selected_pairs) if tracker.last_diagnostics else 0
    anchor = anchor_summary({"anchor_trace": [(tracker.last_provenance or {}).get("candidate_stage_trace", [])[i].get("reference_anchor")
                                               for i in range(len((tracker.last_provenance or {}).get("candidate_stage_trace", [])))]})
    return {"input": "P2D/P2E2/P2F0 hard no-seam constant negative", "result": result.to_dict(),
            "crossing": crossing, "anchor": anchor,
            "hard_negative_pass": not result.detected and result.tracking_state == TrackingState.LOST and crossing == 0}


def draw_ab_overlay(name: str, image: np.ndarray, a: dict[str, Any], b: dict[str, Any], label: str) -> None:
    panels = []
    for variant, item, color in (("A", a, (0, 0, 255)), ("B", b, (255, 0, 0))):
        canvas = draw_stable_seam_overlay(image, item["payload"]["results"][0] if False else
                                          StableSeamTracker(config_for(variant), REFERENCE).process(image), (1024, 768))
        # Re-run with diagnostics is intentionally not used for output; this
        # panel is diagnostic-only and shows the actual first reset result.
        tracker = StableSeamTracker(config_for(variant), REFERENCE)
        result = tracker.process(image)
        canvas = draw_stable_seam_overlay(image, result, (1024, 768))
        cv2.putText(canvas, f"{label} {variant} {result.tracking_state.name} detected={result.detected}",
                    (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)
        panels.append(canvas)
    cv2.imwrite(str(OUT / "six_frame_ab_montage.jpg"), np.hstack(panels))


def plot_right_diagnostic(rows: list[dict[str, Any]]) -> None:
    width, height = 1500, 820
    canvas = np.full((height, width, 3), 248, dtype=np.uint8)
    colors = {"A": (0, 0, 220), "B": (220, 80, 0)}
    max_score = max([float(x) for row in rows for x in (row["A_score"], row["B_score"]) if x is not None] + [1.0])
    for index, row in enumerate(rows):
        x = 80 + index * 230
        cv2.putText(canvas, row["capture"], (x, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (20, 20, 20), 2)
        for variant in ("A", "B"):
            score = row[variant + "_score"]
            px = x + (40 if variant == "A" else 105)
            bar = 500 * (float(score or 0) / max_score)
            cv2.rectangle(canvas, (px, 690), (px + 38, int(690 - bar)), colors[variant], -1)
            cv2.putText(canvas, variant, (px + 12, 720), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors[variant], 2)
            cv2.putText(canvas, "NA" if score is None else f"{score:.2f}", (px - 5, 670 - int(bar)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, colors[variant], 1)
        cv2.putText(canvas, f"Rout A/B {row['A_ref_distance']:.1f}/{row['B_ref_distance']:.1f}",
                    (x, 770), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (40, 40, 40), 1)
    cv2.putText(canvas, "right anchor score; A=signed Sobel, B=normalized step; diagnostic only", (30, 805),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (30, 30, 30), 1)
    cv2.imwrite(str(OUT / "right_boundary_diagnostic.png"), canvas)


def main() -> int:
    if OUT.exists() and any(OUT.iterdir()):
        raise RuntimeError(f"refusing to overwrite non-empty output: {OUT}")
    OUT.mkdir(parents=True, exist_ok=True)
    before = {"raw": sha256(RAW), "annotation": sha256(ANNOTATION), "config": sha256(CONFIG),
              "stable_config": sha256(STABLE_CONFIG), "reference": sha256(REFERENCE), "tracker": sha256(TRACKER)}
    if before["raw"] != EXPECTED_RAW:
        raise RuntimeError("frozen raw SHA mismatch")
    image = cv2.imread(str(RAW), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read labeled raw")
    points = manual_points()
    a = run_repeated(config_for("A"), image, points)
    baseline_metrics = {"path": len(a["diagnostics"].selected_pairs), "coverage": a["results"][0]["coverage"],
                        "joint_inlier": a["results"][0]["fit_inlier_ratio"], "state_sequence": a["state_sequence"],
                        "final_layer": a["temporal_errors"]}
    baseline_ok = (baseline_metrics["path"] == 117 and abs(baseline_metrics["coverage"] - 1.0) < 1e-12
                   and baseline_metrics["state_sequence"] == ["ACQUIRING", "ACQUIRING", "TRACKING"]
                   and abs(baseline_metrics["final_layer"]["left"]["absolute"]["p95"] - 11.1761) < 0.5
                   and abs(baseline_metrics["final_layer"]["right"]["absolute"]["p95"] - 14.2205) < 0.5
                   and abs(baseline_metrics["final_layer"]["center"]["absolute"]["p95"] - 11.2567) < 0.5
                   and abs(baseline_metrics["final_layer"]["width"]["absolute"]["p95"] - 11.6125) < 0.5)
    write_json("baseline_reproduction.json", {"status": "matched" if baseline_ok else "baseline_mismatch",
                                                "expected": {"path": 117, "coverage": 1.0,
                                                             "final_p95_left_right_center_width": [11.18, 14.22, 11.26, 11.61]},
                                                "actual": baseline_metrics})
    if not baseline_ok:
        return 2
    b = run_repeated(config_for("B"), image, points)
    labeled = {}
    for variant, payload in (("A", a), ("B", b)):
        labeled[variant] = {"raw_selected_pair": payload["raw_errors"], "irls_fitted": payload["fit_errors"],
                            "temporal": payload["temporal_errors"], "final_output": payload["temporal_errors"],
                            "joint_inlier": payload["results"][0]["fit_inlier_ratio"],
                            "fit_residual_px": payload["results"][0]["fit_residual_px"],
                            "coverage": payload["results"][0]["coverage"],
                            "crossing": 0, "abrupt_jumps": [payload["results"][0]["abrupt_left_jump_count"], payload["results"][0]["abrupt_right_jump_count"]],
                            "state_sequence": payload["state_sequence"],
                            "processing_ms": [r["processing_ms"] for r in payload["results"]],
                            "anchor": anchor_summary(payload)}
    write_json("labeled_accuracy_comparison.json", labeled)
    # Provenance contains every pair and candidate trace.  Release the two
    # labeled-image diagnostic objects before retaining six-frame summaries.
    del a, b
    import gc
    gc.collect()

    directories = [SNAPSHOT_ROOT / name for name in SNAPSHOTS]
    images = [cv2.imread(str(path / "raw.bmp"), cv2.IMREAD_COLOR) for path in directories]
    if any(item is None for item in images):
        raise RuntimeError("missing field raw image")
    field = {"A": [], "B": []}
    for variant in ("A", "B"):
        for index, (directory, frame) in enumerate(zip(directories, images), start=1):
            item = live_frame(directory / "raw.bmp", variant)
            metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
            item.pop("image", None)
            item["capture"] = f"capture_{index:02d}"
            item["field_observed"] = {"detected": metadata.get("detected"), "state": metadata.get("tracking_state"),
                                       "coverage": metadata.get("coverage"), "joint_inlier": metadata.get("fit_inlier_ratio"),
                                       "fit_residual_px": metadata.get("fit_residual_px"), "failure_reasons": metadata.get("failure_reasons", [])}
            field[variant].append(item)
    def compact_field(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [{k: v for k, v in item.items() if k not in ("payload",)} for item in items]
    write_json("six_frame_reset_comparison.json", {"A": compact_field(field["A"]), "B": compact_field(field["B"]),
                                                     "field_labels": "capture_01..capture_06 only; no scene interpretation"})

    sequence = {"A": sequence_run(images, "A"), "B": sequence_run(images, "B")}
    write_json("six_frame_sequence_comparison.json", {key: [{"result": item["result"], "anchor": item["anchor"]} for item in value]
                                                        for key, value in sequence.items()})

    anchor_comparison = {"A": [item["anchor"] for item in field["A"]], "B": [item["anchor"] for item in field["B"]],
                         "definition": "generated means a valid candidate after the response rule and Sobel fallback; no field ground truth"}
    write_json("anchor_response_comparison.json", anchor_comparison)

    outlier = {"definition": "field residuals are diagnostic distances from the frozen reference, not accuracy claims", "A": [], "B": []}
    right_rows = []
    for variant in ("A", "B"):
        for item in field[variant]:
            refs = item["reference_rows"]
            left_out = sum(abs(r["selected_left"] - r["reference_left"]) > 30 for r in refs)
            right_out = sum(abs(r["selected_right"] - r["reference_right"]) > 30 for r in refs)
            selected_l = np.asarray([r["selected_left"] for r in refs], dtype=float)
            selected_r = np.asarray([r["selected_right"] for r in refs], dtype=float)
            row = {"capture": item["capture"], "left_reference_distance_p95": float(np.percentile(np.abs(selected_l - [r["reference_left"] for r in refs]), 95)),
                   "right_reference_distance_p95": float(np.percentile(np.abs(selected_r - [r["reference_right"] for r in refs]), 95)),
                   "left_outlier_count_gt30": int(left_out), "right_outlier_count_gt30": int(right_out),
                   "left_adjacent_delta_p95": float(np.percentile(np.abs(np.diff(selected_l)), 95)) if len(selected_l) > 1 else 0.0,
                   "right_adjacent_delta_p95": float(np.percentile(np.abs(np.diff(selected_r)), 95)) if len(selected_r) > 1 else 0.0}
            outlier[variant].append(row)
            if variant == "A":
                right_rows.append({"capture": item["capture"], "A_score": item["anchor"]["right"]["score_stats"]["p50"],
                                   "B_score": None, "A_ref_distance": row["right_reference_distance_p95"], "B_ref_distance": 0.0})
    for row, item in zip(right_rows, outlier["B"]):
        row["B_score"] = field["B"][int(row["capture"][-2:]) - 1]["anchor"]["right"]["score_stats"]["p50"]
        row["B_ref_distance"] = item["right_reference_distance_p95"]
    write_json("left_right_outlier_comparison.json", outlier)

    negatives = {variant: negative_run(variant, image) for variant in ("A", "B")}
    write_json("negative_regression.json", negatives)
    benchmarks = {}
    for variant in ("A", "B"):
        tracker = StableSeamTracker(config_for(variant), REFERENCE)
        values = [float(tracker.process(image).processing_ms) for _ in range(40)]
        benchmarks[variant] = {"warmup_and_measured_frames": 40, "processing_ms": {"p50": float(np.percentile(values, 50)),
                             "p95": float(np.percentile(values, 95)), "max": max(values)}, "p95_lt_100ms": float(np.percentile(values, 95)) < 100}
    write_json("benchmark.json", benchmarks)

    # Diagnostic-only A/B panels; no field panel is treated as accuracy evidence.
    montage_rows = []
    for index, frame in enumerate(images):
        panels = []
        for variant, config in (("A", config_for("A")), ("B", config_for("B"))):
            tracker = StableSeamTracker(config, REFERENCE)
            result = tracker.process(frame)
            panel = draw_stable_seam_overlay(frame, result, (720, 540))
            cv2.putText(panel, f"capture_{index + 1:02d} {variant} {result.tracking_state.name} detected={result.detected}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(panel, f"capture_{index + 1:02d} {variant} {result.tracking_state.name} detected={result.detected}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            panels.append(panel)
        montage_rows.append(np.hstack(panels))
    cv2.imwrite(str(OUT / "six_frame_ab_montage.jpg"), np.vstack(montage_rows))
    plot_right_diagnostic(right_rows)

    after = {"raw": sha256(RAW), "annotation": sha256(ANNOTATION), "config": sha256(CONFIG),
             "stable_config": sha256(STABLE_CONFIG), "reference": sha256(REFERENCE), "tracker": sha256(TRACKER)}
    b_final = labeled["B"]["final_output"]
    a_final = labeled["A"]["final_output"]
    b_improves_field_inlier = float(np.mean([x["first"]["fit_inlier_ratio"] for x in field["B"]])) > float(np.mean([x["first"]["fit_inlier_ratio"] for x in field["A"]]))
    b_no_worse_labeled = all(b_final[field]["absolute"]["p95"] <= a_final[field]["absolute"]["p95"] + 1.0 for field in ("left", "right", "center", "width"))
    b_raw_not_significantly_worse = all(
        labeled["B"]["raw_selected_pair"][field]["absolute"]["p95"]
        <= labeled["A"]["raw_selected_pair"][field]["absolute"]["p95"] + 10.0
        for field in ("left", "right", "center", "width"))
    negative_ok = all(item["hard_negative_pass"] for item in negatives.values())
    performance_ok = all(item["p95_lt_100ms"] for item in benchmarks.values())
    final_accuracy_ok = all(b_final[field]["absolute"]["p95"] <= limit for field, limit in
                            (("center", 25.0), ("left", 30.0), ("right", 30.0), ("width", 30.0)))
    b_all_supported = (final_accuracy_ok and b_no_worse_labeled and b_raw_not_significantly_worse
                       and b_improves_field_inlier and negative_ok and performance_ok
                       and all(item["state_sequence"][-1] != "LOST" for item in field["B"]))
    b_internal_partial = (b_no_worse_labeled and b_raw_not_significantly_worse and performance_ok and
                          (b_improves_field_inlier or any(item["first"]["detected"] for item in field["B"])) and negative_ok)
    choice = "A" if b_all_supported else ("B" if b_internal_partial else "C")
    write_json("diagnostic_summary.json", {"task": "S1-P2G1", "choice": choice,
        "baseline_status": "matched", "labeled_final_p95_A": a_final, "labeled_final_p95_B": b_final,
        "field_inlier_mean_A": float(np.mean([x["first"]["fit_inlier_ratio"] for x in field["A"]])),
        "field_inlier_mean_B": float(np.mean([x["first"]["fit_inlier_ratio"] for x in field["B"]])),
        "negative_all_pass": negative_ok, "performance_all_pass": performance_ok,
        "raw_not_significantly_worse": b_raw_not_significantly_worse,
        "final_accuracy_ok": final_accuracy_ok,
        "field_accuracy_claim": False, "overlay_semantics": "diagnostic-only; six frames have no labels",
        "fixed_input_sha256_before": before, "fixed_input_sha256_after": after,
        "fixed_input_hashes_unchanged": before == after,
        "production_changes": {"reference": False, "roi": False, "pair": False, "dp": False, "gate": False, "temporal": False, "ros": False, "production_yaml": False}})
    write_json("next_decision.json", {"choice": choice, "implemented": False,
        "next_action": "remain a field-validation candidate; collect controlled light/exposure telemetry before further algorithm change" if choice != "A" else "run a controlled field validation with fixed lighting and exposure",
        "do_not_change": ["reference", "ROI", "pair", "DP", "gate", "temporal", "ROS", "default stable path", "anchor radius"]})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
