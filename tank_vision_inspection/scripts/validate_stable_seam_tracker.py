#!/usr/bin/env python3
"""Generate S1-P1 reference, sequence, real-image, and benchmark evidence."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import resource
import platform
import statistics
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tank_vision.stable_seam_tracker import StableSeamTracker, draw_stable_seam_overlay
from tank_vision.seam_width_reference import sha256_file

CONFIG = ROOT / "configs/s100p_stable_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_width_reference_s100p_v1.json"
IMAGE = ROOT / "output/s1_real_seam_images/seam_center_lit_quality_01.bmp"
V2 = ROOT / "output/s1_d7_manual_ground_truth_v2"
OUTPUT = ROOT / "output/s1_p1_stable_seam_tracker"
EXPECTED_SOURCE_SHA256 = "f49df2fd8e7cffa72b87f5d004096f1b2410a88cf503984b2dc89e53d9bbaf92"
EXPECTED_ANNOTATION_SHA256 = "f960bf210508e011d3646d64a181df01d10ad4e416834e42cf23cfac7f3a68c6"


def _stats(values: list[float]) -> dict:
    if not values:
        return {key: None for key in ("p50", "p90", "p95", "max")}
    return {"p50": float(np.percentile(values, 50)), "p90": float(np.percentile(values, 90)),
            "p95": float(np.percentile(values, 95)), "max": float(np.max(values))}


def _manual(annotation: dict, ys: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    left = annotation["left_boundary_manual_points"]
    right = annotation["right_boundary_manual_points"]
    return (np.interp(ys, [p["y"] for p in left], [p["x"] for p in left]),
            np.interp(ys, [p["y"] for p in right], [p["x"] for p in right]))


def _reference_validation(image: np.ndarray, annotation: dict) -> dict:
    out = OUTPUT / "reference_validation"
    out.mkdir(parents=True, exist_ok=True)
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    result = tracker.process(image, diagnostics=True)
    diagnostics = tracker.last_diagnostics
    x, y, w, h = result.roi_original_px
    roi_preview = image.copy()
    cv2.rectangle(roi_preview, (x, y), (x + w - 1, y + h - 1), (0, 255, 255), 8)
    cv2.imwrite(str(out / "roi_preview.png"), roi_preview)
    edge = cv2.normalize(np.abs(diagnostics.edge_response), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    cv2.imwrite(str(out / "candidate_edges.png"), edge)
    selected = cv2.cvtColor(diagnostics.processed_gray, cv2.COLOR_GRAY2BGR)
    for pair in diagnostics.selected_pairs:
        cv2.circle(selected, (round(pair.left), round(pair.y)), 3, (255, 0, 0), -1)
        cv2.circle(selected, (round(pair.right), round(pair.y)), 3, (0, 255, 0), -1)
    cv2.imwrite(str(out / "selected_boundaries.png"), selected)
    overlay = draw_stable_seam_overlay(image, result)
    cv2.imwrite(str(out / "final_overlay.png"), overlay)
    ys = np.asarray([point[1] for point in result.centerline_original_px])
    common = json.loads((V2 / "ground_truth_manifest.json").read_text())["common_y_range"]
    valid = (ys >= max(common[0], y)) & (ys <= min(common[1], y + h))
    ys = ys[valid]
    ml, mr = _manual(annotation, ys) if len(ys) else (np.asarray([]), np.asarray([]))
    al = np.asarray([point[0] for point in result.left_boundary_original_px])[valid]
    ar = np.asarray([point[0] for point in result.right_boundary_original_px])[valid]
    ac = np.asarray([point[0] for point in result.centerline_original_px])[valid]
    report = result.to_dict()
    report.update({"coordinate_system": "original_image_pixels", "roi_error_statistics_px": {
        "left": _stats(np.abs(al - ml).tolist()), "right": _stats(np.abs(ar - mr).tolist()),
        "center": _stats(np.abs(ac - 0.5 * (ml + mr)).tolist()),
        "width": _stats(np.abs((ar - al) - (mr - ml)).tolist())},
        "manual_evaluation_sample_count": int(len(ys))})
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def _transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    return cv2.transform(points.reshape(-1, 1, 2).astype(np.float64), matrix).reshape(-1, 2)


def _synthetic_sequence(image: np.ndarray, annotation: dict) -> dict:
    out = OUTPUT / "synthetic_sequence"
    out.mkdir(parents=True, exist_ok=True)
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    left0 = np.asarray([(p["x"], p["y"]) for p in annotation["left_boundary_manual_points"]])
    right0 = np.asarray([(p["x"], p["y"]) for p in annotation["right_boundary_manual_points"]])
    specs = [(-12, -1.0, .92, 2), (-6, -.5, 1.0, 1), (0, 0, 1.05, 0), (7, .6, .96, 2),
             (12, 1.0, 1.0, 1), (5, .3, 1.08, 0), (0, 0, 1.0, 251), (0, 0, 1.0, 251),
             (-4, -.4, 1.0, 1), (0, 0, 1.0, 0), (4, .4, .95, 2), (0, 0, 1.0, 0)]
    records, panels = [], []
    rng = np.random.default_rng(7)
    previous_lateral_residual = previous_heading_residual = None
    false_jumps = 0
    common_y0 = max(left0[:, 1].min(), right0[:, 1].min())
    common_y1 = min(left0[:, 1].max(), right0[:, 1].max())
    truth_ys0 = np.linspace(common_y0, common_y1, 96)
    left_order0, right_order0 = np.argsort(left0[:, 1]), np.argsort(right0[:, 1])
    truth_center0 = np.column_stack((
        0.5 * (np.interp(truth_ys0, left0[left_order0, 1], left0[left_order0, 0])
               + np.interp(truth_ys0, right0[right_order0, 1], right0[right_order0, 0])), truth_ys0))
    for index, (shift, angle, brightness, blur) in enumerate(specs):
        matrix = cv2.getRotationMatrix2D((image.shape[1] / 2, image.shape[0] / 2), angle, 1.0)
        matrix[0, 2] += shift
        frame = cv2.warpAffine(image, matrix, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REFLECT)
        frame = np.clip(frame.astype(np.float32) * brightness + rng.normal(0, 1.5, frame.shape), 0, 255).astype(np.uint8)
        if blur:
            kernel = blur if blur % 2 else blur + 1
            frame = cv2.GaussianBlur(frame, (kernel, kernel), 0)
            if blur >= 200:
                # Severe temporary defocus also collapses local contrast.  The
                # retained 0.5% image signal makes this a deterministic
                # no-evidence regression for DEGRADED rather than a fake seam.
                flat = np.full_like(frame, int(np.mean(frame)))
                frame = cv2.addWeighted(frame, 0.005, flat, 0.995, 0.0)
        cv2.line(frame, (50, 80 + index * 5), (900, 80 + index * 5), (255, 255, 255), 10)  # ROI-external distractor
        result = tracker.process(frame)
        truth_left, truth_right = _transform_points(left0, matrix), _transform_points(right0, matrix)
        truth_center = _transform_points(truth_center0, matrix)
        center_order = np.argsort(truth_center[:, 1])
        truth_coefficients = np.polyfit(truth_center[center_order, 1], truth_center[center_order, 0], 2)
        reference_y = float(tracker.config["geometry"]["reference_y_ratio"]) * image.shape[0]
        truth_lateral = float(np.polyval(truth_coefficients, reference_y) - image.shape[1] * 0.5)
        truth_heading = float(np.degrees(np.arctan(np.polyval(np.polyder(truth_coefficients), reference_y))))
        lateral_residual = None if result.lateral_error_px is None else result.lateral_error_px - truth_lateral
        heading_residual = None if result.heading_error_deg is None else result.heading_error_deg - truth_heading
        _, reference_width = tracker.reference.at(reference_y, image.shape[1], image.shape[0])
        lateral_jump_limit = (float(tracker.config["tracking"]["max_lateral_jump_width_scale"])
                              * float(reference_width))
        heading_jump_limit = float(tracker.config["tracking"]["max_heading_change_deg"])
        if previous_lateral_residual is not None and lateral_residual is not None:
            false_jumps += int(abs(lateral_residual - previous_lateral_residual) > lateral_jump_limit
                               or abs(heading_residual - previous_heading_residual) > heading_jump_limit)
        if lateral_residual is not None:
            previous_lateral_residual = lateral_residual
            previous_heading_residual = heading_residual
        center_errors = []
        if result.centerline_original_px:
            order_l, order_r = np.argsort(truth_left[:, 1]), np.argsort(truth_right[:, 1])
            for cx, cy in result.centerline_original_px:
                if max(truth_left[order_l[0], 1], truth_right[order_r[0], 1]) <= cy <= min(truth_left[order_l[-1], 1], truth_right[order_r[-1], 1]):
                    tx = 0.5 * (np.interp(cy, truth_left[order_l, 1], truth_left[order_l, 0]) + np.interp(cy, truth_right[order_r, 1], truth_right[order_r, 0]))
                    center_errors.append(abs(cx - tx))
        records.append({"frame": index, "detected": result.detected, "predicted_only": result.predicted_only,
                        "state": result.tracking_state.name, "lateral": result.lateral_error_px,
                        "heading": result.heading_error_deg, "truth_lateral": truth_lateral,
                        "truth_heading": truth_heading,
                        "lateral_residual": lateral_residual, "heading_residual": heading_residual,
                        "center_errors_px": center_errors, "failure_reasons": result.failure_reasons})
        if index in (0, 5, 7, 11):
            panels.append(cv2.resize(draw_stable_seam_overlay(frame, result), (512, 384)))
    montage = np.vstack((np.hstack(panels[:2]), np.hstack(panels[2:])))
    cv2.imwrite(str(OUTPUT / "synthetic_sequence_montage.jpg"), montage)
    detected = [r["detected"] for r in records]
    lateral_residuals = [r["lateral_residual"] for r in records if r["lateral_residual"] is not None]
    heading_residuals = [r["heading_residual"] for r in records if r["heading_residual"] is not None]
    errors = [v for r in records for v in r["center_errors_px"]]
    degraded_indices = [index for index, record in enumerate(records) if record["state"] == "DEGRADED"]
    lost_indices = [index for index, record in enumerate(records) if record["state"] == "LOST"]
    last_failure = max(degraded_indices + lost_indices, default=-1)
    recovered_detection = (None if last_failure < 0 else
                           next((index for index in range(last_failure + 1, len(records))
                                 if records[index]["detected"]), None))
    recovered_tracking = (None if last_failure < 0 else
                          next((index for index in range(last_failure + 1, len(records))
                                if records[index]["state"] == "TRACKING"), None))
    state_semantics_valid = all(
        (record["state"] != "DEGRADED" or (not record["detected"] and record["predicted_only"]))
        and (record["state"] != "LOST" or (not record["detected"] and not record["predicted_only"]))
        for record in records)
    report = {"frame_count": len(records), "detected_rate": float(np.mean(detected)), "false_jump_count": false_jumps,
              "lateral_jitter_std_px": float(np.std(lateral_residuals)) if len(lateral_residuals) > 1 else None,
              "heading_jitter_std_deg": float(np.std(heading_residuals)) if len(heading_residuals) > 1 else None,
              "centerline_error_p95_px": float(np.percentile(errors, 95)) if errors else None,
              "degraded_frame_indices": degraded_indices, "lost_frame_indices": lost_indices,
              "state_semantics_valid": state_semantics_valid,
              "recovery_to_detection_frames": None if recovered_detection is None else recovered_detection - last_failure,
              "recovery_to_tracking_frames": None if recovered_tracking is None else recovered_tracking - last_failure,
              "perturbations": {"translation_px": "-12..+12", "rotation_deg": "-1..+1",
                                "brightness_scale": "0.92..1.08", "gaussian_noise_sigma_gray": 1.5,
                                "severe_blur_kernel": 251, "severe_blur_frame_count": 2,
                                "severe_blur_retained_contrast_ratio": 0.005,
                                "bright_distractor": "ROI-external bright line"},
              "states": [r["state"] for r in records], "records": records,
              "scope_warning": "synthetic regression only; not real-site accuracy"}
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def _real_images() -> dict:
    out = OUTPUT / "real_images_unlabelled"
    out.mkdir(parents=True, exist_ok=True)
    records = {}
    for path in sorted((ROOT / "output/s1_real_seam_images").glob("*.bmp")):
        image = cv2.imread(str(path))
        if image is None or image.shape[:2] != (3072, 4096) or path == IMAGE:
            continue
        tracker = StableSeamTracker(CONFIG, REFERENCE)
        result = tracker.process(image)
        x, y, width, height = result.roi_original_px
        paired = zip(result.left_boundary_original_px, result.right_boundary_original_px)
        crossing = any(left[0] >= right[0] for left, right in paired)
        points = result.left_boundary_original_px + result.right_boundary_original_px + result.centerline_original_px
        outside = any(not (x <= px < x + width and y <= py < y + height) for px, py in points)
        records[path.name] = {
            **result.to_dict(), "precision_claimed": False, "boundaries_cross": crossing,
            "boundary_out_of_roi": outside,
            "abrupt_jump_observed": bool(result.abrupt_left_jump_count or result.abrupt_right_jump_count),
        }
        cv2.imwrite(str(out / f"{path.stem}_overlay.jpg"), draw_stable_seam_overlay(image, result, (1280, 960)))
    (out / "summary.json").write_text(json.dumps(records, indent=2) + "\n")
    return records


def _benchmark(image: np.ndarray) -> dict:
    tracker = StableSeamTracker(CONFIG, REFERENCE)
    for _ in range(30):
        tracker.process(image)
    total, stages = [], {}
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    for _ in range(300):
        result = tracker.process(image)
        total.append(result.processing_ms)
        for name, value in result.stage_timings_ms.items():
            stages.setdefault(name, []).append(value)
    rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    def values(items):
        return {"mean": statistics.fmean(items), "p50": float(np.percentile(items, 50)),
                "p95": float(np.percentile(items, 95)), "max": max(items)}
    report = {"warmup_frames": 30, "measured_frames": 300, "total_ms": values(total),
              "stages_ms": {name: values(items) for name, items in stages.items()},
              "algorithm_output_hz": 1000 / statistics.fmean(total), "mean_below_100ms": statistics.fmean(total) < 100,
              "p95_below_100ms": float(np.percentile(total, 95)) < 100,
              "max_rss_before_kb": rss_before, "max_rss_after_kb": rss_after,
              "max_rss_growth_kb": max(0, rss_after - rss_before), "display_debug_save_disabled": True,
              "platform": platform.platform(), "processor": platform.processor(),
              "hardware_scope": "host CPU algorithm benchmark; MindVision live-camera realtime validation pending"}
    (OUTPUT / "benchmark.json").write_text(json.dumps(report, indent=2) + "\n")
    return report


def main() -> None:
    manifest = json.loads((V2 / "ground_truth_manifest.json").read_text())
    annotation_path = V2 / "seam_center_lit_quality_01_annotation_v2.json"
    overlay_path = V2 / "seam_center_lit_quality_01_annotation_overlay_v2.png"
    checks = {"manifest_status": manifest["status"] == "frozen", "human_overlay_approved": manifest["human_overlay_approved"],
              "json_overlay_consistent": manifest["json_overlay_consistent"],
              "image_sha_matches": sha256_file(IMAGE) == manifest["input_sha256"] == EXPECTED_SOURCE_SHA256,
              "annotation_sha_matches": (sha256_file(annotation_path) == manifest["json_sha256"]
                                         == EXPECTED_ANNOTATION_SHA256),
              "overlay_sha_matches": sha256_file(overlay_path) == manifest["overlay_sha256"]}
    if not all(checks.values()):
        raise RuntimeError(f"Frozen V2 validation failed: {checks}")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(str(IMAGE))
    annotation = json.loads(annotation_path.read_text())
    summary = {"frozen_v2_checks": checks, "reference_validation": _reference_validation(image, annotation),
               "synthetic_sequence": _synthetic_sequence(image, annotation), "unlabelled_real_images": _real_images(),
               "benchmark": _benchmark(image)}
    (OUTPUT / "validation_summary.json").write_text(json.dumps(summary, indent=2) + "\n")


if __name__ == "__main__":
    main()
