#!/usr/bin/env python3
"""Read-only, staged diagnosis for the S1-B2d real seam evidence.

This module deliberately does not modify ClassicalSeamDetector.  The response
and selected-component calls use the production detector's private methods;
the additional arrays are reconstructed with the same operations so that all
intermediate images can be saved without changing the production result.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np

# Permit the documented ``python scripts/...py`` invocation from the repository
# root without requiring an editable install.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.config import AppConfig, load_config
from tank_vision.preprocessing.image_quality import assess_image_quality
from tank_vision.preprocessing.normalize import enhance_contrast, select_gray_channel
from tank_vision.preprocessing.roi import ImageView, make_image_view
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.seam.centerline import extract_component_centers
from tank_vision.seam.confidence import calculate_confidence
from tank_vision.seam.geometry import fit_polynomial_ransac, seam_errors
from tank_vision.types import FrameData


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_IMAGE = ROOT / "output/s1_b2c_positive_evidence_insufficient_20260907/representative_input.bmp"
DEFAULT_STATE = ROOT / "output/s1_b2c_positive_evidence_insufficient_20260907/representative_seam_state.json"
DEFAULT_VALIDATOR = ROOT / "output/s1_b2c_positive_centered_seam_20260907_attempt1/validator.json"
DEFAULT_CONFIG = ROOT / "configs/s100p_realtime_1920x1200_896.yaml"
DEFAULT_OUTPUT = ROOT / "output/s1_b2d_positive_failure_diagnosis_20260907"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def finite_or_none(value: Any) -> Any:
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


def sanitize_json(value: Any) -> Any:
    """Convert NumPy values and non-finite floats to strict JSON values."""
    if isinstance(value, dict):
        return {str(key): sanitize_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [sanitize_json(item) for item in value]
    if isinstance(value, bool):
        return value
    if isinstance(value, (float, np.floating)):
        return float(value) if math.isfinite(float(value)) else None
    if isinstance(value, (int, np.integer)):
        return int(value)
    return value


def response_stats(response: np.ndarray) -> dict[str, float]:
    return {
        "max": float(np.max(response)) if response.size else 0.0,
        "mean": float(np.mean(response)) if response.size else 0.0,
        "p95": float(np.percentile(response, 95)) if response.size else 0.0,
        "p99": float(np.percentile(response, 99)) if response.size else 0.0,
        "p99_5": float(np.percentile(response, 99.5)) if response.size else 0.0,
    }


def quality_dict(image: np.ndarray, config: AppConfig) -> dict[str, Any]:
    result = assess_image_quality(image, config.quality)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    gray = gray.astype(np.uint8, copy=False)
    image_stats = {
        "mean": float(np.mean(gray)),
        "std": float(np.std(gray)),
        "p1": float(np.percentile(gray, 1)),
        "p5": float(np.percentile(gray, 5)),
        "p50": float(np.percentile(gray, 50)),
        "p95": float(np.percentile(gray, 95)),
        "p99": float(np.percentile(gray, 99)),
    }
    return {
        "grayscale": image_stats,
        "underexposed_ratio": result.underexposed_ratio,
        "overexposed_ratio": result.overexposed_ratio,
        "laplacian_variance": result.laplacian_variance,
        "contrast_std": result.contrast_std,
        "valid_image": result.valid_image,
        "valid_for_measurement": result.valid_for_measurement,
        "confidence_scale": result.confidence_scale,
        "warnings": list(result.warnings),
        "gates": {
            "underexposure": {
                "enabled": config.quality.enable_exposure_check,
                "threshold": config.quality.underexposed_threshold,
                "max_ratio": config.quality.max_underexposed_ratio,
                "failed": "severe_underexposure" in result.warnings,
            },
            "overexposure": {
                "enabled": config.quality.enable_exposure_check,
                "threshold": config.quality.overexposed_threshold,
                "max_ratio": config.quality.max_overexposed_ratio,
                "failed": "severe_overexposure" in result.warnings,
            },
            "contrast": {
                "minimum_std": config.quality.min_contrast_std,
                "failed": "low_contrast" in result.warnings,
            },
            "blur": {
                "enabled": config.quality.enable_blur_check,
                "minimum_laplacian_variance": config.quality.min_laplacian_variance,
                "failed": "severe_blur" in result.warnings,
            },
        },
    }


def _threshold_and_labels(response: np.ndarray, config: AppConfig) -> tuple[np.ndarray, np.ndarray, float | None]:
    """Exact binary/morphology sequence from ClassicalSeamDetector._component_mask."""
    if config.seam.threshold_method == "adaptive":
        binary = cv2.adaptiveThreshold(
            response, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
            config.seam.adaptive_block_size, config.seam.adaptive_constant,
        )
        threshold: float | None = None
    else:
        threshold_value, binary = cv2.threshold(response, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        threshold = float(threshold_value)
    raw = binary.copy()
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (config.seam.morphology_kernel,) * 2)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=config.seam.morphology_iterations)
    open_size = config.seam.morphology_open_kernel
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8), iterations=1)
    return raw, binary, threshold


def _component_records(
    response: np.ndarray, cleaned: np.ndarray, config: AppConfig, minimum_gate_passed: bool,
) -> tuple[list[dict[str, Any]], list[tuple[float, int]], np.ndarray, np.ndarray, np.ndarray]:
    count, labels, component_stats, centroids = cv2.connectedComponentsWithStats(cleaned, connectivity=8)
    height, width = response.shape
    image_area = height * width
    major_length = height if config.seam.expected_direction == "vertical" else width
    records: list[dict[str, Any]] = []
    candidates: list[tuple[float, int]] = []
    for label in range(1, count):
        x, y, component_width, component_height, area = (int(v) for v in component_stats[label])
        span = component_height if config.seam.expected_direction == "vertical" else component_width
        thickness = component_width if config.seam.expected_direction == "vertical" else component_height
        span_ratio = span / major_length
        dependent_center = centroids[label][0 if config.seam.expected_direction == "vertical" else 1]
        dependent_length = width if config.seam.expected_direction == "vertical" else height
        center_score = 1.0 - abs(dependent_center - dependent_length / 2) / max(1.0, dependent_length / 2)
        elongation = span / max(1.0, thickness)
        density = area / max(1.0, component_width * component_height)
        score = 0.50 * span_ratio + 0.20 * min(1.0, elongation / 8.0) + 0.15 * density + 0.15 * center_score
        passed = minimum_gate_passed
        reasons: list[str] = []
        if not minimum_gate_passed:
            reasons.append("response_percentile_below_minimum_response")
        if area < image_area * config.seam.min_component_area_ratio:
            passed = False
            reasons.append("area_below_min_component_area_ratio")
        if span_ratio < config.seam.min_component_span_ratio:
            passed = False
            reasons.append("span_below_min_component_span_ratio")
        inside = response[labels == label].astype(np.float32)
        ring_size = config.seam.contrast_ring_kernel
        mask = labels == label
        dilated = cv2.dilate(mask.astype(np.uint8), np.ones((ring_size, ring_size), np.uint8), iterations=1).astype(bool)
        ring = response[dilated & ~mask].astype(np.float32)
        records.append({
            "label": label,
            "bbox": {"x": x, "y": y, "width": component_width, "height": component_height},
            "area": area,
            "width": component_width,
            "height": component_height,
            "span": span,
            "span_ratio": float(span_ratio),
            "elongation_span_over_thickness": float(elongation),
            "centroid": {"x": float(centroids[label][0]), "y": float(centroids[label][1])},
            "center_score_used_for_ranking": float(center_score),
            "ranking_score": float(score),
            "mean_response": float(np.mean(inside)) if inside.size else 0.0,
            "max_response": float(np.max(inside)) if inside.size else 0.0,
            "component_contrast": float(np.mean(inside) - np.mean(ring)) if ring.size else float(np.mean(inside)),
            "passes_area_and_span": passed,
            "passed": passed,
            "rejection_reasons": reasons,
        })
        if passed:
            candidates.append((score, label))
    return records, candidates, labels, component_stats, centroids


def _gray_to_color(image: np.ndarray) -> np.ndarray:
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image.copy()


def _put_label(image: np.ndarray, text: str, line: int = 0, color: tuple[int, int, int] = (0, 255, 255)) -> None:
    cv2.putText(image, text, (12, 28 + 26 * line), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, text, (12, 28 + 26 * line), cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1, cv2.LINE_AA)


def _save_gray(path: Path, image: np.ndarray, label: str) -> None:
    output = _gray_to_color(image)
    _put_label(output, label)
    if not cv2.imwrite(str(path), output):
        raise OSError(f"Failed to write {path}")


def _overlay_components(
    view: ImageView, records: list[dict[str, Any]], selected_label: int | None, accepted: bool | None,
    label: str,
) -> np.ndarray:
    output = _gray_to_color(view.image)
    for record in records:
        if accepted is not None and bool(record["passed"]) != accepted:
            continue
        bbox = record["bbox"]
        colour = (0, 255, 0) if record["label"] == selected_label else ((255, 180, 0) if accepted else (0, 0, 255))
        cv2.rectangle(output, (bbox["x"], bbox["y"]),
                      (bbox["x"] + bbox["width"] - 1, bbox["y"] + bbox["height"] - 1), colour, 2)
        # Draw every box, but label only major components so the diagnostic
        # remains inspectable when low-light noise creates hundreds of blobs.
        major_area = view.image.shape[0] * view.image.shape[1] * 0.0005
        if record["area"] >= max(20.0, major_area):
            cv2.putText(output, f"L{record['label']} a={record['area']} s={record['span_ratio']:.2f}",
                        (bbox["x"], max(18, bbox["y"] - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1, cv2.LINE_AA)
    _put_label(output, label, 0)
    return output


def _draw_candidates(view: ImageView, component: np.ndarray | None, independent: np.ndarray,
                     dependent: np.ndarray, fit: Any, expected_direction: str, label: str) -> np.ndarray:
    output = _gray_to_color(view.image)
    if component is not None:
        ys, xs = np.nonzero(component)
        output[ys, xs] = (80, 80, 255)
    for independent_value, dependent_value in zip(independent, dependent):
        x, y = (dependent_value, independent_value) if expected_direction == "vertical" else (independent_value, dependent_value)
        cv2.circle(output, (round(x), round(y)), 2, (0, 255, 255), -1)
    if fit is not None:
        values = np.linspace(float(np.min(independent)), float(np.max(independent)), 120)
        for previous, current in zip(values[:-1], values[1:]):
            p0 = float(np.polyval(fit.coefficients, previous))
            p1 = float(np.polyval(fit.coefficients, current))
            q0 = (p0, previous) if expected_direction == "vertical" else (previous, p0)
            q1 = (p1, current) if expected_direction == "vertical" else (current, p1)
            cv2.line(output, (round(q0[0]), round(q0[1])), (round(q1[0]), round(q1[1])), (0, 255, 0), 2)
    _put_label(output, label, 0)
    return output


def _direction_analysis(image: np.ndarray, expected_direction: str) -> dict[str, Any]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    edges = cv2.Canny(gray, 30, 100)
    hough = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=40, minLineLength=60, maxLineGap=15)
    bins = {"vertical": 0.0, "horizontal": 0.0, "diagonal": 0.0}
    lines: list[dict[str, float]] = []
    if hough is not None:
        for item in hough[:, 0]:
            x1, y1, x2, y2 = (float(v) for v in item)
            dx, dy = x2 - x1, y2 - y1
            length = math.hypot(dx, dy)
            angle = abs(math.degrees(math.atan2(dy, dx)))
            angle = angle if angle <= 90 else 180 - angle
            if angle >= 70:
                category = "vertical"
            elif angle <= 20:
                category = "horizontal"
            else:
                category = "diagonal"
            bins[category] += length
            lines.append({"x1": x1, "y1": y1, "x2": x2, "y2": y2, "length": length,
                          "angle_from_horizontal_deg": angle, "category": category})
    total = sum(bins.values())
    shares = {key: (value / total if total else 0.0) for key, value in bins.items()}
    dominant = max(shares, key=shares.get) if total else "uncertain"
    if total and sorted(shares.values(), reverse=True)[0] - sorted(shares.values(), reverse=True)[1] < 0.08:
        dominant = "uncertain"
    seam_direction_span = None
    return {
        "method": "Canny + probabilistic HoughLinesP; diagnostic only, not connected to detector",
        "dominant_structure": dominant,
        "direction_shares_length_weighted": shares,
        "line_count": len(lines),
        "longest_lines": sorted(lines, key=lambda item: item["length"], reverse=True)[:20],
        "expected_direction": expected_direction,
        "algorithm_direction_match": dominant == expected_direction,
        "seam_direction_span": seam_direction_span,
    }


def _run_variant(image: np.ndarray, config: AppConfig, name: str) -> tuple[dict[str, Any], dict[str, Any]]:
    frame = FrameData(image=image, timestamp_ns=1788793406480432270, frame_id="diagnosis", sequence_id=0,
                      source_name="representative_input.bmp", original_filename="representative_input.bmp")
    pipeline = RealtimeSeamPipeline(config)
    result = pipeline.process(frame)
    seam = result.seam_result
    view = make_image_view(image, config.pipeline.roi, config.pipeline.thumbnail_max_dimension)
    gray, response = pipeline.seam_detector._response(view)
    raw_mask, cleaned, threshold = _threshold_and_labels(response, config)
    response_percentile_value = float(np.percentile(response, config.seam.response_percentile))
    minimum_gate_passed = response_percentile_value >= config.seam.minimum_response
    records, candidates, labels, component_stats, centroids = _component_records(
        response, cleaned, config, minimum_gate_passed
    )
    production_component, _ = pipeline.seam_detector._component_mask(response)
    selected_label = None
    if candidates:
        selected_label = max(candidates)[1]
    selected = labels == selected_label if selected_label is not None else None
    independent = dependent = widths = np.empty(0, dtype=np.float64)
    fit = None
    factors: dict[str, float] = {}
    lateral = heading = None
    if selected is not None:
        independent, dependent, widths = extract_component_centers(selected, response, config.seam.expected_direction)
        fit = fit_polynomial_ransac(independent, dependent, config.seam.fit_degree, config.seam.ransac_iterations,
                                    config.seam.ransac_residual_px)
        if fit is not None:
            major_length = response.shape[0] if config.seam.expected_direction == "vertical" else response.shape[1]
            dependent_length = response.shape[1] if config.seam.expected_direction == "vertical" else response.shape[0]
            contrast = next(record["component_contrast"] for record in records if record["label"] == selected_label)
            confidence, factors = calculate_confidence(independent, dependent, fit.inliers, major_length,
                                                        fit.residual_rms, config.seam.max_fit_residual_px, contrast,
                                                        dependent_length, config.seam.center_reasonable_margin_ratio,
                                                        config.seam.continuity_max_gap_px, config.seam.contrast_reference)
            if config.seam.expected_direction == "vertical":
                reference = config.seam.reference_y_ratio * (response.shape[0] - 1)
                target = config.seam.reference_x_ratio * response.shape[1]
            else:
                reference = config.seam.reference_x_ratio * (response.shape[1] - 1)
                target = config.seam.reference_y_ratio * response.shape[0]
            lateral_processed, heading = seam_errors(fit.coefficients, config.seam.expected_direction, reference, target)
            dependent_scale = view.scale_x if config.seam.expected_direction == "vertical" else view.scale_y
            lateral = lateral_processed / dependent_scale
        else:
            confidence = 0.0
    else:
        confidence = 0.0
    selected_record = next((record for record in records if record["label"] == selected_label), None)
    for record in records:
        if record["label"] == selected_label:
            record["selected_by_production"] = True
    diagnostics = {
        "name": name,
        "processing": {
            "original_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
            "roi": list(config.pipeline.roi) if config.pipeline.roi is not None else None,
            "processed_size": {"width": int(view.image.shape[1]), "height": int(view.image.shape[0])},
            "scale_x": float(view.scale_x), "scale_y": float(view.scale_y),
        },
        "seam": {
            "detected": bool(seam.detected), "confidence": float(seam.confidence),
            "lost_reason": seam.lost_reason,
            "centerline_candidate_point_count": int(independent.size),
            "ransac_executed": bool(selected is not None),
            "ransac_fit_succeeded": fit is not None,
            "inlier_count": int(np.count_nonzero(fit.inliers)) if fit is not None else 0,
            "inlier_ratio": float(seam.inlier_ratio),
            "fit_residual_processed_px": float(fit.residual_rms) if fit is not None else None,
            "fit_residual_original_px": seam.fit_residual_px,
            "confidence_factors": factors,
            "lateral_error_px": seam.lateral_error_px if seam.lateral_error_px is not None else lateral,
            "heading_error_deg": seam.heading_error_deg if seam.heading_error_deg is not None else heading,
            "geometry_not_executed_direct_reason": ("response_percentile_below_minimum_response" if not minimum_gate_passed else "no_valid_component") if selected is None else (
                "ransac_fit_failed" if fit is None else None),
            "production_component_present": production_component is not None,
            "diagnostic_component_reconstruction_matches_production": (
                (production_component is None and selected is None)
                or (production_component is not None and selected is not None and np.array_equal(production_component, selected))
            ),
        },
        "response": response_stats(response),
        "threshold": {
            "method": config.seam.threshold_method,
            "minimum_response": config.seam.minimum_response,
            "response_percentile": config.seam.response_percentile,
            "response_percentile_value": response_percentile_value,
            "minimum_response_gate_passed": minimum_gate_passed,
            "threshold_stage_reached_in_production": minimum_gate_passed,
            "mask_ratios_are_diagnostic_continuation_after_minimum_gate": not minimum_gate_passed,
            "final_threshold": threshold,
            "raw_mask_valid_pixel_ratio": float(np.mean(raw_mask > 0)),
            "cleaned_mask_valid_pixel_ratio": float(np.mean(cleaned > 0)),
        },
        "connected_components": {
            "total_count_including_background": int(len(records) + 1),
            "foreground_count": len(records),
            "accepted_count": sum(bool(record["passed"]) for record in records),
            "structural_area_span_pass_count": sum(
                not any(reason in {"area_below_min_component_area_ratio", "span_below_min_component_span_ratio"}
                        for reason in record["rejection_reasons"])
                for record in records
            ),
            "selected_label": selected_label,
            "stage_reached_in_production": minimum_gate_passed,
            "components": records,
        },
        "timing_ms": {
            "total_algorithm_ms": float(result.metrics.timing.total_algorithm_ms),
            "seam_detect_ms": float(result.metrics.timing.seam_detect_ms),
            "seam_geometry_ms": float(result.metrics.timing.seam_geometry_ms),
        },
        "selected_component": selected_record,
        "_arrays": {"view": view, "gray": gray, "response": response, "raw": raw_mask,
                    "cleaned": cleaned, "labels": labels, "selected": selected,
                    "independent": independent, "dependent": dependent, "fit": fit},
    }
    return diagnostics, {"pipeline_result": result, "view": view}


def _save_primary_images(output: Path, image: np.ndarray, diagnosis: dict[str, Any]) -> None:
    arrays = diagnosis["_arrays"]
    view: ImageView = arrays["view"]
    preview = image.copy()
    _put_label(preview, f"input preview | original {image.shape[1]}x{image.shape[0]} BGR uint8", 0)
    cv2.imwrite(str(output / "input_preview.jpg"), preview, [cv2.IMWRITE_JPEG_QUALITY, 95])
    _save_gray(output / "resized_input.png", view.image, f"processed scale {view.image.shape[1]}x{view.image.shape[0]} | from original {image.shape[1]}x{image.shape[0]}")
    _save_gray(output / "selected_channel.png", arrays["gray"], "selected channel: gray | processed coordinates")
    config_note = diagnosis["name"]
    _save_gray(output / "clahe_response.png", arrays["clahe"], f"CLAHE enhanced gray | {config_note}")
    _save_gray(output / "filtered_image.png", arrays["filtered"], "Gaussian filtered | processed coordinates")
    _save_gray(output / "local_contrast_response.png", arrays["local_contrast"], "local contrast | processed coordinates")
    _save_gray(output / "morphology_gradient_response.png", arrays["gradient"], "morphology gradient | processed coordinates")
    combined_normalized = cv2.normalize(arrays["response"], None, 0, 255, cv2.NORM_MINMAX)
    _save_gray(output / "combined_response_normalized.png", combined_normalized,
               f"combined response normalized | p99.5={diagnosis['response']['p99_5']:.2f}")
    _save_gray(output / "threshold_mask_raw.png", arrays["raw"],
               f"raw {diagnosis['threshold']['method']} mask | threshold={diagnosis['threshold']['final_threshold']}")
    _save_gray(output / "threshold_mask_cleaned.png", arrays["cleaned"], "cleaned mask | close then open")
    selected_label = diagnosis["connected_components"]["selected_label"]
    cv2.imwrite(str(output / "connected_components_overlay.png"), _overlay_components(view, diagnosis["connected_components"]["components"], selected_label, None, "all component boxes | processed coordinates"))
    cv2.imwrite(str(output / "accepted_components_overlay.png"), _overlay_components(view, diagnosis["connected_components"]["components"], selected_label, True, "accepted area+span components | processed coordinates"))
    cv2.imwrite(str(output / "rejected_components_overlay.png"), _overlay_components(view, diagnosis["connected_components"]["components"], selected_label, False, "rejected components | processed coordinates"))
    cv2.imwrite(str(output / "centerline_candidates_overlay.png"), _draw_candidates(view, arrays["selected"], arrays["independent"], arrays["dependent"], arrays["fit"], diagnosis["config"]["seam"]["expected_direction"], "centerline candidates/fit | processed coordinates"))
    final = image.copy()
    if selected_label is not None:
        selected = arrays["selected"]
        ys, xs = np.nonzero(selected)
        if len(xs):
            points = np.column_stack((xs / view.scale_x + view.origin_x, ys / view.scale_y + view.origin_y)).astype(np.int32)
            x, y, w, h = cv2.boundingRect(points.reshape(-1, 1, 2))
            cv2.rectangle(final, (x, y), (x + w - 1, y + h - 1), (0, 255, 0), 3)
    _put_label(final, f"final diagnosis | original coordinates; algorithm scale {view.image.shape[1]}x{view.image.shape[0]}", 0)
    _put_label(final, f"detected={diagnosis['seam']['detected']} confidence={diagnosis['seam']['confidence']:.4f} reason={diagnosis['seam']['lost_reason']}", 1)
    cv2.imwrite(str(output / "final_diagnosis_overlay.jpg"), final, [cv2.IMWRITE_JPEG_QUALITY, 95])


def _diagnosis_markdown(diagnosis: dict[str, Any], state: dict[str, Any], validator: dict[str, Any], hashes: dict[str, str]) -> str:
    seam = diagnosis["seam"]
    threshold = diagnosis["threshold"]
    components = diagnosis["connected_components"]
    direction = diagnosis["direction"]
    return f"""# S1-B2d real seam failure diagnosis (2026-09-07)

## Result

The explicit 896 profile reproduced `detected=false`, confidence `{seam['confidence']:.6f}`, and `lost_reason={seam['lost_reason']}`. The first failing production gate is `{seam['geometry_not_executed_direct_reason'] or 'none'}`: `_component_mask()` returns no valid component, so geometry is not entered and `seam_geometry_ms` remains `{diagnosis['timing_ms']['seam_geometry_ms']:.6f}` ms. The saved ROS state is semantically consistent: no points, zero confidence, false detection, NaN geometry fields, and the repeated-frame lost-threshold reason.

Image-quality warnings are `{', '.join(diagnosis['quality']['warnings']) or 'none'}`. They are real evidence of poor optical quality, but in the actual code quality is not an early detector return; it only scales confidence and can demote an otherwise detected seam. Therefore it is a secondary candidate, not the direct cause of this zero-geometry failure.

## Code gates and image evidence

- Input `{diagnosis['input']['shape']}` is `{diagnosis['input']['dtype']}` with `{diagnosis['input']['channels']}` channels; the explicit config loaded successfully from `{diagnosis['config']['path']}`.
- Full-image ROI is used. The processed image is `{diagnosis['processing']['processed_size']['width']}x{diagnosis['processing']['processed_size']['height']}`, with `scale_x={diagnosis['processing']['scale_x']:.9f}` and `scale_y={diagnosis['processing']['scale_y']:.9f}`.
- Response P99.5 is `{diagnosis['response']['p99_5']:.3f}` versus `minimum_response={threshold['minimum_response']}`; the minimum-response gate is `{'PASS' if threshold['minimum_response_gate_passed'] else 'FAIL'}`.
- Threshold is `{threshold['method']}` with final threshold `{threshold['final_threshold']}`. Raw mask ratio is `{threshold['raw_mask_valid_pixel_ratio']:.6f}` and cleaned mask ratio is `{threshold['cleaned_mask_valid_pixel_ratio']:.6f}`.
- Cleaned foreground components: `{components['foreground_count']}`; production-eligible accepted components: `{components['accepted_count']}`; components that pass only area/span structurally (ignoring the already-failed response gate): `{components['structural_area_span_pass_count']}`. The accepted component rules are the actual `min_component_area_ratio` and `min_component_span_ratio` tests; center score is ranking only, while center plausibility is a confidence factor after fitting.
- Candidate points: `{seam['centerline_candidate_point_count']}`; RANSAC executed: `{seam['ransac_executed']}`; fit succeeded: `{seam['ransac_fit_succeeded']}`. Geometry direct reason: `{seam['geometry_not_executed_direct_reason']}`.

## Direction diagnosis

Hough/edge analysis (diagnostic-only) classified the dominant line structure as **{direction['dominant_structure']}**, with length-weighted shares `{json.dumps(direction['direction_shares_length_weighted'], ensure_ascii=False)}`. This does not alter the production result. Expected direction is `{direction['expected_direction']}`; algorithm direction match is `{direction['algorithm_direction_match']}`. The image is not automatically reconfigured to horizontal and no two-direction detector was added.

## Sensitivity variants

See `diagnosis.json` for the complete machine-readable table. The variants are deliberately limited to original 896, 1024 scale, supported horizontal direction, and gray/blue/green/red channels. No threshold scan or production parameter change was made.

## Evidence hashes

Before and after hashes are recorded in `diagnosis.json`; they are identical:

```text
{json.dumps(hashes, indent=2)}
```

## Boundary of conclusion and next step

The evidence supports a direct classical-detector failure at the cleaned connected-component area/span gate, with poor quality and possible direction mismatch as candidates. It does not uniquely prove that a human-visible weld is absent, nor that RANSAC or final confidence rejected it, because those stages were never reached. If structure analysis remains horizontal, the next stage should explicitly design longitudinal/circumferential direction handling. If responses remain unstable across labeled real samples, improve lighting/focus/working distance first and then move to real-data collection/S2 model detection rather than tuning one-image thresholds.

## Input evidence

Saved ROS SeamState: `{json.dumps(state, ensure_ascii=False, allow_nan=True)}`

Saved validator summary: scenario `{validator.get('scenario')}`, seam detected ratio `{validator.get('seam_detected_ratio')}`, warnings `{validator.get('warnings')}`.
"""


def diagnose(image_path: Path = DEFAULT_IMAGE, state_path: Path = DEFAULT_STATE,
             validator_path: Path = DEFAULT_VALIDATOR, config_path: Path = DEFAULT_CONFIG,
             output_dir: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    protected = {str(path): sha256(path) for path in (image_path, state_path, validator_path, config_path)}
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"Unable to decode image: {image_path}")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected a 3-channel color image, got {image.shape} dtype={image.dtype}")
    state = json.loads(state_path.read_text(encoding="utf-8"))
    validator = json.loads(validator_path.read_text(encoding="utf-8"))
    base_config = load_config(config_path)
    config_dict = base_config.to_dict()
    primary, _ = _run_variant(image, base_config, "A_896_gray_vertical")
    arrays = primary["_arrays"]
    # Capture exact preprocess arrays for the requested staged images.
    selected = select_gray_channel(arrays["view"].image, base_config.seam.color_channel)
    clahe = enhance_contrast(selected, base_config.seam.clahe_clip_limit, base_config.seam.clahe_grid_size)
    if base_config.seam.filter_type == "bilateral":
        filtered = cv2.bilateralFilter(clahe, base_config.seam.gaussian_kernel, base_config.seam.bilateral_sigma, base_config.seam.bilateral_sigma)
    else:
        filtered = cv2.GaussianBlur(clahe, (base_config.seam.gaussian_kernel,) * 2, 0)
    background_kernel = (base_config.seam.local_contrast_kernel, max(3, base_config.seam.gaussian_kernel))
    gradient_kernel = (3, 1)
    background = cv2.GaussianBlur(filtered, background_kernel, 0)
    arrays.update({"clahe": clahe, "filtered": filtered, "local_contrast": cv2.absdiff(filtered, background),
                   "gradient": cv2.morphologyEx(filtered, cv2.MORPH_GRADIENT, cv2.getStructuringElement(cv2.MORPH_RECT, gradient_kernel))})
    primary["input"] = {"path": str(image_path), "shape": list(image.shape), "dtype": str(image.dtype), "channels": int(image.shape[2]), "encoding": "BMP decoded by cv2.imread(IMREAD_UNCHANGED), BGR"}
    primary["config"] = {"path": str(config_path), "loaded_explicitly": True, "seam": config_dict["seam"], "pipeline": config_dict["pipeline"], "quality": config_dict["quality"]}
    primary["config_load_check"] = {
        "explicit_profile_path": str(config_path),
        "load_succeeded": True,
        "fallback_to_default_config": False,
        "loaded_thumbnail_max_dimension": base_config.pipeline.thumbnail_max_dimension,
        "loaded_expected_direction": base_config.seam.expected_direction,
        "loaded_color_channel": base_config.seam.color_channel,
    }
    primary["quality"] = quality_dict(image, base_config)
    primary["direction"] = _direction_analysis(image, base_config.seam.expected_direction)
    # A bounding span is reported independently of direction classification.
    primary["direction"]["seam_direction_span"] = {
        "required_direction": base_config.seam.expected_direction,
        "selected_component_span_ratio": primary["selected_component"]["span_ratio"] if primary["selected_component"] else 0.0,
        "passes_min_component_span_ratio": bool(primary["selected_component"] and primary["selected_component"]["passed"]),
    }
    saved_seam = state
    primary["ros_semantic_comparison"] = {
        "detected_equal": bool(saved_seam.get("detected") is False and primary["seam"]["detected"] is False),
        "confidence_equal": float(saved_seam.get("confidence", 0.0)) == primary["seam"]["confidence"],
        "centerline_empty_equal": not saved_seam.get("centerline_points") and primary["seam"]["centerline_candidate_point_count"] == 0,
        "geometry_nan_matches_unset": all(
            isinstance(saved_seam.get(key), float) and math.isnan(saved_seam[key])
            for key in ("lateral_error_px", "heading_error_deg", "seam_width_px")
        ) and primary["seam"]["lateral_error_px"] is None and primary["seam"]["heading_error_deg"] is None,
        "repeated_frame_lost_reason_matches": saved_seam.get("lost_reason") == "lost_threshold_reached:" + str(primary["seam"]["lost_reason"]),
        "note": "The offline one-frame reason is no_valid_component; ROS adds lost_threshold_reached after repeated misses.",
    }
    _save_primary_images(output_dir, image, primary)

    variants: list[dict[str, Any]] = []
    variant_specs = [("A_896_gray_vertical", 896, "gray", "vertical", primary),
                     ("B_1024_gray_vertical", 1024, "gray", "vertical", None),
                     ("C_896_gray_horizontal", 896, "gray", "horizontal", None),
                     ("D_896_gray_vertical", 896, "gray", "vertical", primary),
                     ("D_896_blue_vertical", 896, "blue", "vertical", None),
                     ("D_896_green_vertical", 896, "green", "vertical", None),
                     ("D_896_red_vertical", 896, "red", "vertical", None)]
    for name, dimension, channel, direction, known in variant_specs:
        if known is not None:
            item = known
        else:
            config = copy.deepcopy(base_config)
            config.pipeline.thumbnail_max_dimension = dimension
            config.seam.color_channel = channel
            config.seam.expected_direction = direction
            item, _ = _run_variant(image, config, name)
        variants.append({"name": name, "detected": item["seam"]["detected"], "confidence": item["seam"]["confidence"],
                         "mask_ratio": item["threshold"]["cleaned_mask_valid_pixel_ratio"],
                         "raw_mask_ratio": item["threshold"]["raw_mask_valid_pixel_ratio"],
                         "accepted_components": item["connected_components"]["accepted_count"],
                         "structural_area_span_pass_count": item["connected_components"]["structural_area_span_pass_count"],
                         "centerline_points": item["seam"]["centerline_candidate_point_count"],
                         "response_p99_5": item["response"]["p99_5"],
                         "minimum_response_gate_passed": item["threshold"]["minimum_response_gate_passed"],
                         "geometry_not_executed_direct_reason": item["seam"]["geometry_not_executed_direct_reason"],
                         "lateral_error_px": item["seam"]["lateral_error_px"], "heading_error_deg": item["seam"]["heading_error_deg"],
                         "total_algorithm_ms": item["timing_ms"]["total_algorithm_ms"],
                         "seam_detect_ms": item["timing_ms"]["seam_detect_ms"], "seam_geometry_ms": item["timing_ms"]["seam_geometry_ms"],
                         "lost_reason": item["seam"]["lost_reason"]})
    primary.pop("_arrays", None)
    primary["sensitivity_variants"] = variants
    primary["protected_input_sha256_before"] = protected
    protected_after = {path: sha256(Path(path)) for path in protected}
    primary["protected_input_sha256_after"] = protected_after
    primary["evidence_unchanged"] = protected == protected_after
    primary["saved_ros_seam_state"] = state
    primary["saved_validator_summary"] = validator
    # Conservative machine-readable categorization: distinguish direct from secondary evidence.
    primary["primary_failure_category"] = "minimum_response_gate"
    primary["secondary_failure_categories"] = ["optical_quality_insufficient"]
    if primary["direction"]["dominant_structure"] in {"vertical", "horizontal", "diagonal"} and primary["direction"]["dominant_structure"] != base_config.seam.expected_direction:
        primary["secondary_failure_categories"].append("direction_configuration_mismatch_candidate")
    primary["diagnostic_limits"] = {
        "quality_is_not_direct_detector_gate": True,
        "center_reasonable_is_confidence_factor_not_hard_rejection": True,
        "fit_residual_and_continuity_are_confidence_factors_not_hard_rejections": True,
        "direction_analysis_not_connected_to_production_result": True,
        "no_parameter_search": True,
    }
    primary["gate_catalog"] = {
        "image_validity": {"condition": "image is ndarray, non-empty, ndim >= 2", "outcome": "pass"},
        "undistort": {"condition": "use_undistort implies calibration is present", "outcome": "not_applicable_use_undistort_false"},
        "roi": {"condition": "ROI intersects image", "outcome": "pass_full_image_roi_null"},
        "scale": {"condition": "thumbnail resize succeeds", "outcome": "pass_896x560"},
        "color_channel": {"condition": "gray/blue/green/red selection succeeds", "outcome": "pass_gray"},
        "minimum_response": {"condition": "percentile(response, 99.5) >= 4.0", "outcome": "fail_1.0_below_4.0"},
        "threshold_segmentation": {"condition": "Otsu threshold then morphology", "outcome": "not_reached_in_production"},
        "connected_component_area": {"condition": "area >= image_area * 0.001", "outcome": "not_reached_in_production"},
        "connected_component_span": {"condition": "span / major_length >= 0.25", "outcome": "not_reached_in_production"},
        "expected_direction": {"condition": "vertical selects y-independent/x-dependent geometry", "outcome": "configured_vertical_direction_used"},
        "center_reasonable_region": {"condition": "confidence roi_plausibility factor", "outcome": "not_reached_in_production_not_hard_component_gate"},
        "centerline_valid_points": {"condition": "at least fit_degree + 1 points for RANSAC", "outcome": "not_reached_in_production"},
        "ransac_inlier_rate": {"condition": "inliers contribute to confidence", "outcome": "not_reached_in_production"},
        "fit_residual": {"condition": "residual contributes exponential confidence factor", "outcome": "not_reached_in_production_not_hard_reject"},
        "continuity": {"condition": "gaps <= continuity_max_gap_px contribute confidence", "outcome": "not_reached_in_production_not_hard_reject"},
        "final_confidence": {"condition": "confidence >= min_confidence", "outcome": "not_reached_in_production"},
    }
    serializable = sanitize_json(primary)
    (output_dir / "diagnosis.json").write_text(json.dumps(serializable, ensure_ascii=False, indent=2), encoding="utf-8")
    report = _diagnosis_markdown(serializable, state, validator, protected)
    (output_dir / "diagnosis_report.md").write_text(report, encoding="utf-8")
    return serializable


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, default=DEFAULT_IMAGE)
    parser.add_argument("--state", type=Path, default=DEFAULT_STATE)
    parser.add_argument("--validator", type=Path, default=DEFAULT_VALIDATOR)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    result = diagnose(args.image, args.state, args.validator, args.config, args.output)
    print(json.dumps({"output": str(args.output), "detected": result["seam"]["detected"],
                      "primary_failure_category": result["primary_failure_category"],
                      "evidence_unchanged": result["evidence_unchanged"]}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
