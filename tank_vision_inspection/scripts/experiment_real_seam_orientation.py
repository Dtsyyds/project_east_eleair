#!/usr/bin/env python3
"""Offline S1-D3 orientation experiment for the S1-D1 seam-band method.

The S1-D1 detector remains the source of all boundary candidates and fitting
parameters.  This module only rotates the already resized image, calls that
detector, and maps the selected evidence back with the inverse affine map.
It is deliberately not imported by the production pipeline.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
import csv
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Sequence

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.experiment_real_seam_band import (  # noqa: E402
    DEFAULT_CONFIG,
    BoundaryTrack,
    _band_mask,
    _draw_edges,
    _fit_center,
    _quality_summary,
    find_boundary_track,
    sha256,
)
from tank_vision.config import load_config  # noqa: E402
from tank_vision.preprocessing.normalize import select_gray_channel  # noqa: E402
from tank_vision.preprocessing.roi import ImageView, make_image_view  # noqa: E402
from tank_vision.seam.geometry import PolynomialFit, fit_polynomial_ransac  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "output/s1_d3_orientation_experiment"
DEFAULT_ANGLES = (-45, -35, -25, -15, -5, 0, 5, 15, 25, 35, 45)


def _json_number(value: float | int | None) -> float | int | None:
    if value is None:
        return None
    return float(value) if math.isfinite(float(value)) else None


def rotate_processing_image(gray: np.ndarray, angle_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate around the processing-image centre and return image, valid mask, M.

    Positive angles follow OpenCV's ``getRotationMatrix2D`` convention: in the
    displayed image coordinate system they rotate counter-clockwise.  Border
    replication avoids introducing a black stripe into the edge response; the
    warped valid-region mask is still used to reject all geometry outside the
    original processing image.
    """
    height, width = gray.shape[:2]
    center = ((width - 1) * 0.5, (height - 1) * 0.5)
    matrix = cv2.getRotationMatrix2D(center, float(angle_deg), 1.0)
    rotated = cv2.warpAffine(
        gray, matrix, (width, height), flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    source_valid = np.full((height, width), 255, dtype=np.uint8)
    valid = cv2.warpAffine(
        source_valid, matrix, (width, height), flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )
    return rotated, valid, matrix


def inverse_affine_points(points: Sequence[Sequence[float]] | np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Map Nx2 points from rotated processing coordinates to original ones."""
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if not len(values):
        return values
    inverse = cv2.invertAffineTransform(matrix)
    homogeneous = np.column_stack((values, np.ones(len(values), dtype=np.float64)))
    return homogeneous @ inverse.T


def forward_affine_points(points: Sequence[Sequence[float]] | np.ndarray, matrix: np.ndarray) -> np.ndarray:
    """Map Nx2 points from original processing coordinates to rotated ones."""
    values = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if not len(values):
        return values
    homogeneous = np.column_stack((values, np.ones(len(values), dtype=np.float64)))
    return homogeneous @ matrix.T


def _track_inside_valid_region(track: BoundaryTrack, valid: np.ndarray) -> BoundaryTrack:
    """Apply only the rotated-canvas validity check to an existing track."""
    height, width = valid.shape[:2]
    points = [
        point for point in track.points
        if 0 <= point.y < height
        and 0 <= point.left < width
        and 0 <= point.right < width
        and valid[point.y, point.left] != 0
        and valid[point.y, point.right] != 0
    ]
    reason = track.failure_reason
    if track.points and not points:
        reason = "all_boundary_pairs_outside_original_rotated_region"
    elif track.points and len(points) < 3:
        reason = "valid_boundary_path_has_fewer_than_three_points"
    return BoundaryTrack(points, track.all_edge_candidates, reason)


def _continuity_metrics(track: BoundaryTrack) -> tuple[float, float]:
    ordered = sorted(track.points, key=lambda point: point.y)
    if len(ordered) < 2:
        return 0.0, 0.0
    widths = np.asarray([point.width for point in ordered], dtype=np.float64)
    centers = np.asarray([point.center for point in ordered], dtype=np.float64)
    width_scale = max(float(np.median(widths)), 1.0)
    width_continuity = math.exp(-float(np.median(np.abs(np.diff(widths)))) / width_scale)
    center_continuity = math.exp(-float(np.median(np.abs(np.diff(centers)))) / width_scale)
    return float(width_continuity), float(center_continuity)


def _effective_coverage(track: BoundaryTrack, valid: np.ndarray) -> float:
    if not track.points:
        return 0.0
    ordered = sorted(track.points, key=lambda point: point.y)
    valid_rows = np.flatnonzero(np.any(valid != 0, axis=1))
    if not len(valid_rows):
        return 0.0
    return float((ordered[-1].y - ordered[0].y + 1) / max(1, valid_rows[-1] - valid_rows[0] + 1))


def _candidate_metrics(track: BoundaryTrack, valid: np.ndarray) -> dict[str, Any]:
    ordered = sorted(track.points, key=lambda point: point.y)
    widths = np.asarray([point.width for point in ordered], dtype=np.float64)
    strengths = np.asarray([min(point.left_strength, point.right_strength) for point in ordered], dtype=np.float64)
    width_continuity, center_continuity = _continuity_metrics(track)
    _, fit = _fit_center(track)
    coverage = _effective_coverage(track, valid)
    center_between = all(point.left < point.center < point.right for point in ordered)
    return {
        "geometry_candidate_valid": bool(fit is not None and len(ordered) >= 3 and coverage >= 0.60 and center_between),
        "failure_reason": (
            None if fit is not None and len(ordered) >= 3 and coverage >= 0.60 and center_between
            else (track.failure_reason or ("vertical_boundary_coverage_below_0.60" if coverage < 0.60 else "centerline_geometry_validation_failed"))
        ),
        "valid_center_point_count": len(ordered),
        "valid_left_boundary_point_count": len(ordered),
        "valid_right_boundary_point_count": len(ordered),
        "vertical_coverage_ratio": coverage,
        "weld_width_px_median_processed": float(np.median(widths)) if len(widths) else None,
        "weld_width_px_p05_processed": float(np.percentile(widths, 5)) if len(widths) else None,
        "weld_width_px_p95_processed": float(np.percentile(widths, 95)) if len(widths) else None,
        "centerline_inlier_ratio": float(np.mean(fit.inliers)) if fit is not None else 0.0,
        "centerline_fit_residual_processed_px": float(fit.residual_rms) if fit is not None else None,
        "boundary_gradient_evidence_processed": float(np.median(strengths)) if len(strengths) else 0.0,
        "boundary_gradient_p05_processed": float(np.percentile(strengths, 5)) if len(strengths) else 0.0,
        "width_continuity_score": width_continuity,
        "center_continuity_score": center_continuity,
        "centerline_between_boundaries": center_between,
        "track_failure_reason": track.failure_reason,
        "fit": fit,
    }


def _select_best_candidate(candidates: list[dict[str, Any]]) -> dict[str, Any] | None:
    """Select deterministically using only the five specified image evidences."""
    if not candidates:
        return None
    max_gradient = max(float(item["boundary_gradient_evidence_processed"]) for item in candidates)
    max_gradient = max(max_gradient, 1e-9)
    for item in candidates:
        gradient_score = min(1.0, float(item["boundary_gradient_evidence_processed"]) / max_gradient)
        item["boundary_gradient_score"] = gradient_score
        item["selection_score"] = (
            0.50 * float(item["vertical_coverage_ratio"])
            + 0.15 * gradient_score
            + 0.15 * float(item["width_continuity_score"])
            + 0.10 * float(item["center_continuity_score"])
            + 0.10 * float(item["centerline_inlier_ratio"])
        )
        item["selection_eligible"] = bool(
            item["geometry_candidate_valid"]
            and item["vertical_coverage_ratio"] >= 0.60
            and item["centerline_between_boundaries"]
            and item.get("restored_coordinates_valid", True)
        )
    eligible = [item for item in candidates if item["selection_eligible"]]
    if not eligible:
        return None
    # The fixed candidate order is the deterministic tie-break; angle itself
    # is not a score term and is never preferred as a result.
    return max(eligible, key=lambda item: float(item["selection_score"]))


def _safe_points(track: BoundaryTrack, matrix: np.ndarray, view: ImageView, image: np.ndarray) -> dict[str, np.ndarray]:
    ordered = sorted(track.points, key=lambda point: point.y)
    rotated_left = np.asarray([(point.left, point.y) for point in ordered], dtype=np.float64)
    rotated_right = np.asarray([(point.right, point.y) for point in ordered], dtype=np.float64)
    rotated_center = np.asarray([(point.center, point.y) for point in ordered], dtype=np.float64)
    result = {}
    for name, points in (("left", rotated_left), ("right", rotated_right), ("center", rotated_center)):
        processing_points = inverse_affine_points(points, matrix)
        original_points = np.asarray([view.to_original(x, y) for x, y in processing_points], dtype=np.float64)
        result[name] = original_points
    return result


def _points_valid(points: np.ndarray, width: int, height: int) -> bool:
    return bool(len(points) == 0 or (np.isfinite(points).all() and np.all(points[:, 0] >= 0) and np.all(points[:, 0] < width) and np.all(points[:, 1] >= 0) and np.all(points[:, 1] < height)))


def _draw_polyline(image: np.ndarray, points: np.ndarray, colour: tuple[int, int, int], thickness: int) -> None:
    if len(points) >= 2:
        cv2.polylines(image, [np.round(points).astype(np.int32).reshape(-1, 1, 2)], False, colour, thickness, cv2.LINE_AA)


def _draw_rotated_overlay(rotated: np.ndarray, valid: np.ndarray, track: BoundaryTrack, fit: PolynomialFit | None, angle: float) -> np.ndarray:
    output = cv2.cvtColor(rotated, cv2.COLOR_GRAY2BGR)
    invalid = valid == 0
    output[invalid] = (20, 20, 20)
    ordered = sorted(track.points, key=lambda point: point.y)
    _draw_polyline(output, np.asarray([(point.left, point.y) for point in ordered]), (255, 0, 0), 3)
    _draw_polyline(output, np.asarray([(point.right, point.y) for point in ordered]), (0, 255, 0), 3)
    if fit is not None and ordered:
        ys = np.linspace(ordered[0].y, ordered[-1].y, 180)
        _draw_polyline(output, np.column_stack((np.polyval(fit.coefficients, ys), ys)), (0, 255, 255), 3)
    cv2.line(output, (output.shape[1] // 2, 0), (output.shape[1] // 2, output.shape[0] - 1), (255, 255, 255), 1)
    cv2.putText(output, f"angle={angle:+.0f} deg | rotated processing coordinates", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(output, f"angle={angle:+.0f} deg | rotated processing coordinates", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def _draw_restored_overlay(image: np.ndarray, points: dict[str, np.ndarray], label: str, fit_points: np.ndarray | None = None) -> np.ndarray:
    output = image.copy()
    center_x = output.shape[1] // 2
    cv2.line(output, (center_x, 0), (center_x, output.shape[0] - 1), (255, 255, 255), 2)
    _draw_polyline(output, points["left"], (255, 0, 0), 8)
    _draw_polyline(output, points["right"], (0, 255, 0), 8)
    _draw_polyline(output, points["center"], (0, 255, 255), 8)
    if fit_points is not None:
        _draw_polyline(output, fit_points, (0, 255, 255), 8)
    cv2.putText(output, label, (32, 56), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(output, label, (32, 56), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(output, "left=blue right=green center=yellow reference_center=white | original coordinates", (32, 100), cv2.FONT_HERSHEY_SIMPLEX, .72, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(output, "left=blue right=green center=yellow reference_center=white | original coordinates", (32, 100), cv2.FONT_HERSHEY_SIMPLEX, .72, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def _angle_comparison(candidates: list[dict[str, Any]], rotated_images: dict[float, np.ndarray], selected_angle: float | None) -> np.ndarray:
    tiles = []
    for candidate in candidates:
        angle = float(candidate["angle_deg"])
        tile = rotated_images[angle].copy()
        tile = cv2.resize(tile, (320, 240), interpolation=cv2.INTER_AREA)
        colour = (0, 255, 0) if candidate.get("selection_eligible") else (0, 0, 255)
        text = f"{angle:+.0f}deg cov={candidate['vertical_coverage_ratio']:.3f} score={candidate.get('selection_score', 0):.3f}"
        cv2.putText(tile, text, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, .38, (0, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(tile, text, (6, 22), cv2.FONT_HERSHEY_SIMPLEX, .38, colour, 1, cv2.LINE_AA)
        tiles.append(tile)
    rows = []
    for index in range(0, len(tiles), 4):
        row = tiles[index:index + 4]
        while len(row) < 4:
            row.append(np.zeros_like(tiles[0]))
        rows.append(np.hstack(row))
    output = np.vstack(rows) if rows else np.zeros((240, 320, 3), dtype=np.uint8)
    title = f"finite orientation candidates | selected={selected_angle:+.0f}deg" if selected_angle is not None else "finite orientation candidates | no eligible angle"
    cv2.putText(output, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(output, title, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, .65, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def _fit_restored_center(points: np.ndarray, scale_x: float) -> PolynomialFit | None:
    if len(points) < 3:
        return None
    y = points[:, 1]
    x = points[:, 0]
    return fit_polynomial_ransac(y, x, degree=2, iterations=120, residual_threshold=8.0 / scale_x)


def run_orientation_experiment(
    input_path: Path,
    config_path: Path = DEFAULT_CONFIG,
    output_dir: Path = DEFAULT_OUTPUT,
    max_dimension: int = 896,
    angles: Sequence[float] = DEFAULT_ANGLES,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    input_hash_before = sha256(input_path)
    image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"Unable to decode image: {input_path}")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected BGR 3-channel input, got shape={image.shape} dtype={image.dtype}")
    config = load_config(config_path)
    view = make_image_view(image, config.pipeline.roi, max_dimension)
    selected = select_gray_channel(view.image, config.seam.color_channel)
    track_input = cv2.GaussianBlur(selected, (5, 5), 0)
    candidates: list[dict[str, Any]] = []
    rotated_images: dict[float, np.ndarray] = {}
    rotated_valid: dict[float, np.ndarray] = {}
    rotated_tracks: dict[float, BoundaryTrack] = {}
    for angle in angles:
        angle = float(angle)
        rotated, valid, matrix = rotate_processing_image(track_input, angle)
        track = _track_inside_valid_region(find_boundary_track(rotated), valid)
        metrics = _candidate_metrics(track, valid)
        valid_pixels = int(np.count_nonzero(valid))
        valid_rows = np.flatnonzero(np.any(valid != 0, axis=1))
        metrics.update({
            "angle_deg": angle,
            "rotation_sign_definition": "positive is counter-clockwise in displayed image coordinates (OpenCV getRotationMatrix2D)",
            "valid_rotated_pixel_ratio": float(valid_pixels / valid.size),
            "valid_rotated_bbox": (
                {"x": int(np.min(np.where(valid != 0)[1])), "y": int(np.min(np.where(valid != 0)[0])),
                 "width": int(np.max(np.where(valid != 0)[1]) - np.min(np.where(valid != 0)[1]) + 1),
                 "height": int(np.max(np.where(valid != 0)[0]) - np.min(np.where(valid != 0)[0]) + 1)}
                if valid_pixels else None
            ),
            "valid_rotated_row_span": [int(valid_rows[0]), int(valid_rows[-1])] if len(valid_rows) else None,
            "matrix": np.asarray(matrix, dtype=np.float64).round(12).tolist(),
        })
        for name in ("median", "p05", "p95"):
            processed_key = f"weld_width_px_{name}_processed"
            metrics[f"weld_width_px_{name}"] = (
                float(metrics[processed_key] / view.scale_x)
                if metrics[processed_key] is not None else None
            )
        metrics["centerline_fit_residual_px"] = (
            float(metrics["centerline_fit_residual_processed_px"] / view.scale_x)
            if metrics["centerline_fit_residual_processed_px"] is not None else None
        )
        metrics["centerline_fit_residual"] = metrics["centerline_fit_residual_px"]
        restored_candidate = _safe_points(track, matrix, view, image)
        metrics["restored_coordinates_valid"] = all(
            _points_valid(values, image.shape[1], image.shape[0]) for values in restored_candidate.values()
        )
        # Fit objects are internal only and cannot be JSON serialized.
        metrics.pop("fit", None)
        candidates.append(metrics)
        rotated_images[angle] = _draw_rotated_overlay(rotated, valid, track, _fit_center(track)[1], angle)
        rotated_valid[angle] = valid
        rotated_tracks[angle] = track
    chosen = _select_best_candidate(candidates)
    selected_angle = float(chosen["angle_deg"]) if chosen else None
    comparison = _angle_comparison(candidates, rotated_images, selected_angle)
    cv2.imwrite(str(output_dir / "angle_comparison.jpg"), comparison, [cv2.IMWRITE_JPEG_QUALITY, 95])

    quality = _quality_summary(image, config)
    selected_track = rotated_tracks[selected_angle] if selected_angle is not None else BoundaryTrack([], [], "no_angle_with_sufficient_evidence")
    selected_matrix = np.asarray(chosen["matrix"], dtype=np.float64) if chosen else np.eye(2, 3, dtype=np.float64)
    restored = _safe_points(selected_track, selected_matrix, view, image)
    restored_valid = all(_points_valid(values, image.shape[1], image.shape[0]) for values in restored.values())
    center_between = bool(len(selected_track.points) and all(p.left < p.center < p.right for p in selected_track.points))
    restored_fit = _fit_restored_center(restored["center"], view.scale_x)
    ordered = sorted(selected_track.points, key=lambda point: point.y)
    if restored_fit is not None:
        reference_y = config.seam.reference_y_ratio * (image.shape[0] - 1)
        target_x = config.seam.reference_x_ratio * image.shape[1]
        lateral = float(np.polyval(restored_fit.coefficients, reference_y) - target_x)
        heading = float(math.degrees(math.atan(float(np.polyval(np.polyder(restored_fit.coefficients), reference_y)))))
        residual = float(restored_fit.residual_rms)
        inlier_ratio = float(np.mean(restored_fit.inliers))
        y0 = float(np.min(restored["center"][:, 1])) if len(restored["center"]) else 0.0
        y1 = float(np.max(restored["center"][:, 1])) if len(restored["center"]) else 0.0
        fit_y = np.linspace(y0, y1, 180)
        restored_fit_points = np.column_stack((np.polyval(restored_fit.coefficients, fit_y), fit_y))
    else:
        lateral = heading = residual = None
        inlier_ratio = 0.0
        restored_fit_points = None
    geometry_valid = bool(chosen is not None and chosen["geometry_candidate_valid"] and restored_valid and center_between)
    if chosen is None:
        failure_reason = "no_angle_with_sufficient_evidence"
    elif not restored_valid:
        failure_reason = "restored_coordinates_out_of_original_bounds"
    elif not center_between:
        failure_reason = "restored_centerline_geometry_validation_failed"
    elif not geometry_valid:
        failure_reason = str(chosen["failure_reason"] or "selected_orientation_geometry_invalid")
    else:
        failure_reason = None
    selected_overlay = rotated_images[selected_angle] if selected_angle is not None else cv2.cvtColor(track_input, cv2.COLOR_GRAY2BGR)
    cv2.imwrite(str(output_dir / "selected_orientation_overlay.jpg"), selected_overlay, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "restored_boundaries_overlay.jpg"), _draw_restored_overlay(image, restored, "S1-D3 restored boundaries | experiment only"), [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "restored_centerline_overlay.jpg"), _draw_restored_overlay(image, restored, "S1-D3 restored centerline | experiment only", restored_fit_points), [cv2.IMWRITE_JPEG_QUALITY, 95])

    scale_x = float(view.scale_x)
    widths = np.asarray([point.width / scale_x for point in ordered], dtype=np.float64)
    input_hash_after = sha256(input_path)
    result: dict[str, Any] = {
        "experiment": "S1-D3",
        "input_path": str(input_path),
        "input_sha256": input_hash_before,
        "input_sha256_after": input_hash_after,
        "input_unchanged": input_hash_before == input_hash_after,
        "original_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
        "input_dtype": str(image.dtype),
        "input_channels": int(image.shape[2]),
        "processing_size": {"width": int(view.image.shape[1]), "height": int(view.image.shape[0])},
        "scale_x": scale_x,
        "scale_y": float(view.scale_y),
        "selected_angle_deg": selected_angle,
        "rotation_sign_definition": "positive is counter-clockwise in displayed image coordinates",
        "valid_left_boundary_point_count": len(ordered),
        "valid_right_boundary_point_count": len(ordered),
        "valid_center_point_count": len(ordered),
        "vertical_coverage_ratio": float(chosen["vertical_coverage_ratio"]) if chosen else 0.0,
        "weld_width_px_median": float(np.median(widths)) if len(widths) else None,
        "weld_width_px_p05": float(np.percentile(widths, 5)) if len(widths) else None,
        "weld_width_px_p95": float(np.percentile(widths, 95)) if len(widths) else None,
        "centerline_fit_residual_px": residual,
        "centerline_inlier_ratio": inlier_ratio,
        "lateral_error_px": lateral,
        "heading_error_deg": heading,
        "processing_time_ms": (time.perf_counter() - started) * 1000.0,
        "geometry_candidate_valid": geometry_valid,
        "image_quality_valid": bool(quality["valid_for_measurement"]),
        "production_valid": False,
        "experimental_only": True,
        "failure_reason": failure_reason,
        "centerline_between_boundaries": center_between,
        "restored_coordinates_valid": restored_valid,
        "quality": quality,
        "parameters_inherited_from_s1_d1": {
            "max_dimension": max_dimension,
            "scan_step_processed_px": 8,
            "scan_band_half_height_processed_px": 5,
            "centerline_ransac_degree": 2,
            "centerline_ransac_iterations": 120,
            "centerline_ransac_residual_threshold_processed_px": 8.0,
            "boundary_detector_source": "scripts/experiment_real_seam_band.py",
        },
        "limitations": [
            "finite 11-angle offline search only; no claim of generalization",
            "rotated valid-region mask excludes the original-image exterior; no black border is used as a boundary",
            "quality threshold and production confidence threshold were not changed",
            "right-image lower-half boundary jump is outside S1-D3 scope",
            "production_valid is always false; no ROS or production-pipeline integration",
        ],
    }
    serial_candidates = []
    for candidate in candidates:
        serial_candidates.append({key: value for key, value in candidate.items() if key != "matrix"} | {"matrix": candidate["matrix"]})
    (output_dir / "angle_candidates.json").write_text(json.dumps(serial_candidates, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    (output_dir / "experiment_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return result


def _write_root_summary(results: list[tuple[str, dict[str, Any]]], output_root: Path) -> None:
    rows = []
    for case, result in results:
        rows.append({"case": case, **{key: result.get(key) for key in (
            "input_path", "input_sha256", "selected_angle_deg", "geometry_candidate_valid", "failure_reason",
            "image_quality_valid", "valid_left_boundary_point_count", "valid_right_boundary_point_count",
            "valid_center_point_count", "vertical_coverage_ratio", "weld_width_px_median", "weld_width_px_p05",
            "weld_width_px_p95", "centerline_fit_residual_px", "centerline_inlier_ratio", "lateral_error_px",
            "heading_error_deg", "centerline_between_boundaries", "restored_coordinates_valid", "processing_time_ms",
        )}})
    summary = {
        "experiment": "S1-D3",
        "algorithm_source": "scripts/experiment_real_seam_band.py",
        "angles_deg": list(DEFAULT_ANGLES),
        "results": rows,
        "checks": {
            "all_input_unchanged": all(r["input_unchanged"] for _, r in results),
            "all_coordinates_valid": all(r["restored_coordinates_valid"] for _, r in results),
            "center_left_right_selected_near_zero": all(
                r["selected_angle_deg"] is not None and abs(float(r["selected_angle_deg"])) <= 15
                for case, r in results if case != "tilt"
            ),
            "left_lateral_more_negative_than_center": rows[1]["lateral_error_px"] < rows[0]["lateral_error_px"] if len(rows) >= 2 else False,
            "right_lateral_more_positive_than_center": rows[2]["lateral_error_px"] > rows[0]["lateral_error_px"] if len(rows) >= 3 else False,
            "tilt_valid": next((r["geometry_candidate_valid"] for case, r in results if case == "tilt"), False),
        },
        "note": "This is a validation experiment. No filename-based correction or per-image parameter tuning was used.",
    }
    (output_root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    fields = list(rows[0].keys()) if rows else ["case"]
    with (output_root / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _make_montage(results: list[tuple[str, dict[str, Any]]], output_root: Path) -> None:
    tiles = []
    for case, result in results:
        path = output_root / case / "restored_centerline_overlay.jpg"
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            image = np.zeros((672, 896, 3), dtype=np.uint8)
        image = cv2.resize(image, (896, 672), interpolation=cv2.INTER_AREA)
        text = f"{case} | geom={str(result['geometry_candidate_valid']).lower()} cov={result['vertical_coverage_ratio']:.3f} width={result['weld_width_px_median'] if result['weld_width_px_median'] is not None else 'n/a'} lateral={result['lateral_error_px'] if result['lateral_error_px'] is not None else 'n/a'} heading={result['heading_error_deg'] if result['heading_error_deg'] is not None else 'n/a'}"
        cv2.rectangle(image, (0, 0), (896, 44), (0, 0, 0), -1)
        cv2.putText(image, text, (8, 29), cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(image)
    while len(tiles) < 4:
        tiles.append(np.zeros_like(tiles[0]))
    montage = np.vstack((np.hstack(tiles[:2]), np.hstack(tiles[2:4])))
    cv2.imwrite(str(output_root / "comparison_montage.jpg"), montage, [cv2.IMWRITE_JPEG_QUALITY, 95])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-dimension", type=int, default=896)
    args = parser.parse_args()
    if args.input:
        result = run_orientation_experiment(args.input, args.config, args.output, args.max_dimension)
        print(json.dumps({key: result[key] for key in ("input_path", "selected_angle_deg", "geometry_candidate_valid", "vertical_coverage_ratio", "failure_reason")}, ensure_ascii=False))
        return 0
    inputs = {
        "center": ROOT / "output/s1_real_seam_images/seam_center_full_fov_01.bmp",
        "left": ROOT / "output/s1_real_seam_images/seam_left_full_fov_01.bmp",
        "right": ROOT / "output/s1_real_seam_images/seam_right_full_fov_01.bmp",
        "tilt": ROOT / "output/s1_real_seam_images/seam_tilt_full_fov_01.bmp",
    }
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for case, path in inputs.items():
        results.append((case, run_orientation_experiment(path, args.config, args.output / case, args.max_dimension)))
    _write_root_summary(results, args.output)
    _make_montage(results, args.output)
    print(json.dumps([{"case": case, "angle": result["selected_angle_deg"], "geometry": result["geometry_candidate_valid"]} for case, result in results], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
