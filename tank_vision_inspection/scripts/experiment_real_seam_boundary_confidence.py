#!/usr/bin/env python3
"""Offline S1-D4 confidence-gated boundary experiment.

This experiment keeps the S1-D1 signed-gradient candidate generator intact.
It adds an explicit, image-derived width model and rejects candidate pairs that
do not have enough two-sided evidence or that jump away from a stable path.
The S1-D3 orientation search is called unchanged and remains outside the
production detector.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Any, Sequence

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.experiment_real_seam_band import (  # noqa: E402
    DEFAULT_CONFIG,
    BandPair,
    BoundaryTrack,
    _mean_profile,
    _pair_candidates,
    _quality_summary,
    find_boundary_track,
    sha256,
)
from scripts.experiment_real_seam_orientation import (  # noqa: E402
    DEFAULT_ANGLES,
    _points_valid,
    _track_inside_valid_region,
    inverse_affine_points,
    rotate_processing_image,
    run_orientation_experiment,
)
from tank_vision.config import load_config  # noqa: E402
from tank_vision.preprocessing.normalize import select_gray_channel  # noqa: E402
from tank_vision.preprocessing.roi import ImageView, make_image_view  # noqa: E402
from tank_vision.seam.geometry import PolynomialFit, fit_polynomial_ransac  # noqa: E402


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "output/s1_d4_boundary_confidence_experiment"
SCAN_STEP = 8
SCAN_HALF_BAND = 5
MAX_INTERPOLATED_MISSING_BANDS = 2
MIN_LOCAL_CONFIDENCE = 0.18


@dataclass(frozen=True)
class ConfidenceModel:
    width_median: float
    width_mad: float
    width_tolerance: float
    edge_median: float
    contrast_median: float
    seed_count: int
    seed_edge_quantile: float


@dataclass(frozen=True)
class CandidateEvidence:
    pair: BandPair
    local_confidence: float
    edge_score: float
    contrast_score: float
    width_score: float
    reasons: tuple[str, ...]


@dataclass
class ScanRecord:
    y: int
    candidates: list[CandidateEvidence]
    all_pairs: list[BandPair]
    edge_candidates: tuple[list[Any], list[Any]]


@dataclass
class PathResult:
    reliable_points: list[BandPair]
    display_points: list[BandPair]
    interpolated_y: set[int]
    selected_candidates: set[tuple[int, int, int]]
    rejected_candidates: list[dict[str, Any]]
    rejected_scan_band_count: int
    longest_missing_boundary_run: int
    failure_reason: str | None


def _finite(value: float | int | None) -> float | int | None:
    if value is None:
        return None
    return float(value) if math.isfinite(float(value)) else None


def _robust_model(provisional: BoundaryTrack) -> ConfidenceModel:
    points = sorted(provisional.points, key=lambda item: item.y)
    if not points:
        return ConfidenceModel(0.0, 0.0, 1.0, 1.0, 1.0, 0, 60.0)
    widths = np.asarray([point.width for point in points], dtype=np.float64)
    edges = np.asarray([min(point.left_strength, point.right_strength) for point in points], dtype=np.float64)
    contrasts = np.asarray([max(0.0, point.contrast) for point in points], dtype=np.float64)
    quantile = 60.0
    high = edges >= np.percentile(edges, quantile)
    if int(np.count_nonzero(high)) < 3:
        high = np.ones(len(points), dtype=bool)
    high_widths = widths[high]
    high_edges = edges[high]
    high_contrasts = contrasts[high]
    width_median = float(np.median(high_widths))
    width_mad = float(np.median(np.abs(high_widths - width_median)))
    # The tolerance is relative to the image-derived distribution; it is not
    # a fixed weld width and is used only to reject anomalous candidates.
    width_tolerance = max(3.0 * width_mad, 0.40 * max(width_median, 1.0))
    return ConfidenceModel(
        width_median=width_median,
        width_mad=width_mad,
        width_tolerance=width_tolerance,
        edge_median=max(float(np.median(high_edges)), 1e-6),
        contrast_median=max(float(np.median(high_contrasts)), 1.0),
        seed_count=int(np.count_nonzero(high)),
        seed_edge_quantile=quantile,
    )


def _candidate_evidence(pair: BandPair, model: ConfidenceModel) -> CandidateEvidence:
    edge = min(float(pair.left_strength), float(pair.right_strength))
    edge_score = min(1.0, edge / model.edge_median)
    contrast_score = min(1.0, max(0.0, float(pair.contrast)) / model.contrast_median)
    width_delta = abs(float(pair.width) - model.width_median)
    width_score = max(0.0, 1.0 - width_delta / max(model.width_tolerance, 1e-6))
    confidence = 0.45 * edge_score + 0.30 * contrast_score + 0.25 * width_score
    reasons: list[str] = []
    if width_delta > model.width_tolerance:
        reasons.append("width_outside_robust_model")
    if confidence < MIN_LOCAL_CONFIDENCE:
        reasons.append("weak_two_sided_boundary_evidence")
    if pair.left_strength <= 0 or pair.right_strength <= 0:
        reasons.append("incorrect_signed_gradient_direction")
    if pair.contrast <= 0:
        reasons.append("non_positive_interior_external_contrast")
    return CandidateEvidence(pair, float(confidence), float(edge_score), float(contrast_score), float(width_score), tuple(reasons))


def _scan_records(image: np.ndarray, valid: np.ndarray, model: ConfidenceModel) -> list[ScanRecord]:
    records: list[ScanRecord] = []
    for y in range(0, image.shape[0], SCAN_STEP):
        profile = _mean_profile(image, y, SCAN_HALF_BAND)
        pairs, left_edges, right_edges = _pair_candidates(profile, y, image.shape[1])
        evidence = []
        for pair in pairs:
            item = _candidate_evidence(pair, model)
            if valid[y, pair.left] == 0 or valid[y, pair.right] == 0:
                item = CandidateEvidence(item.pair, item.local_confidence, item.edge_score, item.contrast_score, item.width_score, item.reasons + ("outside_rotated_valid_region",))
            evidence.append(item)
        records.append(ScanRecord(y, evidence, pairs, (left_edges, right_edges)))
    return records


def _continuity_limits(model: ConfidenceModel, gap_bands: int = 0) -> tuple[float, float]:
    multiplier = max(1, gap_bands + 1)
    return max(10.0, 0.20 * model.width_median) * multiplier, max(12.0, 0.35 * model.width_median) * multiplier


def _pair_key(pair: BandPair) -> tuple[int, int, int]:
    return (int(pair.y), int(pair.left), int(pair.right))


def _path_for_records(records: list[ScanRecord], model: ConfidenceModel) -> PathResult:
    """Choose a longest evidence-supported path, allowing only short gaps."""
    # Each state is (last pair, reliable path, score, missing bands).
    states: list[tuple[BandPair, list[BandPair], float, int]] = []
    completed: list[tuple[list[BandPair], float]] = []
    rejected: list[dict[str, Any]] = []
    selected_keys: set[tuple[int, int, int]] = set()
    invalid_run = 0
    longest_invalid_run = 0
    for record in records:
        viable = [item for item in record.candidates if not item.reasons]
        if viable:
            invalid_run = 0
        else:
            invalid_run += 1
            longest_invalid_run = max(longest_invalid_run, invalid_run)
        next_states: list[tuple[BandPair, list[BandPair], float, int]] = []
        # Preserve a path across at most the explicitly allowed number of
        # invalid scan bands. No point is added while a state is carried.
        for previous, path, score, missing in states:
            if missing < MAX_INTERPOLATED_MISSING_BANDS:
                next_states.append((previous, path, score, missing + 1))
            else:
                completed.append((path, score))
        for item in record.candidates:
            if item.reasons:
                rejected.append({
                    "y_processed": record.y,
                    "left_processed": item.pair.left,
                    "right_processed": item.pair.right,
                    "width_processed": item.pair.width,
                    "left_strength": item.pair.left_strength,
                    "right_strength": item.pair.right_strength,
                    "contrast": item.pair.contrast,
                    "local_confidence": item.local_confidence,
                    "rejection_reasons": list(item.reasons),
                })
        for item in viable:
            best: tuple[float, list[BandPair], int] | None = None
            for previous, path, score, missing in states:
                gap = max(0, (record.y - previous.y) // SCAN_STEP - 1)
                if gap > MAX_INTERPOLATED_MISSING_BANDS:
                    completed.append((path, score))
                    continue
                max_center, max_width = _continuity_limits(model, gap)
                center_delta = abs(item.pair.center - previous.center)
                width_delta = abs(item.pair.width - previous.width)
                left_delta = abs(item.pair.left - previous.left)
                right_delta = abs(item.pair.right - previous.right)
                # Endpoint continuity is checked independently. A stable
                # midpoint can otherwise hide one boundary jumping to a
                # remote background edge while the other boundary moves too.
                max_boundary_step = max(12.0, 0.20 * model.width_median)
                continuity_reasons = []
                if center_delta > max_center:
                    continuity_reasons.append("center_jump_from_previous_accepted_pair")
                if width_delta > max_width:
                    continuity_reasons.append("width_jump_from_previous_accepted_pair")
                if left_delta > max_boundary_step:
                    continuity_reasons.append("left_boundary_jump_from_previous_accepted_pair")
                if right_delta > max_boundary_step:
                    continuity_reasons.append("right_boundary_jump_from_previous_accepted_pair")
                if continuity_reasons:
                    rejected.append({
                        "y_processed": record.y,
                        "left_processed": item.pair.left,
                        "right_processed": item.pair.right,
                        "width_processed": item.pair.width,
                        "left_strength": item.pair.left_strength,
                        "right_strength": item.pair.right_strength,
                        "contrast": item.pair.contrast,
                        "local_confidence": item.local_confidence,
                        "rejection_reasons": continuity_reasons,
                    })
                    continue
                continuity = 0.15 * max(0.0, 1.0 - center_delta / max_center) + 0.10 * max(0.0, 1.0 - width_delta / max_width)
                candidate = (score + item.local_confidence + continuity, path + [item.pair], 0)
                if best is None or candidate[0] > best[0] or (candidate[0] == best[0] and len(candidate[1]) > len(best[1])):
                    best = candidate
            if best is not None:
                next_states.append((best[1][-1], best[1], best[0], best[2]))
            else:
                # Starting a new path is allowed only from a pair with local
                # evidence; it never fills a missing side or uses image centre.
                next_states.append((item.pair, [item.pair], item.local_confidence, 0))
        # Keep the best state for each endpoint to make selection deterministic.
        compact: dict[tuple[int, int, int], tuple[BandPair, list[BandPair], float, int]] = {}
        for state in next_states:
            key = _pair_key(state[0])
            old = compact.get(key)
            if old is None or (len(state[1]), state[2]) > (len(old[1]), old[2]):
                compact[key] = state
        states = list(compact.values())
    completed.extend((state[1], state[2]) for state in states)
    if not completed:
        return PathResult([], [], set(), set(), rejected, len(records), max(len(records), longest_invalid_run), "no_reliable_boundary_pair")
    best_path, _ = max(completed, key=lambda item: (len(item[0]), item[1]))
    best_path = sorted(best_path, key=lambda item: item.y)
    selected_keys = {_pair_key(point) for point in best_path}
    display = list(best_path)
    interpolated_y: set[int] = set()
    for previous, current in zip(best_path[:-1], best_path[1:]):
        missing = list(range(previous.y + SCAN_STEP, current.y, SCAN_STEP))
        if 0 < len(missing) <= MAX_INTERPOLATED_MISSING_BANDS:
            for y in missing:
                fraction = (y - previous.y) / (current.y - previous.y)
                left = round(previous.left + fraction * (current.left - previous.left))
                right = round(previous.right + fraction * (current.right - previous.right))
                display.append(BandPair(y, left, right, (previous.left_strength + current.left_strength) * 0.5, (previous.right_strength + current.right_strength) * 0.5, (previous.contrast + current.contrast) * 0.5, 0.0))
                interpolated_y.add(y)
    display.sort(key=lambda item: item.y)
    gaps = [max(0, (b.y - a.y) // SCAN_STEP - 1) for a, b in zip(best_path[:-1], best_path[1:])]
    longest_gap = max(max(gaps, default=0), longest_invalid_run)
    rejected_count = max(0, len(records) - len(best_path))
    failure_reason = None if len(best_path) >= 3 else "reliable_boundary_path_has_fewer_than_three_points"
    return PathResult(best_path, display, interpolated_y, selected_keys, rejected, rejected_count, longest_gap, failure_reason)


def _restore_pair(pair: BandPair, matrix: np.ndarray, view: ImageView) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    processing = inverse_affine_points([(pair.left, pair.y), (pair.right, pair.y), (pair.center, pair.y)], matrix)
    original = np.asarray([view.to_original(x, y) for x, y in processing], dtype=np.float64)
    return original[0], original[1], original[2]


def _restore_points(points: Sequence[BandPair], matrix: np.ndarray, view: ImageView) -> dict[str, np.ndarray]:
    left, right, center = [], [], []
    for point in sorted(points, key=lambda item: item.y):
        a, b, c = _restore_pair(point, matrix, view)
        left.append(a)
        right.append(b)
        center.append(c)
    return {"left": np.asarray(left, dtype=np.float64).reshape(-1, 2), "right": np.asarray(right, dtype=np.float64).reshape(-1, 2), "center": np.asarray(center, dtype=np.float64).reshape(-1, 2)}


def _segment_points(points: np.ndarray, ys: Sequence[int] | None = None) -> list[np.ndarray]:
    if len(points) < 2:
        return []
    if ys is None:
        return [points]
    segments: list[list[np.ndarray]] = [[points[0]]]
    for index in range(1, len(points)):
        if ys[index] - ys[index - 1] > SCAN_STEP:
            segments.append([])
        segments[-1].append(points[index])
    return [np.asarray(segment) for segment in segments if len(segment) >= 2]


def _draw_segments(image: np.ndarray, points: np.ndarray, colour: tuple[int, int, int], thickness: int, ys: Sequence[int] | None = None) -> None:
    for segment in _segment_points(points, ys):
        cv2.polylines(image, [np.round(segment).astype(np.int32).reshape(-1, 1, 2)], False, colour, thickness, cv2.LINE_AA)


def _label(image: np.ndarray, text: str, y: int, colour: tuple[int, int, int] = (255, 255, 255)) -> None:
    cv2.putText(image, text, (24, y), cv2.FONT_HERSHEY_SIMPLEX, .78, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (24, y), cv2.FONT_HERSHEY_SIMPLEX, .78, colour, 2, cv2.LINE_AA)


def _base_overlay(image: np.ndarray, label: str) -> np.ndarray:
    output = image.copy()
    center_x = output.shape[1] // 2
    cv2.line(output, (center_x, 0), (center_x, output.shape[0] - 1), (255, 255, 255), 2)
    _label(output, label, 44, (255, 255, 255))
    _label(output, "left=blue right=green center=yellow rejected=red X | original coordinates", 82, (255, 255, 255))
    return output


def _draw_candidate_overlay(image: np.ndarray, records: list[ScanRecord], matrix: np.ndarray, view: ImageView, selected: set[tuple[int, int, int]], label: str, rejected_only: bool = False) -> np.ndarray:
    output = _base_overlay(image, label)
    for record in records:
        for pair in record.all_pairs:
            if rejected_only and _pair_key(pair) in selected:
                continue
            a, b, _ = _restore_pair(pair, matrix, view)
            colour = (0, 0, 255) if _pair_key(pair) not in selected else (255, 255, 0)
            for point in (a, b):
                x, y = np.round(point).astype(int)
                if 0 <= x < output.shape[1] and 0 <= y < output.shape[0]:
                    if colour == (0, 0, 255):
                        cv2.drawMarker(output, (x, y), colour, cv2.MARKER_TILTED_CROSS, 10, 3, cv2.LINE_AA)
                    else:
                        cv2.circle(output, (x, y), 2, colour, -1)
    return output


def _draw_accepted_overlay(image: np.ndarray, path: PathResult, matrix: np.ndarray, view: ImageView, label: str, centerline: bool = False) -> np.ndarray:
    output = _base_overlay(image, label)
    restored = _restore_points(path.display_points, matrix, view)
    ys = [point.y for point in sorted(path.display_points, key=lambda item: item.y)]
    _draw_segments(output, restored["left"], (255, 0, 0), 7, ys)
    _draw_segments(output, restored["right"], (0, 255, 0), 7, ys)
    _draw_segments(output, restored["center"], (0, 255, 255), 7, ys)
    if centerline:
        reliable = _restore_points(path.reliable_points, matrix, view)
        if len(reliable["center"]) >= 3:
            fit = fit_polynomial_ransac(reliable["center"][:, 1], reliable["center"][:, 0], 2, 120, 8.0 / view.scale_x)
            if fit is not None:
                for segment in _segment_points(reliable["center"], [p.y for p in sorted(path.reliable_points, key=lambda item: item.y)]):
                    y0, y1 = float(np.min(segment[:, 1])), float(np.max(segment[:, 1]))
                    curve = np.column_stack((np.polyval(fit.coefficients, np.linspace(y0, y1, 100)), np.linspace(y0, y1, 100)))
                    _draw_segments(output, curve, (0, 255, 255), 9)
    return output


def _track_jump_stats(points: dict[str, np.ndarray], scan_points: list[BandPair], view: ImageView, reference_width: float | None = None) -> dict[str, Any]:
    def distances(values: np.ndarray) -> np.ndarray:
        return np.linalg.norm(np.diff(values, axis=0), axis=1) if len(values) > 1 else np.empty(0)
    left_step = distances(points["left"])
    right_step = distances(points["right"])
    widths = np.linalg.norm(points["right"] - points["left"], axis=1) if len(points["left"]) else np.empty(0)
    width_change = np.abs(np.diff(widths)) if len(widths) > 1 else np.empty(0)
    median_width = float(np.median(widths)) if len(widths) else 0.0
    threshold_width = float(reference_width) if reference_width is not None else median_width
    step_threshold = max(12.0, 0.20 * threshold_width)
    width_threshold = max(12.0, 0.30 * threshold_width)
    y_values = np.asarray([point.y / view.scale_y for point in sorted(scan_points, key=lambda item: item.y)], dtype=np.float64)
    def positions(values: np.ndarray, threshold: float) -> list[float]:
        return [float(y_values[index + 1]) for index, value in enumerate(values) if value > threshold and index + 1 < len(y_values)]
    return {
        "left_boundary_step_px": {"p50": float(np.percentile(left_step, 50)) if len(left_step) else 0.0, "p95": float(np.percentile(left_step, 95)) if len(left_step) else 0.0, "max": float(np.max(left_step)) if len(left_step) else 0.0},
        "right_boundary_step_px": {"p50": float(np.percentile(right_step, 50)) if len(right_step) else 0.0, "p95": float(np.percentile(right_step, 95)) if len(right_step) else 0.0, "max": float(np.max(right_step)) if len(right_step) else 0.0},
        "weld_width_change_px": {"p50": float(np.percentile(width_change, 50)) if len(width_change) else 0.0, "p95": float(np.percentile(width_change, 95)) if len(width_change) else 0.0, "max": float(np.max(width_change)) if len(width_change) else 0.0},
        "width_p95_to_median_ratio": float(np.percentile(widths, 95) / median_width) if len(widths) and median_width else None,
        "abrupt_left_jump_count": int(np.count_nonzero(left_step > step_threshold)),
        "abrupt_right_jump_count": int(np.count_nonzero(right_step > step_threshold)),
        "abrupt_left_jump_y_original": positions(left_step, step_threshold),
        "abrupt_right_jump_y_original": positions(right_step, step_threshold),
        "abrupt_width_change_count": int(np.count_nonzero(width_change > width_threshold)),
        "width_change_y_original": positions(width_change, width_threshold),
        "step_threshold_px": float(step_threshold),
        "width_change_threshold_px": float(width_threshold),
        "threshold_reference_width_px": float(threshold_width),
        "measured_width_median_px": float(median_width),
    }


def _legacy_baseline(image: np.ndarray, view: ImageView, gray: np.ndarray, angle: float, matrix: np.ndarray, valid: np.ndarray) -> dict[str, Any]:
    track = _track_inside_valid_region(find_boundary_track(gray), valid)
    restored = _restore_points(track.points, matrix, view)
    stats = _track_jump_stats(restored, track.points, view)
    total_bands = len(range(0, gray.shape[0], SCAN_STEP))
    stats.update({
        "selected_angle_deg": angle,
        "valid_left_boundary_point_count": len(track.points),
        "valid_right_boundary_point_count": len(track.points),
        "valid_center_point_count": len(track.points),
        "rejected_scan_band_count": 0,
        "longest_missing_boundary_run": 0,
        "vertical_coverage_ratio": float((track.points[-1].y - track.points[0].y + 1) / gray.shape[0]) if track.points else 0.0,
        "legacy_note": "S1-D1 exposes no per-scan rejection state; rejected_scan_band_count is recorded as zero rather than inferred from path truncation.",
        "total_scan_bands": total_bands,
        "reference_width_px": float(np.median(np.linalg.norm(restored["right"] - restored["left"], axis=1))) if len(restored["left"]) else None,
    })
    return stats


def run_case(input_path: Path, config_path: Path, output_dir: Path, d3_reference_dir: Path) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    input_hash_before = sha256(input_path)
    image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise RuntimeError(f"Unable to decode expected BGR image: {input_path}")
    config = load_config(config_path)
    view = make_image_view(image, config.pipeline.roi, 896)
    gray = cv2.GaussianBlur(select_gray_channel(view.image, config.seam.color_channel), (5, 5), 0)
    with tempfile.TemporaryDirectory(prefix="s1_d4_orientation_") as temp_dir:
        d3 = run_orientation_experiment(input_path, config_path, Path(temp_dir) / "orientation", 896)
    angle = float(d3["selected_angle_deg"]) if d3["selected_angle_deg"] is not None else None
    if angle is None:
        raise RuntimeError(f"S1-D3 orientation search failed for {input_path}")
    rotated, valid, matrix = rotate_processing_image(gray, angle)
    provisional = _track_inside_valid_region(find_boundary_track(rotated), valid)
    model = _robust_model(provisional)
    records = _scan_records(rotated, valid, model)
    path = _path_for_records(records, model)
    restored = _restore_points(path.display_points, matrix, view)
    reliable_restored = _restore_points(path.reliable_points, matrix, view)
    coordinates_valid = all(_points_valid(values, image.shape[1], image.shape[0]) for values in restored.values())
    center_between = bool(path.reliable_points and all(p.left < p.center < p.right for p in path.reliable_points))
    restored_fit = None
    if len(reliable_restored["center"]) >= 3:
        restored_fit = fit_polynomial_ransac(reliable_restored["center"][:, 1], reliable_restored["center"][:, 0], 2, 120, 8.0 / view.scale_x)
    total_bands = len(range(0, rotated.shape[0], SCAN_STEP))
    coverage = len(path.reliable_points) / max(total_bands, 1)
    geometry_valid = bool(restored_fit is not None and coverage >= 0.60 and center_between and coordinates_valid)
    if not path.reliable_points:
        failure_reason = path.failure_reason or "no_reliable_boundary_pair"
    elif coverage < 0.60:
        failure_reason = "reliable_vertical_coverage_below_0.60"
    elif restored_fit is None:
        failure_reason = "centerline_ransac_fit_failed_on_reliable_midpoints"
    elif not coordinates_valid:
        failure_reason = "restored_coordinates_out_of_original_bounds"
    elif not center_between:
        failure_reason = "centerline_not_between_reliable_boundaries"
    else:
        failure_reason = None
    if restored_fit is not None:
        reference_y = config.seam.reference_y_ratio * (image.shape[0] - 1)
        target_x = config.seam.reference_x_ratio * image.shape[1]
        lateral = float(np.polyval(restored_fit.coefficients, reference_y) - target_x)
        heading = float(math.degrees(math.atan(float(np.polyval(np.polyder(restored_fit.coefficients), reference_y)))))
        residual = float(restored_fit.residual_rms)
        inlier_ratio = float(np.mean(restored_fit.inliers))
    else:
        lateral = heading = residual = None
        inlier_ratio = 0.0
    widths = np.linalg.norm(reliable_restored["right"] - reliable_restored["left"], axis=1) if len(reliable_restored["left"]) else np.empty(0)
    baseline = _legacy_baseline(image, view, rotated, angle, matrix, valid)
    after_jumps = _track_jump_stats(reliable_restored, path.reliable_points, view, baseline.get("reference_width_px"))
    quality = _quality_summary(image, config)
    input_hash_after = sha256(input_path)
    candidate_overlay = _draw_candidate_overlay(image, records, matrix, view, path.selected_candidates, "S1-D4 boundary candidates | red X=rejected | original coordinates")
    accepted_overlay = _draw_accepted_overlay(image, path, matrix, view, "S1-D4 accepted boundaries | short gaps only interpolated", False)
    rejected_overlay = _draw_candidate_overlay(image, records, matrix, view, path.selected_candidates, "S1-D4 rejected candidates | red X | original coordinates", True)
    center_overlay = _draw_accepted_overlay(image, path, matrix, view, "S1-D4 restored centerline | reliable midpoints only", True)
    cv2.imwrite(str(output_dir / "boundary_candidates_overlay.jpg"), candidate_overlay, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "accepted_boundaries_overlay.jpg"), accepted_overlay, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "rejected_boundaries_overlay.jpg"), rejected_overlay, [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(output_dir / "restored_centerline_overlay.jpg"), center_overlay, [cv2.IMWRITE_JPEG_QUALITY, 95])
    angle_stats = {
        "angle_deg": angle,
        "rotation_matrix": np.asarray(matrix, dtype=np.float64).round(12).tolist(),
        "valid_rotated_pixel_ratio": float(np.count_nonzero(valid) / valid.size),
        "valid_rotated_bbox": {
            "x": int(np.min(np.where(valid != 0)[1])), "y": int(np.min(np.where(valid != 0)[0])),
            "width": int(np.max(np.where(valid != 0)[1]) - np.min(np.where(valid != 0)[1]) + 1),
            "height": int(np.max(np.where(valid != 0)[0]) - np.min(np.where(valid != 0)[0]) + 1),
        } if np.count_nonzero(valid) else None,
        "model": {key: _finite(value) for key, value in vars(model).items()},
        "candidate_scan_band_count": len(records),
        "rejected_scan_band_count": path.rejected_scan_band_count,
        "longest_missing_boundary_run": path.longest_missing_boundary_run,
        "interpolated_scan_band_count": len(path.interpolated_y),
        "candidates": [
            {
                "y_processed": record.y,
                "candidate_count": len(record.all_pairs),
                "candidates": [
                    {"left_processed": item.pair.left, "right_processed": item.pair.right, "width_processed": item.pair.width, "left_strength": item.pair.left_strength, "right_strength": item.pair.right_strength, "contrast": item.pair.contrast, "local_confidence": item.local_confidence, "edge_score": item.edge_score, "contrast_score": item.contrast_score, "width_score": item.width_score, "rejection_reasons": list(item.reasons), "selected": _pair_key(item.pair) in path.selected_candidates}
                    for item in record.candidates
                ],
            }
            for record in records
        ],
        "rejected_path_candidates": path.rejected_candidates,
    }
    (output_dir / "boundary_confidence.json").write_text(json.dumps(angle_stats, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    result = {
        "experiment": "S1-D4",
        "input_path": str(input_path),
        "input_sha256": input_hash_before,
        "input_sha256_after": input_hash_after,
        "input_unchanged": input_hash_before == input_hash_after,
        "original_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
        "processing_size": {"width": int(view.image.shape[1]), "height": int(view.image.shape[0])},
        "scale_x": float(view.scale_x),
        "scale_y": float(view.scale_y),
        "selected_angle_deg": angle,
        "geometry_candidate_valid": geometry_valid,
        "failure_reason": failure_reason,
        "image_quality_valid": bool(quality["valid_for_measurement"]),
        "valid_left_boundary_point_count": len(path.reliable_points),
        "valid_right_boundary_point_count": len(path.reliable_points),
        "valid_center_point_count": len(path.reliable_points),
        "vertical_coverage_ratio": float(coverage),
        "vertical_span_coverage_ratio": float((path.reliable_points[-1].y - path.reliable_points[0].y + 1) / rotated.shape[0]) if path.reliable_points else 0.0,
        "weld_width_px_median": float(np.median(widths)) if len(widths) else None,
        "weld_width_px_p05": float(np.percentile(widths, 5)) if len(widths) else None,
        "weld_width_px_p95": float(np.percentile(widths, 95)) if len(widths) else None,
        "width_p95_to_median_ratio": float(np.percentile(widths, 95) / np.median(widths)) if len(widths) and np.median(widths) else None,
        "centerline_fit_residual_px": residual,
        "centerline_inlier_ratio": inlier_ratio,
        "lateral_error_px": lateral,
        "heading_error_deg": heading,
        "rejected_scan_band_count": path.rejected_scan_band_count,
        "interpolated_scan_band_count": len(path.interpolated_y),
        "longest_missing_boundary_run": path.longest_missing_boundary_run,
        "centerline_between_boundaries": center_between,
        "restored_coordinates_valid": coordinates_valid,
        "processing_time_ms": (time.perf_counter() - started) * 1000.0,
        "quality": quality,
        "before_baseline": baseline,
        "after_jump_stats": after_jumps,
        "abrupt_left_jump_count": after_jumps["abrupt_left_jump_count"],
        "abrupt_right_jump_count": after_jumps["abrupt_right_jump_count"],
        "abrupt_width_change_count": after_jumps["abrupt_width_change_count"],
        "production_valid": False,
        "experimental_only": True,
        "parameters": {
            "max_dimension": 896,
            "scan_step_processed_px": SCAN_STEP,
            "scan_band_half_height_processed_px": SCAN_HALF_BAND,
            "max_interpolated_missing_bands": MAX_INTERPOLATED_MISSING_BANDS,
            "min_local_confidence": MIN_LOCAL_CONFIDENCE,
            "width_model": "median plus MAD from relative top-40-percent two-sided evidence; no fixed width",
            "s1_d3_direction_search_source": "scripts/experiment_real_seam_orientation.py (called unchanged)",
        },
        "limitations": [
            "offline experiment on four images; no generalization claim",
            "confidence gating can reduce coverage when only one or both boundaries are weak",
            "short gaps are interpolated only when bounded by reliable pairs; long gaps remain unconnected",
            "right and tilt lower-edge appearance still requires manual review",
            "quality threshold, production detector, ROS, and formal pipeline were not changed",
        ],
    }
    (output_dir / "experiment_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return result


def _write_montages(results: list[tuple[str, dict[str, Any]]], root: Path, before_root: Path) -> None:
    after_tiles, before_after_rows = [], []
    for case, result in results:
        after = cv2.imread(str(root / case / "restored_centerline_overlay.jpg"), cv2.IMREAD_COLOR)
        before = cv2.imread(str(before_root / case / "restored_centerline_overlay.jpg"), cv2.IMREAD_COLOR)
        if after is None:
            after = np.zeros((672, 896, 3), dtype=np.uint8)
        if before is None:
            before = np.zeros_like(after)
        after = cv2.resize(after, (896, 672), interpolation=cv2.INTER_AREA)
        before = cv2.resize(before, (896, 672), interpolation=cv2.INTER_AREA)
        text = f"{case} geom={str(result['geometry_candidate_valid']).lower()} cov={result['vertical_coverage_ratio']:.3f} rej={result['rejected_scan_band_count']} width={result['weld_width_px_median'] if result['weld_width_px_median'] is not None else 'n/a'}"
        for tile, title in ((after, "after"), (before, "before")):
            cv2.rectangle(tile, (0, 0), (896, 42), (0, 0, 0), -1)
            cv2.putText(tile, f"{title} | {text}", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1, cv2.LINE_AA)
        after_tiles.append(after)
        before_after_rows.append(np.hstack((before, after)))
    while len(after_tiles) < 4:
        after_tiles.append(np.zeros_like(after_tiles[0]))
    cv2.imwrite(str(root / "comparison_montage.jpg"), np.vstack((np.hstack(after_tiles[:2]), np.hstack(after_tiles[2:4]))), [cv2.IMWRITE_JPEG_QUALITY, 95])
    cv2.imwrite(str(root / "before_after_montage.jpg"), np.vstack(before_after_rows), [cv2.IMWRITE_JPEG_QUALITY, 95])


def _write_summary(results: list[tuple[str, dict[str, Any]]], root: Path) -> None:
    fields = ["case", "selected_angle_deg", "geometry_candidate_valid", "failure_reason", "image_quality_valid", "valid_left_boundary_point_count", "valid_right_boundary_point_count", "valid_center_point_count", "vertical_coverage_ratio", "weld_width_px_median", "weld_width_px_p05", "weld_width_px_p95", "width_p95_to_median_ratio", "centerline_fit_residual_px", "centerline_inlier_ratio", "lateral_error_px", "heading_error_deg", "rejected_scan_band_count", "interpolated_scan_band_count", "longest_missing_boundary_run", "abrupt_right_jump_count", "abrupt_width_change_count", "centerline_between_boundaries", "restored_coordinates_valid", "processing_time_ms", "input_sha256", "input_unchanged"]
    rows = [{"case": case, **{key: result.get(key) for key in fields if key != "case"}} for case, result in results]
    summary = {
        "experiment": "S1-D4",
        "algorithm_source": "S1-D1 candidate generator + S1-D4 confidence gating; S1-D3 direction search unchanged",
        "angles_deg": list(DEFAULT_ANGLES),
        "results": rows,
        "checks": {
            "all_input_unchanged": all(result["input_unchanged"] for _, result in results),
            "all_coordinates_valid": all(result["restored_coordinates_valid"] for _, result in results),
            "s1_d3_angles_preserved": [result["selected_angle_deg"] for _, result in results] == [0.0, 0.0, 0.0, -25.0],
            "left_lateral_more_negative_than_center": (
                rows[1]["lateral_error_px"] is not None and rows[0]["lateral_error_px"] is not None
                and rows[1]["lateral_error_px"] < rows[0]["lateral_error_px"]
            ),
            "right_lateral_more_positive_than_center": (
                rows[2]["lateral_error_px"] is not None and rows[0]["lateral_error_px"] is not None
                and rows[2]["lateral_error_px"] > rows[0]["lateral_error_px"]
            ),
            "tilt_heading_positive": rows[3]["heading_error_deg"] is not None and rows[3]["heading_error_deg"] > 0,
            "right_jump_rejection_recorded": rows[2]["rejected_scan_band_count"] > 0,
            "tilt_jump_rejection_recorded": rows[3]["rejected_scan_band_count"] > 0,
        },
        "note": "No fixed width, filename correction, image-centre result, or formal detector change was used.",
    }
    (root / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    before = {
        "experiment": "S1-D4 baseline before confidence gating",
        "algorithm_source": "S1-D1/S1-D3 legacy selected boundary path",
        "results": {case: result["before_baseline"] for case, result in results},
        "note": "The legacy detector does not expose per-scan rejection state; its rejected_scan_band_count is explicitly recorded as zero rather than inferred.",
    }
    (root / "before_baseline.json").write_text(json.dumps(before, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    with (root / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    inputs = {"center": ROOT / "output/s1_real_seam_images/seam_center_full_fov_01.bmp", "left": ROOT / "output/s1_real_seam_images/seam_left_full_fov_01.bmp", "right": ROOT / "output/s1_real_seam_images/seam_right_full_fov_01.bmp", "tilt": ROOT / "output/s1_real_seam_images/seam_tilt_full_fov_01.bmp"}
    args.output.mkdir(parents=True, exist_ok=True)
    results = [(case, run_case(path, args.config, args.output / case, ROOT / "output/s1_d3_orientation_experiment")) for case, path in inputs.items()]
    _write_summary(results, args.output)
    _write_montages(results, args.output, ROOT / "output/s1_d3_orientation_experiment")
    print(json.dumps([{"case": case, "angle": result["selected_angle_deg"], "geometry": result["geometry_candidate_valid"], "coverage": result["vertical_coverage_ratio"]} for case, result in results], ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
