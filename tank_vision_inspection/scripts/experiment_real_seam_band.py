#!/usr/bin/env python3
"""Offline S1-D1 experiment: estimate a wide seam from two boundaries.

This file is intentionally independent from the production seam detector.
It uses the existing image-view and robust-fit utilities, but it never changes
or calls the production detector, realtime pipeline, ROS, or defect path.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Sequence

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tank_vision.config import AppConfig, load_config
from tank_vision.preprocessing.image_quality import assess_image_quality
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import ImageView, make_image_view
from tank_vision.seam.geometry import PolynomialFit, fit_polynomial_ransac, seam_errors


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT / "output/s1_real_seam_images/seam_center_full_fov_01.bmp"
DEFAULT_CONFIG = ROOT / "configs/default.yaml"
DEFAULT_OUTPUT = ROOT / "output/s1_d1_real_seam_band_experiment"


@dataclass(frozen=True)
class EdgeCandidate:
    x: int
    strength: float
    polarity: str


@dataclass(frozen=True)
class BandPair:
    y: int
    left: int
    right: int
    left_strength: float
    right_strength: float
    contrast: float
    score: float

    @property
    def width(self) -> int:
        return self.right - self.left

    @property
    def center(self) -> float:
        return (self.left + self.right) * 0.5


@dataclass
class BoundaryTrack:
    points: list[BandPair]
    all_edge_candidates: list[tuple[int, list[EdgeCandidate], list[EdgeCandidate]]]
    failure_reason: str | None = None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _local_maxima(values: np.ndarray, positive: bool, threshold: float, min_distance: int) -> list[int]:
    candidates: list[int] = []
    for index in range(1, len(values) - 1):
        value = float(values[index]) if positive else float(-values[index])
        left = float(values[index - 1]) if positive else float(-values[index - 1])
        right = float(values[index + 1]) if positive else float(-values[index + 1])
        if value >= threshold and value >= left and value >= right:
            candidates.append(index)
    # Keep strong peaks while enforcing a small separation, so local weld
    # reflections cannot create an unbounded number of pair combinations.
    candidates.sort(key=lambda index: float(values[index]) if positive else float(-values[index]), reverse=True)
    selected: list[int] = []
    for index in candidates:
        if all(abs(index - other) >= min_distance for other in selected):
            selected.append(index)
    return sorted(selected)


def _mean_profile(image: np.ndarray, y: int, half_band: int) -> np.ndarray:
    y0 = max(0, y - half_band)
    y1 = min(image.shape[0], y + half_band + 1)
    profile = np.mean(image[y0:y1].astype(np.float32), axis=0)
    # Smooth across x to favor a physical boundary over isolated specular dots.
    return cv2.GaussianBlur(profile.reshape(1, -1), (9, 1), 0).reshape(-1)


def _pair_candidates(profile: np.ndarray, y: int, width: int) -> tuple[list[BandPair], list[EdgeCandidate], list[EdgeCandidate]]:
    derivative = np.gradient(profile)
    magnitude = np.abs(derivative)
    edge_floor = max(0.8, float(np.percentile(magnitude, 88)))
    margin = max(12, int(round(width * 0.04)))
    left_x = _local_maxima(derivative, True, edge_floor, 5)
    right_x = _local_maxima(derivative, False, edge_floor, 5)
    left_x = [x for x in left_x if margin <= x <= width - margin]
    right_x = [x for x in right_x if margin <= x <= width - margin]
    # The broad bound excludes pairing unrelated objects across the full FOV;
    # it is not a fixed weld width and the final width is measured per band.
    # Short 20–40 px processed intervals in this image are repeated weld
    # reflections/textures, not the full wide band. This rejects those short
    # pairs without supplying a missing side or imposing a result width.
    min_width = max(24, int(round(width * 0.06)))
    max_width = int(round(width * 0.42))
    left_candidates = [EdgeCandidate(x, float(derivative[x]), "left") for x in left_x]
    right_candidates = [EdgeCandidate(x, float(-derivative[x]), "right") for x in right_x]
    pairs: list[BandPair] = []
    for left in left_candidates:
        for right in right_candidates:
            if not left.x < right.x:
                continue
            pair_width = right.x - left.x
            if not min_width <= pair_width <= max_width:
                continue
            interior = profile[left.x + 2 : right.x - 1]
            if interior.size < 4:
                continue
            outside_left = profile[max(0, left.x - max(8, pair_width // 4)) : left.x]
            outside_right = profile[right.x + 1 : min(width, right.x + max(8, pair_width // 4))]
            outside = np.concatenate((outside_left, outside_right))
            if outside.size == 0:
                continue
            contrast = float(np.mean(interior) - np.mean(outside))
            edge_strength = math.log1p(left.strength) + math.log1p(right.strength)
            # Positive interior contrast is expected for the visible bright
            # band. A small negative value remains possible for dark seams.
            score = 1.5 * edge_strength + 0.08 * contrast
            pairs.append(BandPair(y, left.x, right.x, left.strength, right.strength, contrast, score))
    pairs.sort(key=lambda pair: pair.score, reverse=True)
    return pairs[:80], left_candidates, right_candidates


def _transition(previous: BandPair, current: BandPair, y_step: int, width: int) -> float | None:
    center_delta = abs(current.center - previous.center)
    width_delta = abs(current.width - previous.width)
    # These are continuity constraints per scan step, not image-width
    # assumptions. They reject jumps to a different reflection/texture.
    max_center_delta = max(10.0, width * 0.075 * max(1.0, y_step / 8.0))
    max_width_delta = max(12.0, width * 0.12 * max(1.0, y_step / 8.0))
    if center_delta > max_center_delta or width_delta > max_width_delta:
        return None
    return 0.10 * center_delta + 0.05 * width_delta


def find_boundary_track(processed_gray: np.ndarray, scan_step: int = 8, half_band: int = 5) -> BoundaryTrack:
    """Find a continuous sequence of left/right edge pairs in processed coords."""
    if processed_gray.ndim != 2 or processed_gray.size == 0:
        return BoundaryTrack([], [], "invalid_processed_gray")
    bands: list[list[BandPair]] = []
    edge_records: list[tuple[int, list[EdgeCandidate], list[EdgeCandidate]]] = []
    for y in range(0, processed_gray.shape[0], scan_step):
        profile = _mean_profile(processed_gray, y, half_band)
        pairs, left, right = _pair_candidates(profile, y, processed_gray.shape[1])
        bands.append(pairs)
        edge_records.append((y, left, right))
    if not any(bands):
        return BoundaryTrack([], edge_records, "no_paired_left_right_edges")

    # Dynamic programming over adjacent scan bands. It prefers a long
    # continuous path, with the pair score as a tie breaker.
    paths: list[list[BandPair]] = []
    path_scores: list[float] = []
    completed_paths: list[list[BandPair]] = []
    for band_index, pairs in enumerate(bands):
        if not pairs:
            completed_paths.extend(paths)
            paths = []
            path_scores = []
            continue
        if not paths:
            paths = [[pair] for pair in pairs]
            path_scores = [pair.score for pair in pairs]
            continue
        new_paths: list[list[BandPair]] = []
        new_scores: list[float] = []
        previous_y = paths[0][-1].y
        y_step = max(1, bands[band_index][0].y - previous_y)
        for pair in pairs:
            best_index = -1
            best_score = pair.score
            for index, previous_path in enumerate(paths):
                penalty = _transition(previous_path[-1], pair, y_step, processed_gray.shape[1])
                if penalty is None:
                    continue
                score = path_scores[index] + pair.score - penalty
                if best_index < 0 or score > best_score:
                    best_index = index
                    best_score = score
            if best_index >= 0:
                new_paths.append(paths[best_index] + [pair])
                new_scores.append(best_score)
            else:
                new_paths.append([pair])
                new_scores.append(pair.score)
        paths, path_scores = new_paths, new_scores
    completed_paths.extend(paths)
    if not completed_paths:
        return BoundaryTrack([], edge_records, "no_continuous_boundary_path")
    # Coverage is evaluated below; selecting by path length prevents a strong
    # short reflection from winning over a weaker but continuous weld band.
    best = max(completed_paths, key=lambda path: (len(path), sum(pair.score for pair in path)))
    if len(best) < 3:
        return BoundaryTrack(best, edge_records, "continuous_boundary_path_has_fewer_than_three_points")
    return BoundaryTrack(best, edge_records)


def _restore(view: ImageView, x: float, y: float) -> tuple[float, float]:
    return view.to_original(float(x), float(y))


def _finite_point(point: Sequence[float], width: int, height: int) -> bool:
    return len(point) == 2 and all(math.isfinite(float(value)) for value in point) and 0 <= point[0] < width and 0 <= point[1] < height


def _quality_summary(image: np.ndarray, config: AppConfig) -> dict[str, Any]:
    result = assess_image_quality(image, config.quality)
    return {
        "valid_image": bool(result.valid_image),
        "valid_for_measurement": bool(result.valid_for_measurement),
        "mean_brightness": float(result.mean_brightness),
        "underexposed_ratio": float(result.underexposed_ratio),
        "overexposed_ratio": float(result.overexposed_ratio),
        "laplacian_variance": float(result.laplacian_variance),
        "contrast_std": float(result.contrast_std),
        "warnings": list(result.warnings),
        "min_laplacian_variance_unchanged": float(config.quality.min_laplacian_variance),
    }


def _draw_text(image: np.ndarray, text: str, line: int, colour: tuple[int, int, int] = (0, 255, 255)) -> None:
    origin = (18, 36 + line * 34)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.78, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.78, colour, 2, cv2.LINE_AA)


def _save_labeled(path: Path, image: np.ndarray, label: str, colour: tuple[int, int, int] = (0, 255, 255)) -> None:
    output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    _draw_text(output, label, 0, colour)
    if not cv2.imwrite(str(path), output):
        raise OSError(f"Failed to write {path}")


def _draw_edges(processed_gray: np.ndarray, track: BoundaryTrack) -> np.ndarray:
    output = cv2.cvtColor(processed_gray, cv2.COLOR_GRAY2BGR)
    for y, left, right in track.all_edge_candidates:
        for edge in left:
            cv2.circle(output, (edge.x, y), 2, (255, 120, 0), -1)
        for edge in right:
            cv2.circle(output, (edge.x, y), 2, (0, 120, 255), -1)
    _draw_text(output, "candidate signed edges | processed coordinates", 0)
    _draw_text(output, f"track points={len(track.points)}", 1)
    return output


def _band_mask(shape: tuple[int, int], points: list[BandPair]) -> np.ndarray:
    mask = np.zeros(shape, dtype=np.uint8)
    if len(points) < 2:
        return mask
    ordered = sorted(points, key=lambda point: point.y)
    polygon = np.asarray([(point.left, point.y) for point in ordered] +
                         [(point.right, point.y) for point in reversed(ordered)], dtype=np.int32)
    cv2.fillPoly(mask, [polygon], 255)
    return mask


def _overlay(
    original: np.ndarray, view: ImageView, track: BoundaryTrack, fit: PolynomialFit | None,
    center_points: np.ndarray, result_label: str,
) -> np.ndarray:
    output = original.copy()
    original_center_x = original.shape[1] * 0.5
    cv2.line(output, (round(original_center_x), 0), (round(original_center_x), original.shape[0] - 1), (255, 255, 255), 2)
    ordered = sorted(track.points, key=lambda point: point.y)
    left_points = np.asarray([_restore(view, point.left, point.y) for point in ordered], dtype=np.float32)
    right_points = np.asarray([_restore(view, point.right, point.y) for point in ordered], dtype=np.float32)
    center_original = np.asarray([_restore(view, point.center, point.y) for point in ordered], dtype=np.float32)
    if len(left_points) >= 2:
        cv2.polylines(output, [np.round(left_points).astype(np.int32).reshape(-1, 1, 2)], False, (255, 0, 0), 8, cv2.LINE_AA)
        cv2.polylines(output, [np.round(right_points).astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 0), 8, cv2.LINE_AA)
        if fit is not None:
            fit_y = np.linspace(float(ordered[0].y), float(ordered[-1].y), 180)
            fit_x = np.polyval(fit.coefficients, fit_y)
            fit_original = np.asarray([_restore(view, x, y) for x, y in zip(fit_x, fit_y)], dtype=np.float32)
            cv2.polylines(output, [np.round(fit_original).astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 255), 8, cv2.LINE_AA)
            # Midpoint samples remain visible as thin yellow evidence under
            # the fitted centerline, without treating them as a second result.
            cv2.polylines(output, [np.round(center_original).astype(np.int32).reshape(-1, 1, 2)], False, (80, 180, 180), 2, cv2.LINE_AA)
        else:
            cv2.polylines(output, [np.round(center_original).astype(np.int32).reshape(-1, 1, 2)], False, (0, 255, 255), 8, cv2.LINE_AA)
    _draw_text(output, result_label, 0, (0, 255, 255))
    _draw_text(output, "left=blue right=green center=yellow reference_center=white", 1, (255, 255, 255))
    _draw_text(output, f"processed={view.image.shape[1]}x{view.image.shape[0]} restored_to_original={original.shape[1]}x{original.shape[0]}", 2, (255, 255, 255))
    return output


def _fit_center(track: BoundaryTrack) -> tuple[np.ndarray, PolynomialFit | None]:
    if not track.points:
        return np.empty((0, 2), dtype=np.float64), None
    independent = np.asarray([point.y for point in track.points], dtype=np.float64)
    center = np.asarray([point.center for point in track.points], dtype=np.float64)
    points = np.column_stack((independent, center))
    fit = fit_polynomial_ransac(independent, center, degree=2, iterations=120, residual_threshold=8.0)
    return points, fit


def run_experiment(input_path: Path = DEFAULT_INPUT, config_path: Path = DEFAULT_CONFIG,
                   output_dir: Path = DEFAULT_OUTPUT, max_dimension: int = 896) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    input_hash_before = sha256(input_path)
    started = time.perf_counter()
    image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise RuntimeError(f"Unable to decode image: {input_path}")
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError(f"Expected BGR 3-channel input, got shape={image.shape} dtype={image.dtype}")
    config = load_config(config_path)
    view = make_image_view(image, config.pipeline.roi, max_dimension)
    selected = select_gray_channel(view.image, config.seam.color_channel)
    # Use a lightly blurred original gray envelope for the experiment. The
    # production CLAHE response is useful for finding a single sharp edge,
    # but it amplifies this sample's internal weld reflections and can hide
    # the outer two-boundary structure.
    track_input = cv2.GaussianBlur(selected, (5, 5), 0)
    horizontal_gradient = cv2.Sobel(track_input, cv2.CV_32F, 1, 0, ksize=3)
    response = cv2.GaussianBlur(np.abs(horizontal_gradient), (5, 5), 0)
    response_u8 = cv2.normalize(response, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    # Preserve the broad intensity envelope. CLAHE is retained for the
    # diagnostic response image, but it amplifies the low-frequency reflection
    # texture enough to obscure the outer band on this real sample.
    track = find_boundary_track(track_input)
    center_points, fit = _fit_center(track)
    ordered = sorted(track.points, key=lambda point: point.y)
    valid_left = [(float(point.left), float(point.y)) for point in ordered]
    valid_right = [(float(point.right), float(point.y)) for point in ordered]
    valid_center = [(float(point.center), float(point.y)) for point in ordered]
    center_between = all(point.left < point.center < point.right for point in ordered)
    coverage = ((ordered[-1].y - ordered[0].y + 1) / view.image.shape[0]) if ordered else 0.0
    geometry_candidate_valid = bool(fit is not None and len(ordered) >= 3 and coverage >= 0.60 and center_between)
    quality = _quality_summary(image, config)
    restored_left = [_restore(view, x, y) for x, y in valid_left]
    restored_right = [_restore(view, x, y) for x, y in valid_right]
    restored_center = [_restore(view, x, y) for x, y in valid_center]
    coordinates_valid = all(_finite_point(point, image.shape[1], image.shape[0]) for point in restored_left + restored_right + restored_center)
    if fit is not None:
        residual_original = float(fit.residual_rms / view.scale_x)
        inlier_ratio = float(np.mean(fit.inliers))
        reference_y = config.seam.reference_y_ratio * (view.image.shape[0] - 1)
        target_x = config.seam.reference_x_ratio * view.image.shape[1]
        lateral_processed, heading = seam_errors(fit.coefficients, "vertical", reference_y, target_x)
        lateral = float(lateral_processed / view.scale_x)
        heading = float(heading)
    else:
        residual_original = None
        inlier_ratio = 0.0
        lateral = None
        heading = None
    if not ordered:
        failure_reason = track.failure_reason or "no_valid_boundary_pair"
    elif coverage < 0.60:
        failure_reason = "vertical_boundary_coverage_below_0.60"
    elif fit is None:
        failure_reason = "centerline_ransac_fit_failed"
    elif not center_between or not coordinates_valid:
        failure_reason = "centerline_geometry_validation_failed"
    else:
        failure_reason = None
    # Empty/partial results are intentionally preserved as empty/partial
    # diagnostic evidence; no missing boundary is fabricated.
    preview = image.copy()
    _draw_text(preview, f"S1-D1 input | original {image.shape[1]}x{image.shape[0]} BGR uint8", 0)
    cv2.imwrite(str(output_dir / "input_preview.jpg"), preview, [cv2.IMWRITE_JPEG_QUALITY, 95])
    _save_labeled(output_dir / "response.png", response_u8, "horizontal-gradient response | processed coordinates")
    cv2.imwrite(str(output_dir / "candidate_edges.png"), _draw_edges(track_input, track))
    _save_labeled(output_dir / "weld_band_mask.png", _band_mask(view.image.shape[:2], track.points), "paired boundary band | processed coordinates")
    boundaries = _overlay(image, view, track, fit, center_points, "S1-D1 boundaries | experiment only")
    cv2.imwrite(str(output_dir / "boundaries_overlay.jpg"), boundaries, [cv2.IMWRITE_JPEG_QUALITY, 95])
    centerline = _overlay(image, view, track, fit, center_points, "S1-D1 centerline | experiment only")
    cv2.imwrite(str(output_dir / "centerline_overlay.jpg"), centerline, [cv2.IMWRITE_JPEG_QUALITY, 95])
    processing_ms = (time.perf_counter() - started) * 1000.0
    input_hash_after = sha256(input_path)
    widths_original = np.asarray([point.width / view.scale_x for point in ordered], dtype=np.float64)
    result: dict[str, Any] = {
        "input_path": str(input_path),
        "input_sha256": input_hash_before,
        "input_sha256_after": input_hash_after,
        "input_unchanged": input_hash_before == input_hash_after,
        "original_size": {"width": int(image.shape[1]), "height": int(image.shape[0])},
        "input_dtype": str(image.dtype),
        "input_channels": int(image.shape[2]),
        "processing_size": {"width": int(view.image.shape[1]), "height": int(view.image.shape[0])},
        "scale_x": float(view.scale_x),
        "scale_y": float(view.scale_y),
        "valid_left_boundary_point_count": len(valid_left),
        "valid_right_boundary_point_count": len(valid_right),
        "valid_center_point_count": len(valid_center),
        "vertical_coverage_ratio": float(coverage),
        "weld_width_px_median": float(np.median(widths_original)) if widths_original.size else None,
        "weld_width_px_p05": float(np.percentile(widths_original, 5)) if widths_original.size else None,
        "weld_width_px_p95": float(np.percentile(widths_original, 95)) if widths_original.size else None,
        "centerline_fit_residual_px": residual_original,
        "centerline_inlier_ratio": inlier_ratio,
        "centerline_inlier_point_count": int(np.count_nonzero(fit.inliers)) if fit is not None else 0,
        "lateral_error_px": lateral,
        "heading_error_deg": heading,
        "processing_time_ms": processing_ms,
        "geometry_candidate_valid": geometry_candidate_valid,
        "image_quality_valid": quality["valid_for_measurement"],
        "production_valid": False,
        "experimental_only": True,
        "failure_reason": failure_reason,
        "centerline_between_boundaries": center_between,
        "restored_coordinates_valid": coordinates_valid,
        "quality": quality,
        "algorithm": {
            "method": "horizontal scan-band signed gradient pairs with DP continuity, midpoint center, existing polynomial RANSAC",
            "max_dimension": max_dimension,
            "scan_step_processed_px": 8,
            "scan_band_half_height_processed_px": 5,
            "uses_fixed_width_as_result": False,
            "uses_image_center_as_result": False,
            "uses_brightest_pixel_as_result": False,
        },
        "limitations": [
            "single-image offline experiment; no claim of generalization",
            "quality.min_laplacian_variance and seam.min_confidence were not changed",
            "local specular highlights can create competing signed edges",
            "weld band is accepted only when both boundaries form a continuous path",
            "production_valid is always false and this result is not connected to ROS or the production pipeline",
        ],
    }
    (output_dir / "experiment_result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-dimension", type=int, default=896)
    args = parser.parse_args()
    result = run_experiment(args.input, args.config, args.output, args.max_dimension)
    print(json.dumps({key: result[key] for key in ("output_dir", "geometry_candidate_valid", "production_valid", "failure_reason") if key in result} | {
        "output_dir": str(args.output),
        "geometry_candidate_valid": result["geometry_candidate_valid"],
        "production_valid": result["production_valid"],
        "failure_reason": result["failure_reason"],
    }, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
