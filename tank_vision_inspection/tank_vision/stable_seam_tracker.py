"""Stable, fixed-ROI, two-boundary seam tracker.

This module is deliberately independent of ROS, camera SDKs, and the existing
classical detector. Coordinates returned by :class:`StableSeamTracker` are
always original-image pixels.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import time
from typing import Any

import cv2
import numpy as np
import yaml

from tank_vision.seam_tracking_types import StableSeamResult, TrackingState
from tank_vision.seam_width_reference import SeamWidthReference


@dataclass(frozen=True, slots=True)
class BoundaryPair:
    band_index: int
    y: float
    left: float
    right: float
    score: float
    score_parts: dict[str, float]

    @property
    def center(self) -> float:
        return 0.5 * (self.left + self.right)

    @property
    def width(self) -> float:
        return self.right - self.left


@dataclass(slots=True)
class StableSeamDiagnostics:
    roi_bgr: np.ndarray
    processed_gray: np.ndarray
    edge_response: np.ndarray
    scan_ys: np.ndarray
    all_pairs: list[list[BoundaryPair]]
    selected_pairs: list[BoundaryPair]
    candidate_search: list[dict[str, Any]] = field(default_factory=list)
    left_candidates: list[np.ndarray] = field(default_factory=list)
    right_candidates: list[np.ndarray] = field(default_factory=list)


def scale_effective_response_threshold(thresholds: np.ndarray, scale: float) -> np.ndarray:
    """Apply the offline response-threshold scale exactly once."""
    if not 0.6 <= float(scale) <= 1.0:
        raise ValueError("response_threshold_scale must be within [0.60, 1.00]")
    return np.asarray(thresholds) * float(scale)


def load_stable_config(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != "1.0":
        raise ValueError("Stable seam config must be a schema_version 1.0 mapping")
    required = {"camera", "processing", "roi", "reference", "pairing", "path", "geometry", "tracking", "output"}
    missing = required - set(raw)
    if missing:
        raise ValueError(f"Stable seam config is missing sections: {sorted(missing)}")
    weights = raw["pairing"].get("score_weights", {})
    required_weights = {
        "left_edge_strength", "right_edge_strength", "edge_balance", "contrast",
        "width_reference", "center_prior", "temporal_center", "temporal_width",
    }
    if set(weights) != required_weights or not math.isclose(
            sum(float(value) for value in weights.values()), 1.0, abs_tol=1e-6):
        raise ValueError("Pair score weights must contain every normalized component and sum to 1.0")
    guided = raw.get("guided_search")
    if guided is not None:
        if not isinstance(guided, dict):
            raise ValueError("guided_search must be a mapping")
        if float(guided.get("local_search_radius_width_scale", 0.0)) <= 0:
            raise ValueError("guided_search.local_search_radius_width_scale must be positive")
        if int(guided.get("local_top_k_per_side", 0)) < 1:
            raise ValueError("guided_search.local_top_k_per_side must be positive")
        if "reference_anchor_radius_width_scale" in guided and float(guided["reference_anchor_radius_width_scale"]) <= 0:
            raise ValueError("guided_search.reference_anchor_radius_width_scale must be positive")
        if "reference_anchor_response" in guided and guided["reference_anchor_response"] not in {
            "signed_sobel", "normalized_step",
        }:
            raise ValueError("guided_search.reference_anchor_response must be signed_sobel or normalized_step")
    return raw


def _local_peaks(values: np.ndarray, positive: bool, threshold: float, top_k: int, separation: int,
                 x0: int, x1: int) -> np.ndarray:
    signed = values if positive else -values
    kernel = np.ones((1, 2 * separation + 1), dtype=np.uint8)
    local_max = cv2.dilate(signed.reshape(1, -1), kernel).ravel()
    indices = np.flatnonzero((signed >= local_max) & (signed >= threshold))
    indices = indices[(indices >= x0) & (indices < x1)]
    if not len(indices):
        return indices
    order = indices[np.argsort(signed[indices])[::-1]]
    kept: list[int] = []
    for value in order:
        if all(abs(int(value) - other) >= separation for other in kept):
            kept.append(int(value))
            if len(kept) == top_k:
                break
    return np.asarray(kept, dtype=np.int32)


def _local_peaks_batch(values: np.ndarray, positive: bool, thresholds: np.ndarray, top_k: int,
                       separation: int, x0: int | np.ndarray, x1: int | np.ndarray,
                       trace: list[dict[str, Any]] | None = None, trace_name: str = "") -> list[np.ndarray]:
    """Find row-wise extrema using one morphology pass and no image-pixel loop."""
    signed = values if positive else -values
    kernel = np.ones((1, 2 * separation + 1), dtype=np.uint8)
    local_max = cv2.dilate(signed, kernel)
    eligible = (signed >= local_max) & (signed >= thresholds[:, None])
    row_count, width = signed.shape
    x0_array = np.broadcast_to(np.asarray(x0, dtype=np.int32), (row_count,))
    x1_array = np.broadcast_to(np.asarray(x1, dtype=np.int32), (row_count,))
    columns = np.arange(width, dtype=np.int32)[None, :]
    eligible &= (columns >= x0_array[:, None]) & (columns < x1_array[:, None])
    # Keep extra extrema before resolving equal-valued plateaus so the final
    # separation pass can still return Top-K distinct candidates.
    provisional_count = min(signed.shape[1], 3 * top_k)
    masked = np.where(eligible, signed, -np.inf)
    provisional = np.argpartition(masked, -provisional_count, axis=1)[:, -provisional_count:]
    provisional_scores = np.take_along_axis(masked, provisional, axis=1)
    order = np.argsort(provisional_scores, axis=1)[:, ::-1]
    provisional = np.take_along_axis(provisional, order, axis=1)
    output: list[np.ndarray] = []
    for row_index, indices in enumerate(provisional):
        if trace is None:
            kept: list[int] = []
            for value in indices:
                if not np.isfinite(masked[row_index, value]):
                    break
                candidate = int(value)
                if all(abs(candidate - other) >= separation for other in kept):
                    kept.append(candidate)
                    if len(kept) == top_k:
                        break
        else:
            separated: list[int] = []
            for value in indices:
                if not np.isfinite(masked[row_index, value]):
                    break
                candidate = int(value)
                if all(abs(candidate - other) >= separation for other in separated):
                    separated.append(candidate)
            kept = separated[:top_k]
            x0_row = int(np.broadcast_to(np.asarray(x0, dtype=np.int32), (row_count,))[row_index])
            x1_row = int(np.broadcast_to(np.asarray(x1, dtype=np.int32), (row_count,))[row_index])
            window_columns = np.arange(max(0, x0_row), min(width, x1_row), dtype=np.int32)
            local_max_columns = window_columns[signed[row_index, window_columns] >= local_max[row_index, window_columns]]
            threshold_rejected = [int(index) for index in local_max_columns if signed[row_index, index] < thresholds[row_index]]
            separation_suppressed = [int(index) for index in indices if np.isfinite(masked[row_index, index]) and int(index) not in separated]
            trace.append({
                "stage": trace_name,
                "row_index": int(row_index),
                "positive_edge": bool(positive),
                "threshold": float(thresholds[row_index]),
                "x0": int(np.broadcast_to(np.asarray(x0, dtype=np.int32), (row_count,))[row_index]),
                "x1": int(np.broadcast_to(np.asarray(x1, dtype=np.int32), (row_count,))[row_index]),
                "threshold_rejected_local_maxima": [{"x": index, "signed_response": float(signed[row_index, index]), "reason": "response_below_threshold"} for index in threshold_rejected],
                "not_local_extremum_count_in_window": int(len(window_columns) - len(local_max_columns)),
                "all_local_signed_edge_peaks": [
                    {"x": int(index), "signed_response": float(signed[row_index, index])}
                    for index in np.flatnonzero(eligible[row_index])
                ],
                "provisional_peaks_considered": [
                    {"x": int(index), "signed_response": float(signed[row_index, index])}
                    for index in indices if np.isfinite(masked[row_index, index])
                ],
                "separation_filtered_peaks": [
                    {"x": int(index), "signed_response": float(signed[row_index, index])}
                    for index in separated
                ],
                "separation_suppressed": [{"x": index, "signed_response": float(signed[row_index, index]), "reason": "edge_separation_suppressed"} for index in separation_suppressed],
                "top_k": int(top_k),
                "top_k_peaks": [
                    {"x": int(index), "signed_response": float(signed[row_index, index])}
                    for index in kept
                ],
                "dropped_after_top_k": [
                    {"x": int(index), "signed_response": float(signed[row_index, index]),
                     "reason": "side_top_k_truncated"}
                    for index in separated[top_k:]
                ],
            })
        output.append(np.asarray(kept, dtype=np.int32))
    return output


def _find_reference_anchor(response_profile: np.ndarray, x0: int, x1: int, positive: bool,
                           minimum_response: float, margin: int) -> dict[str, Any]:
    """Find one directional response maximum in an already bounded window."""
    width = int(response_profile.shape[0])
    lo, hi = max(0, int(x0)), min(width, int(x1))
    result: dict[str, Any] = {"window_processed_px": [lo, hi], "position_processed": None,
                              "signed_response": None, "strength": None, "valid": False,
                              "reason": "empty_window" if hi <= lo else None}
    if hi <= lo:
        return result
    signed = response_profile if positive else -response_profile
    position = lo + int(np.argmax(signed[lo:hi]))
    strength = float(signed[position])
    result.update({"position_processed": position, "signed_response": float(response_profile[position]),
                   "strength": strength})
    if not np.isfinite(strength):
        result["reason"] = "non_finite_response"
    elif not (margin <= position < width - margin):
        result["reason"] = "outside_roi_edge_margin"
    elif strength < float(minimum_response):
        result["reason"] = "below_edge_min_response"
    else:
        result["valid"] = True
        result["reason"] = None
    return result


def _mad(values: np.ndarray) -> float:
    """Deterministic median absolute deviation for a one-dimensional profile."""
    if values.size == 0:
        return 0.0
    median = float(np.median(values))
    return float(np.median(np.abs(values - median)))


def _find_normalized_step_anchor(profile: np.ndarray, x0: int, x1: int, positive: bool,
                                 expected_width_processed: float, margin: int) -> dict[str, Any]:
    """Find a polarity-aware, locally MAD-normalized step in a bounded window.

    This is deliberately separate from the production signed-Sobel helper.  It
    has no response threshold: only a positive step, complete support windows,
    and the existing ROI margin are required.  The caller decides whether this
    experiment is enabled and may fall back to the signed-Sobel helper.
    """
    width = int(profile.shape[0])
    lo, hi = max(0, int(x0)), min(width, int(x1))
    support_width = int(np.clip(round(0.08 * float(expected_width_processed)), 5, 21))
    result: dict[str, Any] = {
        "window_processed_px": [lo, hi], "support_width_processed": support_width,
        "position_processed": None, "signed_response": None, "strength": None,
        "step": None, "local_noise": None, "score": None, "valid": False,
        "reason": "empty_window" if hi <= lo else None,
        "invalid_window_count": 0, "non_positive_step_count": 0,
    }
    if hi <= lo:
        return result
    valid_lo = max(lo, int(margin), support_width)
    valid_hi = min(hi, width - int(margin), width - support_width - 1)
    result["invalid_window_count"] = max(0, (hi - lo) - max(0, valid_hi - valid_lo))
    if valid_hi <= valid_lo:
        result["reason"] = "invalid_window"
        return result
    positions = np.arange(valid_lo, valid_hi, dtype=np.int32)
    windows = np.lib.stride_tricks.sliding_window_view(profile, support_width)
    starts_left = positions - support_width
    starts_right = positions + 1
    left_windows = windows[starts_left]
    right_windows = windows[starts_right]
    left_medians = np.median(left_windows, axis=1)
    right_medians = np.median(right_windows, axis=1)
    left_mad = np.median(np.abs(left_windows - left_medians[:, None]), axis=1)
    right_mad = np.median(np.abs(right_windows - right_medians[:, None]), axis=1)
    if positive:
        steps = right_medians - left_medians
    else:
        steps = left_medians - right_medians
    noises = 1.4826 * 0.5 * (left_mad + right_mad)
    scores = steps / (noises + 1.0)
    valid = (steps > 0.0) & np.isfinite(scores)
    result["non_positive_step_count"] = int(np.count_nonzero(~valid))
    if not np.any(valid):
        result["reason"] = "non_positive_step"
        return result
    valid_indices = np.flatnonzero(valid)
    best_index = int(valid_indices[np.argmax(scores[valid])])
    score = float(scores[best_index])
    position = int(positions[best_index])
    step = float(steps[best_index])
    noise = float(noises[best_index])
    result.update({"position_processed": int(position), "strength": float(score),
                   "score": float(score), "step": float(step), "local_noise": float(noise),
                   "valid": True, "reason": None})
    return result


def _find_normalized_step_anchor_batch(profiles: np.ndarray, x0: np.ndarray, x1: np.ndarray,
                                       positive: bool, expected_widths: np.ndarray,
                                       margin: int) -> list[dict[str, Any]]:
    """Vectorized equivalent of ``_find_normalized_step_anchor`` for scan bands."""
    band_count, width = profiles.shape
    output: list[dict[str, Any] | None] = [None] * band_count
    for support_width in sorted(set(int(np.clip(round(0.08 * float(value)), 5, 21))
                                   for value in expected_widths)):
        band_indices = np.asarray([
            index for index, value in enumerate(expected_widths)
            if int(np.clip(round(0.08 * float(value)), 5, 21)) == support_width
        ], dtype=np.int32)
        band_profiles = profiles[band_indices]
        row_x0 = x0[band_indices].astype(np.int32)
        row_x1 = x1[band_indices].astype(np.int32)
        max_span = int(np.max(np.maximum(row_x1 - row_x0, 0)))
        offsets = np.arange(max_span, dtype=np.int32)[None, :]
        positions = row_x0[:, None] + offsets
        # Each row has a contiguous bounded search interval.  Materialize only
        # the small support neighborhood around that interval; this preserves
        # the exact median/MAD definition without scanning the full 896-pixel
        # profile for every band.
        region = np.zeros((len(band_indices), max_span + 2 * support_width + 1), dtype=band_profiles.dtype)
        for row in range(len(band_indices)):
            span = max(0, int(row_x1[row] - row_x0[row]))
            start = max(0, int(row_x0[row] - support_width))
            end = min(width, int(row_x1[row] + support_width))
            region[row, :end - start] = band_profiles[row, start:end]
        windows = np.lib.stride_tricks.sliding_window_view(region, support_width, axis=1)
        left_windows = windows[:, :max_span, :]
        right_windows = windows[:, support_width + 1:support_width + 1 + max_span, :]
        left_medians = np.median(left_windows, axis=2)
        right_medians = np.median(right_windows, axis=2)
        left_mads = np.median(np.abs(left_windows - left_medians[:, :, None]), axis=2)
        right_mads = np.median(np.abs(right_windows - right_medians[:, :, None]), axis=2)
        steps = right_medians - left_medians if positive else left_medians - right_medians
        noises = 1.4826 * 0.5 * (left_mads + right_mads)
        scores = steps / (noises + 1.0)
        valid_lo = np.maximum(np.maximum(row_x0, int(margin)), support_width)
        valid_hi = np.minimum(np.minimum(row_x1, width - int(margin)), width - support_width - 1)
        in_window = ((positions >= valid_lo[:, None]) &
                     (positions < valid_hi[:, None]) &
                     (positions < row_x1[:, None]))
        valid = in_window & (steps > 0.0) & np.isfinite(scores)
        for row, band_index in enumerate(band_indices):
            lo, hi = int(x0[band_index]), int(x1[band_index])
            item: dict[str, Any] = {
                "window_processed_px": [lo, hi], "support_width_processed": support_width,
                "position_processed": None, "signed_response": None, "strength": None,
                "step": None, "local_noise": None, "score": None, "valid": False,
                "reason": "empty_window" if hi <= lo else None,
                "invalid_window_count": max(0, (hi - lo) - max(0, int(valid_hi[row] - valid_lo[row]))),
                "non_positive_step_count": int(np.count_nonzero(in_window[row] & ~((steps[row] > 0.0) & np.isfinite(scores[row])))),
            }
            if valid_hi[row] <= valid_lo[row]:
                item["reason"] = "invalid_window"
            elif not np.any(valid[row]):
                item["reason"] = "non_positive_step"
            else:
                candidates = np.flatnonzero(valid[row])
                best = int(candidates[np.argmax(scores[row, candidates])])
                position = int(positions[row, best])
                item.update({"position_processed": position, "strength": float(scores[row, best]),
                             "score": float(scores[row, best]), "step": float(steps[row, best]),
                             "local_noise": float(noises[row, best]), "valid": True, "reason": None})
            output[int(band_index)] = item
    return [item if item is not None else {"valid": False, "reason": "empty_window"} for item in output]


def _merge_reference_anchor(existing: np.ndarray, anchor: dict[str, Any], response_profile: np.ndarray,
                            positive: bool, separation: int, maximum: int = 3) -> tuple[np.ndarray, dict[str, Any]]:
    """Merge one anchor while retaining at most ``maximum`` side candidates."""
    values = [int(x) for x in existing]
    anchor["merged_with_existing"] = False
    anchor["replaced_existing"] = False
    anchor["retained_in_final_candidates"] = False
    if not anchor.get("valid") or anchor.get("position_processed") is None:
        anchor["final_candidates_processed"] = values
        return np.asarray(values, dtype=np.int32), anchor
    position = int(anchor["position_processed"])
    signed = response_profile if positive else -response_profile
    matching = [index for index, value in enumerate(values) if abs(value - position) < int(separation)]
    if matching:
        index = matching[0]
        anchor["merged_with_existing"] = True
        if float(signed[position]) > float(signed[values[index]]):
            values[index] = position
            anchor["replaced_existing"] = True
        anchor["retained_in_final_candidates"] = True
    else:
        # An anchor occupies one slot; only the two strongest original peaks
        # survive alongside it.  This keeps the existing per-side limit.
        original = sorted(values, key=lambda value: float(signed[value]), reverse=True)[:max(0, maximum - 1)]
        values = [position] + original
        values = sorted(set(values), key=lambda value: float(signed[value]), reverse=True)[:maximum]
        anchor["retained_in_final_candidates"] = position in values
    anchor["final_candidates_processed"] = values
    return np.asarray(values, dtype=np.int32), anchor


def _longest_missing_run(path: list[BoundaryPair]) -> int:
    if len(path) < 2:
        return 0
    return max((b.band_index - a.band_index - 1 for a, b in zip(path, path[1:])), default=0)


def robust_polynomial_fit(independent: np.ndarray, dependent: np.ndarray, degree: int,
                          iterations: int, huber_delta: float) -> tuple[np.ndarray, np.ndarray, float] | None:
    """Small deterministic Huber IRLS fit used instead of costly random trials."""
    if len(independent) < degree + 1 or len(independent) != len(dependent):
        return None
    weights = np.ones(len(independent), dtype=np.float64)
    coefficients = None
    for _ in range(iterations):
        try:
            coefficients = np.polyfit(independent, dependent, degree, w=np.sqrt(weights))
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            return None
        residuals = dependent - np.polyval(coefficients, independent)
        absolute = np.abs(residuals)
        weights = np.where(absolute <= huber_delta, 1.0, huber_delta / np.maximum(absolute, 1e-12))
    if coefficients is None:
        return None
    residuals = dependent - np.polyval(coefficients, independent)
    inliers = np.abs(residuals) <= huber_delta
    if np.count_nonzero(inliers) < degree + 1:
        return None
    coefficients = np.polyfit(independent[inliers], dependent[inliers], degree)
    rms = float(np.sqrt(np.mean(np.square(dependent[inliers] - np.polyval(coefficients, independent[inliers])))))
    return coefficients, inliers, rms


class StableSeamTracker:
    """Fixed-ROI detector with spatial DP and explicit temporal state."""

    def __init__(self, config: str | Path | dict[str, Any], reference: str | Path | SeamWidthReference | None = None) -> None:
        self.config_path = Path(config).resolve() if not isinstance(config, dict) else None
        self.config = load_stable_config(config) if not isinstance(config, dict) else config
        reference_value = reference or self.config["reference"]["path"]
        if isinstance(reference_value, SeamWidthReference):
            self.reference = reference_value
        else:
            ref_path = Path(reference_value)
            if not ref_path.is_absolute():
                root = self.config_path.parent.parent if self.config_path is not None else Path.cwd()
                ref_path = root / ref_path
            self.reference = SeamWidthReference.load(ref_path)
        self.state = TrackingState.LOST
        self.success_count = 0
        self.failure_count = 0
        self._center_coefficients: np.ndarray | None = None
        self._width_coefficients: np.ndarray | None = None
        self._last_lateral: float | None = None
        self._last_heading: float | None = None
        self._last_confidence = 0.0
        self._last_model_y_range: tuple[float, float] | None = None
        self._last_candidate_search: list[dict[str, Any]] = []
        self._last_left_candidates: list[np.ndarray] = []
        self._last_right_candidates: list[np.ndarray] = []
        self.last_diagnostics: StableSeamDiagnostics | None = None
        self.last_provenance: dict[str, Any] | None = None

    def reset(self) -> None:
        self.state = TrackingState.LOST
        self.success_count = self.failure_count = 0
        self._center_coefficients = self._width_coefficients = None
        self._last_lateral = self._last_heading = None
        self._last_confidence = 0.0
        self._last_model_y_range = None
        self._last_candidate_search = []
        self._last_left_candidates = self._last_right_candidates = []
        self.last_provenance = None

    def snapshot_temporal_state(self) -> dict[str, Any]:
        """Capture only mutable tracking state for a non-mutating trace pass."""
        return {
            "state": self.state, "success_count": self.success_count, "failure_count": self.failure_count,
            "center_coefficients": None if self._center_coefficients is None else self._center_coefficients.copy(),
            "width_coefficients": None if self._width_coefficients is None else self._width_coefficients.copy(),
            "last_lateral": self._last_lateral, "last_heading": self._last_heading,
            "last_confidence": self._last_confidence, "last_model_y_range": self._last_model_y_range,
        }

    def restore_temporal_state(self, snapshot: dict[str, Any]) -> None:
        """Restore state after an optional provenance-only replay pass."""
        self.state = snapshot["state"]
        self.success_count = int(snapshot["success_count"])
        self.failure_count = int(snapshot["failure_count"])
        self._center_coefficients = None if snapshot["center_coefficients"] is None else snapshot["center_coefficients"].copy()
        self._width_coefficients = None if snapshot["width_coefficients"] is None else snapshot["width_coefficients"].copy()
        self._last_lateral = snapshot["last_lateral"]
        self._last_heading = snapshot["last_heading"]
        self._last_confidence = snapshot["last_confidence"]
        self._last_model_y_range = snapshot["last_model_y_range"]

    def _roi(self, width: int, height: int) -> tuple[int, int, int, int]:
        roi = self.config["roi"]
        x0 = int(round(float(roi["x_min_ratio"]) * width))
        x1 = int(round(float(roi["x_max_ratio"]) * width))
        y0 = int(round(float(roi["y_min_ratio"]) * height))
        y1 = int(round(float(roi["y_max_ratio"]) * height))
        if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
            raise ValueError("Configured ROI is outside the input image")
        return x0, y0, x1 - x0, y1 - y0

    def _prepare(self, image: np.ndarray) -> tuple[np.ndarray, np.ndarray, tuple[int, int, int, int], float, float]:
        if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim not in (2, 3):
            raise ValueError("Input must be a non-empty mono or color NumPy image")
        height, width = image.shape[:2]
        self.reference.validate_image(width, height, bool(self.config["camera"]["same_fov_scaled"]))
        roi = self._roi(width, height)
        x, y, rw, rh = roi
        cropped = image[y:y + rh, x:x + rw]
        if cropped.ndim == 2:
            gray = cropped.astype(np.uint8, copy=False)
            bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        elif cropped.shape[2] == 3:
            bgr = cropped
            gray = cv2.cvtColor(cropped, cv2.COLOR_BGR2GRAY)
        elif cropped.shape[2] == 4:
            bgr = cv2.cvtColor(cropped, cv2.COLOR_BGRA2BGR)
            gray = cv2.cvtColor(cropped, cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError("Color input must have 3 or 4 channels")
        max_width = int(self.config["processing"]["max_roi_width"])
        scale = min(1.0, max_width / rw)
        size = (max(1, round(rw * scale)), max(1, round(rh * scale)))
        processed = cv2.resize(gray, size, interpolation=cv2.INTER_AREA) if size != (rw, rh) else gray.copy()
        kernel = int(self.config["processing"]["gaussian_kernel"])
        processed = cv2.GaussianBlur(processed, (kernel, kernel), 0)
        return bgr, processed, roi, processed.shape[1] / rw, processed.shape[0] / rh

    def _generate_candidates(self, gray: np.ndarray, roi: tuple[int, int, int, int], sx: float, sy: float,
                             image_width: int, image_height: int, keep_score_parts: bool
                             , provenance: dict[str, Any] | None = None,
                             response_threshold_scale: float = 1.0
                             ) -> tuple[np.ndarray, list[list[BoundaryPair]], np.ndarray, dict[str, float]]:
        started = time.perf_counter_ns()
        pcfg, qcfg = self.config["processing"], self.config["pairing"]
        response = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=int(pcfg["sobel_kernel"]))
        half = int(pcfg["scan_band_half_height_px"])
        step = int(pcfg["scan_band_step_px"])
        ys = np.arange(half, gray.shape[0] - half, step, dtype=np.int32)
        # Box/Gaussian filtering all scan bands in C++ avoids repeated Python
        # slices while preserving the configured band average and x smoothing.
        response_profiles = cv2.blur(response, (1, 2 * half + 1))
        response_profiles = cv2.GaussianBlur(response_profiles, (int(pcfg["profile_gaussian_kernel"]), 1), 0)[ys]
        intensity_profiles = cv2.blur(gray.astype(np.float32), (1, 2 * half + 1))[ys]
        step_profiles = None
        integrals = np.pad(np.cumsum(intensity_profiles, axis=1, dtype=np.float64), ((0, 0), (1, 0)))
        thresholds = np.maximum(float(pcfg["edge_min_response"]),
                                np.percentile(np.abs(response_profiles), float(pcfg["edge_percentile"]), axis=1))
        # Experiment-only ablation hook.  The production threshold is first
        # computed exactly as before; only the threshold used by candidate
        # filtering is scaled.  Anchor, NMS, Top-K, pairing, DP, fitting and
        # all downstream gates remain untouched.  At the default scale of
        # 1.0 this multiplication is exactly identity for finite float32/64
        # values and preserves the existing call behavior.
        thresholds = scale_effective_response_threshold(thresholds, response_threshold_scale)
        margin = max(int(pcfg["edge_margin_min_px"]),
                     int(round(gray.shape[1] * float(pcfg["edge_margin_ratio"]))))
        separation = int(pcfg["edge_min_separation_px"])
        y_originals = roi[1] + ys.astype(np.float64) / sy
        ref_centers_o, ref_widths_o = self.reference.at(y_originals, image_width, image_height)
        ref_centers = (ref_centers_o - roi[0]) * sx
        ref_widths = ref_widths_o * sx
        global_top_k = int(pcfg["top_k_edges_per_side"])
        global_left_trace: list[dict[str, Any]] = []
        global_right_trace: list[dict[str, Any]] = []
        if provenance is None:
            global_left = _local_peaks_batch(response_profiles, True, thresholds, global_top_k, separation, margin, gray.shape[1] - margin)
            global_right = _local_peaks_batch(response_profiles, False, thresholds, global_top_k, separation, margin, gray.shape[1] - margin)
        else:
            global_left = _local_peaks_batch(response_profiles, True, thresholds, global_top_k, separation, margin, gray.shape[1] - margin, global_left_trace, "global_left")
            global_right = _local_peaks_batch(response_profiles, False, thresholds, global_top_k, separation, margin, gray.shape[1] - margin, global_right_trace, "global_right")
        candidate_search: list[dict[str, Any]] = []
        local_left_trace: list[dict[str, Any]] = []
        local_right_trace: list[dict[str, Any]] = []
        guided = self.config.get("guided_search", {})
        use_guided = bool(guided.get("enabled", False))
        if use_guided:
            # LOST/ACQUIRING intentionally use only the frozen reference.  Once
            # a fit exists, TRACKING/DEGRADED use the previous accepted model.
            expected_centers_o = ref_centers_o.copy()
            expected_widths_o = ref_widths_o.copy()
            source = "manual_reference_model"
            if (self.state in (TrackingState.TRACKING, TrackingState.DEGRADED)
                    and self._center_coefficients is not None and self._width_coefficients is not None):
                expected_centers_o = np.polyval(self._center_coefficients, y_originals)
                expected_widths_o = np.polyval(self._width_coefficients, y_originals)
                source = "previous_accepted_fit"
            radius = np.maximum(
                float(guided["local_search_radius_width_scale"]) * expected_widths_o * sx, 1.0)
            expected_centers = (expected_centers_o - roi[0]) * sx
            expected_widths = expected_widths_o * sx
            expected_left = expected_centers - 0.5 * expected_widths
            expected_right = expected_centers + 0.5 * expected_widths
            local_top_k = min(global_top_k, int(guided["local_top_k_per_side"]))
            local_left_x0 = np.clip(np.floor(expected_left - radius).astype(np.int32), margin, gray.shape[1] - margin)
            local_left_x1 = np.clip(np.ceil(expected_left + radius).astype(np.int32) + 1, margin + 1, gray.shape[1] - margin)
            local_right_x0 = np.clip(np.floor(expected_right - radius).astype(np.int32), margin, gray.shape[1] - margin)
            local_right_x1 = np.clip(np.ceil(expected_right + radius).astype(np.int32) + 1, margin + 1, gray.shape[1] - margin)
            if provenance is None:
                local_left = _local_peaks_batch(response_profiles, True, thresholds, local_top_k, separation, local_left_x0, local_left_x1)
                local_right = _local_peaks_batch(response_profiles, False, thresholds, local_top_k, separation, local_right_x0, local_right_x1)
            else:
                local_left = _local_peaks_batch(response_profiles, True, thresholds, local_top_k, separation, local_left_x0, local_left_x1, local_left_trace, "guided_left")
                local_right = _local_peaks_batch(response_profiles, False, thresholds, local_top_k, separation, local_right_x0, local_right_x1, local_right_trace, "guided_right")
            fallback = bool(guided.get("fallback_to_global_search", True))
            left_candidates = []
            right_candidates = []
            for index in range(len(ys)):
                left_candidates.append(
                    local_left[index] if len(local_left[index]) or not fallback
                    else global_left[index][:local_top_k])
                right_candidates.append(
                    local_right[index] if len(local_right[index]) or not fallback
                    else global_right[index][:local_top_k])
                candidate_search.append({
                    "band_index": index,
                    "source": source,
                    "left_mode": "local" if len(local_left[index]) else ("global_fallback" if fallback else "empty"),
                    "right_mode": "local" if len(local_right[index]) else ("global_fallback" if fallback else "empty"),
                    "left_window_processed_px": [int(local_left_x0[index]), int(local_left_x1[index])],
                    "right_window_processed_px": [int(local_right_x0[index]), int(local_right_x1[index])],
                    "expected_center_original_px": float(expected_centers_o[index]),
                    "expected_width_original_px": float(expected_widths_o[index]),
                    "local_radius_original_px": float(radius[index] / max(sx, 1e-12)),
                })
            anchor_enabled = bool(guided.get("reference_anchor_enabled", False))
            if anchor_enabled:
                anchor_scale = float(guided.get("reference_anchor_radius_width_scale", 0.12))
                anchor_response_mode = str(guided.get("reference_anchor_response", "signed_sobel"))
                anchor_trace: list[dict[str, Any]] = []
                anchor_radius = np.maximum(anchor_scale * ref_widths_o * sx, 1.0)
                anchor_left = (ref_centers_o - 0.5 * ref_widths_o - roi[0]) * sx
                anchor_right = (ref_centers_o + 0.5 * ref_widths_o - roi[0]) * sx
                step_left_anchors = step_right_anchors = None
                if anchor_response_mode == "normalized_step":
                    step_profiles = cv2.GaussianBlur(
                        cv2.blur(gray.astype(np.float32), (1, 2 * half + 1)),
                        (int(pcfg["profile_gaussian_kernel"]), 1), 0)[ys]
                    step_left_anchors = _find_normalized_step_anchor_batch(
                        step_profiles, np.asarray([int(np.clip(np.floor(value - radius[index]), margin, gray.shape[1] - margin))
                                                  for index, value in enumerate(anchor_left)], dtype=np.int32),
                        np.asarray([int(np.clip(np.ceil(value + radius[index]) + 1, margin + 1, gray.shape[1] - margin))
                                    for index, value in enumerate(anchor_left)], dtype=np.int32),
                        True, ref_widths, margin)
                    step_right_anchors = _find_normalized_step_anchor_batch(
                        step_profiles, np.asarray([int(np.clip(np.floor(value - radius[index]), margin, gray.shape[1] - margin))
                                                  for index, value in enumerate(anchor_right)], dtype=np.int32),
                        np.asarray([int(np.clip(np.ceil(value + radius[index]) + 1, margin + 1, gray.shape[1] - margin))
                                    for index, value in enumerate(anchor_right)], dtype=np.int32),
                        False, ref_widths, margin)
                for index in range(len(ys)):
                    left_x0 = int(np.clip(np.floor(anchor_left[index] - anchor_radius[index]), margin, gray.shape[1] - margin))
                    left_x1 = int(np.clip(np.ceil(anchor_left[index] + anchor_radius[index]) + 1, margin + 1, gray.shape[1] - margin))
                    right_x0 = int(np.clip(np.floor(anchor_right[index] - anchor_radius[index]), margin, gray.shape[1] - margin))
                    right_x1 = int(np.clip(np.ceil(anchor_right[index] + anchor_radius[index]) + 1, margin + 1, gray.shape[1] - margin))
                    if anchor_response_mode == "normalized_step":
                        left_anchor = step_left_anchors[index]
                        right_anchor = step_right_anchors[index]
                        for side, item in (("left", left_anchor), ("right", right_anchor)):
                            item["signed_response"] = None if item["position_processed"] is None else float(
                                response_profiles[index, int(item["position_processed"])])
                            item["step_anchor_used"] = bool(item.get("valid"))
                            item["fallback_to_sobel"] = False
                            item["invalid_window"] = item.get("reason") == "invalid_window"
                            item["non_positive_step"] = item.get("reason") == "non_positive_step"
                            if not item.get("valid"):
                                sobel_anchor = _find_reference_anchor(
                                    response_profiles[index], left_x0 if side == "left" else right_x0,
                                    left_x1 if side == "left" else right_x1, side == "left",
                                    float(pcfg["edge_min_response"]), margin)
                                item["fallback_to_sobel"] = True
                                item["fallback_sobel"] = sobel_anchor
                                item["fallback_sobel_valid"] = bool(sobel_anchor.get("valid"))
                                item["position_processed"] = sobel_anchor.get("position_processed")
                                item["signed_response"] = sobel_anchor.get("signed_response")
                                item["strength"] = sobel_anchor.get("strength")
                                item["valid"] = bool(sobel_anchor.get("valid"))
                                item["fallback_reason"] = item.get("reason")
                                item["reason"] = None if item["valid"] else sobel_anchor.get("reason")
                    else:
                        left_anchor = _find_reference_anchor(response_profiles[index], left_x0, left_x1, True,
                                                             float(pcfg["edge_min_response"]), margin)
                        right_anchor = _find_reference_anchor(response_profiles[index], right_x0, right_x1, False,
                                                              float(pcfg["edge_min_response"]), margin)
                        for item in (left_anchor, right_anchor):
                            item["step_anchor_used"] = False
                            item["fallback_to_sobel"] = False
                    left_candidates[index], left_anchor = _merge_reference_anchor(
                        left_candidates[index], left_anchor, response_profiles[index], True, separation, 3)
                    right_candidates[index], right_anchor = _merge_reference_anchor(
                        right_candidates[index], right_anchor, response_profiles[index], False, separation, 3)
                    candidate_search[index]["reference_anchor"] = {
                        "enabled": True, "response_mode": anchor_response_mode,
                        "radius_width_scale": anchor_scale,
                        "predicted_left_processed": float(anchor_left[index]),
                        "predicted_right_processed": float(anchor_right[index]),
                        "radius_processed": float(anchor_radius[index]),
                        "left": left_anchor, "right": right_anchor,
                        "final_left_candidates_processed": [int(x) for x in left_candidates[index]],
                        "final_right_candidates_processed": [int(x) for x in right_candidates[index]],
                    }
        else:
            left_candidates, right_candidates = global_left, global_right
            candidate_search = [{
                "band_index": index, "source": "global", "left_mode": "global", "right_mode": "global"
            } for index in range(len(ys))]
        candidate_ms = (time.perf_counter_ns() - started) / 1e6
        pairing_started = time.perf_counter_ns()
        temporal_centers = temporal_widths = None
        if self._center_coefficients is not None:
            temporal_centers = (np.polyval(self._center_coefficients, y_originals) - roi[0]) * sx
            temporal_widths = np.polyval(self._width_coefficients, y_originals) * sx
        band_count = len(ys)
        all_pairs: list[list[BoundaryPair]] = []
        left_matrix = np.zeros((band_count, global_top_k), dtype=np.int32)
        right_matrix = np.zeros((band_count, global_top_k), dtype=np.int32)
        left_present = np.zeros((band_count, global_top_k), dtype=bool)
        right_present = np.zeros((band_count, global_top_k), dtype=bool)
        for band_index, (lefts, rights) in enumerate(zip(left_candidates, right_candidates)):
            left_matrix[band_index, :len(lefts)] = lefts
            right_matrix[band_index, :len(rights)] = rights
            left_present[band_index, :len(lefts)] = True
            right_present[band_index, :len(rights)] = True

        left_values = left_matrix[:, :, None]
        right_values = right_matrix[:, None, :]
        widths = (right_values - left_values).astype(np.float64)
        ref_width_grid = ref_widths[:, None, None]
        valid = (left_present[:, :, None] & right_present[:, None, :] & (widths > 0)
                 & (widths >= float(qcfg["width_min_scale"]) * ref_width_grid)
                 & (widths <= float(qcfg["width_max_scale"]) * ref_width_grid))
        centers = 0.5 * (left_values + right_values)
        if temporal_centers is not None and self.state in (TrackingState.TRACKING, TrackingState.DEGRADED):
            valid &= (np.abs(centers - temporal_centers[:, None, None])
                      <= float(self.config["tracking"]["tracking_search_width_scale"]) * ref_width_grid)

        row_indices = np.arange(band_count)[:, None]
        edge_l = response_profiles[row_indices, left_matrix][:, :, None]
        edge_r = -response_profiles[row_indices, right_matrix][:, None, :]
        threshold_grid = np.maximum(thresholds[:, None, None], 1e-6)
        left_edge_norm = np.clip(edge_l / threshold_grid, 0.0, 1.0) + np.zeros_like(widths)
        right_edge_norm = np.clip(edge_r / threshold_grid, 0.0, 1.0) + np.zeros_like(widths)
        balance = np.clip(
            np.minimum(edge_l, edge_r) / np.maximum(np.maximum(edge_l, edge_r), 1e-6), 0.0, 1.0)

        outside_widths = np.maximum(
            int(qcfg["exterior_sample_min_px"]),
            np.rint(widths * float(qcfg["exterior_sample_width_scale"])).astype(np.int32))
        inner_margin = int(qcfg["interior_margin_px"])
        ia = np.clip(left_values + inner_margin, 0, gray.shape[1]) + np.zeros_like(right_values)
        ib = np.clip(right_values - inner_margin, 0, gray.shape[1]) + np.zeros_like(left_values)
        la = np.clip(left_values - outside_widths, 0, gray.shape[1])
        lb = left_values + np.zeros_like(right_values)
        ra = right_values + np.zeros_like(left_values)
        rb = np.clip(right_values + outside_widths, 0, gray.shape[1])
        valid &= (ib > ia) & (lb > la) & (rb > ra)
        band_grid = np.arange(band_count)[:, None, None]
        inside_mean = (integrals[band_grid, ib] - integrals[band_grid, ia]) / np.maximum(ib - ia, 1)
        outside_mean = 0.5 * (
            (integrals[band_grid, lb] - integrals[band_grid, la]) / np.maximum(lb - la, 1)
            + (integrals[band_grid, rb] - integrals[band_grid, ra]) / np.maximum(rb - ra, 1))
        contrast = np.clip(
            float(qcfg["contrast_score_offset"])
            + (inside_mean - outside_mean) / float(qcfg["contrast_normalization_gray"]), 0.0, 1.0)
        # Continuous consistency score: exp(-|log(w / w_ref)|).
        # It is symmetric for reciprocal width ratios and has no hard switch
        # inside the separately configured legal gate.
        width_scores = np.exp(-np.abs(np.log(np.maximum(widths, 1e-6) / ref_width_grid)))
        reference_weight = float(qcfg["initial_reference_center_weight"])
        prior_centers = (reference_weight * ref_centers
                         + (1.0 - reference_weight) * (gray.shape[1] * 0.5))[:, None, None]
        center_scores = np.exp(
            -np.abs(centers - prior_centers)
            / np.maximum(float(qcfg["center_prior_decay_width_scale"]) * ref_width_grid, 1.0))
        neutral = float(qcfg["unavailable_temporal_score"])
        temporal_center_scores = np.full_like(widths, neutral) if temporal_centers is None else np.exp(
            -np.abs(centers - temporal_centers[:, None, None])
            / np.maximum(float(qcfg["temporal_center_decay_width_scale"]) * ref_width_grid, 1.0))
        temporal_width_scores = np.full_like(widths, neutral) if temporal_widths is None else np.exp(
            -np.abs(np.log(np.maximum(widths, 1e-6) / np.maximum(temporal_widths[:, None, None], 1e-6))))
        names = ("left_edge_strength", "right_edge_strength", "edge_balance", "contrast",
                 "width_reference", "center_prior", "temporal_center", "temporal_width")
        components = (left_edge_norm, right_edge_norm, balance, contrast, width_scores, center_scores,
                      temporal_center_scores, temporal_width_scores)
        weights = qcfg["score_weights"]
        scores = sum(float(weights[name]) * component for name, component in zip(names, components))
        scores[~valid] = -np.inf
        flat_scores = scores.reshape(band_count, -1)
        max_pairs = int(qcfg["max_pairs_per_band"])
        pair_trace_rows: list[dict[str, Any]] = []
        if provenance is not None:
            minimum_width = float(qcfg["width_min_scale"])
            maximum_width = float(qcfg["width_max_scale"])
            tracking_active = temporal_centers is not None and self.state in (TrackingState.TRACKING, TrackingState.DEGRADED)
            for band_index, (lefts, rights) in enumerate(zip(left_candidates, right_candidates)):
                cartesian: list[dict[str, Any]] = []
                for left_index, left_value in enumerate(lefts):
                    for right_index, right_value in enumerate(rights):
                        left_value, right_value = int(left_value), int(right_value)
                        width_value = float(right_value - left_value)
                        reason = None
                        if right_value <= left_value:
                            reason = "invalid_left_right_order"
                        elif width_value < minimum_width * float(ref_widths[band_index]):
                            reason = "width_below_minimum"
                        elif width_value > maximum_width * float(ref_widths[band_index]):
                            reason = "width_above_maximum"
                        elif tracking_active and abs(float(centers[band_index, left_index, right_index]) - float(temporal_centers[band_index])) > float(self.config["tracking"]["tracking_search_width_scale"]) * float(ref_widths[band_index]):
                            reason = "tracking_search_rejected"
                        elif not bool(ib[band_index, left_index, right_index] > ia[band_index, left_index, right_index]):
                            reason = "invalid_interior_sample"
                        elif not bool(lb[band_index, left_index, right_index] > la[band_index, left_index, right_index]):
                            reason = "invalid_left_exterior_sample"
                        elif not bool(rb[band_index, left_index, right_index] > ra[band_index, left_index, right_index]):
                            reason = "invalid_right_exterior_sample"
                        flat_index = left_index * global_top_k + right_index
                        cartesian.append({"pair_id": f"b{band_index}:l{left_value}:r{right_value}",
                                          "left_processed": left_value, "right_processed": right_value,
                                          "left_candidate_index": int(left_index), "right_candidate_index": int(right_index),
                                          "width_processed": width_value, "hard_gate_passed": reason is None,
                                          "hard_gate_rejection_reason": reason,
                                          "score": (float(flat_scores[band_index, flat_index]) if math.isfinite(float(flat_scores[band_index, flat_index])) else None)})
                valid_indices = np.flatnonzero(np.isfinite(flat_scores[band_index]))
                pre_order = valid_indices[np.argsort(flat_scores[band_index, valid_indices])[::-1]] if len(valid_indices) else np.empty(0, dtype=np.int64)
                post_order = pre_order[:max_pairs]
                pre_by_flat = {int(item["left_candidate_index"] * global_top_k + item["right_candidate_index"]): item for item in cartesian if item["hard_gate_passed"]}
                for rank, flat_index in enumerate(pre_order):
                    left_index, right_index = divmod(int(flat_index), global_top_k)
                    item = pre_by_flat[int(flat_index)]
                    item["pre_truncation_rank"] = int(rank + 1)
                    item["pair_score_parts"] = {name: float(component[band_index, left_index, right_index]) for name, component in zip(names, components)}
                    item["pair_id"] = f"b{band_index}:l{int(left_matrix[band_index, left_index])}:r{int(right_matrix[band_index, right_index])}"
                    item["retained_after_pair_top_k"] = bool(rank < max_pairs)
                    if rank >= max_pairs:
                        item["removal_reason"] = "pair_top_k_truncated"
                pair_trace_rows.append({"band_index": int(band_index), "cartesian_pairs": cartesian,
                                        "pair_hard_gate_before": [item for item in cartesian if not item["hard_gate_passed"]],
                                        "pair_hard_gate_after": [pre_by_flat[int(flat_index)] for flat_index in pre_order],
                                        "pair_after_max_pairs_per_band": [pre_by_flat[int(flat_index)] for flat_index in post_order],
                                        "max_pairs_per_band": max_pairs,
                                        "pre_truncation_count": int(len(pre_order)), "post_truncation_count": int(len(post_order))})
        for band_index, yp in enumerate(ys):
            valid_indices = np.flatnonzero(np.isfinite(flat_scores[band_index]))
            if len(valid_indices):
                order = valid_indices[np.argsort(flat_scores[band_index, valid_indices])[::-1][:max_pairs]]
            else:
                order = np.empty(0, dtype=np.int64)
            pairs: list[BoundaryPair] = []
            for flat_index in order:
                left_index, right_index = divmod(int(flat_index), global_top_k)
                parts = ({name: float(component[band_index, left_index, right_index])
                          for name, component in zip(names, components)} if keep_score_parts else {})
                pairs.append(BoundaryPair(
                    band_index, float(yp), float(left_matrix[band_index, left_index]),
                    float(right_matrix[band_index, right_index]), float(flat_scores[band_index, flat_index]), parts))
            all_pairs.append(pairs)
        if provenance is not None:
            candidate_stage_rows = []
            global_left_by_band = {item["row_index"]: item for item in global_left_trace}
            global_right_by_band = {item["row_index"]: item for item in global_right_trace}
            local_left_by_band = {item["row_index"]: item for item in local_left_trace}
            local_right_by_band = {item["row_index"]: item for item in local_right_trace}
            for band_index in range(band_count):
                search = candidate_search[band_index]
                gl = global_left_by_band.get(band_index, {})
                gr = global_right_by_band.get(band_index, {})
                ll = local_left_by_band.get(band_index, {})
                lr = local_right_by_band.get(band_index, {})
                def values(item: dict[str, Any], key: str) -> list[dict[str, Any]]:
                    return list(item.get(key, []))
                left_window = search.get("left_window_processed_px")
                right_window = search.get("right_window_processed_px")
                outside_left = []
                outside_right = []
                if left_window is not None:
                    outside_left = [item for item in values(gl, "all_local_signed_edge_peaks") if not left_window[0] <= item["x"] < left_window[1]]
                if right_window is not None:
                    outside_right = [item for item in values(gr, "all_local_signed_edge_peaks") if not right_window[0] <= item["x"] < right_window[1]]
                left_source = "guided_local" if search.get("left_mode") == "local" else ("global_fallback" if search.get("left_mode") == "global_fallback" else "empty")
                right_source = "guided_local" if search.get("right_mode") == "local" else ("global_fallback" if search.get("right_mode") == "global_fallback" else "empty")
                candidate_stage_rows.append({"band_index": int(band_index), "y_processed": int(ys[band_index]),
                    "all_local_signed_edge_peaks": {"left": values(gl, "all_local_signed_edge_peaks"), "right": values(gr, "all_local_signed_edge_peaks")},
                    "guided_window_peaks": {"left": values(ll, "all_local_signed_edge_peaks"), "right": values(lr, "all_local_signed_edge_peaks")},
                    "outside_guided_window": {"left": outside_left, "right": outside_right},
                    "separation_filtered_peaks": {"left": values(ll if use_guided else gl, "separation_filtered_peaks"), "right": values(lr if use_guided else gr, "separation_filtered_peaks")},
                    "top_k_before": {"left": values(ll if use_guided else gl, "separation_filtered_peaks"), "right": values(lr if use_guided else gr, "separation_filtered_peaks")},
                    "top_k_after": {"left": [{"x": int(x), "signed_response": float(response_profiles[band_index, int(x)])} for x in left_candidates[band_index]], "right": [{"x": int(x), "signed_response": float(-response_profiles[band_index, int(x)])} for x in right_candidates[band_index]]},
                    "global_fallback": {"left": [{"x": int(x), "signed_response": float(response_profiles[band_index, int(x)])} for x in global_left[band_index]] if left_source == "global_fallback" else [], "right": [{"x": int(x), "signed_response": float(-response_profiles[band_index, int(x)])} for x in global_right[band_index]] if right_source == "global_fallback" else []},
                    "selected_side_source": {"left": left_source, "right": right_source},
                    **({"reference_anchor": search["reference_anchor"]} if "reference_anchor" in search else {}),
                    "removed_peak_reasons": {"outside_guided_window": {"left": outside_left, "right": outside_right}, "side_top_k_truncated": {"left": list((ll if use_guided else gl).get("dropped_after_top_k", [])), "right": list((lr if use_guided else gr).get("dropped_after_top_k", []))}},
                    "pair_stages": pair_trace_rows[band_index]})
            provenance["candidate_stage_trace"] = candidate_stage_rows
        self._last_candidate_search = candidate_search
        self._last_left_candidates = [values.copy() for values in left_candidates]
        self._last_right_candidates = [values.copy() for values in right_candidates]
        return response, all_pairs, ys, {
            "candidate_generation": candidate_ms,
            "pairing": (time.perf_counter_ns() - pairing_started) / 1e6,
        }

    def _transition_detail(self, previous: BoundaryPair, current: BoundaryPair, missing: int,
                          ref_width: float) -> dict[str, Any]:
        cfg = self.config["path"]
        cd = abs(current.center - previous.center) / ref_width
        ld = abs(current.left - previous.left) / ref_width
        rd = abs(current.right - previous.right) / ref_width
        wr = abs(math.log(current.width / previous.width))
        step_scale = missing + 1
        reasons = []
        if cd > float(cfg["max_center_jump_width_scale"]) * step_scale:
            reasons.append("center_jump_above_max")
        if max(ld, rd) > float(cfg["max_boundary_jump_width_scale"]) * step_scale:
            reasons.append("boundary_jump_above_max")
        if wr > float(cfg["max_width_ratio_change"]) * step_scale:
            reasons.append("width_ratio_change_above_max")
        center_cost = float(cfg["center_transition_weight"]) * cd
        width_cost = float(cfg["width_transition_weight"]) * wr
        left_cost = float(cfg["left_transition_weight"]) * ld
        right_cost = float(cfg["right_transition_weight"]) * rd
        skip_cost = float(cfg["skip_penalty"]) * missing
        return {"passed": not reasons, "reasons": reasons, "missing": int(missing),
                "center_normalized": float(cd), "width_normalized": float(wr),
                "left_normalized": float(ld), "right_normalized": float(rd),
                "center_cost": center_cost, "width_cost": width_cost,
                "left_cost": left_cost, "right_cost": right_cost,
                "skip_cost": skip_cost,
                "total": center_cost + left_cost + right_cost + width_cost + skip_cost}

    def _transition(self, previous: BoundaryPair, current: BoundaryPair, missing: int, ref_width: float) -> float | None:
        detail = self._transition_detail(previous, current, missing, ref_width)
        return float(detail["total"]) if detail["passed"] else None

    @staticmethod
    def _pair_id(pair: BoundaryPair) -> str:
        return f"b{pair.band_index}:l{int(round(pair.left))}:r{int(round(pair.right))}"

    def _select_path(self, all_pairs: list[list[BoundaryPair]], roi: tuple[int, int, int, int], sx: float, sy: float,
                     image_width: int, image_height: int, provenance: dict[str, Any] | None = None) -> list[BoundaryPair]:
        if provenance is not None:
            provenance["_dp_states"] = []
        scores: list[np.ndarray] = [np.full(len(items), -np.inf) for items in all_pairs]
        predecessors: list[list[tuple[int, int] | None]] = [[None] * len(items) for items in all_pairs]
        max_gap = int(self.config["path"]["maximum_missing_run"])
        for band, items in enumerate(all_pairs):
            if not items:
                continue
            current_center = np.asarray([item.center for item in items])[:, None]
            current_left = np.asarray([item.left for item in items])[:, None]
            current_right = np.asarray([item.right for item in items])[:, None]
            current_width = np.asarray([item.width for item in items])[:, None]
            current_local = np.asarray([item.score for item in items])
            scores[band][:] = current_local
            y_original = roi[1] + items[0].y / sy
            _, ref_width_o = self.reference.at(y_original, image_width, image_height)
            ref_width = max(float(ref_width_o) * sx, 1.0)
            cfg = self.config["path"]
            previous_bands = [pb for pb in range(band - 1, max(-1, band - max_gap - 2), -1) if all_pairs[pb]]
            state_rows = []
            if provenance is not None:
                state_rows = [{"pair_id": self._pair_id(item), "band_index": int(band),
                               "left": float(item.left), "right": float(item.right),
                               "center": float(item.center), "width": float(item.width),
                               "unary_score": float(item.score), "cumulative_score": float(item.score),
                               "predecessor_pair_id": None, "predecessor_band": None,
                               "best_predecessor": None, "second_predecessor": None,
                               "cumulative_score_margin_best_second": None,
                               "cumulative_score_margin_best_unary": None,
                               "predecessor_candidates": []} for item in items]
            # Consider every predecessor inside the configured gap.  This is
            # the explicit skip state: an isolated reflection may have pair
            # candidates yet still be bypassed at the configured skip cost.
            if previous_bands:
                previous_items = [item for pb in previous_bands for item in all_pairs[pb]]
                previous_keys = [(pb, index) for pb in previous_bands for index in range(len(all_pairs[pb]))]
                previous_center = np.asarray([item.center for item in previous_items])[None, :]
                previous_left = np.asarray([item.left for item in previous_items])[None, :]
                previous_right = np.asarray([item.right for item in previous_items])[None, :]
                previous_width = np.asarray([item.width for item in previous_items])[None, :]
                previous_scores = np.concatenate([scores[pb] for pb in previous_bands])
                missing = np.asarray([band - pb - 1 for pb, _ in previous_keys], dtype=np.float64)[None, :]
                step_scale = missing + 1.0
                cd = np.abs(current_center - previous_center) / ref_width
                ld = np.abs(current_left - previous_left) / ref_width
                rd = np.abs(current_right - previous_right) / ref_width
                wr = np.abs(np.log(current_width / previous_width))
                valid = ((cd <= float(cfg["max_center_jump_width_scale"]) * step_scale)
                         & (np.maximum(ld, rd) <= float(cfg["max_boundary_jump_width_scale"]) * step_scale)
                         & (wr <= float(cfg["max_width_ratio_change"]) * step_scale))
                penalty = (float(cfg["center_transition_weight"]) * cd + float(cfg["left_transition_weight"]) * ld
                           + float(cfg["right_transition_weight"]) * rd + float(cfg["width_transition_weight"]) * wr
                           + float(cfg["skip_penalty"]) * missing)
                totals = previous_scores[None, :] + current_local[:, None] - penalty
                totals[~valid] = -np.inf
                if provenance is not None:
                    for ci, current in enumerate(items):
                        candidates = []
                        for predecessor_index, (pb, pi) in enumerate(previous_keys):
                            previous = all_pairs[pb][pi]
                            missing_value = band - pb - 1
                            detail = self._transition_detail(previous, current, missing_value, ref_width)
                            candidate_score = float(scores[pb][pi] + current.score - detail["total"]) if detail["passed"] else None
                            candidates.append({"pair_id": self._pair_id(previous), "predecessor_band": int(pb),
                                               "predecessor_index": int(pi), "previous_cumulative_score": float(scores[pb][pi]),
                                               "transition": detail, "candidate_cumulative_score": candidate_score,
                                               "transition_passed": bool(detail["passed"]), "is_best_predecessor": False})
                        feasible = sorted((item for item in candidates if item["transition_passed"]),
                                          key=lambda item: item["candidate_cumulative_score"], reverse=True)
                        if feasible:
                            feasible[0]["is_best_predecessor"] = True
                            state_rows[ci]["best_predecessor"] = feasible[0]
                            if len(feasible) > 1:
                                state_rows[ci]["second_predecessor"] = feasible[1]
                                state_rows[ci]["cumulative_score_margin_best_second"] = float(feasible[0]["candidate_cumulative_score"] - feasible[1]["candidate_cumulative_score"])
                            state_rows[ci]["cumulative_score_margin_best_unary"] = float(feasible[0]["candidate_cumulative_score"] - current.score)
                        state_rows[ci]["predecessor_candidates"] = candidates
                best_pi = np.argmax(totals, axis=1)
                best_values = totals[np.arange(len(items)), best_pi]
                improved = best_values > scores[band]
                scores[band][improved] = best_values[improved]
                for ci in np.flatnonzero(improved):
                    predecessors[band][int(ci)] = previous_keys[int(best_pi[ci])]
            if provenance is not None:
                for ci, state in enumerate(state_rows):
                    state["cumulative_score"] = float(scores[band][ci])
                    predecessor = predecessors[band][ci]
                    if predecessor is not None:
                        pb, pi = predecessor
                        state["predecessor_pair_id"] = self._pair_id(all_pairs[pb][pi])
                        state["predecessor_band"] = int(pb)
                    for candidate in state["predecessor_candidates"]:
                        candidate["is_selected_predecessor"] = candidate["pair_id"] == state["predecessor_pair_id"] and candidate["predecessor_band"] == state["predecessor_band"]
                provenance["_dp_states"].extend(state_rows)
        endpoints = [(float(value), band, index) for band, row in enumerate(scores) for index, value in enumerate(row)]
        if not endpoints:
            if provenance is not None:
                provenance["dp_state_trace"] = []
                provenance["dp_final_path"] = []
            return []
        _, band, index = max(endpoints, key=lambda item: (item[0], item[1], -item[2]))
        path: list[BoundaryPair] = []
        while True:
            path.append(all_pairs[band][index])
            predecessor = predecessors[band][index]
            if predecessor is None:
                break
            band, index = predecessor
        path = list(reversed(path))
        if provenance is not None:
            path_keys = {(item.band_index, item.left, item.right) for item in path}
            states = provenance.get("_dp_states", [])
            for state in states:
                state["on_final_path"] = (state["band_index"], state["left"], state["right"]) in path_keys
            provenance["dp_state_trace"] = states
            provenance["dp_final_path"] = [self._pair_id(item) for item in path]
            provenance.pop("_dp_states", None)
        return path

    def _fit(self, path: list[BoundaryPair], roi: tuple[int, int, int, int], sx: float, sy: float,
             image_width: int, image_height: int) -> dict[str, Any] | None:
        cfg = self.config["geometry"]
        if len(path) < int(cfg["minimum_point_count"]):
            return None
        ys_o = np.asarray([roi[1] + p.y / sy for p in path])
        centers_o = np.asarray([roi[0] + p.center / sx for p in path])
        widths_o = np.asarray([p.width / sx for p in path])
        # The public result and this threshold are both original-image pixels.
        threshold = float(cfg["robust_fit_huber_delta_px"])
        fit = robust_polynomial_fit(ys_o, centers_o, int(cfg["fit_degree"]),
                                    int(cfg["robust_fit_iterations"]), threshold)
        if fit is None:
            return None
        center_coefficients, inliers, residual_rms = fit
        width_fit = robust_polynomial_fit(
            ys_o, widths_o, int(cfg["width_fit_degree"]), int(cfg["robust_fit_iterations"]),
            float(cfg["width_fit_huber_delta_scale"]) * threshold)
        if width_fit is None:
            return None
        width_coefficients, width_inliers, _ = width_fit
        joint_inliers = inliers & width_inliers
        if np.count_nonzero(joint_inliers) < max(int(cfg["fit_degree"]), int(cfg["width_fit_degree"])) + 1:
            return None
        center_coefficients = np.polyfit(ys_o[joint_inliers], centers_o[joint_inliers], int(cfg["fit_degree"]))
        width_coefficients = np.polyfit(ys_o[joint_inliers], widths_o[joint_inliers], int(cfg["width_fit_degree"]))
        center_residuals = centers_o - np.polyval(center_coefficients, ys_o)
        half_width_residuals = 0.5 * (widths_o - np.polyval(width_coefficients, ys_o))
        joint_residuals = np.sqrt(np.square(center_residuals) + np.square(half_width_residuals))
        residual_rms = float(np.sqrt(np.mean(np.square(joint_residuals[joint_inliers]))))
        return {"center": center_coefficients, "width": width_coefficients, "inliers": joint_inliers,
                "center_inliers": inliers, "width_inliers": width_inliers,
                "center_residuals": center_residuals, "width_residuals": widths_o - np.polyval(width_coefficients, ys_o),
                "joint_residuals": joint_residuals,
                "center_inlier_ratio": float(np.mean(inliers)),
                "width_inlier_ratio": float(np.mean(width_inliers)),
                "residual_px": residual_rms, "ys": ys_o, "centers": centers_o, "widths": widths_o}

    def _geometry(self, path: list[BoundaryPair], fit: dict[str, Any] | None, band_count: int,
                  roi: tuple[int, int, int, int], sx: float, sy: float, image_width: int, image_height: int) -> tuple[bool, dict[str, Any], list[str]]:
        cfg = self.config["geometry"]
        coverage = len(path) / max(band_count, 1)
        span = 0.0 if not path else (path[-1].band_index - path[0].band_index + 1) / max(band_count, 1)
        missing = _longest_missing_run(path)
        widths = np.asarray([p.width / sx for p in path], dtype=np.float64)
        jumps_l = jumps_r = 0
        if len(path) > 1:
            for a, b in zip(path, path[1:]):
                y_o = roi[1] + b.y / sy
                _, rw = self.reference.at(y_o, image_width, image_height)
                threshold = float(cfg["abrupt_jump_width_scale"]) * float(rw)
                jumps_l += abs(b.left - a.left) / sx > threshold
                jumps_r += abs(b.right - a.right) / sx > threshold
        inlier_ratio = 0.0 if fit is None else float(np.mean(fit["inliers"]))
        residual = None if fit is None else float(fit["residual_px"])
        path_confidence = 0.0 if not path else float(np.mean([p.score for p in path]))
        model_geometry_valid = False
        if fit is not None and path:
            model_ys = np.linspace(fit["ys"][0], fit["ys"][-1], int(self.config["output"]["centerline_sample_count"]))
            model_centers = np.polyval(fit["center"], model_ys)
            model_widths = np.polyval(fit["width"], model_ys)
            model_left = model_centers - 0.5 * model_widths
            model_right = model_centers + 0.5 * model_widths
            model_geometry_valid = bool(
                np.isfinite(model_left + model_right).all() and (model_widths > 0).all()
                and (model_left >= roi[0]).all() and (model_right < roi[0] + roi[2]).all())
        checks = [
            ("insufficient_points", len(path) >= int(cfg["minimum_point_count"])),
            ("coverage_below_minimum", coverage >= float(cfg["minimum_coverage"])),
            ("effective_span_below_minimum", span >= float(cfg["minimum_effective_span"])),
            ("missing_run_too_long", missing <= int(cfg["maximum_missing_run"])),
            ("too_many_abrupt_jumps", max(jumps_l, jumps_r) <= int(cfg["maximum_abrupt_jump_count"])),
            ("fit_failed", fit is not None),
            ("inlier_ratio_below_minimum", inlier_ratio >= float(cfg["minimum_inlier_ratio"])),
            ("fit_residual_above_maximum", residual is not None and residual <= float(cfg["maximum_fit_residual_px"])),
            ("path_confidence_below_minimum", path_confidence >= float(cfg["minimum_path_confidence"])),
            ("fitted_boundaries_invalid_or_outside_roi", model_geometry_valid),
        ]
        reasons = [name for name, passed in checks if not passed]
        return not reasons, {"coverage": coverage, "span": span, "missing": missing, "jumps_l": int(jumps_l),
                            "jumps_r": int(jumps_r), "inlier_ratio": inlier_ratio, "residual": residual,
                            "path_confidence": path_confidence, "widths": widths}, reasons

    @staticmethod
    def _model_measurements(center: np.ndarray, width: np.ndarray, image_width: int, image_height: int,
                            reference_y_ratio: float) -> tuple[float, float, float]:
        y = reference_y_ratio * image_height
        center_x = float(np.polyval(center, y))
        heading = math.degrees(math.atan(float(np.polyval(np.polyder(center), y))))
        median_width = float(np.polyval(width, y))
        return center_x - image_width * 0.5, heading, median_width

    def _temporal_accept(self, center: np.ndarray, width: np.ndarray, image_width: int, image_height: int) -> list[str]:
        if self._center_coefficients is None or not self.config["tracking"]["enabled"]:
            return []
        ref_y = float(self.config["geometry"]["reference_y_ratio"])
        lateral, heading, median_width = self._model_measurements(center, width, image_width, image_height, ref_y)
        old_lateral, old_heading, old_width = self._model_measurements(
            self._center_coefficients, self._width_coefficients, image_width, image_height, ref_y)
        cfg = self.config["tracking"]
        reasons = []
        if abs(lateral - old_lateral) > float(cfg["max_lateral_jump_width_scale"]) * max(old_width, 1.0):
            reasons.append("temporal_lateral_jump")
        if abs(median_width / max(old_width, 1e-6) - 1.0) > float(cfg["max_width_change_ratio"]):
            reasons.append("temporal_width_jump")
        if abs(heading - old_heading) > float(cfg["max_heading_change_deg"]):
            reasons.append("temporal_heading_jump")
        return reasons

    def _sample_model(self, center: np.ndarray, width: np.ndarray, path: list[BoundaryPair], roi: tuple[int, int, int, int], sy: float,
                      image_width: int, image_height: int) -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[tuple[float, float]], list[float]]:
        count = int(self.config["output"]["centerline_sample_count"])
        if path:
            y0, y1 = roi[1] + path[0].y / sy, roi[1] + path[-1].y / sy
        elif self._last_model_y_range is not None:
            y0, y1 = self._last_model_y_range
        else:
            y0, y1 = roi[1], roi[1] + roi[3]
        ys = np.linspace(max(roi[1], y0), min(roi[1] + roi[3] - 1, y1), count)
        centers = np.polyval(center, ys)
        widths = np.polyval(width, ys)
        lefts, rights = centers - widths * 0.5, centers + widths * 0.5
        valid = np.isfinite(lefts + rights) & (widths > 0) & (lefts >= roi[0]) & (rights < roi[0] + roi[2])
        return ([(float(x), float(y)) for x, y in zip(lefts[valid], ys[valid])],
                [(float(x), float(y)) for x, y in zip(rights[valid], ys[valid])],
                [(float(x), float(y)) for x, y in zip(centers[valid], ys[valid])],
                [float(value) for value in widths[valid]])

    def process(self, image: np.ndarray, diagnostics: bool = False, provenance: bool = False,
                response_threshold_scale: float = 1.0) -> StableSeamResult:
        started = time.perf_counter_ns()
        timings: dict[str, float] = {}
        self.last_provenance = None
        provenance_trace: dict[str, Any] | None = {"enabled": True} if provenance else None
        temporal_input = {
            "state": self.state.name,
            "center_coefficients": None if self._center_coefficients is None else [float(value) for value in self._center_coefficients],
            "width_coefficients": None if self._width_coefficients is None else [float(value) for value in self._width_coefficients],
        }
        fit: dict[str, Any] | None = None
        metrics: dict[str, Any] = {}
        valid = False
        reasons: list[str] = []
        model_center: np.ndarray | None = None
        model_width: np.ndarray | None = None
        selected_pair_crossing_count = 0
        fitted_output_crossing_count = 0
        minimum_fitted_width_px: float | None = None
        height, width = image.shape[:2] if isinstance(image, np.ndarray) and image.ndim >= 2 else (0, 0)
        try:
            mark = time.perf_counter_ns()
            roi_bgr, gray, roi, sx, sy = self._prepare(image)
            timings["preprocess"] = (time.perf_counter_ns() - mark) / 1e6
            mark = time.perf_counter_ns()
            response, candidates, scan_ys, candidate_timings = self._generate_candidates(
                gray, roi, sx, sy, width, height, diagnostics or provenance_trace is not None, provenance_trace,
                response_threshold_scale=response_threshold_scale)
            timings.update(candidate_timings)
            mark = time.perf_counter_ns()
            path = self._select_path(candidates, roi, sx, sy, width, height, provenance_trace)
            # Lightweight evidence only: do not enable full provenance or
            # diagnostics in the realtime path.  Pair construction already
            # enforces left < right; recording the count makes that invariant
            # observable without changing selection or geometry gates.
            selected_pair_crossing_count = sum(int(pair.left >= pair.right) for pair in path)
            if provenance_trace is not None:
                provenance_trace["scales"] = {"sx": float(sx), "sy": float(sy)}
                provenance_trace["scan_bands"] = [
                    {"band_index": int(index), "y_processed": int(value),
                     "y_original": float(roi[1] + value / sy)}
                    for index, value in enumerate(scan_ys)
                ]
                provenance_trace["selected_raw_path"] = [
                    {"band_index": int(pair.band_index), "y": float(roi[1] + pair.y / sy),
                     "left": float(roi[0] + pair.left / sx), "right": float(roi[0] + pair.right / sx),
                     "center": float(roi[0] + pair.center / sx), "width": float(pair.width / sx),
                     "unary_score": float(pair.score), "unary_score_parts": dict(pair.score_parts)}
                    for pair in path
                ]
            timings["dynamic_programming"] = (time.perf_counter_ns() - mark) / 1e6
            mark = time.perf_counter_ns()
            fit = self._fit(path, roi, sx, sy, width, height)
            valid, metrics, reasons = self._geometry(path, fit, len(scan_ys), roi, sx, sy, width, height)
            timings["fitting"] = (time.perf_counter_ns() - mark) / 1e6
            if fit is not None:
                reasons.extend(self._temporal_accept(fit["center"], fit["width"], width, height))
                valid = valid and not any(reason.startswith("temporal_") for reason in reasons)
            mark = time.perf_counter_ns()
            if valid and fit is not None:
                alpha = float(self.config["tracking"]["ema_alpha"])
                if self._center_coefficients is None:
                    self._center_coefficients = fit["center"].copy()
                    self._width_coefficients = fit["width"].copy()
                else:
                    self._center_coefficients = alpha * fit["center"] + (1.0 - alpha) * self._center_coefficients
                    self._width_coefficients = alpha * fit["width"] + (1.0 - alpha) * self._width_coefficients
                self.success_count += 1
                self.failure_count = 0
                self._last_model_y_range = (float(fit["ys"][0]), float(fit["ys"][-1]))
                self.state = TrackingState.TRACKING if self.success_count >= int(self.config["tracking"]["acquiring_success_frames"]) else TrackingState.ACQUIRING
                detected, predicted = True, False
                model_center, model_width = self._center_coefficients, self._width_coefficients
            else:
                self.failure_count += 1
                self.success_count = 0
                limit = int(self.config["tracking"]["lost_frame_threshold"])
                degraded_limit = int(self.config["tracking"]["degraded_max_frames"])
                can_predict = (self._center_coefficients is not None and self.failure_count < limit
                               and self.failure_count <= degraded_limit)
                self.state = TrackingState.DEGRADED if can_predict else TrackingState.LOST
                if not can_predict:
                    self._center_coefficients = self._width_coefficients = None
                    self._last_model_y_range = None
                detected, predicted = False, can_predict
                model_center, model_width = self._center_coefficients, self._width_coefficients
            timings["temporal_update"] = (time.perf_counter_ns() - mark) / 1e6
            left: list[tuple[float, float]] = []
            right: list[tuple[float, float]] = []
            centerline: list[tuple[float, float]] = []
            width_profile: list[float] = []
            lateral = heading = median_width = None
            if model_center is not None and model_width is not None:
                sample_path = path if valid else []
                left, right, centerline, width_profile = self._sample_model(
                    model_center, model_width, sample_path, roi, sy, width, height)
                fitted_output_crossing_count = sum(int(left_point[0] >= right_point[0])
                                                   for left_point, right_point in zip(left, right))
                if width_profile:
                    minimum_fitted_width_px = float(np.min(np.asarray(width_profile, dtype=np.float64)))
                lateral, heading, median_width = self._model_measurements(model_center, model_width, width, height, float(self.config["geometry"]["reference_y_ratio"]))
                if valid:
                    self._last_lateral, self._last_heading = lateral, heading
            confidence = metrics["path_confidence"] * min(1.0, metrics["coverage"] / max(float(self.config["geometry"]["minimum_coverage"]), 1e-6))
            if valid:
                self._last_confidence = confidence
            elif predicted:
                confidence = self._last_confidence * (
                    float(self.config["tracking"]["predicted_confidence_decay"]) ** self.failure_count)
            else:
                confidence = 0.0
            timings["overlay"] = 0.0
            result = StableSeamResult(
                detected=detected, predicted_only=predicted, tracking_state=self.state, confidence=float(np.clip(confidence, 0, 1)),
                left_boundary_original_px=left, right_boundary_original_px=right, centerline_original_px=centerline,
                median_width_px=median_width, width_profile_px=width_profile, lateral_error_px=lateral,
                heading_error_deg=heading, roi_original_px=roi, coverage=metrics["coverage"], effective_span=metrics["span"],
                maximum_missing_run=metrics["missing"], abrupt_left_jump_count=metrics["jumps_l"], abrupt_right_jump_count=metrics["jumps_r"],
                selected_pair_crossing_count=int(selected_pair_crossing_count),
                fitted_output_crossing_count=int(fitted_output_crossing_count),
                minimum_fitted_width_px=minimum_fitted_width_px,
                fit_inlier_ratio=metrics["inlier_ratio"], fit_residual_px=metrics["residual"], stage_timings_ms=timings,
                failure_reasons=[] if valid else reasons, accepted_scan_bands=len(path), rejected_scan_bands=len(scan_ys) - len(path),
            )
            if diagnostics:
                self.last_diagnostics = StableSeamDiagnostics(
                    roi_bgr.copy(), gray.copy(), response.copy(), scan_ys.copy(), candidates, path,
                    [dict(item) for item in self._last_candidate_search],
                    [values.copy() for values in self._last_left_candidates],
                    [values.copy() for values in self._last_right_candidates])
        except (ValueError, cv2.error, np.linalg.LinAlgError, FloatingPointError) as exc:
            self.failure_count += 1
            self.success_count = 0
            self.state = TrackingState.LOST
            self._center_coefficients = self._width_coefficients = None
            self._last_lateral = self._last_heading = None
            self._last_confidence = 0.0
            self._last_model_y_range = None
            result = StableSeamResult(tracking_state=self.state, failure_reasons=[f"input_or_processing_error:{exc}"])
            result.processing_ms = (time.perf_counter_ns() - started) / 1e6
            result.stage_timings_ms = timings
        result.processing_ms = (time.perf_counter_ns() - started) / 1e6
        result.stage_timings_ms = timings
        if provenance_trace is not None:
            fitted_y: list[float] = []
            fitted_center: list[float] = []
            fitted_width: list[float] = []
            fitted_left: list[float] = []
            fitted_right: list[float] = []
            fitted_min_width: float | None = None
            fitted_inside_roi = False
            if fit is not None:
                fitted_y_array = np.linspace(float(fit["ys"][0]), float(fit["ys"][-1]),
                                              int(self.config["output"]["centerline_sample_count"]))
                fitted_center_array = np.polyval(fit["center"], fitted_y_array)
                fitted_width_array = np.polyval(fit["width"], fitted_y_array)
                fitted_left_array = fitted_center_array - 0.5 * fitted_width_array
                fitted_right_array = fitted_center_array + 0.5 * fitted_width_array
                fitted_y = [float(value) for value in fitted_y_array]
                fitted_center = [float(value) for value in fitted_center_array]
                fitted_width = [float(value) for value in fitted_width_array]
                fitted_left = [float(value) for value in fitted_left_array]
                fitted_right = [float(value) for value in fitted_right_array]
                if len(fitted_width_array):
                    fitted_min_width = float(np.min(fitted_width_array))
                fitted_inside_roi = bool(np.isfinite(fitted_left_array + fitted_right_array).all()
                                         and (fitted_width_array > 0).all()
                                         and (fitted_left_array >= roi[0]).all()
                                         and (fitted_right_array < roi[0] + roi[2]).all())

            def gate(name: str, value: Any, threshold: Any, passed: bool | None, direction: str = "boolean") -> dict[str, Any]:
                if passed is None:
                    margin = None
                elif direction == "lower" and value is not None:
                    margin = float(value) - float(threshold)
                elif direction == "upper" and value is not None:
                    margin = float(threshold) - float(value)
                else:
                    margin = 1.0 if passed else 0.0
                return {"value": value, "threshold": threshold, "passed": passed, "margin": margin}

            inlier_value = None if fit is None else float(fit["inliers"].mean())
            residual_value = None if fit is None else float(fit["residual_px"])
            geometry_gates = {
                "point_count": gate("point_count", len(path), int(self.config["geometry"]["minimum_point_count"]), len(path) >= int(self.config["geometry"]["minimum_point_count"]), "lower"),
                "coverage": gate("coverage", metrics.get("coverage"), float(self.config["geometry"]["minimum_coverage"]), metrics.get("coverage", 0.0) >= float(self.config["geometry"]["minimum_coverage"]), "lower"),
                "effective_span": gate("effective_span", metrics.get("span"), float(self.config["geometry"]["minimum_effective_span"]), metrics.get("span", 0.0) >= float(self.config["geometry"]["minimum_effective_span"]), "lower"),
                "maximum_missing_run": gate("maximum_missing_run", metrics.get("missing"), int(self.config["geometry"]["maximum_missing_run"]), metrics.get("missing", 0) <= int(self.config["geometry"]["maximum_missing_run"]), "upper"),
                "abrupt_left_jump_count": gate("abrupt_left_jump_count", metrics.get("jumps_l"), int(self.config["geometry"]["maximum_abrupt_jump_count"]), metrics.get("jumps_l", 0) <= int(self.config["geometry"]["maximum_abrupt_jump_count"]), "upper"),
                "abrupt_right_jump_count": gate("abrupt_right_jump_count", metrics.get("jumps_r"), int(self.config["geometry"]["maximum_abrupt_jump_count"]), metrics.get("jumps_r", 0) <= int(self.config["geometry"]["maximum_abrupt_jump_count"]), "upper"),
                "fit_success": gate("fit_success", fit is not None, True, fit is not None),
                "center_inlier_ratio": gate("center_inlier_ratio", None if fit is None else float(fit["center_inlier_ratio"]), float(self.config["geometry"]["minimum_inlier_ratio"]), None if fit is None else float(fit["center_inlier_ratio"]) >= float(self.config["geometry"]["minimum_inlier_ratio"]), "lower"),
                "width_inlier_ratio": gate("width_inlier_ratio", None if fit is None else float(fit["width_inlier_ratio"]), float(self.config["geometry"]["minimum_inlier_ratio"]), None if fit is None else float(fit["width_inlier_ratio"]) >= float(self.config["geometry"]["minimum_inlier_ratio"]), "lower"),
                "joint_inlier_ratio": gate("joint_inlier_ratio", inlier_value, float(self.config["geometry"]["minimum_inlier_ratio"]), None if inlier_value is None else inlier_value >= float(self.config["geometry"]["minimum_inlier_ratio"]), "lower"),
                "fit_residual_px": gate("fit_residual_px", residual_value, float(self.config["geometry"]["maximum_fit_residual_px"]), None if residual_value is None else residual_value <= float(self.config["geometry"]["maximum_fit_residual_px"]), "upper"),
                "path_confidence": gate("path_confidence", metrics.get("path_confidence"), float(self.config["geometry"]["minimum_path_confidence"]), metrics.get("path_confidence", 0.0) >= float(self.config["geometry"]["minimum_path_confidence"]), "lower"),
                "positive_width": gate("positive_width", fitted_min_width, 0.0, None if fitted_min_width is None else fitted_min_width > 0.0, "lower"),
                "inside_roi": gate("inside_roi", fitted_inside_roi, True, fitted_inside_roi),
                "selected_pair_crossing": gate("selected_pair_crossing", int(selected_pair_crossing_count), 0, selected_pair_crossing_count == 0, "upper"),
                "fitted_output_crossing": gate("fitted_output_crossing", int(fitted_output_crossing_count), 0, fitted_output_crossing_count == 0, "upper"),
                "minimum_fitted_width": gate("minimum_fitted_width", fitted_min_width, 0.0, None if fitted_min_width is None else fitted_min_width > 0.0, "lower"),
                "temporal": gate("temporal", [reason for reason in reasons if reason.startswith("temporal_")], "no temporal_* failure", not any(reason.startswith("temporal_") for reason in reasons)),
            }
            provenance_trace["result"] = {
                "fit_center_coefficients": None if fit is None else [float(value) for value in fit["center"]],
                "fit_width_coefficients": None if fit is None else [float(value) for value in fit["width"]],
                "center_inlier_ratio": None if fit is None else float(fit["center_inlier_ratio"]),
                "width_inlier_ratio": None if fit is None else float(fit["width_inlier_ratio"]),
                "center_raw_samples": [] if fit is None else [float(value) for value in fit["centers"]],
                "width_raw_samples": [] if fit is None else [float(value) for value in fit["widths"]],
                "raw_y_samples": [] if fit is None else [float(value) for value in fit["ys"]],
                "center_residuals": [] if fit is None else [float(value) for value in fit["center_residuals"]],
                "width_residuals": [] if fit is None else [float(value) for value in fit["width_residuals"]],
                "joint_residuals": [] if fit is None else [float(value) for value in fit["joint_residuals"]],
                "center_inlier_mask": [] if fit is None else [bool(value) for value in fit["center_inliers"]],
                "width_inlier_mask": [] if fit is None else [bool(value) for value in fit["width_inliers"]],
                "joint_inlier_mask": [] if fit is None else [bool(value) for value in fit["inliers"]],
                "fitted_y": fitted_y, "fitted_left": fitted_left, "fitted_right": fitted_right,
                "fitted_center": fitted_center, "fitted_width": fitted_width,
                "fitted_minimum_width_px": fitted_min_width,
                "geometry_gates": geometry_gates,
                "temporal_input": temporal_input,
                "temporal_output": {"predicted_only": bool(result.predicted_only), "detected": bool(result.detected),
                                     "y": [float(value[1]) for value in result.centerline_original_px],
                                     "left": [float(value[0]) for value in result.left_boundary_original_px],
                                     "right": [float(value[0]) for value in result.right_boundary_original_px],
                                     "center": [float(value[0]) for value in result.centerline_original_px],
                                     "width": [float(value) for value in result.width_profile_px],
                                     "center_coefficients": None if model_center is None else [float(value) for value in model_center],
                                     "width_coefficients": None if model_width is None else [float(value) for value in model_width]},
                "geometry_metrics": ({key: (value.tolist() if isinstance(value, np.ndarray) else value)
                                       for key, value in metrics.items()} if metrics else None),
                "geometry_valid": bool(valid) if "valid" in locals() else False,
                "geometry_failure_reasons": list(reasons) if "reasons" in locals() else list(result.failure_reasons),
                "detected": bool(result.detected), "tracking_state": result.tracking_state.name,
                "failure_reasons": list(result.failure_reasons),
            }
            self.last_provenance = provenance_trace
        return result


def draw_stable_seam_overlay(image: np.ndarray, result: StableSeamResult, max_size: tuple[int, int] | None = None) -> np.ndarray:
    output = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    x, y, w, h = result.roi_original_px
    cv2.rectangle(output, (x, y), (x + w - 1, y + h - 1), (255, 255, 255), 2)
    def line(points: list[tuple[float, float]], color: tuple[int, int, int]) -> None:
        if len(points) >= 2:
            cv2.polylines(output, [np.round(points).astype(np.int32).reshape(-1, 1, 2)], False, color, 7, cv2.LINE_AA)
    line(result.left_boundary_original_px, (255, 0, 0))
    line(result.right_boundary_original_px, (0, 255, 0))
    line(result.centerline_original_px, (0, 255, 255))
    label = f"{result.tracking_state.name} detected={result.detected} predicted={result.predicted_only} conf={result.confidence:.3f}"
    cv2.putText(output, label, (30, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(output, label, (30, 55), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2, cv2.LINE_AA)
    if max_size is not None:
        scale = min(1.0, max_size[0] / output.shape[1], max_size[1] / output.shape[0])
        if scale < 1:
            output = cv2.resize(output, (round(output.shape[1] * scale), round(output.shape[0] * scale)), interpolation=cv2.INTER_AREA)
    return output
