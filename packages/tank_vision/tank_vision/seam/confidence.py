"""Multi-signal seam confidence scoring."""

from __future__ import annotations

import math

import numpy as np


def calculate_confidence(
    independent: np.ndarray,
    dependent: np.ndarray,
    inliers: np.ndarray,
    major_length: int,
    residual_rms: float,
    max_residual: float,
    component_contrast: float,
    dependent_length: int,
    center_margin_ratio: float,
    continuity_max_gap: float,
    contrast_reference: float,
) -> tuple[float, dict[str, float]]:
    if independent.size == 0 or major_length <= 0:
        return 0.0, {}
    point_coverage = min(1.0, independent.size / major_length)
    inlier_ratio = float(np.mean(inliers)) if inliers.size else 0.0
    residual_score = math.exp(-residual_rms / max(1e-6, max_residual))
    sorted_coordinates = np.sort(independent[inliers])
    if sorted_coordinates.size > 1:
        gaps = np.diff(sorted_coordinates)
        continuity = float(np.mean(gaps <= continuity_max_gap))
    else:
        continuity = 0.0
    contrast_score = float(np.clip(component_contrast / max(contrast_reference, 1e-6), 0.0, 1.0))
    center = float(np.median(dependent[inliers])) if np.any(inliers) else float(np.median(dependent))
    normalized_offset = abs(center - dependent_length * 0.5) / max(1.0, dependent_length * 0.5)
    plausibility = float(np.clip(1.0 - normalized_offset / max(center_margin_ratio, 1e-6), 0.0, 1.0))
    factors = {
        "point_coverage": point_coverage,
        "inlier_ratio": inlier_ratio,
        "residual": residual_score,
        "continuity": continuity,
        "contrast": contrast_score,
        "roi_plausibility": plausibility,
    }
    weights = {
        "point_coverage": 0.18,
        "inlier_ratio": 0.22,
        "residual": 0.20,
        "continuity": 0.14,
        "contrast": 0.16,
        "roi_plausibility": 0.10,
    }
    score = sum(factors[name] * weights[name] for name in factors)
    return float(np.clip(score, 0.0, 1.0)), factors
