"""Robust polynomial fitting and seam error geometry."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(slots=True)
class PolynomialFit:
    coefficients: np.ndarray
    inliers: np.ndarray
    residual_rms: float


def fit_polynomial_ransac(
    independent: np.ndarray,
    dependent: np.ndarray,
    degree: int,
    iterations: int,
    residual_threshold: float,
    random_seed: int = 0,
) -> PolynomialFit | None:
    sample_size = degree + 1
    if independent.size < sample_size or independent.size != dependent.size:
        return None
    rng = np.random.default_rng(random_seed)
    best_mask: np.ndarray | None = None
    best_score = (-1, float("inf"))
    for _ in range(max(1, iterations)):
        indices = rng.choice(independent.size, size=sample_size, replace=False)
        sample_x = independent[indices]
        if np.ptp(sample_x) < max(1.0, degree):
            continue
        try:
            coefficients = np.polyfit(sample_x, dependent[indices], degree)
        except (np.linalg.LinAlgError, ValueError, FloatingPointError):
            continue
        residuals = np.abs(dependent - np.polyval(coefficients, independent))
        mask = residuals <= residual_threshold
        count = int(np.count_nonzero(mask))
        median = float(np.median(residuals[mask])) if count else float("inf")
        score = (count, -median)
        if score > (best_score[0], -best_score[1]):
            best_mask = mask
            best_score = (count, median)
    if best_mask is None or np.count_nonzero(best_mask) < sample_size:
        return None
    try:
        coefficients = np.polyfit(independent[best_mask], dependent[best_mask], degree)
    except (np.linalg.LinAlgError, ValueError, FloatingPointError):
        return None
    final_residuals = dependent[best_mask] - np.polyval(coefficients, independent[best_mask])
    rms = float(np.sqrt(np.mean(np.square(final_residuals))))
    return PolynomialFit(coefficients=coefficients, inliers=best_mask, residual_rms=rms)


def polynomial_derivative(coefficients: np.ndarray, coordinate: float) -> float:
    return float(np.polyval(np.polyder(coefficients), coordinate))


def seam_errors(
    coefficients: np.ndarray,
    expected_direction: str,
    reference_coordinate: float,
    target_coordinate: float,
) -> tuple[float, float]:
    dependent_at_reference = float(np.polyval(coefficients, reference_coordinate))
    lateral_error = dependent_at_reference - target_coordinate
    slope = polynomial_derivative(coefficients, reference_coordinate)
    # For vertical travel the polynomial is x(y); for horizontal travel it is y(x).
    heading_error = math.degrees(math.atan(slope))
    return lateral_error, heading_error

