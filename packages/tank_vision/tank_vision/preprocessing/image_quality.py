"""Non-throwing image-quality assessment."""

from __future__ import annotations

import cv2
import numpy as np

from tank_vision.config import QualityConfig
from tank_vision.types import QualityResult


def assess_image_quality(image: np.ndarray, config: QualityConfig) -> QualityResult:
    if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim < 2:
        return QualityResult(
            valid_image=False,
            valid_for_measurement=False,
            confidence_scale=0.0,
            warnings=["empty_image"],
        )
    try:
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8, copy=False)
        mean = float(np.mean(gray))
        under_ratio = float(np.mean(gray <= config.underexposed_threshold))
        over_ratio = float(np.mean(gray >= config.overexposed_threshold))
        contrast = float(np.std(gray))
        laplacian = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        warnings: list[str] = []
        penalties: list[float] = []
        if config.enable_exposure_check and under_ratio > config.max_underexposed_ratio:
            warnings.append("severe_underexposure")
            penalties.append(0.45)
        if config.enable_exposure_check and over_ratio > config.max_overexposed_ratio:
            warnings.append("severe_overexposure")
            penalties.append(0.45)
        if contrast < config.min_contrast_std:
            warnings.append("low_contrast")
            penalties.append(0.6)
        if config.enable_blur_check and laplacian < config.min_laplacian_variance:
            warnings.append("severe_blur")
            penalties.append(0.55)
        confidence_scale = min(penalties, default=1.0)
        valid_for_measurement = not any(
            warning in {"severe_underexposure", "severe_overexposure", "severe_blur"} for warning in warnings
        )
        return QualityResult(
            mean_brightness=mean,
            overexposed_ratio=over_ratio,
            underexposed_ratio=under_ratio,
            laplacian_variance=laplacian,
            contrast_std=contrast,
            motion_blur_score=None,
            valid_image=True,
            valid_for_measurement=valid_for_measurement,
            confidence_scale=confidence_scale,
            warnings=warnings,
        )
    except (cv2.error, ValueError, TypeError) as exc:
        return QualityResult(
            valid_image=False,
            valid_for_measurement=False,
            confidence_scale=0.0,
            warnings=[f"quality_assessment_failed:{exc}"],
        )
