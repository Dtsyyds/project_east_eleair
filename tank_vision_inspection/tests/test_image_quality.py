from __future__ import annotations

import numpy as np

from tank_vision.preprocessing import assess_image_quality


def test_uniform_dark_frame_reports_exposure_and_blur(config) -> None:
    result = assess_image_quality(np.zeros((120, 160, 3), dtype=np.uint8), config.quality)
    assert result.valid_image
    assert not result.valid_for_measurement
    assert result.confidence_scale < 1.0
    assert "severe_underexposure" in result.warnings
    assert "severe_blur" in result.warnings


def test_uniform_bright_frame_reports_overexposure(config) -> None:
    result = assess_image_quality(np.full((120, 160, 3), 255, dtype=np.uint8), config.quality)
    assert result.overexposed_ratio == 1.0
    assert "severe_overexposure" in result.warnings
