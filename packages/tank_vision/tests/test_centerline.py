from __future__ import annotations

import numpy as np

from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import seam_image
from tank_vision.types import FrameData


def _process(config, image: np.ndarray):
    frame = FrameData(image, 123456789, "camera", 0, "test")
    return VisionPipeline(config).process(frame)


def _x_at_reference(result, target_y: float) -> float:
    return min(result.seam_result.centerline_points, key=lambda point: abs(point[1] - target_y))[0]


def test_clear_straight_center_error_within_two_pixels(config) -> None:
    result = _process(config, seam_image(center_x=320.0))
    assert result.seam_result.detected
    assert abs(_x_at_reference(result, 336.0) - 320.0) <= 2.0
    assert abs(result.seam_result.lateral_error_px or 0.0) <= 2.0


def test_noisy_seam_center_error_within_five_pixels(config) -> None:
    result = _process(config, seam_image(center_x=347.0, noise_sigma=18.0))
    assert result.seam_result.detected
    assert abs(_x_at_reference(result, 336.0) - 347.0) <= 5.0


def test_low_contrast_seam_is_handled(config) -> None:
    result = _process(config, seam_image(center_x=315.0, contrast=32))
    assert result.seam_result.detected
    assert abs(_x_at_reference(result, 336.0) - 315.0) <= 3.0


def test_curved_and_partially_occluded_seams_do_not_crash(config) -> None:
    curved = _process(config, seam_image(curvature=0.15))
    occluded = _process(config, seam_image(occlusion=(170, 290)))
    assert curved.seam_result.centerline_points
    assert occluded.seam_result.centerline_points
