from __future__ import annotations

import math

from tank_vision.pipeline import VisionPipeline
from tank_vision.synthetic import seam_image, steel_background
from tank_vision.types import FrameData


def _seam(config, image):
    result = VisionPipeline(config).process(FrameData(image, 99, "camera", 0, "test"))
    return result.seam_result


def test_lateral_error_sign(config) -> None:
    left = _seam(config, seam_image(center_x=275.0))
    right = _seam(config, seam_image(center_x=365.0))
    assert left.detected and right.detected
    assert left.lateral_error_px is not None and left.lateral_error_px < 0
    assert right.lateral_error_px is not None and right.lateral_error_px > 0


def test_heading_error_matches_line_slope(config) -> None:
    seam = _seam(config, seam_image(slope=0.10))
    assert seam.detected
    assert seam.heading_error_deg is not None
    assert abs(seam.heading_error_deg - math.degrees(math.atan(0.10))) < 1.0


def test_no_seam_is_reported_lost(config) -> None:
    seam = _seam(config, steel_background(seed=91))
    assert not seam.detected
    assert seam.lost_reason


def test_roi_and_thumbnail_restore_original_coordinates(config) -> None:
    config.pipeline.roi = (180, 0, 300, 480)
    config.pipeline.thumbnail_max_dimension = 240
    seam = _seam(config, seam_image(center_x=350.0))
    assert seam.detected
    reference_x = min(seam.centerline_points, key=lambda point: abs(point[1] - 336.0))[0]
    assert abs(reference_x - 350.0) <= 3.0

