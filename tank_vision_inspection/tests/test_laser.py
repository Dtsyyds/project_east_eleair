from __future__ import annotations

import numpy as np

from tank_vision.laser import LaserCalibration, LaserLineDetector, Profile3DInterface
from tank_vision.synthetic import blue_laser_image
from tank_vision.types import LaserLineResult


def test_blue_laser_subpixel_center(config) -> None:
    target = 115.35
    result = LaserLineDetector(config.laser).detect(blue_laser_image(center_y=target), 123)
    assert result.detected
    assert len(result.points_uv) > 500
    median_y = float(np.median([point[1] for point in result.points_uv]))
    assert abs(median_y - target) < 0.5
    assert result.timestamp_ns == 123


def test_profile_refuses_uncalibrated_measurement() -> None:
    profile = Profile3DInterface(LaserCalibration()).reconstruct(LaserLineResult(1, True, [(10.0, 20.0)], 1.0))
    assert profile.points_xyz == []


def test_profile_reports_missing_calibration(config) -> None:
    line = LaserLineDetector(config.laser).detect(blue_laser_image(), 44)
    result = Profile3DInterface().reconstruct(line)
    assert not result.calibrated
    assert result.reason and result.reason.startswith("uncalibrated")
    assert result.points_xyz == []


def test_profile_with_calibration_still_does_not_invent_3d_points() -> None:
    calibration = LaserCalibration(
        camera_matrix=np.eye(3),
        distortion_coefficients=np.zeros(5),
        laser_plane_camera=np.array([0.0, 0.0, 1.0, -1.0]),
        camera_to_base_link=np.eye(4),
        version="test",
    )
    result = Profile3DInterface(calibration).reconstruct(LaserLineResult(2, True, [(10.0, 20.0)], 1.0))
    assert result.calibrated
    assert result.points_xyz == []
    assert result.reason == "triangulation_not_implemented_after_calibration_validation"
