"""Laser triangulation calibration contract."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from tank_vision.types import LaserLineResult, Profile3DResult


@dataclass(slots=True)
class LaserCalibration:
    camera_matrix: np.ndarray | None = None
    distortion_coefficients: np.ndarray | None = None
    laser_plane_camera: np.ndarray | None = None
    camera_to_base_link: np.ndarray | None = None
    version: str | None = None

    def missing_fields(self) -> list[str]:
        return [
            name
            for name, value in (
                ("camera_intrinsics", self.camera_matrix),
                ("camera_distortion", self.distortion_coefficients),
                ("laser_plane_parameters", self.laser_plane_camera),
                ("camera_to_base_link_extrinsics", self.camera_to_base_link),
                ("calibration_version", self.version),
            )
            if value is None
        ]


class Profile3DInterface:
    """Guarded 3D interface; triangulation is disabled until all calibration exists."""

    def __init__(self, calibration: LaserCalibration | None = None) -> None:
        self.calibration = calibration or LaserCalibration()

    def reconstruct(self, line: LaserLineResult) -> Profile3DResult:
        missing = self.calibration.missing_fields()
        if missing:
            return Profile3DResult(
                timestamp_ns=line.timestamp_ns,
                calibrated=False,
                reason=f"uncalibrated:missing_{','.join(missing)}",
            )
        if not line.detected:
            return Profile3DResult(line.timestamp_ns, True, reason="laser_line_not_detected")
        # The interface is intentionally guarded until the project selects conventions,
        # validates the laser plane, and adds calibration-quality tests.
        return Profile3DResult(
            timestamp_ns=line.timestamp_ns,
            calibrated=True,
            reason="triangulation_not_implemented_after_calibration_validation",
        )
