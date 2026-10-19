"""Cached OpenCV undistortion maps."""

from __future__ import annotations

import cv2
import numpy as np

from .camera_calibration import CameraCalibration


class Undistorter:
    def __init__(self, calibration: CameraCalibration) -> None:
        self.calibration = calibration
        self._maps: dict[tuple[int, int], tuple[np.ndarray, np.ndarray]] = {}

    def apply(self, image: np.ndarray) -> np.ndarray:
        if image is None or image.size == 0:
            return image
        height, width = image.shape[:2]
        key = (width, height)
        if key not in self._maps:
            sx = width / self.calibration.image_width
            sy = height / self.calibration.image_height
            matrix = self.calibration.camera_matrix.copy()
            matrix[0, :] *= sx
            matrix[1, :] *= sy
            self._maps[key] = cv2.initUndistortRectifyMap(
                matrix,
                self.calibration.distortion_coefficients,
                None,
                matrix,
                (width, height),
                cv2.CV_32FC1,
            )
        map_x, map_y = self._maps[key]
        return cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR)

