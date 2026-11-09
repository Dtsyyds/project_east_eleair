"""450 nm blue laser-line extraction without fabricated 3D output."""

from __future__ import annotations

import cv2
import numpy as np

from tank_vision.config import LaserConfig
from tank_vision.laser.subpixel import column_weighted_centers
from tank_vision.seam.geometry import fit_polynomial_ransac
from tank_vision.types import LaserLineResult


class LaserLineDetector:
    def __init__(self, config: LaserConfig) -> None:
        self.config = config
        self.last_mask: np.ndarray | None = None

    @staticmethod
    def subtract_laser_frames(laser_on: np.ndarray, laser_off: np.ndarray) -> np.ndarray:
        if laser_on.shape != laser_off.shape:
            raise ValueError("Laser on/off frames must have the same shape")
        return cv2.subtract(laser_on, laser_off)

    def detect(
        self,
        image: np.ndarray,
        timestamp_ns: int,
        laser_off_image: np.ndarray | None = None,
    ) -> LaserLineResult:
        if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim != 3 or image.shape[2] < 3:
            return LaserLineResult(timestamp_ns, False, reason="invalid_color_image")
        try:
            working = self.subtract_laser_frames(image, laser_off_image) if laser_off_image is not None else image
            blue, green, red = cv2.split(working[:, :, :3])
            difference = blue.astype(np.float32) - 0.5 * (green.astype(np.float32) + red.astype(np.float32))
            response = np.clip(difference, 0, 255).astype(np.uint8)
            hsv = cv2.cvtColor(working[:, :, :3], cv2.COLOR_BGR2HSV)
            hsv_mask = cv2.inRange(
                hsv,
                (self.config.hsv_h_min, self.config.saturation_min, self.config.value_min),
                (self.config.hsv_h_max, 255, 255),
            )
            difference_mask = (response >= self.config.blue_difference_threshold).astype(np.uint8) * 255
            mask = cv2.bitwise_and(hsv_mask, difference_mask)
            open_size = self.config.morphology_open_kernel
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8), iterations=1)
            mask = cv2.morphologyEx(
                mask,
                cv2.MORPH_CLOSE,
                np.ones((self.config.morphology_close_height, self.config.morphology_close_width), np.uint8),
                iterations=1,
            )
            self.last_mask = mask
            xs, ys = column_weighted_centers(response, mask)
            if xs.size < self.config.min_points:
                return LaserLineResult(timestamp_ns, False, reason="insufficient_laser_points")
            degree = 2 if xs.size >= self.config.quadratic_fit_min_points else 1
            fit = fit_polynomial_ransac(
                xs,
                ys,
                degree,
                self.config.ransac_iterations,
                self.config.ransac_residual_px,
            )
            if fit is None:
                return LaserLineResult(timestamp_ns, False, reason="laser_ransac_failed")
            inlier_x = xs[fit.inliers]
            inlier_y = ys[fit.inliers]
            points = [(float(x), float(y)) for x, y in zip(inlier_x, inlier_y)]
            coverage = len(points) / max(1, image.shape[1])
            inlier_ratio = float(np.mean(fit.inliers))
            residual_score = float(np.exp(-fit.residual_rms / max(self.config.ransac_residual_px, 1e-6)))
            confidence = float(np.clip(0.4 * coverage + 0.35 * inlier_ratio + 0.25 * residual_score, 0.0, 1.0))
            return LaserLineResult(timestamp_ns, True, points, confidence, None)
        except (cv2.error, ValueError, TypeError) as exc:
            self.last_mask = None
            return LaserLineResult(timestamp_ns, False, reason=f"laser_detection_error:{type(exc).__name__}")
