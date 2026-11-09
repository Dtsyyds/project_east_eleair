"""CPU-only classical weld seam detector."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

from tank_vision.calibration.undistort import Undistorter
from tank_vision.config import PipelineConfig, SeamConfig
from tank_vision.metrics import StageTiming
from tank_vision.preprocessing.normalize import enhance_contrast, select_gray_channel
from tank_vision.preprocessing.roi import ImageView, make_image_view
from tank_vision.seam.centerline import extract_component_centers
from tank_vision.seam.confidence import calculate_confidence
from tank_vision.seam.detector import SeamDetector
from tank_vision.seam.geometry import fit_polynomial_ransac, seam_errors
from tank_vision.types import FrameData, SeamResult

LOGGER = logging.getLogger(__name__)


class ClassicalSeamDetector(SeamDetector):
    def __init__(
        self,
        seam_config: SeamConfig,
        pipeline_config: PipelineConfig,
        undistorter: Undistorter | None = None,
    ) -> None:
        self.config = seam_config
        self.pipeline_config = pipeline_config
        self.undistorter = undistorter
        self.consecutive_misses = 0
        self.last_debug_mask: np.ndarray | None = None
        self.last_stage_timing = StageTiming()

    def _lost(self, frame: FrameData, started: float, reason: str, confidence: float = 0.0) -> SeamResult:
        self.consecutive_misses += 1
        if self.consecutive_misses >= self.config.lost_frame_threshold:
            reason = f"lost_threshold_reached:{reason}"
        return SeamResult(
            timestamp_ns=frame.timestamp_ns,
            detected=False,
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            lost_reason=reason,
            processing_time_ms=(time.perf_counter() - started) * 1000.0,
        )

    def _response(self, view: ImageView) -> tuple[np.ndarray, np.ndarray]:
        gray = select_gray_channel(view.image, self.config.color_channel)
        enhanced = enhance_contrast(gray, self.config.clahe_clip_limit, self.config.clahe_grid_size)
        if self.config.filter_type == "bilateral":
            filtered = cv2.bilateralFilter(
                enhanced,
                self.config.gaussian_kernel,
                self.config.bilateral_sigma,
                self.config.bilateral_sigma,
            )
        else:
            filtered = cv2.GaussianBlur(enhanced, (self.config.gaussian_kernel, self.config.gaussian_kernel), 0)
        if self.config.expected_direction == "vertical":
            background_kernel = (self.config.local_contrast_kernel, max(3, self.config.gaussian_kernel))
            gradient_kernel = (3, 1)
        else:
            background_kernel = (max(3, self.config.gaussian_kernel), self.config.local_contrast_kernel)
            gradient_kernel = (1, 3)
        background = cv2.GaussianBlur(filtered, background_kernel, 0)
        local_contrast = cv2.absdiff(filtered, background)
        gradient_element = cv2.getStructuringElement(cv2.MORPH_RECT, gradient_kernel)
        gradient = cv2.morphologyEx(filtered, cv2.MORPH_GRADIENT, gradient_element)
        response = cv2.addWeighted(
            local_contrast,
            self.config.local_contrast_weight,
            gradient,
            self.config.gradient_weight,
            0.0,
        )
        return gray, response

    def _component_mask(self, response: np.ndarray) -> tuple[np.ndarray | None, float]:
        if float(np.percentile(response, self.config.response_percentile)) < self.config.minimum_response:
            return None, 0.0
        if self.config.threshold_method == "adaptive":
            binary = cv2.adaptiveThreshold(
                response,
                255,
                cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY,
                self.config.adaptive_block_size,
                self.config.adaptive_constant,
            )
        else:
            _, binary = cv2.threshold(response, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.morphology_kernel, self.config.morphology_kernel),
        )
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=self.config.morphology_iterations)
        open_size = self.config.morphology_open_kernel
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8), iterations=1)
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
        height, width = response.shape
        image_area = height * width
        major_length = height if self.config.expected_direction == "vertical" else width
        candidates: list[tuple[float, int]] = []
        for label in range(1, count):
            x, y, component_width, component_height, area = stats[label]
            if area < image_area * self.config.min_component_area_ratio:
                continue
            span = component_height if self.config.expected_direction == "vertical" else component_width
            thickness = component_width if self.config.expected_direction == "vertical" else component_height
            span_ratio = span / major_length
            if span_ratio < self.config.min_component_span_ratio:
                continue
            dependent_center = centroids[label][0 if self.config.expected_direction == "vertical" else 1]
            dependent_length = width if self.config.expected_direction == "vertical" else height
            center_score = 1.0 - abs(dependent_center - dependent_length / 2) / max(1.0, dependent_length / 2)
            elongation = span / max(1.0, thickness)
            density = area / max(1.0, component_width * component_height)
            score = 0.50 * span_ratio + 0.20 * min(1.0, elongation / 8.0) + 0.15 * density + 0.15 * center_score
            candidates.append((score, label))
        if not candidates:
            return None, 0.0
        _, best_label = max(candidates)
        mask = labels == best_label
        inside = response[mask].astype(np.float32)
        ring_size = self.config.contrast_ring_kernel
        dilated = cv2.dilate(mask.astype(np.uint8), np.ones((ring_size, ring_size), np.uint8), iterations=1).astype(bool)
        ring = response[dilated & ~mask].astype(np.float32)
        contrast = float(np.mean(inside) - np.mean(ring)) if ring.size else float(np.mean(inside))
        return mask, max(0.0, contrast)

    def detect(self, frame: FrameData) -> SeamResult:
        started = time.perf_counter()
        self.last_stage_timing = StageTiming()
        image = frame.image
        if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim < 2:
            self.last_debug_mask = None
            return self._lost(frame, started, "empty_image")
        try:
            if self.pipeline_config.use_undistort:
                stage_started = time.perf_counter_ns()
                if self.undistorter is None:
                    return self._lost(frame, started, "undistort_enabled_without_calibration")
                image = self.undistorter.apply(image)
                self.last_stage_timing.add_ns("undistort_ms", time.perf_counter_ns() - stage_started)
            stage_started = time.perf_counter_ns()
            view = make_image_view(image, self.pipeline_config.roi, self.pipeline_config.thumbnail_max_dimension)
            self.last_stage_timing.add_ns("roi_ms", time.perf_counter_ns() - stage_started)
            stage_started = time.perf_counter_ns()
            _, response = self._response(view)
            self.last_stage_timing.add_ns("preprocess_ms", time.perf_counter_ns() - stage_started)
            stage_started = time.perf_counter_ns()
            component, component_contrast = self._component_mask(response)
            if component is None:
                self.last_stage_timing.add_ns("seam_detect_ms", time.perf_counter_ns() - stage_started)
                self.last_debug_mask = np.zeros(image.shape[:2], dtype=np.uint8)
                return self._lost(frame, started, "no_valid_component")
            independent, dependent, widths = extract_component_centers(component, response, self.config.expected_direction)
            self.last_stage_timing.add_ns("seam_detect_ms", time.perf_counter_ns() - stage_started)
            stage_started = time.perf_counter_ns()
            fit = fit_polynomial_ransac(
                independent,
                dependent,
                self.config.fit_degree,
                self.config.ransac_iterations,
                self.config.ransac_residual_px,
            )
            if fit is None:
                self.last_stage_timing.add_ns("seam_geometry_ms", time.perf_counter_ns() - stage_started)
                return self._lost(frame, started, "ransac_fit_failed")
            major_length = response.shape[0] if self.config.expected_direction == "vertical" else response.shape[1]
            dependent_length = response.shape[1] if self.config.expected_direction == "vertical" else response.shape[0]
            confidence, _ = calculate_confidence(
                independent,
                dependent,
                fit.inliers,
                major_length,
                fit.residual_rms,
                self.config.max_fit_residual_px,
                component_contrast,
                dependent_length,
                self.config.center_reasonable_margin_ratio,
                self.config.continuity_max_gap_px,
                self.config.contrast_reference,
            )
            if self.config.expected_direction == "vertical":
                reference_processed = self.config.reference_y_ratio * (response.shape[0] - 1)
                target_processed = self.config.reference_x_ratio * response.shape[1]
                independent_scale = view.scale_y
                dependent_scale = view.scale_x
            else:
                reference_processed = self.config.reference_x_ratio * (response.shape[1] - 1)
                target_processed = self.config.reference_y_ratio * response.shape[0]
                independent_scale = view.scale_x
                dependent_scale = view.scale_y
            lateral_processed, heading_error = seam_errors(
                fit.coefficients,
                self.config.expected_direction,
                reference_processed,
                target_processed,
            )
            lateral_error = lateral_processed / dependent_scale
            start_coordinate = int(np.ceil(np.min(independent[fit.inliers])))
            end_coordinate = int(np.floor(np.max(independent[fit.inliers])))
            step = max(1, round(self.config.centerline_step_px * independent_scale))
            centerline: list[tuple[float, float]] = []
            for coordinate in range(start_coordinate, end_coordinate + 1, step):
                fitted = float(np.polyval(fit.coefficients, coordinate))
                if self.config.expected_direction == "vertical":
                    centerline.append(view.to_original(fitted, float(coordinate)))
                else:
                    centerline.append(view.to_original(float(coordinate), fitted))
            full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
            component_u8 = component.astype(np.uint8) * 255
            if component_u8.shape[::-1] != (view.roi_width, view.roi_height):
                component_u8 = cv2.resize(component_u8, (view.roi_width, view.roi_height), interpolation=cv2.INTER_NEAREST)
            full_mask[
                view.origin_y : view.origin_y + view.roi_height,
                view.origin_x : view.origin_x + view.roi_width,
            ] = component_u8
            self.last_debug_mask = full_mask
            detected = confidence >= self.config.min_confidence
            if detected:
                self.consecutive_misses = 0
                lost_reason = None
            else:
                self.consecutive_misses += 1
                lost_reason = "low_confidence"
                if self.consecutive_misses >= self.config.lost_frame_threshold:
                    lost_reason = "lost_threshold_reached:low_confidence"
            width_px = float(np.median(widths[fit.inliers]) / dependent_scale) if widths.size else None
            self.last_stage_timing.add_ns("seam_geometry_ms", time.perf_counter_ns() - stage_started)
            return SeamResult(
                timestamp_ns=frame.timestamp_ns,
                detected=detected,
                confidence=confidence,
                centerline_points=centerline,
                lateral_error_px=float(lateral_error),
                heading_error_deg=float(heading_error),
                seam_width_px=width_px,
                lost_reason=lost_reason,
                processing_time_ms=(time.perf_counter() - started) * 1000.0,
                inlier_ratio=float(np.mean(fit.inliers)),
                fit_residual_px=fit.residual_rms / dependent_scale,
            )
        except (cv2.error, ValueError, TypeError, np.linalg.LinAlgError) as exc:
            LOGGER.exception("Classical seam detection failed")
            self.last_debug_mask = None
            return self._lost(frame, started, f"detector_error:{type(exc).__name__}")
