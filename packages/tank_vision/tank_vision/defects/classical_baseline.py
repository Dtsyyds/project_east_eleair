"""Conservative color/contrast/texture surface-anomaly candidate baseline."""

from __future__ import annotations

import logging
import time

import cv2
import numpy as np

from tank_vision.config import DefectConfig
from tank_vision.defects.detector import DefectDetector
from tank_vision.defects.postprocess import non_maximum_suppression
from tank_vision.defects.tiled_inference import Tile, TiledInference
from tank_vision.metrics import StageTiming
from tank_vision.types import DefectResult, FrameData

LOGGER = logging.getLogger(__name__)


class ClassicalDefectBaseline(DefectDetector):
    """Find anomaly candidates without claiming corrosion, pits, cracks, or scratches."""

    backend_name = "classical_surface_anomaly"

    def __init__(self, config: DefectConfig) -> None:
        self.config = config
        self.tiler = TiledInference(config.tile_size, config.overlap)
        self.last_mask: np.ndarray | None = None
        self.last_warning: str | None = None
        self.last_stage_timing = StageTiming()
        self.last_tiles: list[Tile] = []

    def _tile_mask(self, tile_image: np.ndarray, exclusion: np.ndarray | None) -> tuple[np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(tile_image, cv2.COLOR_BGR2GRAY) if tile_image.ndim == 3 else tile_image
        gray = gray.astype(np.uint8, copy=False)
        local_background = cv2.GaussianBlur(
            gray,
            (self.config.local_contrast_kernel, self.config.local_contrast_kernel),
            0,
        )
        local_difference = cv2.absdiff(gray, local_background).astype(np.float32)
        laplacian = np.abs(cv2.Laplacian(gray, cv2.CV_32F))
        if tile_image.ndim == 3:
            lab = cv2.cvtColor(tile_image, cv2.COLOR_BGR2LAB).astype(np.float32)
            color_difference = np.sqrt(np.square(lab[:, :, 1] - 128.0) + np.square(lab[:, :, 2] - 128.0))
        else:
            color_difference = np.zeros_like(local_difference)
        local_mean, local_std = float(np.mean(local_difference)), float(np.std(local_difference))
        texture_mean, texture_std = float(np.mean(laplacian)), float(np.std(laplacian))
        local_threshold = local_mean + self.config.anomaly_z_threshold * max(local_std, 1.0)
        texture_threshold = texture_mean + (
            self.config.anomaly_z_threshold + self.config.texture_z_offset
        ) * max(texture_std, 1.0)
        # Lab chroma threshold is intentionally generic: it means unusual color, not rust.
        binary = (
            (local_difference > local_threshold)
            | (laplacian > texture_threshold)
            | (color_difference > self.config.color_difference_threshold)
        )
        mask = binary.astype(np.uint8) * 255
        open_size = self.config.morphology_open_kernel
        close_size = self.config.morphology_close_kernel
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((open_size, open_size), np.uint8), iterations=1)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            np.ones((close_size, close_size), np.uint8),
            iterations=self.config.morphology_close_iterations,
        )
        if exclusion is not None:
            exclusion_size = self.config.exclusion_dilation_kernel
            excluded = cv2.dilate(
                (exclusion > 0).astype(np.uint8),
                np.ones((exclusion_size, exclusion_size), np.uint8),
                iterations=1,
            )
            mask[excluded > 0] = 0
        score = np.maximum.reduce(
            [
                local_difference / max(local_threshold, 1.0),
                laplacian / max(texture_threshold, 1.0),
                color_difference / max(self.config.color_difference_threshold, 1e-6),
            ]
        )
        return mask, score

    def _detect_tile(self, tile: Tile, exclusion: np.ndarray | None) -> tuple[list[DefectResult], np.ndarray]:
        mask, score = self._tile_mask(tile.image, exclusion)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        tile_area = tile.valid_width * tile.valid_height
        detections: list[DefectResult] = []
        valid_mask = np.zeros(mask.shape, dtype=np.uint8)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.config.min_area_px or area > tile_area * self.config.max_area_ratio:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if x >= tile.valid_width or y >= tile.valid_height:
                continue
            x2, y2 = min(tile.valid_width, x + width), min(tile.valid_height, y + height)
            if x2 <= x or y2 <= y:
                continue
            component_mask = np.zeros(mask.shape, dtype=np.uint8)
            cv2.drawContours(component_mask, [contour], -1, 255, thickness=-1)
            component_scores = score[component_mask > 0]
            confidence = float(np.clip(0.35 + 0.25 * np.mean(component_scores - 1.0), 0.0, 0.95))
            # Coherent colored regions should clear the conservative default threshold.
            color_pixels = tile.image[component_mask > 0] if tile.image.ndim == 3 else None
            if color_pixels is not None and color_pixels.size:
                channel_spread = float(np.mean(np.max(color_pixels, axis=1) - np.min(color_pixels, axis=1)))
                confidence = max(
                    confidence,
                    float(np.clip(channel_spread / max(self.config.color_confidence_scale, 1e-6), 0.0, 0.95)),
                )
            if confidence < self.config.confidence_threshold:
                continue
            moments = cv2.moments(contour)
            if moments["m00"]:
                centroid = (float(moments["m10"] / moments["m00"]), float(moments["m01"] / moments["m00"]))
            else:
                centroid = (x + width / 2.0, y + height / 2.0)
            polygon = [(float(point[0][0]), float(point[0][1])) for point in cv2.approxPolyDP(contour, 2.0, True)]
            detections.append(
                DefectResult(
                    defect_id="tile_candidate",
                    class_name="surface_anomaly",
                    confidence=confidence,
                    bbox_xyxy=(x, y, x2, y2),
                    polygon=polygon,
                    area_px=area,
                    centroid_uv=centroid,
                    backend_name=self.backend_name,
                )
            )
            cv2.drawContours(valid_mask, [contour], -1, 255, thickness=-1)
        return detections, valid_mask

    def detect(self, frame: FrameData, exclusion_mask: np.ndarray | None = None) -> list[DefectResult]:
        image = frame.image
        self.last_warning = None
        self.last_stage_timing = StageTiming()
        self.last_tiles = []
        if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim < 2:
            self.last_mask = None
            self.last_warning = "defect_detection_skipped:empty_image"
            return []
        full_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        restored: list[DefectResult] = []
        try:
            stage_started = time.perf_counter_ns()
            self.last_tiles = self.tiler.tiles(image)
            self.last_stage_timing.add_ns("tiling_ms", time.perf_counter_ns() - stage_started)
            inference_started = time.perf_counter_ns()
            for tile in self.last_tiles:
                exclusion_tile = None
                if exclusion_mask is not None and exclusion_mask.shape[:2] == image.shape[:2]:
                    crop = exclusion_mask[tile.y : tile.y + tile.valid_height, tile.x : tile.x + tile.valid_width]
                    exclusion_tile = cv2.copyMakeBorder(
                        crop,
                        0,
                        self.config.tile_size - tile.valid_height,
                        0,
                        self.config.tile_size - tile.valid_width,
                        cv2.BORDER_CONSTANT,
                    )
                local_detections, local_mask = self._detect_tile(tile, exclusion_tile)
                valid_local = local_mask[: tile.valid_height, : tile.valid_width]
                target = full_mask[tile.y : tile.y + tile.valid_height, tile.x : tile.x + tile.valid_width]
                np.maximum(target, valid_local, out=target)
                for detection in local_detections:
                    global_detection = self.tiler.restore_detection(detection, tile, image.shape)
                    if global_detection is not None:
                        restored.append(global_detection)
            self.last_mask = full_mask
            self.last_stage_timing.add_ns("defect_inference_ms", time.perf_counter_ns() - inference_started)
            postprocess_started = time.perf_counter_ns()
            result = non_maximum_suppression(restored, self.config.nms_iou_threshold)
            self.last_stage_timing.add_ns("postprocess_ms", time.perf_counter_ns() - postprocess_started)
            return result
        except (cv2.error, ValueError, TypeError) as exc:
            LOGGER.exception("Classical defect baseline failed")
            self.last_mask = full_mask
            self.last_warning = f"defect_detection_failed:{type(exc).__name__}"
            return []
