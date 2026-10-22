"""Overlapping padded tiles and original-coordinate restoration."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from tank_vision.types import DefectResult


@dataclass(slots=True)
class Tile:
    image: np.ndarray
    x: int
    y: int
    valid_width: int
    valid_height: int


def _origins(length: int, tile_size: int, step: int) -> list[int]:
    if length <= tile_size:
        return [0]
    origins = list(range(0, length - tile_size + 1, step))
    final = length - tile_size
    if origins[-1] != final:
        origins.append(final)
    return origins


class TiledInference:
    def __init__(self, tile_size: int = 1024, overlap: int = 128) -> None:
        if tile_size <= 0 or overlap < 0 or overlap >= tile_size:
            raise ValueError("tile_size and overlap must satisfy tile_size > overlap >= 0")
        self.tile_size = tile_size
        self.overlap = overlap

    def tiles(self, image: np.ndarray) -> list[Tile]:
        if image is None or image.size == 0 or image.ndim < 2:
            return []
        height, width = image.shape[:2]
        step = self.tile_size - self.overlap
        result: list[Tile] = []
        for y in _origins(height, self.tile_size, step):
            for x in _origins(width, self.tile_size, step):
                crop = image[y : min(y + self.tile_size, height), x : min(x + self.tile_size, width)]
                valid_height, valid_width = crop.shape[:2]
                pad_right = self.tile_size - valid_width
                pad_bottom = self.tile_size - valid_height
                if pad_right or pad_bottom:
                    crop = cv2.copyMakeBorder(
                        crop,
                        0,
                        pad_bottom,
                        0,
                        pad_right,
                        cv2.BORDER_REFLECT_101 if valid_width > 1 and valid_height > 1 else cv2.BORDER_CONSTANT,
                    )
                result.append(Tile(crop, x, y, valid_width, valid_height))
        return result

    @staticmethod
    def restore_detection(detection: DefectResult, tile: Tile, image_shape: tuple[int, ...]) -> DefectResult | None:
        image_height, image_width = image_shape[:2]
        x1, y1, x2, y2 = detection.bbox_xyxy
        x1 = max(0, min(tile.valid_width, x1))
        y1 = max(0, min(tile.valid_height, y1))
        x2 = max(0, min(tile.valid_width, x2))
        y2 = max(0, min(tile.valid_height, y2))
        if x2 <= x1 or y2 <= y1:
            return None
        global_bbox = (
            max(0, min(image_width, tile.x + x1)),
            max(0, min(image_height, tile.y + y1)),
            max(0, min(image_width, tile.x + x2)),
            max(0, min(image_height, tile.y + y2)),
        )
        polygon = None
        if detection.polygon is not None:
            polygon = [
                (float(np.clip(tile.x + px, 0, image_width - 1)), float(np.clip(tile.y + py, 0, image_height - 1)))
                for px, py in detection.polygon
                if 0 <= px < tile.valid_width and 0 <= py < tile.valid_height
            ]
        return DefectResult(
            defect_id=detection.defect_id,
            class_name=detection.class_name,
            confidence=detection.confidence,
            bbox_xyxy=global_bbox,
            polygon=polygon,
            area_px=detection.area_px,
            centroid_uv=(tile.x + detection.centroid_uv[0], tile.y + detection.centroid_uv[1]),
            backend_name=detection.backend_name,
        )

