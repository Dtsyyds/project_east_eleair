"""Coordinate-preserving crop and thumbnail transforms."""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(slots=True)
class ImageView:
    image: np.ndarray
    origin_x: int
    origin_y: int
    scale_x: float
    scale_y: float
    original_width: int
    original_height: int
    roi_width: int
    roi_height: int

    def to_original(self, x: float, y: float) -> tuple[float, float]:
        return (self.origin_x + x / self.scale_x, self.origin_y + y / self.scale_y)

    def length_to_original_x(self, value: float) -> float:
        return value / self.scale_x


def make_image_view(
    image: np.ndarray,
    roi: tuple[int, int, int, int] | None = None,
    max_dimension: int = 0,
) -> ImageView:
    if image is None or image.size == 0 or image.ndim < 2:
        raise ValueError("Cannot create an image view from an empty image")
    original_height, original_width = image.shape[:2]
    if roi is None:
        x, y, width, height = 0, 0, original_width, original_height
    else:
        x, y, width, height = (int(value) for value in roi)
        if x >= original_width or y >= original_height:
            raise ValueError("ROI origin lies outside the image")
        width = min(width, original_width - x)
        height = min(height, original_height - y)
        if width <= 0 or height <= 0:
            raise ValueError("ROI has no intersection with the image")
    cropped = image[y : y + height, x : x + width]
    scale = 1.0
    if max_dimension > 0 and max(width, height) > max_dimension:
        scale = max_dimension / max(width, height)
        output_size = (max(1, round(width * scale)), max(1, round(height * scale)))
        cropped = cv2.resize(cropped, output_size, interpolation=cv2.INTER_AREA)
    processed_height, processed_width = cropped.shape[:2]
    return ImageView(
        image=cropped,
        origin_x=x,
        origin_y=y,
        scale_x=processed_width / width,
        scale_y=processed_height / height,
        original_width=original_width,
        original_height=original_height,
        roi_width=width,
        roi_height=height,
    )

