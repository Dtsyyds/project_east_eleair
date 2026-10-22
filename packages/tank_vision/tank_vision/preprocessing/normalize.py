"""Configurable channel selection and contrast normalization."""

from __future__ import annotations

import cv2
import numpy as np


def select_gray_channel(image: np.ndarray, channel: str = "gray") -> np.ndarray:
    if image.ndim == 2:
        return image
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError(f"Unsupported image shape: {image.shape}")
    normalized = channel.lower()
    if normalized == "gray":
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    channel_indices = {"blue": 0, "green": 1, "red": 2}
    if normalized not in channel_indices:
        raise ValueError(f"Unsupported color channel: {channel}")
    return image[:, :, channel_indices[normalized]]


def enhance_contrast(gray: np.ndarray, clip_limit: float, grid_size: int) -> np.ndarray:
    if gray.dtype != np.uint8:
        gray = cv2.normalize(gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(grid_size, grid_size))
    return clahe.apply(gray)

