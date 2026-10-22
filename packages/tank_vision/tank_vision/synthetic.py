"""Deterministic synthetic images for software regression tests only."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def steel_background(width: int = 640, height: int = 480, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    x_gradient = np.linspace(-8.0, 8.0, width, dtype=np.float32)[None, :]
    noise = rng.normal(0.0, 2.0, (height, width)).astype(np.float32)
    gray = np.clip(105.0 + x_gradient + noise, 0, 255).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def seam_image(
    width: int = 640,
    height: int = 480,
    center_x: float | None = None,
    slope: float = 0.0,
    curvature: float = 0.0,
    contrast: int = 100,
    noise_sigma: float = 0.0,
    occlusion: tuple[int, int] | None = None,
    seed: int = 2,
) -> np.ndarray:
    center_x = width / 2 if center_x is None else center_x
    image = steel_background(width, height, seed)
    ys = np.arange(height, dtype=np.float64)
    normalized_y = ys - height * 0.5
    xs = center_x + slope * normalized_y + curvature * np.square(normalized_y) / max(1.0, height)
    points = np.column_stack((xs, ys)).round().astype(np.int32).reshape(-1, 1, 2)
    base_value = 105
    seam_value = int(np.clip(base_value + contrast, 0, 255))
    cv2.polylines(image, [points], False, (seam_value,) * 3, 15, cv2.LINE_AA)
    cv2.polylines(image, [points], False, (min(255, seam_value + 30),) * 3, 3, cv2.LINE_AA)
    if occlusion is not None:
        y1, y2 = occlusion
        replacement = steel_background(width, max(1, y2 - y1), seed + 19)
        image[y1:y2] = replacement[: max(0, min(y2, height) - y1)]
    if noise_sigma > 0:
        rng = np.random.default_rng(seed + 5)
        noise = rng.normal(0.0, noise_sigma, image.shape).astype(np.float32)
        image = np.clip(image.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return image


def rust_anomaly_image(width: int = 640, height: int = 480) -> np.ndarray:
    image = seam_image(width, height)
    cv2.ellipse(image, (int(width * 0.72), int(height * 0.42)), (48, 32), 18, 0, 360, (35, 82, 175), -1)
    cv2.circle(image, (int(width * 0.74), int(height * 0.40)), 12, (20, 55, 125), -1)
    return image


def blue_laser_image(width: int = 640, height: int = 240, center_y: float = 115.35) -> np.ndarray:
    image = np.full((height, width, 3), 25, dtype=np.uint8)
    y0 = int(np.floor(center_y))
    fraction = center_y - y0
    for offset in range(-3, 4):
        y = y0 + offset
        if 0 <= y < height:
            intensity = int(245 * np.exp(-0.5 * ((offset - fraction) / 1.1) ** 2))
            image[y, :, 0] = np.maximum(image[y, :, 0], intensity)
            image[y, :, 1] = np.maximum(image[y, :, 1], intensity // 7)
    return image


def generate_sample_set(output_dir: str | Path) -> list[Path]:
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    samples = {
        "01_straight_center.png": seam_image(),
        "02_curved.png": seam_image(curvature=0.12),
        "03_noisy.png": seam_image(center_x=350, noise_sigma=16),
        "04_low_contrast.png": seam_image(contrast=32),
        "05_partial_occlusion.png": seam_image(occlusion=(190, 280)),
        "06_no_seam.png": steel_background(seed=22),
        "07_surface_anomaly.png": rust_anomaly_image(),
        "08_blue_laser.png": blue_laser_image(),
    }
    paths: list[Path] = []
    for name, image in samples.items():
        path = output / name
        if not cv2.imwrite(str(path), image):
            raise OSError(f"Could not write synthetic sample: {path}")
        paths.append(path)
    return paths

