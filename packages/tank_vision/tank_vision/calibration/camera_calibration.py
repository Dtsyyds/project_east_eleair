"""OpenCV camera-calibration persistence."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


@dataclass(slots=True)
class CameraCalibration:
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    image_width: int
    image_height: int
    version: str
    rms_error: float | None = None


def load_camera_calibration(path: str | Path) -> CameraCalibration:
    calibration_path = Path(path)
    if not calibration_path.is_file():
        raise FileNotFoundError(f"Calibration file not found: {calibration_path}")
    with calibration_path.open("r", encoding="utf-8") as handle:
        data: dict[str, Any] = yaml.safe_load(handle) or {}
    required = {"camera_matrix", "distortion_coefficients", "image_width", "image_height", "version"}
    missing = required - set(data)
    if missing:
        raise ValueError(f"Calibration file is missing: {sorted(missing)}")
    matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
    distortion = np.asarray(data["distortion_coefficients"], dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("camera_matrix must have shape 3x3")
    return CameraCalibration(
        camera_matrix=matrix,
        distortion_coefficients=distortion,
        image_width=int(data["image_width"]),
        image_height=int(data["image_height"]),
        version=str(data["version"]),
        rms_error=float(data["rms_error"]) if data.get("rms_error") is not None else None,
    )


def save_camera_calibration(calibration: CameraCalibration, path: str | Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "camera_matrix": calibration.camera_matrix.tolist(),
        "distortion_coefficients": calibration.distortion_coefficients.reshape(-1).tolist(),
        "image_width": calibration.image_width,
        "image_height": calibration.image_height,
        "version": calibration.version,
        "rms_error": calibration.rms_error,
    }
    with target.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)

