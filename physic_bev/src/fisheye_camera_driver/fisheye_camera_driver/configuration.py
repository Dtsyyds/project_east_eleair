"""SDK-independent configuration helpers for the fisheye driver."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def resolve_device(configured: str, match: str, by_id_root: Path = Path("/dev/v4l/by-id")) -> Path:
    if configured:
        return Path(configured).expanduser()
    candidates = sorted(
        path for path in by_id_root.glob("*-video-index0")
        if match.lower() in path.name.lower()
    )
    if len(candidates) != 1:
        raise RuntimeError(
            f"fisheye stable device selection matched {len(candidates)} paths for {match!r}: "
            + ", ".join(str(path) for path in candidates)
        )
    return candidates[0]


def validate_capture(width: int, height: int, fps: float, pixel_format: str) -> tuple[int, int, float, str]:
    if width <= 0 or height <= 0 or width > 8192 or height > 8192:
        raise ValueError("width and height must be within 1..8192")
    if fps <= 0.0 or fps > 120.0:
        raise ValueError("fps must be within (0, 120]")
    normalized = pixel_format.upper()
    if normalized not in {"MJPG", "YUYV"}:
        raise ValueError("pixel_format must be MJPG or YUYV")
    return width, height, fps, normalized


def load_camera_info(path_value: str) -> dict[str, Any] | None:
    if not path_value:
        return None
    path = Path(path_value.removeprefix("file://")).expanduser().resolve()
    document = yaml.safe_load(path.read_text()) or {}
    required = ("image_width", "image_height", "camera_matrix", "distortion_coefficients",
                "rectification_matrix", "projection_matrix")
    missing = [key for key in required if key not in document]
    if missing:
        raise ValueError(f"camera calibration missing fields: {', '.join(missing)}")
    sizes = {"camera_matrix": 9, "rectification_matrix": 9, "projection_matrix": 12}
    for key, expected in sizes.items():
        data = document[key].get("data", [])
        if len(data) != expected:
            raise ValueError(f"camera calibration {key} requires {expected} values")
    return document
