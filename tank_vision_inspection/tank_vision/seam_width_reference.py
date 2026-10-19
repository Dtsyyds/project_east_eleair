"""Validated interpolation of a manually derived, normalized seam-width reference."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True, slots=True)
class SeamWidthReference:
    source_width: int
    source_height: int
    y_normalized: np.ndarray
    center_x_normalized: np.ndarray
    width_normalized: np.ndarray
    metadata: dict[str, Any]
    center_coefficients: np.ndarray | None = None
    width_coefficients: np.ndarray | None = None

    @classmethod
    def load(cls, path: str | Path) -> "SeamWidthReference":
        source = Path(path)
        raw = json.loads(source.read_text(encoding="utf-8"))
        if raw.get("schema_version") != "1.0":
            raise ValueError("Unsupported seam width reference schema")
        samples = raw.get("reference_samples")
        if not isinstance(samples, list) or len(samples) < 4:
            raise ValueError("Reference needs at least four samples")
        y = np.asarray([item["y_normalized"] for item in samples], dtype=np.float64)
        center = np.asarray([item["center_x_normalized"] for item in samples], dtype=np.float64)
        width = np.asarray([item["width_normalized"] for item in samples], dtype=np.float64)
        if not (np.isfinite(y).all() and np.isfinite(center).all() and np.isfinite(width).all()):
            raise ValueError("Reference contains non-finite values")
        if np.any(np.diff(y) <= 0) or np.any(width <= 0):
            raise ValueError("Reference y must increase and widths must be positive")
        center_coefficients = _polynomial_coefficients(raw, "center_ref")
        width_coefficients = _polynomial_coefficients(raw, "width_ref")
        if (center_coefficients is None) != (width_coefficients is None):
            raise ValueError("Reference must provide both center_ref and width_ref coefficients")
        return cls(int(raw["source_width"]), int(raw["source_height"]), y, center, width, raw,
                   center_coefficients, width_coefficients)

    def validate_image(self, width: int, height: int, same_fov_scaled: bool) -> None:
        if (width, height) == (self.source_width, self.source_height):
            return
        if not same_fov_scaled:
            raise ValueError(
                f"Input {width}x{height} differs from reference {self.source_width}x{self.source_height}; "
                "set camera.same_fov_scaled=true only for a verified same-FOV resize"
            )
        if abs(width / height - self.source_width / self.source_height) > 1e-3:
            raise ValueError("same_fov_scaled input aspect ratio differs from reference")

    def at(self, y_original: np.ndarray | float, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
        yn = np.asarray(y_original, dtype=np.float64) / float(height)
        if self.center_coefficients is not None and self.width_coefficients is not None:
            center = np.polyval(self.center_coefficients, yn) * width
            seam_width = np.polyval(self.width_coefficients, yn) * width
        else:
            center = np.interp(yn, self.y_normalized, self.center_x_normalized) * width
            seam_width = np.interp(yn, self.y_normalized, self.width_normalized) * width
        return center, seam_width


def _polynomial_coefficients(raw: dict[str, Any], name: str) -> np.ndarray | None:
    """Read the optional normalized polynomial representation of a reference.

    The aliases are intentional: the explicit top-level fields make the frozen
    JSON easy to audit, while the nested object carries degree/coordinate
    metadata for consumers that need it.
    """
    value = raw.get(name)
    if isinstance(value, dict):
        coefficients = value.get("coefficients")
    else:
        coefficients = raw.get(f"{name}_coefficients")
    if coefficients is None:
        return None
    result = np.asarray(coefficients, dtype=np.float64)
    if result.ndim != 1 or len(result) < 2 or not np.isfinite(result).all():
        raise ValueError(f"{name} coefficients must be a finite polynomial")
    return result
