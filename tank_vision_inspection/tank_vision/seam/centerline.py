"""Center-point extraction from a selected seam component."""

from __future__ import annotations

import numpy as np


def extract_component_centers(
    component_mask: np.ndarray,
    response: np.ndarray,
    expected_direction: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return independent coordinate, dependent center, and local seam width."""
    independent: list[float] = []
    dependent: list[float] = []
    widths: list[float] = []
    if expected_direction == "vertical":
        for y in range(component_mask.shape[0]):
            xs = np.flatnonzero(component_mask[y])
            if xs.size == 0:
                continue
            weights = response[y, xs].astype(np.float64) + 1.0
            independent.append(float(y))
            dependent.append(float(np.average(xs, weights=weights)))
            widths.append(float(xs[-1] - xs[0] + 1))
    else:
        for x in range(component_mask.shape[1]):
            ys = np.flatnonzero(component_mask[:, x])
            if ys.size == 0:
                continue
            weights = response[ys, x].astype(np.float64) + 1.0
            independent.append(float(x))
            dependent.append(float(np.average(ys, weights=weights)))
            widths.append(float(ys[-1] - ys[0] + 1))
    return (
        np.asarray(independent, dtype=np.float64),
        np.asarray(dependent, dtype=np.float64),
        np.asarray(widths, dtype=np.float64),
    )

