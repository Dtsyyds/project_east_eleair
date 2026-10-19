"""Subpixel grayscale-centroid extraction."""

from __future__ import annotations

import numpy as np


def column_weighted_centers(response: np.ndarray, mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xs: list[float] = []
    ys: list[float] = []
    for x in range(response.shape[1]):
        rows = np.flatnonzero(mask[:, x] > 0)
        if rows.size == 0:
            continue
        weights = response[rows, x].astype(np.float64)
        total = float(np.sum(weights))
        if total <= 0.0:
            continue
        xs.append(float(x))
        ys.append(float(np.sum(rows * weights) / total))
    return np.asarray(xs, dtype=np.float64), np.asarray(ys, dtype=np.float64)

