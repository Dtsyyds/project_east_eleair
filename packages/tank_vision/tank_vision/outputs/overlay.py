"""Human-readable inspection overlay."""

from __future__ import annotations

import cv2
import numpy as np

from tank_vision.types import VisionResult


def create_overlay(result: VisionResult) -> np.ndarray | None:
    image = result.frame.image
    if not isinstance(image, np.ndarray) or image.size == 0 or image.ndim < 2:
        return None
    if image.ndim == 2:
        overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        overlay = image.copy()
    seam = result.seam_result
    if seam.centerline_points:
        points = np.asarray(seam.centerline_points, dtype=np.float32).round().astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(overlay, [points], False, (0, 255, 0) if seam.detected else (0, 165, 255), 2)
    for defect in result.defect_results:
        x1, y1, x2, y2 = defect.bbox_xyxy
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.putText(
            overlay,
            f"surface_anomaly {defect.confidence:.2f}",
            (x1, max(18, y1 - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )
    status = "SEAM" if seam.detected else f"LOST:{seam.lost_reason or 'unknown'}"
    geometry = ""
    if seam.lateral_error_px is not None and seam.heading_error_deg is not None:
        geometry = f" dx={seam.lateral_error_px:+.1f}px heading={seam.heading_error_deg:+.1f}deg"
    cv2.putText(
        overlay,
        f"{status} conf={seam.confidence:.2f}{geometry}",
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0) if seam.detected else (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    quality_color = (0, 255, 0) if result.quality_result.valid_for_measurement else (0, 165, 255)
    cv2.putText(
        overlay,
        f"brightness={result.quality_result.mean_brightness:.1f} sharpness={result.quality_result.laplacian_variance:.1f}",
        (12, 54),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        quality_color,
        1,
        cv2.LINE_AA,
    )
    return overlay
