"""Overlap suppression for tiled anomaly candidates."""

from __future__ import annotations

from tank_vision.types import DefectResult


def bbox_iou(first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1))
    first_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    second_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def non_maximum_suppression(detections: list[DefectResult], iou_threshold: float) -> list[DefectResult]:
    ordered = sorted(detections, key=lambda item: item.confidence, reverse=True)
    kept: list[DefectResult] = []
    while ordered:
        current = ordered.pop(0)
        kept.append(current)
        ordered = [
            item
            for item in ordered
            if item.class_name != current.class_name or bbox_iou(item.bbox_xyxy, current.bbox_xyxy) < iou_threshold
        ]
    for index, detection in enumerate(kept):
        detection.defect_id = f"anomaly_{index:04d}"
    return kept

