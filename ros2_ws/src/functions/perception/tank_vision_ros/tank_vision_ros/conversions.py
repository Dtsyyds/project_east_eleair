"""Convert core dataclasses to timestamped ROS messages."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point32
from std_msgs.msg import Header
from tank_vision_interfaces.msg import Defect, DefectArray, SeamState, VisionQuality

from tank_vision.types import DefectResult, VisionResult


def _finite_or_nan(value: float | None) -> float:
    return float(value) if value is not None else math.nan


def seam_message(result: VisionResult, header: Header) -> SeamState:
    seam = result.seam_result
    message = SeamState()
    message.header = header
    message.detected = seam.detected
    message.confidence = seam.confidence
    message.centerline_points = [Point32(x=float(x), y=float(y), z=0.0) for x, y in seam.centerline_points]
    message.lateral_error_px = _finite_or_nan(seam.lateral_error_px)
    message.heading_error_deg = _finite_or_nan(seam.heading_error_deg)
    message.seam_width_px = _finite_or_nan(seam.seam_width_px)
    message.lost_reason = seam.lost_reason or ""
    message.processing_time_ms = seam.processing_time_ms
    return message


def defect_array_message(result: VisionResult, header: Header) -> DefectArray:
    return defect_results_message(result.defect_results, header)


def defect_results_message(defects: list[DefectResult], header: Header) -> DefectArray:
    array = DefectArray()
    array.header = header
    for item in defects:
        message = Defect()
        message.defect_id = item.defect_id
        message.class_name = item.class_name
        message.confidence = item.confidence
        message.bbox_xyxy = list(item.bbox_xyxy)
        message.polygon = [Point32(x=float(x), y=float(y), z=0.0) for x, y in item.polygon or []]
        message.area_px = item.area_px
        message.centroid_uv = Point32(x=float(item.centroid_uv[0]), y=float(item.centroid_uv[1]), z=0.0)
        message.backend_name = item.backend_name
        array.defects.append(message)
    return array


def quality_message(result: VisionResult, header: Header) -> VisionQuality:
    quality = result.quality_result
    message = VisionQuality()
    message.header = header
    message.mean_brightness = quality.mean_brightness
    message.overexposed_ratio = quality.overexposed_ratio
    message.underexposed_ratio = quality.underexposed_ratio
    message.laplacian_variance = quality.laplacian_variance
    message.contrast_std = quality.contrast_std
    message.valid_image = quality.valid_image
    message.valid_for_measurement = quality.valid_for_measurement
    message.confidence_scale = quality.confidence_scale
    message.warnings = quality.warnings
    return message
