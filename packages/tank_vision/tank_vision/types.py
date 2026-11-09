"""Strongly typed data exchanged across the inspection pipeline."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, is_dataclass
from pathlib import Path
from typing import Any, Optional

import numpy as np

from tank_vision.metrics import FrameMetrics


Point = tuple[float, float]
BBox = tuple[int, int, int, int]


def to_json_safe(value: Any) -> Any:
    """Recursively convert dataclasses and NumPy values into JSON-safe values."""
    if is_dataclass(value):
        return to_json_safe(asdict(value))
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): to_json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_json_safe(item) for item in value]
    return value


@dataclass(slots=True)
class FrameData:
    image: np.ndarray
    timestamp_ns: int
    frame_id: str
    sequence_id: int
    source_name: str
    exposure_us: Optional[float] = None
    gain_db: Optional[float] = None
    analog_gain_raw: Optional[float] = None
    exposure_mode: Optional[str] = None
    calibration_version: Optional[str] = None
    original_filename: Optional[str] = None
    metrics: FrameMetrics = field(default_factory=FrameMetrics)
    raw_mcap_uri: Optional[str] = None

    @property
    def image_size(self) -> tuple[int, int]:
        if not isinstance(self.image, np.ndarray) or self.image.ndim < 2:
            return (0, 0)
        height, width = self.image.shape[:2]
        return (width, height)

    def metadata_dict(self) -> dict[str, Any]:
        width, height = self.image_size
        return {
            "timestamp_ns": int(self.timestamp_ns),
            "frame_id": self.frame_id,
            "sequence_id": int(self.sequence_id),
            "source_name": self.source_name,
            "exposure_us": self.exposure_us,
            "gain_db": self.gain_db,
            "analog_gain_raw": self.analog_gain_raw,
            "exposure_mode": self.exposure_mode,
            "calibration_version": self.calibration_version,
            "original_filename": self.original_filename,
            "image_size": {"width": width, "height": height},
            "source_timing": self.metrics.timing,
            "raw_mcap_uri": self.raw_mcap_uri,
        }


@dataclass(slots=True)
class QualityResult:
    mean_brightness: float = 0.0
    overexposed_ratio: float = 0.0
    underexposed_ratio: float = 0.0
    laplacian_variance: float = 0.0
    contrast_std: float = 0.0
    motion_blur_score: Optional[float] = None
    valid_image: bool = True
    valid_for_measurement: bool = True
    confidence_scale: float = 1.0
    warnings: list[str] = field(default_factory=list)


@dataclass(slots=True)
class SeamResult:
    timestamp_ns: int
    detected: bool
    confidence: float
    centerline_points: list[Point] = field(default_factory=list)
    lateral_error_px: Optional[float] = None
    heading_error_deg: Optional[float] = None
    seam_width_px: Optional[float] = None
    lost_reason: Optional[str] = None
    processing_time_ms: float = 0.0
    inlier_ratio: float = 0.0
    fit_residual_px: Optional[float] = None


@dataclass(slots=True)
class DefectResult:
    defect_id: str
    class_name: str
    confidence: float
    bbox_xyxy: BBox
    polygon: Optional[list[Point]]
    area_px: float
    centroid_uv: Point
    backend_name: str


@dataclass(slots=True)
class VisionResult:
    frame: FrameData
    seam_result: SeamResult
    defect_results: list[DefectResult]
    quality_result: QualityResult
    processing_time_ms: float
    warnings: list[str] = field(default_factory=list)
    algorithm_version: str = "0.1.0"
    config: dict[str, Any] = field(default_factory=dict)
    metrics: FrameMetrics = field(default_factory=FrameMetrics)

    def to_dict(self) -> dict[str, Any]:
        return to_json_safe(
            {
                "original_filename": self.frame.original_filename,
                "timestamp_ns": self.frame.timestamp_ns,
                "frame": self.frame.metadata_dict(),
                "image_size": self.frame.metadata_dict()["image_size"],
                "seam_result": self.seam_result,
                "defect_results": self.defect_results,
                "quality_result": self.quality_result,
                "processing_time_ms": self.processing_time_ms,
                "warnings": self.warnings,
                "algorithm_version": self.algorithm_version,
                "config": self.config,
                "metrics": self.metrics.to_dict(),
            }
        )


@dataclass(slots=True)
class RealtimeResult:
    frame: FrameData
    seam_result: SeamResult
    quality_result: QualityResult
    metrics: FrameMetrics = field(default_factory=FrameMetrics)
    warnings: list[str] = field(default_factory=list)


@dataclass(slots=True)
class TileTransform:
    x: int
    y: int
    valid_width: int
    valid_height: int
    tile_width: int
    tile_height: int


@dataclass(slots=True)
class InspectionResult:
    frame: FrameData
    defect_results: list[DefectResult]
    tile_transforms: list[TileTransform] = field(default_factory=list)
    metrics: FrameMetrics = field(default_factory=FrameMetrics)
    warnings: list[str] = field(default_factory=list)


@dataclass(slots=True)
class LaserLineResult:
    timestamp_ns: int
    detected: bool
    points_uv: list[Point] = field(default_factory=list)
    confidence: float = 0.0
    reason: Optional[str] = None


@dataclass(slots=True)
class Profile3DResult:
    timestamp_ns: int
    calibrated: bool
    points_xyz: list[tuple[float, float, float]] = field(default_factory=list)
    reason: Optional[str] = None
