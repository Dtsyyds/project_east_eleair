"""ROS-independent result types for the stable seam production candidate."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any


Point = tuple[float, float]


class TrackingState(IntEnum):
    LOST = 0
    ACQUIRING = 1
    TRACKING = 2
    DEGRADED = 3


@dataclass(slots=True)
class StableSeamResult:
    detected: bool = False
    predicted_only: bool = False
    tracking_state: TrackingState = TrackingState.LOST
    confidence: float = 0.0
    left_boundary_original_px: list[Point] = field(default_factory=list)
    right_boundary_original_px: list[Point] = field(default_factory=list)
    centerline_original_px: list[Point] = field(default_factory=list)
    median_width_px: float | None = None
    width_profile_px: list[float] = field(default_factory=list)
    lateral_error_px: float | None = None
    heading_error_deg: float | None = None
    roi_original_px: tuple[int, int, int, int] = (0, 0, 0, 0)
    coverage: float = 0.0
    effective_span: float = 0.0
    maximum_missing_run: int = 0
    abrupt_left_jump_count: int = 0
    abrupt_right_jump_count: int = 0
    selected_pair_crossing_count: int = 0
    fitted_output_crossing_count: int = 0
    minimum_fitted_width_px: float | None = None
    fit_inlier_ratio: float = 0.0
    fit_residual_px: float | None = None
    processing_ms: float = 0.0
    stage_timings_ms: dict[str, float] = field(default_factory=dict)
    failure_reasons: list[str] = field(default_factory=list)
    accepted_scan_bands: int = 0
    rejected_scan_bands: int = 0

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["tracking_state"] = self.tracking_state.name
        return value
