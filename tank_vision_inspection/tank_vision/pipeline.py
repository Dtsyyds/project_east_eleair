"""Compatibility pipeline composed from the realtime and inspection paths."""

from __future__ import annotations

import time

from tank_vision import __version__
from tank_vision.config import AppConfig
from tank_vision.inspection_pipeline import InspectionPipeline
from tank_vision.metrics import FrameMetrics, PipelineStats, StageTiming
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.types import FrameData, VisionResult


class VisionPipeline:
    """Deterministic offline entry point; it never drops valid input frames."""

    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.realtime_pipeline = RealtimeSeamPipeline(config)
        self.inspection_pipeline = InspectionPipeline(config)
        # Original public attributes are retained for existing callers/tests.
        self.seam_detector = self.realtime_pipeline.seam_detector
        self.defect_detector = self.inspection_pipeline.defect_detector
        self.stats = PipelineStats()

    @property
    def last_defect_mask(self):
        return self.inspection_pipeline.last_defect_mask

    def process(self, frame: FrameData) -> VisionResult:
        accepted_ns = time.perf_counter_ns()
        realtime = self.realtime_pipeline.process(frame)
        inspection = self.inspection_pipeline.process(frame, self.realtime_pipeline.last_seam_mask)
        timing = StageTiming(**frame.metrics.timing.to_dict())
        for stage in (
            "undistort_ms",
            "quality_ms",
            "roi_ms",
            "preprocess_ms",
            "seam_detect_ms",
            "seam_geometry_ms",
        ):
            setattr(timing, stage, getattr(realtime.metrics.timing, stage))
        for stage in ("tiling_ms", "defect_inference_ms", "postprocess_ms"):
            setattr(timing, stage, getattr(inspection.metrics.timing, stage))
        timing.total_algorithm_ms = (time.perf_counter_ns() - accepted_ns) / 1_000_000.0
        timing.total_end_to_end_ms = timing.total_algorithm_ms
        metrics = FrameMetrics(timing=timing)
        warnings = [*realtime.warnings, *inspection.warnings]
        result = VisionResult(
            frame=frame,
            seam_result=realtime.seam_result,
            defect_results=inspection.defect_results,
            quality_result=realtime.quality_result,
            processing_time_ms=timing.total_algorithm_ms,
            warnings=warnings,
            algorithm_version=__version__,
            config=self.config.to_dict(),
            metrics=metrics,
        )
        self.stats.record(timing)
        return result


__all__ = ["InspectionPipeline", "RealtimeSeamPipeline", "VisionPipeline"]
