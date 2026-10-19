"""Independent high-resolution defect inspection path."""

from __future__ import annotations

from dataclasses import replace
import logging
from queue import Empty, Full, Queue
import threading
import time
from typing import Callable

import numpy as np

from tank_vision.async_lifecycle import AsyncLifecycle
from tank_vision.config import AppConfig
from tank_vision.defects import ClassicalDefectBaseline, ONNXBackend
from tank_vision.defects.detector import DefectDetector
from tank_vision.metrics import FrameMetrics, PipelineStats, StageTiming
from tank_vision.types import DefectResult, FrameData, InspectionResult, TileTransform

LOGGER = logging.getLogger(__name__)


class _UnavailableModelDetector(DefectDetector):
    backend_name = "onnx_unavailable"

    def __init__(self, backend: ONNXBackend) -> None:
        self.backend = backend
        self.last_warning: str | None = None
        self.last_mask: np.ndarray | None = None
        self.last_stage_timing = StageTiming()
        self.last_tiles: list[object] = []

    def detect(self, frame: FrameData, exclusion_mask: np.ndarray | None = None) -> list[DefectResult]:
        del frame, exclusion_mask
        self.last_warning = f"onnx_detection_skipped:{self.backend.error or 'model_output_adapter_not_configured'}"
        return []


class InspectionPipeline:
    """Tiled inspection with an independent bounded queue and trigger policy."""

    def __init__(
        self,
        config: AppConfig,
        processor: Callable[[FrameData], InspectionResult] | None = None,
    ) -> None:
        self.config = config
        self._custom_processor = processor
        defect_config = replace(
            config.defect,
            tile_size=config.inspection.tile_size,
            overlap=config.inspection.overlap,
        )
        if defect_config.backend == "classical":
            self.defect_detector: DefectDetector = ClassicalDefectBaseline(defect_config)
        elif defect_config.backend == "onnx":
            self.defect_detector = _UnavailableModelDetector(ONNXBackend(defect_config.model_path))
        else:
            raise ValueError(f"Unsupported defect backend: {defect_config.backend}")
        self.stats = PipelineStats()
        self._input: Queue[tuple[FrameData, np.ndarray | None, int] | None] = Queue(config.inspection.queue_size)
        self._output: Queue[InspectionResult] = Queue(max(2, config.inspection.queue_size))
        self._thread: threading.Thread | None = None
        self._lifecycle = AsyncLifecycle(self._input, self._worker, "high-res-inspection")
        self._last_distance_mm: float | None = None

    @property
    def last_defect_mask(self) -> np.ndarray | None:
        return getattr(self.defect_detector, "last_mask", None)

    def should_trigger(
        self,
        frame: FrameData,
        encoder_distance_mm: float | None = None,
        external_event: bool = False,
    ) -> bool:
        if not self.config.inspection.enabled:
            return False
        mode = self.config.inspection.trigger_mode
        if mode == "always":
            return True
        if mode == "frame_interval":
            return frame.sequence_id % self.config.inspection.trigger_every_n_frames == 0
        if mode == "external":
            return external_event
        if encoder_distance_mm is None:
            return False
        if self._last_distance_mm is None:
            self._last_distance_mm = encoder_distance_mm
            return True
        threshold = self.config.inspection.trigger_distance_mm
        if threshold is not None and abs(encoder_distance_mm - self._last_distance_mm) >= threshold:
            self._last_distance_mm = encoder_distance_mm
            return True
        return False

    def _process_internal(
        self,
        frame: FrameData,
        exclusion_mask: np.ndarray | None,
        accepted_ns: int,
    ) -> InspectionResult:
        if self._custom_processor is not None:
            result = self._custom_processor(frame)
            if result.metrics.timing.total_end_to_end_ms <= 0:
                result.metrics.timing.total_end_to_end_ms = (time.perf_counter_ns() - accepted_ns) / 1_000_000.0
            self.stats.record(result.metrics.timing, result.metrics.input_queue_depth)
            return result
        started_ns = time.perf_counter_ns()
        timing = StageTiming(**frame.metrics.timing.to_dict())
        metrics = FrameMetrics(timing=timing)
        warnings: list[str] = []
        try:
            defects = self.defect_detector.detect(frame, exclusion_mask)
            timing.merge(getattr(self.defect_detector, "last_stage_timing", StageTiming()))
        except Exception as exc:
            LOGGER.exception("Unexpected defect detector failure")
            self.stats.mark_failed()
            defects = []
            warnings.append(f"unexpected_defect_error:{type(exc).__name__}")
        detector_warning = getattr(self.defect_detector, "last_warning", None)
        if detector_warning:
            warnings.append(detector_warning)
        transforms = [
            TileTransform(tile.x, tile.y, tile.valid_width, tile.valid_height, tile.image.shape[1], tile.image.shape[0])
            for tile in getattr(self.defect_detector, "last_tiles", [])
        ]
        timing.total_algorithm_ms = (time.perf_counter_ns() - started_ns) / 1_000_000.0
        timing.total_end_to_end_ms = (time.perf_counter_ns() - accepted_ns) / 1_000_000.0
        result = InspectionResult(frame, defects, transforms, metrics, warnings)
        self.stats.record(timing, metrics.input_queue_depth)
        return result

    def process(self, frame: FrameData, exclusion_mask: np.ndarray | None = None) -> InspectionResult:
        if not self.config.inspection.enabled:
            self.stats.mark_skipped()
            return InspectionResult(frame, [])
        return self._process_internal(frame, exclusion_mask, time.perf_counter_ns())

    @property
    def state(self):
        return self._lifecycle.state

    @property
    def lifecycle_status(self):
        return self._lifecycle.status

    def start(self) -> None:
        if not self.config.inspection.enabled:
            return
        self._lifecycle.start()
        self._thread = self._lifecycle.thread

    def submit(
        self,
        frame: FrameData,
        exclusion_mask: np.ndarray | None = None,
        encoder_distance_mm: float | None = None,
        external_event: bool = False,
        force: bool = False,
    ) -> bool:
        if not self.config.inspection.enabled:
            self.stats.mark_skipped()
            return False
        if not force and not self.should_trigger(frame, encoder_distance_mm, external_event):
            self.stats.mark_skipped()
            return False
        self.start()
        item = (frame, exclusion_mask, time.perf_counter_ns())
        accepted = self._lifecycle.submit(item, self.config.inspection.drop_policy,
                                          self.stats.mark_dropped)
        self.stats.observe_queue_depth(self._input.qsize())
        return accepted

    def _worker(self, item) -> None:
        frame, mask, accepted_ns = item
        self._lifecycle.publish(self._output, self._process_internal(frame, mask, accepted_ns))

    def get_result(self, timeout: float | None = None) -> InspectionResult:
        return self._lifecycle.get_result(self._output, timeout)

    def flush(self, timeout: float = 10.0) -> None:
        self._lifecycle.flush(timeout)

    def close(self, timeout: float = 10.0) -> None:
        self._lifecycle.close(timeout)

    def __enter__(self) -> "InspectionPipeline":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
