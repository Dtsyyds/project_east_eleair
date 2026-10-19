"""Low-latency weld tracking path with latest-frame queue semantics."""

from __future__ import annotations

import logging
from queue import Empty, Full, Queue
import threading
import time
from typing import Callable

import numpy as np

from tank_vision.calibration import Undistorter, load_camera_calibration
from tank_vision.async_lifecycle import AsyncLifecycle
from tank_vision.config import AppConfig
from tank_vision.metrics import FrameMetrics, PipelineStats, StageTiming, StageTimer
from tank_vision.preprocessing import assess_image_quality
from tank_vision.seam import ClassicalSeamDetector
from tank_vision.types import FrameData, RealtimeResult, SeamResult

LOGGER = logging.getLogger(__name__)


class RealtimeSeamPipeline:
    """Process only image quality and seam geometry; never performs defect tiling or disk I/O."""

    def __init__(
        self,
        config: AppConfig,
        processor: Callable[[FrameData], RealtimeResult] | None = None,
    ) -> None:
        self.config = config
        self._custom_processor = processor
        undistorter = None
        if config.pipeline.use_undistort:
            if not config.camera.calibration_file:
                raise ValueError("Undistortion is enabled but camera.calibration_file is not configured")
            undistorter = Undistorter(load_camera_calibration(config.camera.calibration_file))
        if config.seam.detector != "classical":
            raise ValueError(f"Unsupported seam detector: {config.seam.detector}")
        self.seam_detector = ClassicalSeamDetector(config.seam, config.pipeline, undistorter)
        self.stats = PipelineStats()
        self.input_frames = 0
        self.output_frames = 0
        self._input: Queue[tuple[FrameData, int] | None] = Queue(config.realtime_seam.queue_size)
        self._output: Queue[RealtimeResult] = Queue(max(2, config.realtime_seam.queue_size))
        self.output_dropped_frames = 0
        self._thread: threading.Thread | None = None
        self._lifecycle = AsyncLifecycle(self._input, self._worker, "realtime-seam")
        self._closing = threading.Event()
        self._rate_started_ns: int | None = None
        self._first_input_ns: int | None = None
        self._last_input_ns: int | None = None
        self._first_output_ns: int | None = None
        self._last_output_ns: int | None = None

    @property
    def last_seam_mask(self) -> np.ndarray | None:
        return self.seam_detector.last_debug_mask

    def _apply_camera_metadata(self, frame: FrameData) -> None:
        if frame.exposure_us is None:
            frame.exposure_us = self.config.camera.exposure_us
        if frame.gain_db is None:
            frame.gain_db = self.config.camera.gain_db
        if frame.calibration_version is None:
            frame.calibration_version = self.config.camera.calibration_version

    def _process_internal(self, frame: FrameData, accepted_ns: int) -> RealtimeResult:
        if self._custom_processor is not None:
            result = self._custom_processor(frame)
            if result.metrics.timing.total_end_to_end_ms <= 0:
                result.metrics.timing.total_end_to_end_ms = (time.perf_counter_ns() - accepted_ns) / 1_000_000.0
            self.stats.record(result.metrics.timing, result.metrics.input_queue_depth)
            return result
        algorithm_started = time.perf_counter_ns()
        timing = StageTiming(**frame.metrics.timing.to_dict())
        metrics = FrameMetrics(timing=timing)
        warnings: list[str] = []
        self._apply_camera_metadata(frame)
        with StageTimer(timing, "quality_ms"):
            quality = assess_image_quality(frame.image, self.config.quality)
        warnings.extend(quality.warnings)
        try:
            seam = self.seam_detector.detect(frame)
            timing.merge(self.seam_detector.last_stage_timing)
        except Exception as exc:  # runtime boundary must preserve the tracking loop
            LOGGER.exception("Unexpected seam detector failure")
            self.stats.mark_failed()
            seam = SeamResult(
                timestamp_ns=frame.timestamp_ns,
                detected=False,
                confidence=0.0,
                lost_reason=f"unexpected_seam_error:{type(exc).__name__}",
            )
        if quality.confidence_scale < 1.0:
            seam.confidence = float(np.clip(seam.confidence * quality.confidence_scale, 0.0, 1.0))
            if seam.detected and seam.confidence < self.config.seam.min_confidence:
                seam.detected = False
                seam.lost_reason = "quality_degraded_below_confidence_threshold"
        timing.total_algorithm_ms = (time.perf_counter_ns() - algorithm_started) / 1_000_000.0
        timing.total_end_to_end_ms = (time.perf_counter_ns() - accepted_ns) / 1_000_000.0
        result = RealtimeResult(frame, seam, quality, metrics, warnings)
        self.stats.record(timing, metrics.input_queue_depth)
        return result

    def process(self, frame: FrameData) -> RealtimeResult:
        """Deterministic synchronous mode; exactly one result per call."""

        return self._process_internal(frame, time.perf_counter_ns())

    @property
    def state(self):
        return self._lifecycle.state

    @property
    def lifecycle_status(self):
        return self._lifecycle.status

    def start(self) -> None:
        self._lifecycle.start()
        self._thread = self._lifecycle.thread

    def submit(self, frame: FrameData) -> bool:
        """Accept a live frame; ``drop_oldest`` ensures queued stale work is removed."""

        self.start()
        accepted_ns = time.perf_counter_ns()
        if self._first_input_ns is None:
            self._first_input_ns = accepted_ns
        self._last_input_ns = accepted_ns
        self.input_frames += 1
        item = (frame, accepted_ns)
        accepted = self._lifecycle.submit(item, self.config.realtime_seam.drop_policy,
                                          self.stats.mark_dropped)
        self.stats.observe_queue_depth(self._input.qsize())
        return accepted

    def _worker(self, item) -> None:
        frame, accepted_ns = item
        result = self._process_internal(frame, accepted_ns)
        result.metrics.input_queue_depth = self._input.qsize()
        output_ns = time.perf_counter_ns()
        if self._first_output_ns is None:
            self._first_output_ns = output_ns
        self._last_output_ns = output_ns
        self.output_frames += 1
        # Live output follows the explicit latest-frame policy; offline block keeps all results.
        dropped = self._lifecycle.publish(self._output, result,
            latest=self.config.realtime_seam.drop_policy != "block")
        self.output_dropped_frames += dropped

    def get_result(self, timeout: float | None = None) -> RealtimeResult:
        return self._lifecycle.get_result(self._output, timeout)

    def flush(self, timeout: float = 10.0) -> None:
        self._lifecycle.flush(timeout)

    def close(self, timeout: float = 5.0) -> None:
        self._lifecycle.close(timeout)

    @property
    def rates(self) -> dict[str, float]:
        def frequency(count: int, first: int | None, last: int | None) -> float:
            if count < 2 or first is None or last is None or last <= first:
                return 0.0
            return (count - 1) / ((last - first) / 1_000_000_000.0)

        return {
            "input_hz": frequency(self.input_frames, self._first_input_ns, self._last_input_ns),
            "output_hz": frequency(self.output_frames, self._first_output_ns, self._last_output_ns),
        }

    def __enter__(self) -> "RealtimeSeamPipeline":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
