"""Monotonic stage timing and bounded pipeline statistics."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import math
import threading
import time
from typing import Iterable


STAGE_NAMES = (
    "source_read_ms",
    "decode_ms",
    "undistort_ms",
    "quality_ms",
    "roi_ms",
    "preprocess_ms",
    "seam_detect_ms",
    "seam_geometry_ms",
    "tiling_ms",
    "defect_inference_ms",
    "postprocess_ms",
    "overlay_ms",
    "serialization_ms",
    "writer_queue_ms",
    "disk_write_ms",
    "total_algorithm_ms",
    "total_end_to_end_ms",
)


@dataclass(slots=True)
class StageTiming:
    """Per-frame timings in milliseconds, measured using ``perf_counter_ns``."""

    source_read_ms: float = 0.0
    decode_ms: float = 0.0
    undistort_ms: float = 0.0
    quality_ms: float = 0.0
    roi_ms: float = 0.0
    preprocess_ms: float = 0.0
    seam_detect_ms: float = 0.0
    seam_geometry_ms: float = 0.0
    tiling_ms: float = 0.0
    defect_inference_ms: float = 0.0
    postprocess_ms: float = 0.0
    overlay_ms: float = 0.0
    serialization_ms: float = 0.0
    writer_queue_ms: float = 0.0
    disk_write_ms: float = 0.0
    total_algorithm_ms: float = 0.0
    total_end_to_end_ms: float = 0.0

    def add_ns(self, stage: str, elapsed_ns: int) -> None:
        if stage not in STAGE_NAMES:
            raise KeyError(f"Unknown timing stage: {stage}")
        setattr(self, stage, getattr(self, stage) + max(0, elapsed_ns) / 1_000_000.0)

    def merge(self, other: "StageTiming") -> None:
        for name in STAGE_NAMES:
            setattr(self, name, getattr(self, name) + getattr(other, name))

    def to_dict(self) -> dict[str, float]:
        return {name: float(value) for name, value in asdict(self).items()}


class StageTimer:
    """Small context manager that accumulates one timing stage."""

    def __init__(self, timing: StageTiming, stage: str) -> None:
        if stage not in STAGE_NAMES:
            raise KeyError(f"Unknown timing stage: {stage}")
        self.timing = timing
        self.stage = stage
        self.started_ns = 0

    def __enter__(self) -> "StageTimer":
        self.started_ns = time.perf_counter_ns()
        return self

    def __exit__(self, *_: object) -> None:
        self.timing.add_ns(self.stage, time.perf_counter_ns() - self.started_ns)


@dataclass(slots=True)
class FrameMetrics:
    """Timing and queue metadata for one accepted frame."""

    timing: StageTiming = field(default_factory=StageTiming)
    input_queue_depth: int = 0
    output_queue_depth: int = 0
    dropped_before_processing: int = 0

    def to_dict(self) -> dict[str, object]:
        return {
            "timing": self.timing.to_dict(),
            "input_queue_depth": int(self.input_queue_depth),
            "output_queue_depth": int(self.output_queue_depth),
            "dropped_before_processing": int(self.dropped_before_processing),
        }


def percentile(values: Iterable[float], percent: float) -> float:
    """Return a linearly interpolated percentile without a NumPy dependency."""

    ordered = sorted(float(value) for value in values if math.isfinite(float(value)))
    if not ordered:
        return 0.0
    if not 0.0 <= percent <= 100.0:
        raise ValueError("percent must be in [0, 100]")
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


@dataclass(slots=True)
class PipelineStats:
    """Thread-safe aggregate counters and latency distributions."""

    processed_frames: int = 0
    skipped_frames: int = 0
    dropped_frames: int = 0
    failed_frames: int = 0
    max_queue_depth: int = 0
    _timings: deque[StageTiming] = field(default_factory=lambda: deque(maxlen=2048), repr=False)
    _sums: dict[str, float] = field(default_factory=lambda: dict.fromkeys(STAGE_NAMES, 0.0), repr=False)
    _maxima: dict[str, float] = field(default_factory=lambda: dict.fromkeys(STAGE_NAMES, 0.0), repr=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def record(self, timing: StageTiming, queue_depth: int = 0) -> None:
        with self._lock:
            self.processed_frames += 1
            self.max_queue_depth = max(self.max_queue_depth, int(queue_depth))
            values = timing.to_dict()
            self._timings.append(StageTiming(**values))
            for name, value in values.items():
                self._sums[name] += value
                self._maxima[name] = max(self._maxima[name], value)

    def mark_skipped(self, count: int = 1) -> None:
        with self._lock:
            self.skipped_frames += max(0, int(count))

    def mark_dropped(self, count: int = 1) -> None:
        with self._lock:
            self.dropped_frames += max(0, int(count))

    def mark_failed(self, count: int = 1) -> None:
        with self._lock:
            self.failed_frames += max(0, int(count))

    def observe_queue_depth(self, depth: int) -> None:
        with self._lock:
            self.max_queue_depth = max(self.max_queue_depth, max(0, int(depth)))

    def summary(self) -> dict[str, object]:
        with self._lock:
            timings = list(self._timings)
            sums, maxima, count = dict(self._sums), dict(self._maxima), self.processed_frames
            counters = {
                "processed_frames": self.processed_frames,
                "skipped_frames": self.skipped_frames,
                "dropped_frames": self.dropped_frames,
                "failed_frames": self.failed_frames,
                "max_queue_depth": self.max_queue_depth,
            }
        stages: dict[str, dict[str, float]] = {}
        for name in STAGE_NAMES:
            values = [getattr(item, name) for item in timings]
            stages[name] = {
                "mean": sums[name] / count if count else 0.0,
                "p50": percentile(values, 50),
                "p90": percentile(values, 90),
                "p95": percentile(values, 95),
                "max": maxima[name],
            }
        algorithm_mean = stages["total_algorithm_ms"]["mean"]
        end_to_end_mean = stages["total_end_to_end_ms"]["mean"]
        return {
            **counters,
            "quantile_basis": "last_2048_frames_linear_interpolation",
            "quantile_sample_count": len(timings),
            "mean_and_max_basis": "all_processed_frames",
            "average_algorithm_fps": 1000.0 / algorithm_mean if algorithm_mean > 0 else 0.0,
            "average_end_to_end_fps": 1000.0 / end_to_end_mean if end_to_end_mean > 0 else 0.0,
            "stages": stages,
        }

