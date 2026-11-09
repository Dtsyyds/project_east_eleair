"""Bounded asynchronous result writer with failure containment."""

from __future__ import annotations

import atexit
from collections import deque
import logging
from pathlib import Path
from queue import Empty, Full, Queue
import shutil
import threading
import time

import numpy as np
import cv2

from tank_vision.async_lifecycle import AsyncLifecycle
from tank_vision.config import AppConfig, PipelineConfig, StorageConfig
from tank_vision.metrics import PipelineStats
from tank_vision.outputs.result_writer import ResultWriter
from tank_vision.types import VisionResult

LOGGER = logging.getLogger(__name__)


class AsyncResultWriter:
    """Disk I/O worker; accepted tasks are drained by :meth:`flush`/:meth:`close`."""

    def __init__(
        self,
        output_dir: str | Path,
        config: AppConfig | PipelineConfig,
        storage: StorageConfig | None = None,
    ) -> None:
        if isinstance(config, AppConfig):
            self.pipeline_config = config.pipeline
            self.storage = config.storage
        else:
            self.pipeline_config = config
            self.storage = storage or StorageConfig()
        self.output_dir = Path(output_dir)
        self._writer = ResultWriter(self.output_dir, self.pipeline_config)
        self.raw_dir = self.output_dir / "raw"
        self.raw_dir.mkdir(parents=True, exist_ok=True)
        self._queue: Queue[tuple[VisionResult, np.ndarray | None, int] | None] = Queue(
            self.storage.output_queue_size
        )
        self._lifecycle = AsyncLifecycle(self._queue, self._worker, "async-result-writer", self._writer.close)
        self._thread = self._lifecycle.thread
        self._closed = False
        self.stats = PipelineStats()
        self.errors: list[str] = []
        self._pre_event: deque[tuple[str, bytes]] = deque(maxlen=self.storage.pre_event_frames)
        self._post_event_remaining = 0
        self._lifecycle.start()
        from tank_vision.async_status import write_status
        write_status(self.output_dir / "writer_status.json", {**self.lifecycle_status,
            "status_time_basis": "initial_incomplete_snapshot"})
        atexit.register(self.close)

    def submit(self, result: VisionResult, defect_mask: np.ndarray | None = None) -> bool:
        if self._closed:
            raise RuntimeError("AsyncResultWriter is closed")
        item = (result, defect_mask, time.perf_counter_ns())
        accepted = self._lifecycle.submit(item, self.storage.drop_policy, self.stats.mark_dropped)
        self.stats.observe_queue_depth(self._queue.qsize())
        return accepted

    @property
    def state(self):
        return self._lifecycle.state

    @property
    def lifecycle_status(self):
        return self._lifecycle.status

    def _low_disk_space(self) -> bool:
        threshold = self.storage.disk_low_space_threshold_mb
        if threshold <= 0:
            return False
        try:
            free_mb = shutil.disk_usage(self.output_dir).free / (1024 * 1024)
        except OSError as exc:
            self.errors.append(f"disk_space_check_failed:{type(exc).__name__}")
            return False
        return free_mb < threshold

    def _worker(self, item) -> None:
        result, mask, enqueued_ns = item
        result.metrics.timing.writer_queue_ms += (time.perf_counter_ns() - enqueued_ns) / 1_000_000.0
        try:
            if self._low_disk_space():
                self.stats.mark_skipped()
                raise OSError("recording_stopped:disk_low_space")
            self._record_raw(result)
            self._writer.write(result, mask)
            self.stats.record(result.metrics.timing, self._queue.qsize())
        except Exception as exc:
            warning = f"writer_failed:{type(exc).__name__}:{exc}"
            result.warnings.append(warning)
            self.errors.append(warning)
            self.stats.mark_failed()
            raise

    def _encode_raw(self, result: VisionResult) -> tuple[str, bytes] | None:
        image = result.frame.image
        if not isinstance(image, np.ndarray) or image.size == 0:
            return None
        extension = {"jpg": ".jpg", "png": ".png", "none": ".bmp"}[self.storage.compression]
        started_ns = time.perf_counter_ns()
        ok, encoded = cv2.imencode(extension, image)
        result.metrics.timing.add_ns("serialization_ms", time.perf_counter_ns() - started_ns)
        if not ok:
            result.warnings.append("raw_encode_failed")
            return None
        return f"{ResultWriter._stem(result)}{extension}", encoded.tobytes()

    def _write_raw_bytes(self, name: str, payload: bytes, result: VisionResult) -> None:
        if (self.raw_dir / name).exists():
            return
        started_ns = time.perf_counter_ns()
        (self.raw_dir / name).write_bytes(payload)
        result.metrics.timing.add_ns("disk_write_ms", time.perf_counter_ns() - started_ns)

    def _record_raw(self, result: VisionResult) -> None:
        policy_needs_frame = (
            self.storage.save_all_raw
            or self.storage.save_abnormal_raw
            or result.frame.sequence_id % self.storage.normal_frame_interval == 0
            or self._post_event_remaining > 0
        )
        encoded = self._encode_raw(result) if policy_needs_frame else None
        abnormal = bool(result.defect_results) or not result.seam_result.detected
        if encoded is not None:
            name, payload = encoded
            if self.storage.save_all_raw:
                self._write_raw_bytes(name, payload, result)
            elif abnormal and self.storage.save_abnormal_raw:
                for buffered_name, buffered_payload in self._pre_event:
                    self._write_raw_bytes(buffered_name, buffered_payload, result)
                self._write_raw_bytes(name, payload, result)
                self._post_event_remaining = self.storage.post_event_frames
            elif self._post_event_remaining > 0:
                self._write_raw_bytes(name, payload, result)
                self._post_event_remaining -= 1
            elif result.frame.sequence_id % self.storage.normal_frame_interval == 0:
                self._write_raw_bytes(name, payload, result)
            if self.storage.save_abnormal_raw and self.storage.pre_event_frames > 0:
                self._pre_event.append((name, payload))

    def flush(self, timeout: float = 10.0) -> None:
        self._lifecycle.flush(timeout)

    def close(self, timeout: float = 10.0) -> None:
        deadline = time.monotonic() + max(0., timeout)
        self._closed = True
        try:
            # Reserve a small part of the same deadline for failure-status I/O.
            reserve = min(.01, max(0., timeout)/4)
            self._lifecycle.close(max(0., deadline-time.monotonic()-reserve))
        finally:
            if not self._thread.is_alive():
                atexit.unregister(self.close)
            from tank_vision.async_status import write_status
            self._lifecycle.finalize_io(
                lambda: write_status(self.output_dir / "writer_status.json", self.lifecycle_status), deadline,
                self.errors.append)

    def __enter__(self) -> "AsyncResultWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
