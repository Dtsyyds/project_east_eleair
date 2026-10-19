"""Bounded processed-ROI evidence recorder for field validation.

The recorder is intentionally independent from the tracker algorithm.  It is
created only when explicitly enabled, copies only the ROI of an already
processed frame, and never waits for disk I/O in the processing thread.
"""

from __future__ import annotations

from tank_vision.async_lifecycle import AsyncLifecycle

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
from queue import Full, Queue
import threading
import time
from typing import Any

import numpy as np


def _sha256(path: str | Path) -> str | None:
    try:
        digest = hashlib.sha256()
        with Path(path).open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()
    except (OSError, ValueError):
        return None


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return [_json_safe(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        return value if np.isfinite(value) else None
    return value


@dataclass(slots=True)
class ROIFrameTask:
    image: np.ndarray
    metadata: dict[str, Any]


class ROIValidationRecorder:
    """Write processed BGR8 ROIs and per-frame metadata asynchronously."""

    def __init__(self, output_root: str | Path, config_path: str | Path,
                 reference_path: str | Path, queue_size: int = 8,
                 max_duration_sec: float = 0.0) -> None:
        if int(queue_size) < 1:
            raise ValueError("roi_validation_queue_size must be >= 1")
        if float(max_duration_sec) < 0.0:
            raise ValueError("roi_validation_max_duration_sec must be >= 0")

        root = Path(output_root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
        session = root / stamp
        suffix = 1
        while session.exists():
            session = root / f"{stamp}_{suffix:02d}"
            suffix += 1
        session.mkdir()

        self.session = session
        self.raw_path = session / "roi_frames.raw"
        self.frames_path = session / "frames.jsonl"
        self.manifest_path = session / "manifest.json"
        self.complete_path = session / "complete.json"
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.reference_path = str(Path(reference_path).expanduser().resolve())
        self.config_sha256 = _sha256(self.config_path)
        self.reference_sha256 = _sha256(self.reference_path)
        self.queue: Queue[ROIFrameTask | None] = Queue(maxsize=int(queue_size))
        self.max_duration_sec = float(max_duration_sec)
        self.started_monotonic = time.monotonic()
        self._lock = threading.Lock()
        self._accepting = True
        self._closed = False
        self._queue_dropped = 0
        self._discarded_after_error = 0
        self._write_errors: list[str] = []
        self._offset_discontinuities = 0
        self._frames_written = 0
        self._raw_bytes = 0
        self._frozen_roi: tuple[int, int, int, int] | None = None
        self._input_full_shape: tuple[int, ...] | None = None
        self._duration_exceeded = False
        if self.config_sha256 is None:
            self._write_errors.append("config_sha256_failed")
        if self.reference_sha256 is None:
            self._write_errors.append("reference_sha256_failed")
        self._raw_handle = None
        self._frames_handle = None
        self._lifecycle = AsyncLifecycle(self.queue, self._writer_loop, "stable-seam-roi-writer", self._finish_writer)
        self._thread = self._lifecycle.thread

        self.raw_path.touch()
        self.frames_path.touch()
        self._write_manifest(recording=True, complete=False)
        self._lifecycle.start()

    @property
    def queue_dropped(self) -> int:
        return self._queue_dropped

    @property
    def frames_written(self) -> int:
        return self._frames_written

    @property
    def session_directory(self) -> str:
        return str(self.session)

    def _mark_error(self, reason: str) -> None:
        with self._lock:
            self._write_errors.append(str(reason))

    def submit(self, image: Any, roi: tuple[int, int, int, int], metadata: dict[str, Any]) -> bool:
        """Copy one processed ROI and enqueue it without waiting for disk."""
        with self._lock:
            if not self._accepting or self._closed or self._lifecycle.state != "RUNNING":
                return False
            if self.max_duration_sec > 0 and time.monotonic() - self.started_monotonic >= self.max_duration_sec:
                self._accepting = False
                self._duration_exceeded = True
                return False

        if not isinstance(image, np.ndarray) or image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            self._mark_error("input_is_not_bgr8")
            return False
        x, y, width, height = (int(value) for value in roi)
        if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > image.shape[1] or y + height > image.shape[0]:
            self._mark_error(f"roi_out_of_bounds:{(x, y, width, height)}:{tuple(image.shape)}")
            return False

        frozen_roi = (x, y, width, height)
        with self._lock:
            if self._frozen_roi is None:
                self._frozen_roi = frozen_roi
                self._input_full_shape = tuple(int(value) for value in image.shape)
            elif self._frozen_roi != frozen_roi:
                self._write_errors.append(f"roi_changed:{self._frozen_roi}->{frozen_roi}")
                return False

        # This is the only copy performed for an ordinary accepted frame when
        # this recorder is enabled.  The tracker has already finished using
        # the input view before submit() is called.
        roi_copy = np.ascontiguousarray(image[y:y + height, x:x + width]).copy()
        task = ROIFrameTask(image=roi_copy, metadata=dict(metadata))
        try:
            if self._lifecycle.submit(task, "drop_newest"): return True
            with self._lock: self._queue_dropped += 1
            return False
        except Full:
            with self._lock:
                self._queue_dropped += 1
            return False

    def _write_manifest(self, recording: bool, complete: bool, counters: dict[str, Any] | None = None) -> None:
        manifest = {
            "schema_version": "1.0",
            "recording": bool(recording),
            "complete": bool(complete),
            "valid_for_formal_validation": bool(complete),
            "storage_mode": "roi",
            "raw_file": self.raw_path.name,
            "frames_file": self.frames_path.name,
            "config": self.config_path,
            "reference": self.reference_path,
            "config_sha256": self.config_sha256,
            "reference_sha256": self.reference_sha256,
            "roi_frozen": self._frozen_roi is not None,
            "roi_original_px": list(self._frozen_roi) if self._frozen_roi is not None else None,
            "input_full_shape": list(self._input_full_shape) if self._input_full_shape is not None else None,
            "encoding": "bgr8",
            "dtype": "uint8",
            "queue_size": self.queue.maxsize,
            "max_duration_sec": self.max_duration_sec,
            "counters": counters or {},
        }
        self.manifest_path.write_text(json.dumps(_json_safe(manifest), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    @property
    def state(self):
        return self._lifecycle.state

    def _writer_loop(self, task) -> None:
        try:
            if self._write_errors:
                self._discarded_after_error += 1
                raise OSError("recording already has evidence errors")
            if self._raw_handle is None:
                self._raw_handle = self.raw_path.open("ab")
                self._frames_handle = self.frames_path.open("a", encoding="utf-8")
            image = task.image
            length = int(image.nbytes)
            if image.dtype != np.uint8 or not image.flags.c_contiguous:
                raise ValueError("roi_payload_not_contiguous_bgr8")
            payload_offset = self._raw_bytes
            written = self._raw_handle.write(memoryview(image).cast("B"))
            if written != length: raise OSError(f"short_raw_write:{written}!={length}")
            frame = {"offset": payload_offset, "length": length,
                "shape": [int(value) for value in image.shape], "dtype": str(image.dtype), **task.metadata}
            self._frames_handle.write(json.dumps(_json_safe(frame), ensure_ascii=False, allow_nan=False) + "\n")
            self._raw_bytes += length
            self._frames_written += 1
        except Exception as exc:
            self._mark_error(f"writer_exception:{exc}")
            raise

    def _finish_writer(self):
        import os
        for handle in [self._raw_handle, self._frames_handle]:
            if handle is not None:
                handle.flush()
                os.fsync(handle.fileno())
                handle.close()

    def close(self, timeout: float = 30.0) -> dict[str, Any]:
        deadline = time.monotonic() + max(0., timeout)
        with self._lock:
            if self._closed: return self.summary
            self._closed = True
            self._accepting = False
        try:
            self._lifecycle.close(max(0., deadline-time.monotonic()))
        except (RuntimeError, TimeoutError) as exc:
            self._mark_error(str(exc))
        self._discarded_after_error += self._lifecycle.abandoned
        try:
            return self._lifecycle.finalize_io(self._finalize_metadata, deadline, self._mark_error)
        except (OSError, RuntimeError, TimeoutError) as exc:
            self._mark_error(str(exc))
            self.summary = {"session": str(self.session), "complete": False, "close_error": str(exc),
                "counters": {"lifecycle": self._lifecycle.status,
                    "unwritten_tasks": self.queue.unfinished_tasks + self._lifecycle.abandoned + self._lifecycle.failed,
                    "write_errors": list(self._write_errors)}}
            return self.summary

    def _finalize_metadata(self):
        counters = {
            "lifecycle": self._lifecycle.status,
            "unwritten_tasks": self._lifecycle.queue.unfinished_tasks + self._lifecycle.abandoned + self._lifecycle.failed,
            "frames_written": self._frames_written,
            "raw_bytes": self._raw_bytes,
            "queue_dropped": self._queue_dropped,
            "discarded_after_error": self._discarded_after_error,
            "offset_discontinuities": self._offset_discontinuities,
            "write_errors": list(self._write_errors),
            "duration_exceeded": self._duration_exceeded,
            "elapsed_sec": time.monotonic() - self.started_monotonic,
        }
        complete = not self._write_errors and self._queue_dropped == 0 and self._offset_discontinuities == 0 and self._thread.is_alive() is False and self._lifecycle.state == "CLOSED"
        self._write_manifest(recording=False, complete=complete, counters=counters)
        # complete.json is deliberately the final evidence file written.
        self.complete_path.write_text(json.dumps(_json_safe({
            "recording": False,
            "complete": complete,
            "valid_for_formal_validation": complete,
            "counters": counters,
            "manifest": self.manifest_path.name,
        }), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.summary = {"session": str(self.session), "complete": complete, "counters": counters}
        return self.summary

    @property
    def summary(self) -> dict[str, Any]:
        value = getattr(self, "_summary", {"session": str(self.session), "complete": False})
        if self._lifecycle.state == "FAILED":
            return {**value, "complete": False, "lifecycle": self._lifecycle.status}
        return value

    @summary.setter
    def summary(self, value: dict[str, Any]) -> None:
        self._summary = value
