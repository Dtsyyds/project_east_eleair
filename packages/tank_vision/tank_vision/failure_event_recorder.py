"""Asynchronous, bounded live-session metrics and failure-event storage.

This module deliberately sits outside :mod:`stable_seam_tracker`.  It owns no
detector state and never changes a result.  The node submits small JSON rows
and, only for selected events, an owned image copy.  Disk work is performed by
two bounded writer threads so a slow eMMC cannot block camera acquisition or
tracker processing.
"""

from __future__ import annotations

from tank_vision.async_lifecycle import AsyncLifecycle

from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
from queue import Empty, Full, PriorityQueue
import shutil
import threading
import time
from typing import Any, Callable
import uuid

import cv2
import numpy as np

from tank_vision.seam_tracking_types import StableSeamResult
from tank_vision.snapshot_writer import sha256_file


FAILURE_REASON_TEXT = {
    "insufficient_points": "路径点数量不足",
    "coverage_below_minimum": "可靠路径覆盖率不足",
    "effective_span_below_minimum": "有效纵向跨度不足",
    "missing_run_too_long": "连续缺失扫描带过长",
    "too_many_abrupt_jumps": "边界突跳数量过多",
    "fit_failed": "中心/宽度拟合失败",
    "inlier_ratio_below_minimum": "左右边界不能被同一中心/宽度模型稳定解释",
    "fit_residual_above_maximum": "拟合残差过大",
    "path_confidence_below_minimum": "路径综合置信度不足",
    "fitted_boundaries_invalid_or_outside_roi": "拟合边界超出ROI或无效",
    "temporal_lateral_jump": "相邻帧横向位置突变过大",
    "temporal_width_jump": "相邻帧焊缝宽度变化过大",
    "temporal_heading_jump": "相邻帧方向变化过大",
    "boundary_crossing": "左右边界发生交叉",
    "input_or_processing_error": "输入或处理异常",
}


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


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


def write_json(path: str | Path, value: Any) -> None:
    Path(path).write_text(
        json.dumps(_json_safe(value), ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def _atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
    write_json(temporary, value)
    os.replace(temporary, path)


def gate_evidence(config: dict[str, Any], result: StableSeamResult) -> dict[str, Any]:
    """Return an honest gate table from the result available with trace off.

    Path confidence and separate fit ratios remain unavailable on the fast
    path.  Crossing counts are now computed by the tracker in O(N) without
    enabling full provenance, so they are recorded directly when available.
    """
    geometry = config["geometry"]
    reasons = list(result.failure_reasons)
    point = {"observed": int(result.accepted_scan_bands),
             "threshold": int(geometry["minimum_point_count"])}
    checks: dict[str, dict[str, Any]] = {
        "point_count": {**point, "pass": point["observed"] >= point["threshold"]},
        "coverage": {"observed": float(result.coverage), "threshold": float(geometry["minimum_coverage"]),
                      "pass": float(result.coverage) >= float(geometry["minimum_coverage"])},
        "effective_span": {"observed": float(result.effective_span), "threshold": float(geometry["minimum_effective_span"]),
                            "pass": float(result.effective_span) >= float(geometry["minimum_effective_span"])},
        "missing_run": {"observed": int(result.maximum_missing_run), "threshold": int(geometry["maximum_missing_run"]),
                         "pass": int(result.maximum_missing_run) <= int(geometry["maximum_missing_run"])},
        "abrupt_left_jump": {"observed": int(result.abrupt_left_jump_count), "threshold": int(geometry["maximum_abrupt_jump_count"]),
                              "pass": int(result.abrupt_left_jump_count) <= int(geometry["maximum_abrupt_jump_count"])},
        "abrupt_right_jump": {"observed": int(result.abrupt_right_jump_count), "threshold": int(geometry["maximum_abrupt_jump_count"]),
                               "pass": int(result.abrupt_right_jump_count) <= int(geometry["maximum_abrupt_jump_count"])},
        "fit_success": {"observed": result.fit_residual_px is not None,
                         "threshold": True, "pass": result.fit_residual_px is not None},
        "joint_inlier": {"observed": float(result.fit_inlier_ratio), "threshold": float(geometry["minimum_inlier_ratio"]),
                          "pass": float(result.fit_inlier_ratio) >= float(geometry["minimum_inlier_ratio"])},
        "fit_residual": {"observed": _finite(result.fit_residual_px), "threshold": float(geometry["maximum_fit_residual_px"]),
                          "pass": result.fit_residual_px is not None and float(result.fit_residual_px) <= float(geometry["maximum_fit_residual_px"])},
        "path_confidence": {"observed": None, "threshold": float(geometry["minimum_path_confidence"]),
                            "pass": None, "unavailable": "trace disabled in realtime path"},
        "positive_width": {"observed": _finite(result.median_width_px), "threshold": 0.0,
                            "pass": None if result.median_width_px is None else float(result.median_width_px) > 0},
        "inside_roi": {"observed": bool(result.left_boundary_original_px and result.right_boundary_original_px),
                        "threshold": True,
                        "pass": None if not result.left_boundary_original_px else _boundaries_inside_roi(result)},
        "non_crossing": {"observed": int(result.fitted_output_crossing_count), "threshold": 0,
                          "pass": int(result.fitted_output_crossing_count) == 0},
        "temporal": {"observed": reasons, "threshold": "no temporal_* failure",
                      "pass": not any(str(reason).startswith("temporal_") for reason in reasons)},
    }
    return {"checks": checks, "all_pass": all(item["pass"] is True for item in checks.values()),
            "failure_reasons": reasons}


def _boundaries_inside_roi(result: StableSeamResult) -> bool:
    x, y, width, height = result.roi_original_px
    points = result.left_boundary_original_px + result.right_boundary_original_px + result.centerline_original_px
    return all(x <= px < x + width and y <= py < y + height for px, py in points)


def human_readable_reason(config: dict[str, Any], result: StableSeamResult) -> str:
    geometry = config["geometry"]
    if result.failure_reasons:
        parts: list[str] = []
        for reason in result.failure_reasons:
            if reason == "inlier_ratio_below_minimum":
                parts.append(f"联合内点率 {result.fit_inlier_ratio:.2f} < {float(geometry['minimum_inlier_ratio']):.2f}")
            elif reason == "fit_residual_above_maximum" and result.fit_residual_px is not None:
                parts.append(f"拟合残差 {result.fit_residual_px:.1f} px > {float(geometry['maximum_fit_residual_px']):.1f} px")
            else:
                parts.append(FAILURE_REASON_TEXT.get(reason, str(reason)))
        return "；".join(parts)
    if result.detected and result.tracking_state.name == "TRACKING":
        return "当前帧通过并处于TRACKING"
    if result.detected and result.tracking_state.name == "ACQUIRING":
        return "单帧几何通过，正在等待连续3帧进入TRACKING"
    if result.predicted_only:
        return "检测失败，当前输出为预测/降级结果"
    return f"当前帧状态为{result.tracking_state.name}"


@dataclass(slots=True)
class FailureEventTask:
    image: np.ndarray
    header: Any
    result: StableSeamResult
    geometry: dict[str, Any]
    gate: dict[str, Any]
    metrics: dict[str, Any]
    trigger_reason: str
    automatic: bool
    source_mode: str
    config_path: str
    reference_path: str
    config_sha256: str | None
    reference_sha256: str | None
    save_full_frame: bool
    save_context_jpeg: bool
    context_jpeg_quality: int
    save_lossless_roi: bool
    event_id: str


class LiveSessionRecorder:
    """Bounded JSONL/event writer used by the ROS node."""

    def __init__(self, output_root: str | Path, config_path: str | Path, reference_path: str | Path,
                 parameters: dict[str, Any], snapshot_queue_size: int = 4,
                 max_snapshots: int = 100, low_space_bytes: int = 500 * 1024 * 1024) -> None:
        root = Path(output_root).expanduser()
        root.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        session = root / stamp
        index = 1
        while session.exists():
            session = root / f"{stamp}_{index:02d}"
            index += 1
        session.mkdir()
        (session / "failure_snapshots").mkdir()
        self.session = session
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.reference_path = str(Path(reference_path).expanduser().resolve())
        self.config_sha256 = _safe_sha(self.config_path)
        self.reference_sha256 = _safe_sha(self.reference_path)
        self.max_snapshots = int(max_snapshots)
        self.low_space_bytes = int(low_space_bytes)
        self.startup_min_free_bytes = 1 * 1024 * 1024 * 1024
        self.startup_free_bytes = shutil.disk_usage(session).free
        self.startup_space_ok = self.startup_free_bytes >= self.startup_min_free_bytes
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot_reserved = 0
        self._snapshot_saved = 0
        self._snapshot_failed = 0
        self._metrics_dropped = 0
        self._event_log_dropped = 0
        self._snapshot_queue_high_watermark = 0
        self._writer_errors: list[str] = []
        self._metric_queue: PriorityQueue[tuple[int, int, tuple[str, dict[str, Any]] | None]] = PriorityQueue(maxsize=1024)
        self._snapshot_queue: PriorityQueue[tuple[int, int, FailureEventTask | None]] = PriorityQueue(maxsize=max(1, int(snapshot_queue_size)))
        self._sequence = 0
        self._manifest: dict[str, Any] = {
            "schema_version": "1.0", "recording": True, "complete": False,
            "session_directory": str(session), "config": self.config_path, "reference": self.reference_path,
            "config_sha256": self.config_sha256, "reference_sha256": self.reference_sha256,
            "parameters": parameters, "validation_rule": "session writer completed without writer errors",
            "startup_free_bytes": self.startup_free_bytes,
            "startup_min_free_bytes": self.startup_min_free_bytes,
            "startup_space_ok": self.startup_space_ok,
            "counters": {},
        }
        write_json(session / "session_manifest.json", self._manifest)
        self._metric_handles = {}
        self._accepting = True
        self._metrics_lifecycle = AsyncLifecycle(self._metric_queue, self._metrics_loop,
                                                 "seam-live-metrics-writer", self._finish_metrics)
        self._snapshot_lifecycle = AsyncLifecycle(self._snapshot_queue, self._snapshot_loop,
                                                  "seam-live-failure-writer")
        self._metrics_thread = self._metrics_lifecycle.thread
        self._snapshot_thread = self._snapshot_lifecycle.thread
        self._metrics_lifecycle.start()
        self._snapshot_lifecycle.start()

    @property
    def metrics_dropped(self) -> int:
        return self._metrics_dropped

    @property
    def snapshot_queue_size(self) -> int:
        return self._snapshot_queue.qsize()

    @property
    def snapshot_saved(self) -> int:
        return self._snapshot_saved

    @property
    def snapshot_failed(self) -> int:
        return self._snapshot_failed

    @property
    def writer_errors(self) -> list[str]:
        return list(self._writer_errors)

    def submit_metrics(self, row: dict[str, Any]) -> bool:
        if not self._accepting or self._metrics_lifecycle.state != "RUNNING": return False
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        try:
            if self._metrics_lifecycle.submit((1, sequence, ("frame", row)), "drop_newest"): return True
            self._metrics_dropped += 1
            return False
        except Full:
            self._metrics_dropped += 1
            return False

    def submit_event_log(self, row: dict[str, Any]) -> bool:
        if not self._accepting or self._metrics_lifecycle.state != "RUNNING": return False
        with self._lock:
            self._sequence += 1
            sequence = self._sequence
        try:
            if self._metrics_lifecycle.submit((0, sequence, ("event", row)), "drop_newest"): return True
            self._event_log_dropped += 1
            return False
        except Full:
            self._event_log_dropped += 1
            return False

    def enqueue_snapshot(self, factory: Callable[[str], FailureEventTask], *, automatic: bool) -> tuple[bool, str | None, str | None]:
        """Reserve queue capacity before factory copies the image."""
        if not self._accepting or self._snapshot_lifecycle.state != "RUNNING":
            return False, None, "snapshot_writer_failed_or_closed"
        if automatic and not self.startup_space_ok:
            return False, None, "startup_free_space_below_1GB"
        if automatic and shutil.disk_usage(self.session).free < self.low_space_bytes:
            return False, None, "low_disk_space"
        with self._lock:
            if self._snapshot_reserved >= self.max_snapshots:
                return False, None, "max_failure_snapshots_per_session"
            if self._snapshot_queue.full():
                return False, None, "snapshot_queue_full"
            self._snapshot_reserved += 1
            self._sequence += 1
            sequence = self._sequence
        event_id = f"event_{time.time_ns()}_{uuid.uuid4().hex[:8]}"
        try:
            task = factory(event_id)
            if not self._snapshot_lifecycle.submit((0 if not automatic else 1, sequence, task), "drop_newest"):
                raise Full
            self._snapshot_queue_high_watermark = max(self._snapshot_queue_high_watermark, self._snapshot_queue.qsize())
            self.submit_event_log({"event_id": event_id, "trigger_reason": task.trigger_reason,
                                   "automatic": bool(automatic), "accepted": True})
            return True, event_id, None
        except (Full, Exception) as exc:  # factory may fail while copying an image
            with self._lock:
                self._snapshot_reserved -= 1
            reason = "snapshot_queue_full" if isinstance(exc, Full) else f"snapshot_prepare:{type(exc).__name__}:{exc}"
            return False, None, reason

    def _metrics_loop(self, item) -> None:
        try:
            if not self._metric_handles:
                self._metric_handles["frame"] = (self.session / "frame_metrics.jsonl").open("w", encoding="utf-8", buffering=1024 * 1024)
                self._metric_handles["event"] = (self.session / "events.jsonl").open("w", encoding="utf-8", buffering=1024 * 1024)
            kind, row = item[2]
            self._metric_handles[kind].write(json.dumps(_json_safe(row), ensure_ascii=False, allow_nan=False) + "\n")
        except Exception as exc:
            self._writer_errors.append(f"metrics:{type(exc).__name__}: {exc}")
            raise

    def _finish_metrics(self):
        # Preserve the normal empty-session output files as well.
        if not self._metric_handles:
            for name in ["frame_metrics.jsonl", "events.jsonl"]: (self.session / name).touch()
        for handle in self._metric_handles.values(): handle.close()

    def _snapshot_loop(self, item) -> None:
        task = item[2]
        try:
            directory = write_failure_event(task, self.session / "failure_snapshots")
            metadata = json.loads((directory / "metadata.json").read_text(encoding="utf-8"))
            if metadata.get("complete") is not True:
                raise RuntimeError(metadata.get("error", "failure event incomplete"))
            self._snapshot_saved += 1
        except Exception as exc:
            self._snapshot_failed += 1
            self._writer_errors.append(f"snapshot:{type(exc).__name__}: {exc}")
            raise

    def finalize(self, counters: dict[str, Any], *, graceful: bool = True, timeout: float = 30.0) -> dict[str, Any]:
        if hasattr(self, "_final_summary"):
            failed = self._metrics_lifecycle.state == "FAILED" or self._snapshot_lifecycle.state == "FAILED"
            return {**self._final_summary, "complete": False} if failed else self._final_summary
        self._accepting = False
        deadline = time.monotonic() + max(0., timeout)
        for lifecycle in [self._metrics_lifecycle, self._snapshot_lifecycle]:
            try: lifecycle.close(max(0., deadline-time.monotonic()))
            except (RuntimeError, TimeoutError) as exc: self._writer_errors.append(str(exc))
        try:
            return self._metrics_lifecycle.finalize_io(
                lambda: self._finalize_metadata(counters, graceful), deadline, self._writer_errors.append)
        except (OSError, RuntimeError, TimeoutError) as exc:
            self._writer_errors.append(str(exc))
            self._final_summary = {"session": str(self.session), "complete": False, "close_error": str(exc),
                "counters": {"lifecycle": {"metrics": self._metrics_lifecycle.status,
                    "snapshots": self._snapshot_lifecycle.status}, "writer_errors": self.writer_errors,
                    "unwritten_tasks": sum(x.queue.unfinished_tasks + x.abandoned + x.failed for x in [self._metrics_lifecycle,self._snapshot_lifecycle])}}
            return self._final_summary

    def _finalize_metadata(self, counters, graceful):
        final_counters = dict(counters)
        final_counters.update({"lifecycle": {"metrics": self._metrics_lifecycle.status, "snapshots": self._snapshot_lifecycle.status},
                               "unwritten_tasks": sum(x.queue.unfinished_tasks + x.abandoned + x.failed for x in [self._metrics_lifecycle, self._snapshot_lifecycle]),
                               "auto_snapshot_saved": self._snapshot_saved,
                               "snapshot_writer_failed": self._snapshot_failed,
                               "metrics_log_dropped": self._metrics_dropped,
                               "event_log_dropped": self._event_log_dropped,
                               "writer_errors": self.writer_errors,
                               "snapshot_queue_high_watermark": self._snapshot_queue_high_watermark})
        complete = bool(graceful and not self._writer_errors and not self._metrics_thread.is_alive() and not self._snapshot_thread.is_alive() and self._metrics_lifecycle.state == "CLOSED" and self._snapshot_lifecycle.state == "CLOSED")
        write_json(self.session / "live_summary.json", {
            "status": "complete" if complete else "incomplete",
            "counters": final_counters,
            "failure_reason_counts": final_counters.get("failure_reason_counts", {}),
            "state_counts": final_counters.get("state_counts", {}),
        })
        self._manifest.update({"recording": False, "complete": complete, "counters": final_counters,
                               "snapshot_directory": "failure_snapshots", "finalized_at_ns": time.time_ns()})
        _atomic_json(self.session / "session_manifest.json", self._manifest)
        _atomic_json(self.session / "complete.json", {"recording": False, "complete": complete,
                     "valid_for_live_diagnostics": complete, "counters": final_counters})
        self._final_summary = {"session": str(self.session), "complete": complete, "counters": final_counters}
        return self._final_summary


def _safe_sha(path: str) -> str | None:
    try:
        return sha256_file(path)
    except OSError:
        return None


def write_failure_event(task: FailureEventTask, output_root: str | Path) -> Path:
    root = Path(output_root); root.mkdir(parents=True, exist_ok=True)
    directory = root / task.event_id
    directory.mkdir(parents=False, exist_ok=False)
    metadata: dict[str, Any] = {
        "schema_version": "1.0", "complete": False, "event_id": task.event_id,
        "trigger_reason": task.trigger_reason, "automatic": bool(task.automatic),
        "source_mode": task.source_mode, "config": task.config_path, "reference": task.reference_path,
        "config_sha256": task.config_sha256, "reference_sha256": task.reference_sha256,
        "header_stamp": _header_stamp(task.header), "roi_original_px": list(task.result.roi_original_px),
        "detected": bool(task.result.detected), "predicted_only": bool(task.result.predicted_only),
        "tracking_state": task.result.tracking_state.name, "processing_ms": float(task.result.processing_ms),
    }
    try:
        raw = np.asarray(task.image)
        if raw.dtype != np.uint8 or raw.ndim not in (2, 3):
            raise ValueError("event image must be uint8 mono or color")
        x, y, width, height = (int(v) for v in task.result.roi_original_px)
        roi = raw[y:y + height, x:x + width].copy()
        if task.save_lossless_roi:
            if not cv2.imwrite(str(directory / "roi_raw.bmp"), roi):
                raise OSError("cv2.imwrite failed for roi_raw.bmp")
            metadata["roi_raw_sha256"] = sha256_file(directory / "roi_raw.bmp")
        if task.save_full_frame:
            if not cv2.imwrite(str(directory / "raw.bmp"), raw):
                raise OSError("cv2.imwrite failed for raw.bmp")
            metadata["raw_sha256"] = sha256_file(directory / "raw.bmp")
        if task.save_context_jpeg:
            if not cv2.imwrite(str(directory / "context.jpg"), raw,
                               [int(cv2.IMWRITE_JPEG_QUALITY), int(task.context_jpeg_quality)]):
                raise OSError("cv2.imwrite failed for context.jpg")
            metadata["context_sha256"] = sha256_file(directory / "context.jpg")
        overlay = _event_overlay(raw, task.result)
        if not cv2.imwrite(str(directory / "overlay.png"), overlay):
            raise OSError("cv2.imwrite failed for overlay.png")
        metadata["overlay_sha256"] = sha256_file(directory / "overlay.png")
        write_json(directory / "seam_geometry.json", task.geometry)
        metadata["seam_geometry_sha256"] = sha256_file(directory / "seam_geometry.json")
        write_json(directory / "failure_reason.json", {
            "failure_reasons": list(task.result.failure_reasons),
            "human_readable_reason": human_readable_reason(_load_config(task.config_path), task.result),
            "gate_truth": task.gate, "pre_state": task.metrics.get("pre_state"),
            "post_state": task.result.tracking_state.name, "detected": bool(task.result.detected),
            "predicted_only": bool(task.result.predicted_only), "config_sha256": task.config_sha256,
            "reference_sha256": task.reference_sha256, "roi_original_px": list(task.result.roi_original_px),
            "telemetry": {key: task.metrics.get(key) for key in
                          ("exposure_mode", "exposure_us", "analog_gain", "analog_gain_raw", "gain_db")},
            "processing_ms": float(task.result.processing_ms), "trigger_reason": task.trigger_reason,
            "automatic": bool(task.automatic),
        })
        metadata["failure_reason_sha256"] = sha256_file(directory / "failure_reason.json")
        metadata["complete"] = True
    except Exception as exc:
        metadata["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        write_json(directory / "metadata.json", metadata)
    return directory


def _load_config(path: str) -> dict[str, Any]:
    import yaml
    with Path(path).open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _header_stamp(header: Any) -> dict[str, int]:
    stamp = getattr(header, "stamp", None)
    return {"sec": int(getattr(stamp, "sec", 0)), "nanosec": int(getattr(stamp, "nanosec", 0))}


def _event_overlay(image: np.ndarray, result: StableSeamResult) -> np.ndarray:
    from tank_vision.stable_seam_tracker import draw_stable_seam_overlay
    output = draw_stable_seam_overlay(image, result, None)
    if not result.detected:
        for points in (result.left_boundary_original_px, result.right_boundary_original_px, result.centerline_original_px):
            if len(points) >= 2:
                cv2.polylines(output, [np.round(points).astype(np.int32).reshape(-1, 1, 2)], False, (0, 0, 255), 3, cv2.LINE_AA)
        text = "REJECTED / DIAGNOSTIC ONLY"
        cv2.putText(output, text, (25, 105), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(output, text, (25, 105), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
    return output
