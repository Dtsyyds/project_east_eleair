"""Bounded asynchronous geometry-provenance evidence writer.

The tracker remains the source of all numerical decisions.  This module only
copies sampled ROIs and serializes the already-computed trace on a writer
thread; it never changes a tracker result.
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

import cv2
import numpy as np

from tank_vision.stable_seam_tracker import StableSeamDiagnostics
from tank_vision.seam_tracking_types import StableSeamResult


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


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(slots=True)
class ProvenanceTask:
    roi_bgr: np.ndarray
    roi: tuple[int, int, int, int]
    metadata: dict[str, Any]
    geometry: dict[str, Any]
    provenance: dict[str, Any]
    diagnostics: StableSeamDiagnostics | None
    result: StableSeamResult
    labels: list[str]

    @property
    def frame_index(self) -> int:
        return int(self.metadata.get("captured_index", 0))


class GeometryProvenanceRecorder:
    """Write at most sampled/extreme evidence, never waiting in the tracker."""

    def __init__(self, output_root: str | Path, config_path: str | Path,
                 reference_path: str | Path, queue_size: int = 8,
                 sample_period_sec: float = 5.0, max_events: int = 30) -> None:
        if int(queue_size) < 1 or float(sample_period_sec) <= 0 or int(max_events) < 1:
            raise ValueError("invalid geometry provenance recorder parameters")
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
        self.config_path = Path(config_path).expanduser().resolve()
        self.reference_path = Path(reference_path).expanduser().resolve()
        self.config_sha256 = _sha256(self.config_path)
        self.reference_sha256 = _sha256(self.reference_path)
        self.sample_period_sec = float(sample_period_sec)
        self.max_events = int(max_events)
        self.queue: Queue[ProvenanceTask | None] = Queue(maxsize=int(queue_size))
        self._lock = threading.Lock()
        self._accepting = True
        self._closed = False
        self._last_sample_monotonic = -float("inf")
        self._queue_dropped = 0
        self._capacity_suppressed = 0
        self._write_errors: list[str] = []
        self._events_written = 0
        self._extreme_candidates: dict[str, list[tuple[float, ProvenanceTask]]] = {
            "joint_inlier_min": [], "joint_inlier_max": [],
            "residual_min": [], "residual_max": [],
        }
        self._enqueued_frame_indices: set[int] = set()
        self._extreme_frame_indices: set[int] = set()
        self._lifecycle = AsyncLifecycle(self.queue, self._writer_loop, "stable-seam-provenance-writer")
        self._thread = self._lifecycle.thread
        self._write_schema()
        (self.session / "manifest.json").write_text(json.dumps({
            "schema_version": "geometry_provenance_session_v1", "status": "recording", "complete": False,
            "config": str(self.config_path), "reference": str(self.reference_path),
            "config_sha256": self.config_sha256, "reference_sha256": self.reference_sha256,
            "sample_period_sec": self.sample_period_sec, "max_events": self.max_events,
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self._lifecycle.start()

    @property
    def session_directory(self) -> str:
        return str(self.session)

    @property
    def queue_dropped(self) -> int:
        return self._queue_dropped

    @property
    def events_written(self) -> int:
        return self._events_written

    def has_capacity(self) -> bool:
        """Return whether one more task can be accepted without waiting.

        The node uses this guard before doing the diagnostic second tracker
        pass or copying an ROI.  The queue and completed-event count are the
        only capacity that can consume the configured event budget while a
        session is running; retained extreme candidates are not queued until
        close and therefore must not make the realtime path do extra work.
        """
        with self._lock:
            return bool(self._has_capacity_locked())

    def _has_capacity_locked(self) -> bool:
        retained_extremes = len(self._extreme_frame_indices - self._enqueued_frame_indices)
        return bool(
            self._accepting
            and not self._closed
            and self._lifecycle.state == "RUNNING"
            and self._events_written + self.queue.qsize() + retained_extremes < self.max_events
        )

    def periodic_due(self) -> bool:
        with self._lock:
            if not self._has_capacity_locked():
                return False
            now = time.monotonic()
            if now - self._last_sample_monotonic < self.sample_period_sec:
                return False
            self._last_sample_monotonic = now
            return True

    def extreme_reasons(self, joint: float | None, residual: float | None) -> list[str]:
        reasons: list[str] = []
        values = {"joint_inlier_min": joint, "joint_inlier_max": joint,
                  "residual_min": residual, "residual_max": residual}
        with self._lock:
            for key, value in values.items():
                if value is None or not np.isfinite(float(value)):
                    continue
                current = [item[0] for item in self._extreme_candidates[key]]
                if len(current) < 3 or ((key.endswith("_min") and float(value) < max(current))
                                        or (key.endswith("_max") and float(value) > min(current))):
                    reasons.append(key)
        return reasons

    def commit_extreme(self, task: ProvenanceTask, reasons: list[str]) -> None:
        values = {"joint_inlier_min": task.result.fit_inlier_ratio,
                  "joint_inlier_max": task.result.fit_inlier_ratio,
                  "residual_min": task.result.fit_residual_px,
                  "residual_max": task.result.fit_residual_px}
        with self._lock:
            stored = False
            for key in reasons:
                value = values[key]
                if value is None or not np.isfinite(float(value)):
                    continue
                entries = [item for item in self._extreme_candidates[key]
                           if item[1].frame_index != task.frame_index]
                entries.append((float(value), task))
                entries.sort(key=lambda item: item[0], reverse=key.endswith("_max"))
                self._extreme_candidates[key] = entries[:3]
                stored = True
            if stored:
                self._extreme_frame_indices = {t.frame_index for entries in self._extreme_candidates.values() for _, t in entries}

    def submit_periodic(self, task: ProvenanceTask) -> bool:
        return self._enqueue(task)

    def _enqueue(self, task: ProvenanceTask) -> bool:
        with self._lock:
            if not self._accepting or self._closed:
                self._queue_dropped += 1
                return False
            if not self._has_capacity_locked():
                # Reaching the configured evidence cap is an intentional
                # sampling decision, not a writer/queue failure.
                self._capacity_suppressed += 1
                return False
            try:
                if self._lifecycle.submit(task, "drop_newest"):
                    self._enqueued_frame_indices.add(task.frame_index)
                    return True
                self._queue_dropped += 1
                return False
            except Full:
                self._queue_dropped += 1
                return False

    def _write_schema(self) -> None:
        schema = {
            "schema_version": "geometry_provenance_v1",
            "sampling": {"period_sec": self.sample_period_sec, "max_events": self.max_events,
                          "extra_extremes": "joint inlier min/max 3 each; residual min/max 3 each"},
            "files": ["roi_raw.bmp", "candidate_overlay.png", "pair_overlay.png", "selected_raw_path_overlay.png",
                      "fitted_geometry_overlay.png", "inlier_mask_overlay.png", "temporal_output_overlay.png",
                      "provenance.json", "geometry.json", "metadata.json"],
            "colors_bgr": {"candidate_cyan": [255, 255, 0], "pair_gray": [160, 160, 160],
                           "selected_left_blue": [255, 0, 0], "selected_right_green": [0, 255, 0],
                           "selected_center_yellow": [0, 255, 255], "fitted_thick_same_colors": True,
                           "outlier_red": [0, 0, 255], "temporal_predicted_orange": [0, 165, 255]},
            "complete_rule": "complete=true only when all queued events and hashes are valid; queue drops or writer errors make complete=false",
            "config_sha256": self.config_sha256, "reference_sha256": self.reference_sha256,
        }
        (self.session / "provenance_schema.json").write_text(json.dumps(schema, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    @staticmethod
    def _pts(values: list[tuple[float, float]], roi: tuple[int, int, int, int]) -> np.ndarray:
        if len(values) < 2:
            return np.empty((0, 1, 2), dtype=np.int32)
        x, y, _, _ = roi
        return np.round(np.asarray([(px - x, py - y) for px, py in values], dtype=np.float64)).astype(np.int32).reshape(-1, 1, 2)

    @staticmethod
    def _draw_line(canvas: np.ndarray, values: list[tuple[float, float]], roi: tuple[int, int, int, int], color: tuple[int, int, int], thickness: int = 2, dashed: bool = False) -> None:
        points = GeometryProvenanceRecorder._pts(values, roi)
        if len(points) < 2:
            return
        if not dashed:
            cv2.polylines(canvas, [points], False, color, thickness, cv2.LINE_AA)
            return
        for first, second in zip(points[:-1], points[1:]):
            if (first[0, 0] // 12) % 2 == 0:
                cv2.line(canvas, tuple(first[0]), tuple(second[0]), color, thickness, cv2.LINE_AA)

    @staticmethod
    def _trace_point(trace: dict[str, Any], roi: tuple[int, int, int, int], sx: float, sy: float, y_processed: float, x_processed: float) -> tuple[int, int]:
        return (int(round((x_processed / max(sx, 1e-12)))), int(round(y_processed / max(sy, 1e-12))))

    def _overlay(self, task: ProvenanceTask, kind: str) -> np.ndarray:
        canvas = task.roi_bgr.copy()
        roi = task.roi
        trace = task.provenance
        diagnostics = task.diagnostics
        result_trace = trace.get("result", {})
        scales = trace.get("scales", {"sx": 1.0, "sy": 1.0})
        sx, sy = float(scales.get("sx", 1.0)), float(scales.get("sy", 1.0))
        if kind == "candidate" and trace.get("candidate_stage_trace"):
            for band in trace["candidate_stage_trace"]:
                yp = float(band.get("y_processed", 0.0))
                for side in ("left", "right"):
                    for item in band.get("top_k_after", {}).get(side, []):
                        point = self._trace_point(trace, roi, sx, sy, yp, float(item["x"]))
                        cv2.circle(canvas, point, 3, (255, 255, 0), -1, cv2.LINE_AA)
        if kind == "pair" and trace.get("candidate_stage_trace"):
            for band in trace["candidate_stage_trace"]:
                yp = float(band.get("y_processed", 0.0))
                for pair in band.get("pair_stages", {}).get("pair_after_max_pairs_per_band", []):
                    left = self._trace_point(trace, roi, sx, sy, yp, float(pair["left_processed"]))
                    right = self._trace_point(trace, roi, sx, sy, yp, float(pair["right_processed"]))
                    cv2.line(canvas, left, right, (160, 160, 160), 1, cv2.LINE_AA)
        selected = []
        for item in trace.get("selected_raw_path", []):
            y0 = float(item["y"])
            selected.append((float(item["left"]), y0, float(item["right"])))
        if kind in {"selected", "inlier"}:
            left = [(item[0], item[1]) for item in selected]
            right = [(item[2], item[1]) for item in selected]
            center = [((item[0] + item[2]) * 0.5, item[1]) for item in selected]
            self._draw_line(canvas, left, roi, (255, 0, 0), 3)
            self._draw_line(canvas, right, roi, (0, 255, 0), 3)
            self._draw_line(canvas, center, roi, (0, 255, 255), 3)
        if kind == "inlier":
            mask = result_trace.get("joint_inlier_mask", [])
            for item, passed in zip(selected, mask):
                if not passed:
                    for px in (item[0], item[2], (item[0] + item[2]) * 0.5):
                        point = (int(round(px - roi[0])), int(round(item[1] - roi[1])))
                        cv2.drawMarker(canvas, point, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        if kind == "fitted":
            y_values = result_trace.get("fitted_y", [])
            fit_left = [(float(x), float(y)) for x, y in zip(result_trace.get("fitted_left", []), y_values)]
            fit_right = [(float(x), float(y)) for x, y in zip(result_trace.get("fitted_right", []), y_values)]
            fit_center = [(float(x), float(y)) for x, y in zip(result_trace.get("fitted_center", []), y_values)]
            self._draw_line(canvas, fit_left, roi, (255, 0, 0), 7)
            self._draw_line(canvas, fit_right, roi, (0, 255, 0), 7)
            self._draw_line(canvas, fit_center, roi, (0, 255, 255), 7)
        if kind == "temporal":
            values = result_trace.get("temporal_output", {})
            y_values = values.get("y", [])
            left = [(float(x), float(y)) for x, y in zip(values.get("left", []), y_values)]
            right = [(float(x), float(y)) for x, y in zip(values.get("right", []), y_values)]
            center = [(float(x), float(y)) for x, y in zip(values.get("center", []), y_values)]
            if values.get("predicted_only"):
                self._draw_line(canvas, left, roi, (0, 165, 255), 5, True)
                self._draw_line(canvas, right, roi, (0, 165, 255), 5, True)
                self._draw_line(canvas, center, roi, (0, 165, 255), 5, True)
            else:
                self._draw_line(canvas, left, roi, (255, 0, 0), 4, True)
                self._draw_line(canvas, right, roi, (0, 255, 0), 4, True)
                self._draw_line(canvas, center, roi, (0, 255, 255), 4, True)
        legend = {"candidate": "candidate cyan", "pair": "DP pairs gray", "selected": "selected raw L/B R/G C/Y",
                  "fitted": "IRLS fitted L/B R/G C/Y", "inlier": "red X=joint outlier", "temporal": "temporal predicted-only orange dashed"}
        text = f"{kind}: {legend[kind]} | state={task.result.tracking_state.name} detected={task.result.detected} predicted={task.result.predicted_only}"
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 42), (0, 0, 0), -1)
        cv2.putText(canvas, text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        return canvas

    def _write_task(self, task: ProvenanceTask) -> None:
        if self._events_written >= self.max_events:
            self._capacity_suppressed += 1
            return
        event = self.session / f"event_{task.metadata.get('header_stamp_ns', time.time_ns())}_{task.frame_index:06d}"
        event.mkdir(parents=False, exist_ok=False)
        cv2.imwrite(str(event / "roi_raw.bmp"), task.roi_bgr)
        for name in ("candidate", "pair", "selected", "fitted", "inlier", "temporal"):
            cv2.imwrite(str(event / {"candidate": "candidate_overlay.png", "pair": "pair_overlay.png",
                                     "selected": "selected_raw_path_overlay.png", "fitted": "fitted_geometry_overlay.png",
                                     "inlier": "inlier_mask_overlay.png", "temporal": "temporal_output_overlay.png"}[name]), self._overlay(task, name))
        provenance_path = event / "provenance.json"
        geometry_path = event / "geometry.json"
        provenance_path.write_text(json.dumps(_json_safe(task.provenance), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        geometry_path.write_text(json.dumps(_json_safe(task.geometry), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        hashes = {path.name: _sha256(path) for path in sorted(event.iterdir()) if path.name not in {"metadata.json"}}
        metadata = dict(task.metadata)
        metadata.update({"schema_version": "geometry_provenance_event_v1", "labels": sorted(set(task.labels)),
                         "complete": True, "config_sha256": self.config_sha256, "reference_sha256": self.reference_sha256,
                         "files_sha256": hashes, "geometry_provenance_frame_consistent": True})
        (event / "metadata.json").write_text(json.dumps(_json_safe(metadata), indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
        self._events_written += 1

    @property
    def state(self):
        return self._lifecycle.state

    def _writer_loop(self, task) -> None:
        try:
            self._write_task(task)
        except Exception as exc:
            self._write_errors.append(f"{type(exc).__name__}:{exc}")
            raise

    def close(self, timeout: float = 30.0) -> dict[str, Any]:
        deadline = time.monotonic() + max(0., timeout)
        with self._lock:
            if self._closed: return self.summary
            self._accepting = False
            self._closed = True
            extremes = {task.frame_index: task for entries in self._extreme_candidates.values() for _,task in entries}
        # No state lock is held while the writer finishes pending evidence.
        try:
            self._lifecycle.flush(max(0., deadline-time.monotonic()))
            for task in extremes.values():
                if task.frame_index not in self._enqueued_frame_indices:
                    self._lifecycle.submit(task, timeout=max(0., deadline-time.monotonic()))
                    self._enqueued_frame_indices.add(task.frame_index)
            self._lifecycle.close(max(0., deadline-time.monotonic()))
        except (RuntimeError, TimeoutError) as exc:
            self._write_errors.append(str(exc))
            try: self._lifecycle.close(max(0., deadline-time.monotonic()))
            except (RuntimeError, TimeoutError): pass
        try:
            return self._lifecycle.finalize_io(self._finalize_metadata, deadline, self._write_errors.append)
        except (OSError, RuntimeError, TimeoutError) as exc:
            self._write_errors.append(str(exc))
            self.summary = {"status": "incomplete", "complete": False, "close_error": str(exc),
                "lifecycle": self._lifecycle.status,
                "unwritten_tasks": self.queue.unfinished_tasks + self._lifecycle.abandoned + self._lifecycle.failed,
                "writer_errors": list(self._write_errors)}
            return self.summary

    def _finalize_metadata(self):
        complete = self._queue_dropped == 0 and not self._write_errors and self._thread.is_alive() is False and self._lifecycle.state == "CLOSED"
        summary = {"status": "complete" if complete else "incomplete", "complete": complete,
                   "lifecycle": self._lifecycle.status,
                   "unwritten_tasks": self._lifecycle.status["unfinished"] + self._lifecycle.abandoned + self._lifecycle.failed,
                   "events_written": self._events_written, "queue_dropped": self._queue_dropped,
                   "dropped_event_count": self._queue_dropped,
                   "capacity_suppressed_count": self._capacity_suppressed,
                   "write_error_count": len(self._write_errors),
                   "writer_errors": list(self._write_errors), "config_sha256": self.config_sha256,
                   "reference_sha256": self.reference_sha256, "max_events": self.max_events,
                   "sample_period_sec": self.sample_period_sec}
        (self.session / "complete.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        (self.session / "manifest.json").write_text(json.dumps({
            "schema_version": "geometry_provenance_session_v1", "status": summary["status"],
            "complete": bool(summary["complete"]), "config": str(self.config_path),
            "reference": str(self.reference_path), "config_sha256": self.config_sha256,
            "reference_sha256": self.reference_sha256, "sample_period_sec": self.sample_period_sec,
            "max_events": self.max_events, "events_written": self._events_written,
            "queue_dropped": self._queue_dropped, "dropped_event_count": self._queue_dropped,
            "capacity_suppressed_count": self._capacity_suppressed,
            "write_error_count": len(self._write_errors), "writer_errors": list(self._write_errors),
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        self.summary = summary
        return summary

    @property
    def summary(self) -> dict[str, Any]:
        value = getattr(self, "_summary", {"status": "recording", "complete": False,
            "events_written": self._events_written})
        if self._lifecycle.state == "FAILED":
            return {**value, "status": "incomplete", "complete": False, "lifecycle": self._lifecycle.status}
        return value

    @summary.setter
    def summary(self, value: dict[str, Any]) -> None:
        self._summary = value
