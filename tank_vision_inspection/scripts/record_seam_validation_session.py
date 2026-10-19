#!/usr/bin/env python3
"""Record every successful MindVision ROI frame and a separate low-cost preview."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import queue
import shutil
import signal
import sys
import tempfile
import threading
import time
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from seam_validation_common import (  # noqa: E402
    diagnostic_overlay,
    camera_telemetry,
    distribution,
    frame_record,
    json_safe,
    preview_frame,
    roi_crop,
    roi_from_config,
    roi_reconstruction_check,
    sha256_file,
    write_json,
)
from tank_vision.latest_frame import LatestFrameSlot  # noqa: E402
from tank_vision.sources import MindVisionCameraSource, SourceError  # noqa: E402
from tank_vision.stable_seam_tracker import StableSeamTracker, load_stable_config  # noqa: E402
from tank_vision.types import FrameData  # noqa: E402


MIN_WRITE_MBPS = 120.0


@dataclass(slots=True)
class CapturePacket:
    captured_index: int
    frame: FrameData
    host_timestamp_ns: int
    roi: np.ndarray
    raw_payload: bytes
    raw_copy_enqueue_ms: float
    telemetry_unavailable_fields: list[str]


@dataclass(slots=True)
class PreviewPacket:
    capture: CapturePacket
    result: Any
    row: dict[str, Any]
    preview: np.ndarray


def _unique_run(root: Path) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
    candidate = root / f"{stamp}_center_run"
    suffix = 1
    while candidate.exists():
        candidate = root / f"{stamp}_center_run_{suffix:02d}"
        suffix += 1
    candidate.mkdir()
    (candidate / "keyframes").mkdir()
    return candidate


def _camera_options(config: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    camera = config["camera"]
    exposure_mode = args.exposure_mode or camera.get("exposure_mode", "auto")
    exposure_us = args.exposure_us if args.exposure_us is not None else camera.get("exposure_us")
    if exposure_mode == "auto":
        exposure_us = None
    return {
        "sdk_root": str(args.sdk_root or camera.get("sdk_root") or ""),
        "resolution_index": int(args.resolution_index if args.resolution_index is not None else camera.get("resolution_index", 0)),
        "width": int(camera["expected_width"]), "height": int(camera["expected_height"]),
        "pixel_format": str(camera.get("pixel_format", "bgr8")),
        "timeout_ms": int(args.camera_timeout_ms or camera.get("timeout_ms", 1000)),
        "exposure_mode": exposure_mode, "exposure_us": exposure_us,
        "ae_target": camera.get("ae_target"),
        "ae_exposure_min_us": args.ae_min_exposure_us if args.ae_min_exposure_us is not None else camera.get("ae_exposure_min_us"),
        "ae_exposure_max_us": args.ae_max_exposure_us if args.ae_max_exposure_us is not None else camera.get("ae_exposure_max_us"),
        "gain_min_raw": camera.get("gain_min_raw"), "gain_max_raw": camera.get("gain_max_raw"),
        "analog_gain_raw": args.analog_gain_raw,
    }


def _telemetry_unavailable(frame: FrameData) -> list[str]:
    return camera_telemetry(frame)["telemetry_unavailable_fields"]


def _estimate(config: dict[str, Any], duration: float, fps: float) -> dict[str, Any]:
    camera = config["camera"]
    roi = roi_from_config(config, int(camera["expected_width"]), int(camera["expected_height"]))
    shape = [roi[3], roi[2], 3]
    bytes_per_frame = int(np.prod(shape))
    estimated_frames = int(np.ceil(max(duration, 0.0) * max(fps, 0.0)))
    estimated_bytes = estimated_frames * bytes_per_frame
    return {"duration_sec": duration, "expected_capture_fps": fps, "roi": {"width": roi[2], "height": roi[3]},
            "bytes_per_frame": bytes_per_frame, "estimated_frames": estimated_frames,
            "estimated_bytes": estimated_bytes, "required_free_bytes": estimated_bytes * 2}


def disk_write_benchmark(root: Path, bytes_per_frame: int, duration_sec: float = 1.0) -> dict[str, Any]:
    """Short sequential-write check on the same filesystem as the session."""
    block = np.zeros(bytes_per_frame, dtype=np.uint8).tobytes()
    path = Path(tempfile.mkstemp(prefix=".seam_validation_write_test_", suffix=".raw", dir=root)[1])
    total = 0
    started = time.perf_counter()
    try:
        with path.open("wb", buffering=0) as handle:
            while time.perf_counter() - started < max(0.2, duration_sec):
                handle.write(block)
                total += len(block)
            handle.flush()
            os.fsync(handle.fileno())
        elapsed = max(time.perf_counter() - started, 1e-9)
    finally:
        path.unlink(missing_ok=True)
    return {"bytes_written": total, "duration_sec": elapsed, "throughput_mbps": total / elapsed / 1_000_000,
            "minimum_required_mbps": MIN_WRITE_MBPS, "pass": total / elapsed / 1_000_000 >= MIN_WRITE_MBPS}


def _packet(captured_index: int, frame: FrameData, config: dict[str, Any]) -> CapturePacket:
    copy_started = time.perf_counter_ns()
    roi = np.ascontiguousarray(roi_crop(frame.image, roi_from_config(config, frame.image.shape[1], frame.image.shape[0])))
    payload = roi.tobytes(order="C")
    copy_ms = (time.perf_counter_ns() - copy_started) / 1e6
    return CapturePacket(captured_index, frame, time.time_ns(), roi, payload, copy_ms, _telemetry_unavailable(frame))


def _stats(rows: list[dict[str, Any]], key: str) -> dict[str, Any]:
    return distribution([float(row[key]) for row in rows if row.get(key) is not None])


def _write_live_summary(run: Path, raw_rows: list[dict[str, Any]], preview_rows: list[dict[str, Any]],
                        counters: dict[str, Any], program_start_ns: int, program_end_ns: int,
                        disk_test: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    camera_span = ((raw_rows[-1]["host_timestamp_ns"] - raw_rows[0]["host_timestamp_ns"]) / 1e9
                   if len(raw_rows) >= 2 else 0.0)
    state_values = [row["tracking_state"] for row in preview_rows]
    summary = {
        "camera_active_duration_sec": camera_span,
        "program_wall_duration_sec": (program_end_ns - program_start_ns) / 1e9,
        "camera_fps": ((len(raw_rows) - 1) / camera_span if camera_span > 0 else 0.0),
        "captured_frames": counters["captured_frames"], "raw_stored_frames": counters["raw_stored_frames"],
        "live_preview_processed_frames": counters["live_preview_processed_frames"],
        "preview_skipped_frames": counters["preview_skipped_frames"],
        "raw_recorder_dropped_frames": counters["raw_recorder_dropped_frames"],
        "capture_errors": counters["capture_errors"], "writer_errors": list(counters["writer_errors"]),
        "detected_rate": float(np.mean([row["detected"] for row in preview_rows])) if preview_rows else 0.0,
        "state_rates": {state: float(np.mean([value == state for value in state_values])) if state_values else 0.0
                        for state in ("TRACKING", "ACQUIRING", "DEGRADED", "LOST")},
        "tracker_process_ms": _stats(preview_rows, "tracker_process_ms"),
        "preview_draw_ms": _stats(preview_rows, "preview_draw_ms"),
        "json_build_ms": _stats(preview_rows, "json_build_ms"),
        "raw_copy_enqueue_ms": _stats(raw_rows, "raw_copy_enqueue_ms"),
        "disk_write_test": disk_test,
        "exposure_us": _stats(raw_rows, "exposure_us"), "analog_gain_raw": _stats(raw_rows, "analog_gain_raw"),
        "gain_db": _stats(raw_rows, "gain_db"),
        "preview_failure_reason_counts": {reason: sum(reason in row.get("failure_reasons", []) for row in preview_rows)
                                           for reason in sorted({reason for row in preview_rows for reason in row.get("failure_reasons", [])})},
        "validation_scope": "preview statistics only; formal per-capture-frame results are in replay_summary.json",
        "command_display": bool(args.display),
    }
    write_json(run / "live_summary.json", summary)
    return summary


def _args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--sdk-root", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "output/seam_validation_sessions")
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--warmup-frames", type=int, default=60)
    parser.add_argument("--display", dest="display", action="store_true")
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.set_defaults(display=False)
    parser.add_argument("--preview-width", type=int, default=1280)
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--camera-timeout-ms", type=int)
    parser.add_argument("--resolution-index", type=int)
    parser.add_argument("--exposure-mode", choices=("auto", "manual"))
    parser.add_argument("--exposure-us", type=float)
    parser.add_argument("--ae-min-exposure-us", type=float)
    parser.add_argument("--ae-max-exposure-us", type=float)
    parser.add_argument("--analog-gain-raw", type=int)
    parser.add_argument("--expected-processed-fps", type=float, default=20.0)
    parser.add_argument("--raw-writer-queue-size", type=int, default=8)
    parser.add_argument("--disk-test-sec", type=float, default=1.0)
    parser.add_argument("--no-disk-check", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def main() -> int:
    args = _args()
    if args.duration_sec <= 0 or args.duration_sec > 30.0:
        raise SystemExit("--duration-sec must be in (0, 30]")
    if args.raw_writer_queue_size < 1:
        raise SystemExit("--raw-writer-queue-size must be positive")
    config = load_stable_config(args.config)
    if args.reference is not None:
        reference = args.reference.expanduser().resolve()
    else:
        reference_spec = Path(config["reference"]["path"])
        reference = reference_spec if reference_spec.is_absolute() else (args.config.parent.parent / reference_spec).resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    estimate = _estimate(config, args.duration_sec, args.expected_processed_fps)
    free_bytes = shutil.disk_usage(args.output).free
    if not args.no_disk_check and free_bytes < estimate["required_free_bytes"]:
        raise SystemExit(f"insufficient free disk: {free_bytes} < required {estimate['required_free_bytes']} bytes")
    disk_test = disk_write_benchmark(args.output, estimate["bytes_per_frame"], args.disk_test_sec)
    if not args.no_disk_check and not disk_test["pass"]:
        raise SystemExit(f"sequential write speed {disk_test['throughput_mbps']:.1f} MB/s is below {MIN_WRITE_MBPS:.1f} MB/s")
    run = _unique_run(args.output)
    config_sha = sha256_file(args.config); reference_sha = sha256_file(reference)
    write_json(run / "manifest.json", {"schema_version": "1.0", "recording": True, "complete": False,
        "config": str(args.config.resolve()), "reference": str(reference), "config_sha256": config_sha,
        "reference_sha256": reference_sha, "estimated_storage": estimate, "disk_write_test": disk_test,
        "storage_mode": "roi", "roi_reconstruction": "pending", "validation_rule": {}})
    stop_event = threading.Event()
    raw_queue: queue.Queue[CapturePacket | None] = queue.Queue(maxsize=args.raw_writer_queue_size)
    preview_slot: LatestFrameSlot[CapturePacket] = LatestFrameSlot()
    display_slot: LatestFrameSlot[PreviewPacket] = LatestFrameSlot()
    counters: dict[str, Any] = {"captured_frames": 0, "raw_stored_frames": 0, "live_preview_processed_frames": 0,
                                 "preview_skipped_frames": 0, "raw_recorder_dropped_frames": 0,
                                 "capture_errors": 0, "writer_errors": [], "preview_video_dropped_frames": 0}
    raw_rows: list[dict[str, Any]] = []
    preview_rows: list[dict[str, Any]] = []
    raw_rows_lock = threading.Lock(); preview_rows_lock = threading.Lock()
    raw_writer_done = threading.Event(); preview_done = threading.Event()
    state_lock = threading.Lock()
    session_state: dict[str, Any] = {"roi_reconstruction": None, "cold_start_confirmed": False,
                                     "first_packet": None, "raw_shape": None}
    camera = MindVisionCameraSource(_camera_options(config, args))
    warmup_errors = 0
    for _ in range(max(0, int(args.warmup_frames))):
        try:
            next(camera)
        except SourceError:
            warmup_errors += 1
    # Validate ROI replay before formal capture begins, using a separate tracker pair.
    probe = next(camera)
    roi_check = roi_reconstruction_check(args.config, reference, probe.image)
    if not roi_check["match"]:
        camera.close()
        raise SystemExit("ROI reconstruction is not field-exact; this recorder requires full-frame fallback implementation")
    probe_packet = _packet(0, probe, config)
    with state_lock:
        session_state.update({"roi_reconstruction": roi_check, "cold_start_confirmed": True,
                              "first_packet": probe_packet, "raw_shape": list(probe_packet.roi.shape)})
    program_start_ns = time.time_ns()
    capture_start = time.monotonic()
    old_sigint = signal.getsignal(signal.SIGINT)

    def stop_handler(_signum: int, _frame: Any) -> None:
        stop_event.set(); preview_slot.close()

    signal.signal(signal.SIGINT, stop_handler)

    def raw_writer_worker() -> None:
        raw_handle = None; jsonl_handle = None
        try:
            raw_handle = (run / "roi_frames.raw").open("wb", buffering=0)
            jsonl_handle = (run / "capture_frames.jsonl").open("w", encoding="utf-8")
            while True:
                item = raw_queue.get()
                if item is None:
                    break
                offset = raw_handle.tell()
                raw_handle.write(item.raw_payload)
                telemetry = camera_telemetry(item.frame)
                row = {"capture_index": item.captured_index, "host_timestamp_ns": item.host_timestamp_ns,
                       "device_timestamp_ns": int(item.frame.timestamp_ns), "raw_offset": offset,
                       "raw_nbytes": len(item.raw_payload), "raw_shape": list(item.roi.shape), "raw_dtype": "uint8",
                       "exposure_mode": telemetry["exposure_mode"], "exposure_us": telemetry["exposure_us"],
                       "analog_gain": telemetry["analog_gain"], "analog_gain_raw": telemetry["analog_gain_raw"],
                       "gain_db": telemetry["gain_db"],
                       "telemetry_unavailable_fields": sorted(set(item.telemetry_unavailable_fields) | set(telemetry["telemetry_unavailable_fields"])),
                       "raw_copy_enqueue_ms": item.raw_copy_enqueue_ms}
                jsonl_handle.write(json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False) + "\n")
                counters["raw_stored_frames"] += 1
                with raw_rows_lock:
                    raw_rows.append(row)
            jsonl_handle.flush(); raw_handle.flush(); os.fsync(raw_handle.fileno())
        except Exception as exc:  # raw persistence failure invalidates the run
            counters["writer_errors"].append(f"{type(exc).__name__}: {exc}")
            stop_event.set(); preview_slot.close()
        finally:
            if jsonl_handle is not None: jsonl_handle.close()
            if raw_handle is not None: raw_handle.close()
            raw_writer_done.set()

    def capture_worker() -> None:
        captured_index = 1
        try:
            while not stop_event.is_set():
                if time.monotonic() - capture_start >= args.duration_sec:
                    break
                if args.max_frames and captured_index >= args.max_frames:
                    break
                try:
                    frame = next(camera)
                except SourceError:
                    counters["capture_errors"] += 1
                    continue
                item = _packet(captured_index, frame, config)
                counters["captured_frames"] += 1
                try:
                    raw_queue.put_nowait(item)
                except queue.Full:
                    counters["raw_recorder_dropped_frames"] += 1
                    stop_event.set(); preview_slot.close()
                    break
                if not preview_slot.put(item):
                    counters["preview_skipped_frames"] += 1
                captured_index += 1
        finally:
            preview_slot.close()
            while True:
                try:
                    raw_queue.put(None, timeout=0.2)
                    break
                except queue.Full:
                    if counters["writer_errors"]:
                        break

    def preview_worker() -> None:
        tracker = StableSeamTracker(args.config, reference)
        tracker.reset()
        history: list[str] = []
        video = None; preview_jsonl = None
        try:
            preview_jsonl = (run / "preview_frames.jsonl").open("w", encoding="utf-8")
            while True:
                item = preview_slot.get(timeout=0.2)
                if item is None:
                    if stop_event.is_set():
                        break
                    continue
                pre_state = tracker.state.name
                process_started = time.perf_counter_ns()
                result = tracker.process(item.frame.image, diagnostics=False, provenance=False)
                process_ms = (time.perf_counter_ns() - process_started) / 1e6
                json_started = time.perf_counter_ns()
                row = frame_record(item, pre_state, result, tracker)
                row.update({"record_type": "preview", "tracker_process_ms": process_ms,
                            "raw_copy_enqueue_ms": item.raw_copy_enqueue_ms})
                row["json_build_ms"] = (time.perf_counter_ns() - json_started) / 1e6
                draw_started = time.perf_counter_ns()
                overlay = diagnostic_overlay(item.frame.image, result, tracker)
                preview = preview_frame(item.frame.image, result, tracker, row, history, args.preview_width)
                row["preview_draw_ms"] = (time.perf_counter_ns() - draw_started) / 1e6
                if video is None:
                    video = cv2.VideoWriter(str(run / "annotated_preview.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                            max(args.preview_fps, 1.0), (args.preview_width, 720))
                    if not video.isOpened():
                        raise OSError("could not open annotated_preview.mp4")
                video.write(preview)
                preview_jsonl.write(json.dumps(json_safe(row), ensure_ascii=False, allow_nan=False) + "\n")
                preview_jsonl.flush()
                with preview_rows_lock:
                    preview_rows.append(row)
                counters["live_preview_processed_frames"] += 1
                history.append(result.tracking_state.name)
                display_slot.put(PreviewPacket(item, result, row, preview))
        except Exception as exc:
            counters["writer_errors"].append(f"preview:{type(exc).__name__}: {exc}")
            stop_event.set(); preview_slot.close()
        finally:
            if preview_jsonl is not None: preview_jsonl.close()
            if video is not None: video.release()
            preview_done.set()

    # The probe is the first successful formal capture frame and is enqueued before threads start.
    raw_queue.put_nowait(probe_packet)
    counters["captured_frames"] = 1
    writer_thread = threading.Thread(target=raw_writer_worker, name="raw-writer", daemon=True)
    capture_thread = threading.Thread(target=capture_worker, name="camera-acquisition", daemon=True)
    preview_thread = threading.Thread(target=preview_worker, name="live-preview-tracker", daemon=True)
    writer_thread.start(); capture_thread.start(); preview_thread.start()
    try:
        while not (raw_writer_done.is_set() and preview_done.is_set()):
            packet = display_slot.get(timeout=0.1)
            if packet is None or not args.display:
                continue
            cv2.imshow("seam validation preview", packet.preview)
            key = cv2.waitKey(max(1, round(1000.0 / max(args.preview_fps, 1.0)))) & 0xFF
            if key in (27, ord("q")):
                stop_event.set(); preview_slot.close()
    except KeyboardInterrupt:
        stop_handler(signal.SIGINT, None)
    finally:
        stop_event.set(); preview_slot.close()
        capture_thread.join(timeout=10); preview_thread.join(timeout=20); writer_thread.join(timeout=20)
        camera.close(); cv2.destroyAllWindows(); signal.signal(signal.SIGINT, old_sigint)
    program_end_ns = time.time_ns()
    with raw_rows_lock: stored_rows = list(raw_rows)
    with preview_rows_lock: shown_rows = list(preview_rows)
    summary = _write_live_summary(run, stored_rows, shown_rows, counters, program_start_ns, program_end_ns, disk_test, args)
    write_json(run / "replay_summary.json", {"status": "not_run", "command": f"/usr/bin/python3 scripts/replay_seam_validation_session.py --session {run}"})
    raw_size = (run / "roi_frames.raw").stat().st_size if (run / "roi_frames.raw").exists() else 0
    offsets_contiguous = all(row["capture_index"] == index and row["raw_offset"] == sum(item["raw_nbytes"] for item in stored_rows[:index])
                             for index, row in enumerate(stored_rows))
    validation_rule = {
        "capture_errors_zero": counters["capture_errors"] == 0,
        "writer_errors_empty": not counters["writer_errors"],
        "raw_recorder_dropped_zero": counters["raw_recorder_dropped_frames"] == 0,
        "raw_stored_equals_captured": counters["raw_stored_frames"] == counters["captured_frames"],
        "capture_index_continuous": offsets_contiguous and counters["raw_stored_frames"] == counters["captured_frames"],
        "raw_offsets_continuous": offsets_contiguous,
        "raw_file_length_correct": raw_size == sum(row["raw_nbytes"] for row in stored_rows),
        "replay_frames_equals_raw_stored": None,
        "config_reference_sha_match": sha256_file(args.config) == config_sha and sha256_file(reference) == reference_sha,
        "roi_reconstruction_match": bool(roi_check["match"]),
        "replay_mismatch_zero": None,
    }
    manifest = {"schema_version": "1.0", "recording": False, "complete": False,
        "config": str(args.config.resolve()), "reference": str(reference), "config_sha256": config_sha,
        "reference_sha256": reference_sha, "estimated_storage": estimate, "disk_write_test": disk_test,
        "storage_mode": "roi", "roi_original_px": list(roi_check["roi_original_px"]), "roi_shape": list(probe_packet.roi.shape),
        "roi_reconstruction": roi_check, "cold_start_confirmed": True, "warmup_errors": warmup_errors,
        "counters": counters, "raw_file": "roi_frames.raw", "raw_capture_jsonl": "capture_frames.jsonl",
        "raw_sequence_count": len(stored_rows), "raw_file_size": raw_size, "validation_rule": validation_rule,
        "program_wall_duration_sec": (program_end_ns - program_start_ns) / 1e9,
        "camera_active_duration_sec": summary["camera_active_duration_sec"]}
    write_json(run / "manifest.json", manifest)
    write_json(run / "complete.json", {"recording": False, "complete": False, "valid_for_formal_validation": False,
        "validation_rule": validation_rule, "counters": counters, "status": "replay_pending"})
    return 0 if not counters["writer_errors"] and counters["raw_recorder_dropped_frames"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
