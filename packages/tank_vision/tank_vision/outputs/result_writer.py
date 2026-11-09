"""Structured offline result directory writer."""

from __future__ import annotations

import csv
from pathlib import Path
import time

import cv2
import numpy as np

from tank_vision.config import PipelineConfig
from tank_vision.metrics import STAGE_NAMES
from tank_vision.outputs.json_writer import serialize_result
from tank_vision.outputs.overlay import create_overlay
from tank_vision.types import VisionResult


class ResultWriter:
    def __init__(self, output_dir: str | Path, config: PipelineConfig) -> None:
        self.output_dir = Path(output_dir)
        self.config = config
        self.overlays_dir = self.output_dir / "overlays"
        self.masks_dir = self.output_dir / "masks"
        self.json_dir = self.output_dir / "json"
        for directory in (self.output_dir, self.overlays_dir, self.masks_dir, self.json_dir):
            directory.mkdir(parents=True, exist_ok=True)
        self.summary_path = self.output_dir / "summary.csv"
        self._summary_handle = self.summary_path.open("w", newline="", encoding="utf-8")
        self._summary = csv.writer(self._summary_handle)
        self._summary.writerow(
            [
                "sequence_id",
                "original_filename",
                "timestamp_ns",
                "seam_detected",
                "seam_confidence",
                "lateral_error_px",
                "heading_error_deg",
                "defect_count",
                "valid_for_measurement",
                "processing_time_ms",
                "warnings",
                *STAGE_NAMES,
            ]
        )

    @staticmethod
    def _stem(result: VisionResult) -> str:
        original = Path(result.frame.original_filename).stem if result.frame.original_filename else "frame"
        return f"{result.frame.sequence_id:06d}_{original}"

    def write(self, result: VisionResult, defect_mask: np.ndarray | None = None) -> None:
        stem = self._stem(result)
        timing = result.metrics.timing
        overlay = None
        if self.config.save_overlay:
            started_ns = time.perf_counter_ns()
            overlay = create_overlay(result)
            timing.add_ns("overlay_ms", time.perf_counter_ns() - started_ns)
        mask = defect_mask
        if self.config.save_masks:
            if mask is None and result.frame.image_size != (0, 0):
                width, height = result.frame.image_size
                mask = np.zeros((height, width), dtype=np.uint8)
        disk_started_ns = time.perf_counter_ns()
        if overlay is not None and not cv2.imwrite(str(self.overlays_dir / f"{stem}.jpg"), overlay):
            result.warnings.append("overlay_write_failed")
        if self.config.save_masks and mask is not None and not cv2.imwrite(str(self.masks_dir / f"{stem}.png"), mask):
            result.warnings.append("mask_write_failed")
        timing.add_ns("disk_write_ms", time.perf_counter_ns() - disk_started_ns)
        if self.config.save_json:
            serialization_started_ns = time.perf_counter_ns()
            payload = serialize_result(result)
            timing.add_ns("serialization_ms", time.perf_counter_ns() - serialization_started_ns)
            disk_started_ns = time.perf_counter_ns()
            (self.json_dir / f"{stem}.json").write_text(payload, encoding="utf-8")
            timing.add_ns("disk_write_ms", time.perf_counter_ns() - disk_started_ns)
        timing.total_end_to_end_ms = (
            timing.source_read_ms
            + timing.decode_ms
            + timing.total_algorithm_ms
            + timing.writer_queue_ms
            + timing.overlay_ms
            + timing.serialization_ms
            + timing.disk_write_ms
        )
        seam = result.seam_result
        self._summary.writerow(
            [
                result.frame.sequence_id,
                result.frame.original_filename or "",
                result.frame.timestamp_ns,
                seam.detected,
                f"{seam.confidence:.6f}",
                "" if seam.lateral_error_px is None else f"{seam.lateral_error_px:.6f}",
                "" if seam.heading_error_deg is None else f"{seam.heading_error_deg:.6f}",
                len(result.defect_results),
                result.quality_result.valid_for_measurement,
                f"{result.processing_time_ms:.6f}",
                ";".join(result.warnings),
                *(f"{getattr(timing, name):.6f}" for name in STAGE_NAMES),
            ]
        )
        self._summary_handle.flush()
        # Summary-row I/O is intentionally outside the serialized row to avoid
        # reporting a timing value that changes while it is being written.

    def close(self) -> None:
        if not self._summary_handle.closed:
            self._summary_handle.close()

    def __enter__(self) -> "ResultWriter":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
