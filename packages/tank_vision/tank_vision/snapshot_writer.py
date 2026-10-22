"""Failure-snapshot serialization helpers.

The stable seam ROS node uses this module only after a one-shot request has
been accepted.  It deliberately contains no ROS or camera code: a task owns
one copied input frame and its already computed result, and the writer turns
that task into a self-contained, hashable evidence directory.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
from typing import Any
import uuid

import cv2
import numpy as np

from tank_vision.seam_tracking_types import StableSeamResult
from tank_vision.stable_seam_tracker import draw_stable_seam_overlay


@dataclass(slots=True)
class SnapshotTask:
    image: np.ndarray
    header: Any
    result: StableSeamResult
    geometry: dict[str, Any]
    request_wall_time: float
    capture_wall_time: float
    source_mode: str
    input_shape: tuple[int, ...]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _stamp(header: Any) -> dict[str, int]:
    stamp = getattr(header, "stamp", None)
    return {"sec": int(getattr(stamp, "sec", 0)), "nanosec": int(getattr(stamp, "nanosec", 0))}


def _snapshot_overlay(image: np.ndarray, result: StableSeamResult) -> np.ndarray:
    """Draw the same detector geometry, with diagnostic status values added."""
    overlay = draw_stable_seam_overlay(image, result, None)
    lines = [
        f"coverage={result.coverage:.3f} span={result.effective_span:.3f} "
        f"lateral={_number(result.lateral_error_px)}px heading={_number(result.heading_error_deg)}deg",
        f"residual={_number(result.fit_residual_px)}px inlier={result.fit_inlier_ratio:.3f}",
    ]
    for index, line in enumerate(lines, start=1):
        y = 95 + index * 38
        cv2.putText(overlay, line, (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(overlay, line, (30, y), cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 1, cv2.LINE_AA)
    return overlay


def _number(value: float | None) -> str:
    return "nan" if value is None else f"{float(value):.2f}"


def _unique_directory(root: Path, stamp: dict[str, int]) -> Path:
    wall = datetime.now(timezone.utc).astimezone().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    base = root / f"{wall}_stamp_{stamp['sec']}_{stamp['nanosec']}"
    candidate = base
    while candidate.exists():
        candidate = root / f"{base.name}_{uuid.uuid4().hex[:8]}"
    candidate.mkdir(parents=True, exist_ok=False)
    return candidate


def write_snapshot(task: SnapshotTask, output_root: str | Path, config_path: str | Path,
                   reference_path: str | Path) -> Path:
    """Write one complete snapshot, or a ``complete=false`` failure record."""
    root = Path(output_root).expanduser()
    stamp = _stamp(task.header)
    directory = _unique_directory(root, stamp)
    metadata: dict[str, Any] = {
        "schema_version": "1.0",
        "reason": "manual_failure_snapshot",
        "request_wall_time": task.request_wall_time,
        "capture_wall_time": task.capture_wall_time,
        "header_stamp": stamp,
        "frame_id": str(getattr(task.header, "frame_id", "")),
        "source_mode": task.source_mode,
        "raw_shape": list(task.image.shape),
        "raw_dtype": str(task.image.dtype),
        "raw_channels": int(task.image.shape[2]) if task.image.ndim == 3 else 1,
        "roi_original_px": list(task.result.roi_original_px),
        "tracker_config_path": str(Path(config_path).expanduser().resolve()),
        "width_reference_path": str(Path(reference_path).expanduser().resolve()),
        "detected": bool(task.result.detected),
        "predicted_only": bool(task.result.predicted_only),
        "tracking_state": task.result.tracking_state.name,
        "confidence": float(task.result.confidence),
        "coverage": float(task.result.coverage),
        "complete": False,
    }
    try:
        config_file = Path(config_path).expanduser()
        reference_file = Path(reference_path).expanduser()
        metadata["tracker_config_sha256"] = sha256_file(config_file)
        metadata["width_reference_sha256"] = sha256_file(reference_file)
        raw = np.asarray(task.image)
        if raw.dtype != np.uint8:
            raise ValueError(f"snapshot image must be uint8, got {raw.dtype}")
        if not cv2.imwrite(str(directory / "raw.bmp"), raw):
            raise OSError("cv2.imwrite failed for raw.bmp")
        overlay = _snapshot_overlay(raw, task.result)
        if not cv2.imwrite(str(directory / "overlay.png"), overlay):
            raise OSError("cv2.imwrite failed for overlay.png")
        x, y, width, height = (int(value) for value in task.result.roi_original_px)
        roi_raw = raw[y:y + height, x:x + width].copy()
        roi_overlay = overlay[y:y + height, x:x + width].copy()
        if not cv2.imwrite(str(directory / "roi_raw.bmp"), roi_raw):
            raise OSError("cv2.imwrite failed for roi_raw.bmp")
        if not cv2.imwrite(str(directory / "roi_overlay.png"), roi_overlay):
            raise OSError("cv2.imwrite failed for roi_overlay.png")
        (directory / "seam_geometry.json").write_text(
            json.dumps(task.geometry, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
        )
        metadata.update({
            "raw_sha256": sha256_file(directory / "raw.bmp"),
            "overlay_sha256": sha256_file(directory / "overlay.png"),
            "roi_raw_sha256": sha256_file(directory / "roi_raw.bmp"),
            "seam_geometry_sha256": sha256_file(directory / "seam_geometry.json"),
            "complete": True,
        })
    except Exception as exc:  # noqa: BLE001 - writer must never kill real-time processing
        metadata["complete"] = False
        metadata["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        # This is intentionally the final write in the directory.  Consumers
        # may safely ignore any directory without complete=true.
        (directory / "metadata.json").write_text(
            json.dumps(metadata, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8"
        )
    return directory
