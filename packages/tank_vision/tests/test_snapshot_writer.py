from __future__ import annotations

import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from tank_vision.seam_tracking_types import StableSeamResult, TrackingState
from tank_vision.snapshot_writer import SnapshotTask, write_snapshot


class _Stamp:
    sec = 1788922627
    nanosec = 983752476


class _Header:
    stamp = _Stamp()
    frame_id = "camera_front"


def _task(image: np.ndarray, state: TrackingState = TrackingState.LOST) -> SnapshotTask:
    result = StableSeamResult(
        detected=state != TrackingState.LOST,
        predicted_only=state == TrackingState.DEGRADED,
        tracking_state=state,
        confidence=0.25,
        roi_original_px=(10, 20, 50, 40),
        coverage=0.5,
        effective_span=0.5,
        failure_reasons=["test_failure"],
    )
    return SnapshotTask(
        image=image,
        header=_Header(),
        result=result,
        geometry={"header": {"stamp": {"sec": _Stamp.sec, "nanosec": _Stamp.nanosec}}, "detected": result.detected},
        request_wall_time=1.0,
        capture_wall_time=2.0,
        source_mode="ros_topic",
        input_shape=tuple(image.shape),
    )


def test_snapshot_writes_same_frame_and_hashes(tmp_path: Path) -> None:
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    image[:, :, 0] = np.arange(100, dtype=np.uint8)[None, :]
    config = tmp_path / "config.yaml"
    reference = tmp_path / "reference.json"
    config.write_text("schema_version: test\n")
    reference.write_text("{}\n")

    directory = write_snapshot(_task(image), tmp_path / "snapshots", config, reference)
    raw = cv2.imread(str(directory / "raw.bmp"), cv2.IMREAD_UNCHANGED)
    assert raw is not None
    assert np.array_equal(raw, image)
    assert cv2.imread(str(directory / "roi_raw.bmp"), cv2.IMREAD_UNCHANGED).shape == (40, 50, 3)
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["complete"] is True
    assert metadata["header_stamp"] == {"sec": _Stamp.sec, "nanosec": _Stamp.nanosec}
    assert metadata["raw_sha256"] == hashlib.sha256((directory / "raw.bmp").read_bytes()).hexdigest()
    assert metadata["seam_geometry_sha256"] == hashlib.sha256((directory / "seam_geometry.json").read_bytes()).hexdigest()
    assert set(p.name for p in directory.iterdir()) == {
        "raw.bmp", "overlay.png", "roi_raw.bmp", "roi_overlay.png", "seam_geometry.json", "metadata.json"
    }


def test_lost_frame_is_saved_without_fabricated_lines(tmp_path: Path) -> None:
    image = np.full((80, 100, 3), 17, dtype=np.uint8)
    directory = write_snapshot(_task(image, TrackingState.LOST), tmp_path / "snapshots", __file__, __file__)
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["complete"] is True
    assert metadata["detected"] is False
    assert metadata["tracking_state"] == "LOST"
    assert json.loads((directory / "seam_geometry.json").read_text())["detected"] is False


def test_bad_dtype_produces_incomplete_metadata(tmp_path: Path) -> None:
    image = np.zeros((80, 100, 3), dtype=np.float32)
    directory = write_snapshot(_task(image), tmp_path / "snapshots", __file__, __file__)
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["complete"] is False
    assert "uint8" in metadata["error"]


def test_repeated_requests_get_unique_directories(tmp_path: Path) -> None:
    image = np.zeros((80, 100, 3), dtype=np.uint8)
    first = write_snapshot(_task(image), tmp_path / "snapshots", __file__, __file__)
    second = write_snapshot(_task(image), tmp_path / "snapshots", __file__, __file__)
    assert first != second
    assert json.loads((first / "metadata.json").read_text())["complete"] is True
    assert json.loads((second / "metadata.json").read_text())["complete"] is True
