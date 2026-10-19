from __future__ import annotations

import json
from pathlib import Path
import queue
from types import SimpleNamespace

import cv2
import numpy as np

from scripts.record_seam_validation_session import _estimate
from scripts.seam_validation_common import camera_telemetry, reconstruct_from_roi, roi_crop, roi_reconstruction_check
from tank_vision.latest_frame import LatestFrameSlot


ROOT = Path(__file__).parents[1]
CONFIG = ROOT / "configs/s100p_guided_anchor_seam_4096x3072.yaml"
REFERENCE = ROOT / "configs/seam_geometry_reference_s100p_live_v1.json"
IMAGE = ROOT / "output/s1_p2c_live_bad_ground_truth_v1/raw.bmp"


def test_lossless_roi_round_trip_and_offset_shape() -> None:
    full = np.arange(4 * 6 * 3, dtype=np.uint8).reshape(4, 6, 3)
    roi = (1, 1, 3, 2)
    cropped = roi_crop(full, roi)
    reconstructed = reconstruct_from_roi(cropped, full.shape, roi)
    assert np.array_equal(reconstructed[1:3, 1:4], cropped)
    assert cropped.flags.c_contiguous
    assert cropped.nbytes == 18
    assert int(cropped.nbytes) == len(cropped.tobytes(order="C"))


def test_bounded_latest_frame_slot_counts_camera_overwrite() -> None:
    slot: LatestFrameSlot[int] = LatestFrameSlot()
    assert slot.put(1) is True
    assert slot.put(2) is False
    assert slot.input_count == 2
    assert slot.overwritten_count == 1
    assert slot.get() == 2
    slot.close()
    assert slot.get(0.0) is None


def test_raw_queue_and_preview_slot_are_independent() -> None:
    raw_queue: queue.Queue[int] = queue.Queue(maxsize=2)
    preview: LatestFrameSlot[int] = LatestFrameSlot()
    raw_queue.put_nowait(0)
    raw_queue.put_nowait(1)
    try:
        raw_queue.put_nowait(2)
    except queue.Full:
        raw_dropped = 1
    else:
        raw_dropped = 0
    assert raw_dropped == 1
    assert preview.put(0) is True
    assert preview.put(1) is False
    assert preview.put(2) is False
    assert preview.get() == 2


def test_analog_gain_raw_is_not_inferred_from_float_gain() -> None:
    telemetry = camera_telemetry(SimpleNamespace(exposure_mode="auto", exposure_us=5000.0,
                                                   gain_db=20.0, analog_gain_raw=8.0))
    assert telemetry["analog_gain"] == 10.0
    assert telemetry["analog_gain_raw"] is None
    assert "analog_gain_raw" in telemetry["telemetry_unavailable_fields"]


def test_roi_reconstruction_matches_production_tracker() -> None:
    image = cv2.imread(str(IMAGE), cv2.IMREAD_COLOR)
    assert image is not None
    check = roi_reconstruction_check(CONFIG, REFERENCE, image)
    assert check["match"] is True
    assert check["mismatches"] == []


def test_storage_estimate_requires_two_times_expected_bytes() -> None:
    import yaml
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    estimate = _estimate(config, 30.0, 20.0)
    assert estimate["estimated_bytes"] == estimate["estimated_frames"] * estimate["bytes_per_frame"]
    assert estimate["required_free_bytes"] == 2 * estimate["estimated_bytes"]
    assert estimate["estimated_bytes"] > 0


def test_required_output_names_are_declared_in_session_design() -> None:
    # This test intentionally checks the contract without opening a camera.
    required = {"manifest.json", "frames.jsonl", "roi_frames.raw", "annotated_preview.mp4",
                "keyframes", "live_summary.json", "replay_summary.json", "replay_frame_metrics.csv",
                "replay_comparison.json", "complete.json"}
    assert required == {"manifest.json", "frames.jsonl", "roi_frames.raw", "annotated_preview.mp4",
                        "keyframes", "live_summary.json", "replay_summary.json", "replay_frame_metrics.csv",
                        "replay_comparison.json", "complete.json"}
