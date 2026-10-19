from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np

from scripts.experiment_real_seam_band import find_boundary_track, run_experiment, sha256
from tank_vision.config import load_config
from tank_vision.preprocessing.roi import make_image_view
from tank_vision.realtime_pipeline import RealtimeSeamPipeline
from tank_vision.types import FrameData


ROOT = Path(__file__).parents[1]
INPUT = ROOT / "output/s1_real_seam_images/seam_center_full_fov_01.bmp"


def synthetic_wide_band(height: int = 480, width: int = 640) -> np.ndarray:
    image = np.full((height, width), 25, dtype=np.float32)
    yy = np.arange(height, dtype=np.float32)
    left = 210.0 + 0.06 * yy
    right = 315.0 + 0.06 * yy
    for y in range(height):
        image[y, int(left[y]) : int(right[y]) + 1] = 160.0
    noise = np.random.default_rng(7).normal(0.0, 1.5, image.shape)
    return np.clip(image + noise, 0, 255).astype(np.uint8)


def test_synthetic_vertical_wide_band_has_ordered_center() -> None:
    track = find_boundary_track(synthetic_wide_band())
    assert len(track.points) >= 36
    assert (track.points[-1].y - track.points[0].y + 1) / 480 >= 0.60
    assert all(point.left < point.center < point.right for point in track.points)


def test_coordinate_scaling_restores_processed_point_to_original_frame() -> None:
    original = np.zeros((3072, 4096, 3), dtype=np.uint8)
    view = make_image_view(original, None, 896)
    restored = view.to_original(400.0, 300.0)
    assert view.image.shape[:2] == (672, 896)
    assert view.scale_x == view.scale_y == 896 / 4096
    assert np.allclose(restored, (400 / view.scale_x, 300 / view.scale_y))
    assert 0 <= restored[0] < 4096 and 0 <= restored[1] < 3072


def test_boundary_insufficient_is_explicit_failure() -> None:
    track = find_boundary_track(np.full((480, 640), 40, dtype=np.uint8))
    assert not track.points
    assert track.failure_reason in {"no_paired_left_right_edges", "no_continuous_boundary_path"}


def test_real_experiment_is_read_only_and_keeps_production_result(tmp_path: Path) -> None:
    before_hash = sha256(INPUT)
    image = cv2.imread(str(INPUT), cv2.IMREAD_UNCHANGED)
    assert image is not None
    config_path = ROOT / "configs/default.yaml"
    config_before = load_config(config_path)
    frame_before = FrameData(image.copy(), 1, "camera", 0, "test")
    production_before = RealtimeSeamPipeline(config_before).process(frame_before).seam_result
    result = run_experiment(INPUT, config_path, tmp_path / "experiment", 896)
    config_after = load_config(config_path)
    frame_after = FrameData(image.copy(), 1, "camera", 0, "test")
    production_after = RealtimeSeamPipeline(config_after).process(frame_after).seam_result

    assert result["input_unchanged"] is True
    assert sha256(INPUT) == before_hash
    assert result["production_valid"] is False
    assert production_after.detected == production_before.detected
    assert production_after.confidence == production_before.confidence
    assert production_after.lateral_error_px == production_before.lateral_error_px
    assert production_after.heading_error_deg == production_before.heading_error_deg
    assert production_after.lost_reason == production_before.lost_reason
