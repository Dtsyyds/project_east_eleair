from __future__ import annotations

import copy
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from tank_vision.latest_frame import LatestFrameSlot
from tank_vision.seam_tracking_types import TrackingState
from tank_vision.seam_width_reference import SeamWidthReference, sha256_file
from tank_vision.stable_seam_tracker import BoundaryPair, StableSeamTracker, robust_polynomial_fit


def _reference() -> SeamWidthReference:
    return SeamWidthReference(160, 120, np.asarray([0.0, .33, .66, 1.0]),
                              np.asarray([.5, .5, .5, .5]), np.asarray([.20, .20, .20, .20]), {})


def _config() -> dict:
    return {
        "schema_version": "1.0", "camera": {"expected_width": 160, "expected_height": 120, "same_fov_scaled": False},
        "processing": {"max_roi_width": 160, "scan_band_step_px": 4, "scan_band_half_height_px": 2,
                       "gaussian_kernel": 5, "profile_gaussian_kernel": 5, "top_k_edges_per_side": 12,
                       "sobel_kernel": 3, "edge_percentile": 75.0, "edge_min_response": .2,
                       "edge_min_separation_px": 3, "edge_margin_ratio": .02, "edge_margin_min_px": 2},
        "roi": {"x_min_ratio": .15, "x_max_ratio": .85, "y_min_ratio": .2, "y_max_ratio": .95},
        "reference": {"path": "unused"},
        "pairing": {"width_min_scale": .55, "width_max_scale": 1.8, "max_pairs_per_band": 12,
                    "interior_margin_px": 1, "exterior_sample_width_scale": .25, "exterior_sample_min_px": 3,
                    "contrast_score_offset": .5, "contrast_normalization_gray": 80.0,
                    "initial_reference_center_weight": .75, "center_prior_decay_width_scale": 2.5,
                    "temporal_center_decay_width_scale": 1.0, "unavailable_temporal_score": .5,
                    "score_weights": {"left_edge_strength": .10, "right_edge_strength": .10,
                                      "edge_balance": .12, "contrast": .18,
                                      "width_reference": .24, "center_prior": .10, "temporal_center": .09, "temporal_width": .07}},
        "path": {"center_transition_weight": .42, "width_transition_weight": .24, "left_transition_weight": .12,
                 "right_transition_weight": .12, "skip_penalty": .38, "maximum_missing_run": 3,
                 "max_center_jump_width_scale": .8, "max_boundary_jump_width_scale": 1.1, "max_width_ratio_change": .55},
        "geometry": {"fit_degree": 1, "width_fit_degree": 1, "robust_fit_method": "irls_huber",
                     "robust_fit_iterations": 8, "robust_fit_huber_delta_px": 4.0, "minimum_point_count": 8,
                     "width_fit_huber_delta_scale": 2.0,
                     "minimum_coverage": .60, "minimum_effective_span": .65, "maximum_missing_run": 3,
                     "minimum_inlier_ratio": .7, "maximum_fit_residual_px": 4.0, "maximum_abrupt_jump_count": 3,
                     "abrupt_jump_width_scale": .65, "minimum_path_confidence": .35, "reference_y_ratio": .85},
        "tracking": {"enabled": True, "acquiring_success_frames": 3, "degraded_max_frames": 2,
                     "lost_frame_threshold": 3, "ema_alpha": .35, "max_lateral_jump_width_scale": .8,
                     "max_width_change_ratio": .4, "max_heading_change_deg": 8.0, "tracking_search_width_scale": 2.5,
                     "predicted_confidence_decay": .5},
        "output": {"centerline_sample_count": 16, "publish_debug_image": False, "publish_raw_image": False},
    }


def _band(mono: bool = False) -> np.ndarray:
    image = np.full((120, 160), 35, np.uint8)
    image[:, 64:96] = 190
    image = cv2.GaussianBlur(image, (5, 5), 0)
    return image if mono else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def test_reference_interpolation_and_resolution_guard() -> None:
    ref = _reference()
    center, width = ref.at(60, 160, 120)
    assert float(center) == pytest.approx(80)
    assert float(width) == pytest.approx(32)
    ref.validate_image(160, 120, False)
    with pytest.raises(ValueError, match="differs from reference"):
        ref.validate_image(80, 60, False)
    ref.validate_image(80, 60, True)


def test_tracker_refuses_unmapped_resolution_with_clear_lost_reason() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    result = tracker.process(cv2.resize(_band(), (80, 60)))
    assert not result.detected and result.tracking_state == TrackingState.LOST
    assert "set camera.same_fov_scaled=true only for a verified same-FOV resize" in result.failure_reasons[0]


def test_verified_same_fov_scaled_coordinates_remain_in_input_pixels() -> None:
    config = copy.deepcopy(_config())
    config["camera"]["same_fov_scaled"] = True
    tracker = StableSeamTracker(config, _reference())
    result = tracker.process(cv2.resize(_band(), (80, 60)))
    assert result.detected
    assert result.roi_original_px == (12, 12, 56, 45)
    assert all(0 <= x < 80 and 0 <= y < 60 for x, y in result.centerline_original_px)


@pytest.mark.parametrize("mono", [False, True])
def test_color_mono_boundaries_order_and_original_coordinates(mono: bool) -> None:
    tracker = StableSeamTracker(_config(), _reference())
    result = tracker.process(_band(mono), diagnostics=True)
    assert result.detected
    assert result.roi_original_px == (24, 24, 112, 90)
    assert result.left_boundary_original_px
    for left, center, right in zip(result.left_boundary_original_px, result.centerline_original_px, result.right_boundary_original_px):
        assert 0 <= left[0] < center[0] < right[0] < 160
        assert 0 <= center[1] < 120
    for candidates in tracker.last_diagnostics.all_pairs:
        assert all(pair.left < pair.right for pair in candidates)
        assert all(.55 * 32 <= pair.width <= 1.8 * 32 for pair in candidates)


def test_signed_edges_and_dp_continuous_path() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    result = tracker.process(_band(), diagnostics=True)
    assert result.coverage >= .6
    path = tracker.last_diagnostics.selected_pairs
    assert len(path) >= 8
    roi_x = result.roi_original_px[0]
    assert np.median([pair.left + roi_x for pair in path]) < 80 < np.median([pair.right + roi_x for pair in path])


def test_dp_can_skip_one_nonempty_reflection_band() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    good = lambda band: BoundaryPair(band, float(20 + band * 4), 64.0, 96.0, 0.8, {})
    reflection = BoundaryPair(2, 28.0, 110.0, 142.0, 0.95, {})
    path = tracker._select_path(
        [[good(0)], [good(1)], [reflection], [good(3)], [good(4)]],
        (0, 0, 160, 120), 1.0, 1.0, 160, 120)
    assert [pair.band_index for pair in path] == [0, 1, 3, 4]


def test_robust_fit_rejects_outlier() -> None:
    y = np.arange(30.0)
    x = 40 + .1 * y
    x[12] += 30
    fit = robust_polynomial_fit(y, x, 1, 8, 2.0)
    assert fit is not None
    coefficients, inliers, residual = fit
    assert not inliers[12]
    assert residual < .1
    assert np.polyval(coefficients, 20) == pytest.approx(42, abs=.1)


def test_temporal_state_degraded_lost_and_reacquire() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    assert tracker.process(_band()).tracking_state == TrackingState.ACQUIRING
    tracker.process(_band())
    assert tracker.process(_band()).tracking_state == TrackingState.TRACKING
    blank = np.full((120, 160, 3), 35, np.uint8)
    first = tracker.process(blank)
    assert not first.detected and first.predicted_only and first.tracking_state == TrackingState.DEGRADED
    tracker.process(blank)
    third = tracker.process(blank)
    assert not third.detected and not third.predicted_only and third.tracking_state == TrackingState.LOST
    assert tracker.process(_band()).tracking_state == TrackingState.ACQUIRING


def test_jump_rejected_without_updating_tracker() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    for _ in range(3):
        tracker.process(_band())
    shifted = np.full((120, 160), 35, np.uint8)
    shifted[:, 96:128] = 190
    result = tracker.process(cv2.cvtColor(shifted, cv2.COLOR_GRAY2BGR))
    assert not result.detected
    assert result.predicted_only


def test_ema_filters_current_measurement_without_fabricating_detection() -> None:
    tracker = StableSeamTracker(_config(), _reference())
    baseline = tracker.process(_band())
    moved = np.full((120, 160), 35, np.uint8)
    moved[:, 68:100] = 190
    moved_result = tracker.process(cv2.cvtColor(moved, cv2.COLOR_GRAY2BGR))
    assert baseline.detected and moved_result.detected and not moved_result.predicted_only
    assert baseline.lateral_error_px < moved_result.lateral_error_px < 3.6


def test_no_debug_work_when_disabled(monkeypatch: pytest.MonkeyPatch) -> None:
    tracker = StableSeamTracker(_config(), _reference())
    monkeypatch.setattr(cv2, "imshow", lambda *_: pytest.fail("imshow must not be called"))
    tracker.process(_band(), diagnostics=False)
    assert tracker.last_diagnostics is None


def test_latest_frame_only_overwrites_and_closes() -> None:
    slot: LatestFrameSlot[int] = LatestFrameSlot()
    assert slot.put(1)
    assert not slot.put(2)
    assert slot.input_count == 2 and slot.overwritten_count == 1
    assert slot.get() == 2
    slot.close()
    assert slot.get(0) is None and not slot.put(3)


def test_frozen_reference_source_hashes() -> None:
    root = Path(__file__).parents[1]
    manifest = json.loads((root / "output/s1_d7_manual_ground_truth_v2/ground_truth_manifest.json").read_text())
    assert sha256_file(root / manifest["input_image_relative"]) == manifest["input_sha256"]
    assert sha256_file(root / "output/s1_d7_manual_ground_truth_v2/seam_center_lit_quality_01_annotation_v2.json") == manifest["json_sha256"]
    assert sha256_file(root / "output/s1_d7_manual_ground_truth_v2/seam_center_lit_quality_01_annotation_overlay_v2.png") == manifest["overlay_sha256"]
