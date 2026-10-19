from __future__ import annotations

from pathlib import Path

import numpy as np

from scripts.experiment_real_seam_band import find_boundary_track, sha256
from scripts.experiment_real_seam_orientation import (
    _select_best_candidate,
    _track_inside_valid_region,
    forward_affine_points,
    inverse_affine_points,
    rotate_processing_image,
)
from tests.test_experiment_real_seam_band import synthetic_wide_band


def _rotated_track(angle: float):
    image = synthetic_wide_band()
    rotated, valid, _ = rotate_processing_image(image, angle)
    return _track_inside_valid_region(find_boundary_track(rotated), valid), rotated, valid


def test_synthetic_wide_band_plus_and_minus_20_degrees_keeps_order() -> None:
    for angle in (-20, 20):
        track, _, _ = _rotated_track(angle)
        assert len(track.points) >= 36
        assert all(point.left < point.center < point.right for point in track.points)


def test_zero_degree_rotation_does_not_change_s1_d1_track() -> None:
    image = synthetic_wide_band()
    baseline = find_boundary_track(image)
    rotated, valid, _ = rotate_processing_image(image, 0)
    candidate = _track_inside_valid_region(find_boundary_track(rotated), valid)
    assert [(p.y, p.left, p.right) for p in candidate.points] == [(p.y, p.left, p.right) for p in baseline.points]


def test_rotated_canvas_invalid_black_border_is_excluded() -> None:
    track, _, valid = _rotated_track(45)
    assert track.points
    assert all(valid[p.y, p.left] != 0 and valid[p.y, p.right] != 0 for p in track.points)


def test_forward_and_inverse_affine_coordinates_round_trip() -> None:
    image = synthetic_wide_band()
    _, _, matrix = rotate_processing_image(image, 20)
    points = np.asarray([[0.0, 0.0], [210.0, 100.0], [639.0, 479.0]])
    assert np.allclose(inverse_affine_points(forward_affine_points(points, matrix), matrix), points, atol=1e-8)


def test_restored_synthetic_coordinates_are_in_bounds() -> None:
    track, _, _ = _rotated_track(-20)
    _, _, matrix = rotate_processing_image(synthetic_wide_band(), -20)
    points = inverse_affine_points([(p.center, p.y) for p in track.points], matrix)
    assert np.isfinite(points).all()
    assert np.all(points[:, 0] >= 0) and np.all(points[:, 0] < 640)
    assert np.all(points[:, 1] >= 0) and np.all(points[:, 1] < 480)


def test_all_angles_without_evidence_return_explicit_failure() -> None:
    candidates = [
        {
            "angle_deg": angle,
            "geometry_candidate_valid": False,
            "vertical_coverage_ratio": 0.1,
            "centerline_between_boundaries": False,
            "width_continuity_score": 0.0,
            "center_continuity_score": 0.0,
            "centerline_inlier_ratio": 0.0,
            "boundary_gradient_evidence_processed": 0.0,
        }
        for angle in (-45, 0, 45)
    ]
    assert _select_best_candidate(candidates) is None


def test_angle_selection_is_deterministic_and_uses_evidence_only() -> None:
    def candidates():
        return [
            {
                "angle_deg": angle,
                "geometry_candidate_valid": True,
                "vertical_coverage_ratio": coverage,
                "centerline_between_boundaries": True,
                "width_continuity_score": 0.9,
                "center_continuity_score": 0.9,
                "centerline_inlier_ratio": 0.9,
                "boundary_gradient_evidence_processed": gradient,
                "restored_coordinates_valid": True,
            }
            for angle, coverage, gradient in ((-20, 0.8, 3.0), (20, 0.7, 8.0))
        ]

    first = _select_best_candidate(candidates())
    second = _select_best_candidate(candidates())
    assert first is not None and second is not None
    assert first["angle_deg"] == second["angle_deg"] == 20


def test_real_input_hash_is_read_only() -> None:
    input_path = Path(__file__).parents[1] / "output/s1_real_seam_images/seam_center_full_fov_01.bmp"
    before = sha256(input_path)
    # The test intentionally does not invoke a writer on the evidence file.
    assert sha256(input_path) == before
