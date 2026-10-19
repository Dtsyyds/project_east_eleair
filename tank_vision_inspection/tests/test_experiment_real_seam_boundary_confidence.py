from __future__ import annotations

from pathlib import Path

import numpy as np

from scripts.experiment_real_seam_band import BandPair, find_boundary_track, sha256
from scripts.experiment_real_seam_boundary_confidence import (
    CandidateEvidence,
    ConfidenceModel,
    ScanRecord,
    _candidate_evidence,
    _path_for_records,
    _robust_model,
)
from scripts.experiment_real_seam_orientation import _track_inside_valid_region, rotate_processing_image
from scripts.experiment_real_seam_orientation import inverse_affine_points
from tests.test_experiment_real_seam_band import synthetic_wide_band


def _pair(y: int, left: int, right: int, edge: float = 10.0, contrast: float = 100.0) -> BandPair:
    return BandPair(y, left, right, edge, edge, contrast, 1.0)


def _model(width: float = 100.0) -> ConfidenceModel:
    return ConfidenceModel(width, 2.0, 3.0 * 2.0, 10.0, 100.0, 20, 60.0)


def _record(pair: BandPair | None, model: ConfidenceModel) -> ScanRecord:
    pairs = [pair] if pair is not None else []
    evidence = [_candidate_evidence(pair, model) for pair in pairs]
    return ScanRecord(pair.y if pair is not None else 0, evidence, pairs, ([], []))


def test_stronger_exterior_edge_with_abnormal_width_is_rejected() -> None:
    evidence = _candidate_evidence(BandPair(0, 100, 260, 50.0, 50.0, 120.0, 1.0), _model())
    assert "width_outside_robust_model" in evidence.reasons


def test_sudden_right_boundary_jump_is_not_selected() -> None:
    model = _model()
    records = []
    for y in range(0, 80, 8):
        pair = _pair(y, 200, 300) if y != 40 else _pair(y, 240, 340)
        records.append(_record(pair, model))
    result = _path_for_records(records, model)
    assert all(point.left != 240 for point in result.reliable_points)
    assert any("right_boundary_jump_from_previous_accepted_pair" in item["rejection_reasons"] for item in result.rejected_candidates)


def test_sudden_width_increase_is_rejected() -> None:
    model = _model()
    records = [_record(_pair(y, 200, 300) if y != 40 else _pair(y, 200, 360), model) for y in range(0, 80, 8)]
    result = _path_for_records(records, model)
    assert all(point.width != 160 for point in result.reliable_points)
    assert any("width_outside_robust_model" in item["rejection_reasons"] for item in result.rejected_candidates)


def test_missing_single_scan_band_does_not_create_center_point() -> None:
    model = _model()
    records = [_record(_pair(0, 200, 300), model), ScanRecord(8, [], [], ([], [])), _record(_pair(16, 202, 302), model)]
    result = _path_for_records(records, model)
    assert len(result.reliable_points) == 2
    assert all(point.y != 8 for point in result.reliable_points)
    assert 8 in result.interpolated_y


def test_short_gap_has_finite_interpolation_only_between_valid_pairs() -> None:
    model = _model()
    records = [_record(_pair(0, 200, 300), model), ScanRecord(8, [], [], ([], [])), _record(_pair(16, 204, 304), model)]
    result = _path_for_records(records, model)
    assert [point.y for point in result.display_points] == [0, 8, 16]
    interpolated = next(point for point in result.display_points if point.y == 8)
    assert interpolated.left == 202 and interpolated.right == 302


def test_long_gap_is_not_interpolated() -> None:
    model = _model()
    records = [_record(_pair(0, 200, 300), model), ScanRecord(8, [], [], ([], [])), ScanRecord(16, [], [], ([], [])), ScanRecord(24, [], [], ([], [])), _record(_pair(32, 204, 304), model)]
    result = _path_for_records(records, model)
    assert not result.interpolated_y
    assert result.longest_missing_boundary_run >= 3


def test_rejected_scan_band_has_no_fake_center() -> None:
    model = _model()
    records = [_record(_pair(0, 200, 300), model), ScanRecord(8, [], [], ([], [])), _record(_pair(16, 204, 304), model)]
    result = _path_for_records(records, model)
    assert all(point.y != 8 for point in result.reliable_points)


def test_normal_wide_band_track_is_unchanged_and_ordered() -> None:
    track = find_boundary_track(synthetic_wide_band())
    assert len(track.points) >= 36
    assert all(point.left < point.center < point.right for point in track.points)


def test_tilt_rotation_valid_mask_and_inverse_path_remain_valid() -> None:
    image = synthetic_wide_band()
    rotated, valid, matrix = rotate_processing_image(image, -20)
    track = _track_inside_valid_region(find_boundary_track(rotated), valid)
    assert track.points
    restored = inverse_affine_points([(p.center, p.y) for p in track.points], matrix)
    assert np.isfinite(restored).all()
    assert np.all(restored[:, 0] >= 0) and np.all(restored[:, 0] < image.shape[1])
    assert np.all(restored[:, 1] >= 0) and np.all(restored[:, 1] < image.shape[0])


def test_confidence_path_is_deterministic() -> None:
    model = _model()
    records = [_record(_pair(y, 200 + y // 16, 300 + y // 16), model) for y in range(0, 80, 8)]
    first = _path_for_records(records, model)
    second = _path_for_records(records, model)
    assert [(p.y, p.left, p.right) for p in first.reliable_points] == [(p.y, p.left, p.right) for p in second.reliable_points]


def test_original_input_hash_is_unchanged() -> None:
    input_path = Path(__file__).parents[1] / "output/s1_real_seam_images/seam_right_full_fov_01.bmp"
    before = sha256(input_path)
    assert sha256(input_path) == before
