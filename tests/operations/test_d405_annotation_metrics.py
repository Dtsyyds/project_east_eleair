"""Missing predictions and unobserved gaps must not inflate label-based accuracy."""
import importlib.util
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.dataset import save_sample


spec = importlib.util.spec_from_file_location('annotation_metrics', Path(__file__).resolve().parents[2] / 'experiments/vision/scripts/evaluate_d405_annotations.py')
metrics = importlib.util.module_from_spec(spec)
spec.loader.exec_module(metrics)


def label():
    return dict(status='approved', origin='human', author='test_fixture', no_seam=False,
                left_xy=[[10, 0], [10, 20]], right_xy=[[20, 0], [20, 20]])


def prediction(points, **kwargs):
    return dict(valid_2d=True, predicted_only=False, points_2d_y_left_right=points, **kwargs)


def test_gap_and_outside_prediction_are_not_interpolated():
    frame = SimpleNamespace(image=np.zeros((30, 40), np.uint8), image_stamp_ns=1)
    result = prediction([[2, 10, 20], [5, 10, 20], [17, 10, 20], [19, 10, 20]])
    row = metrics.score(frame, label(), result)
    assert row['annotated_rows'] == 21
    assert row['covered_rows'] == 7  # 2..5 and 17..19; never bridge 5..17.
    assert row['hits']['2']['both_boundaries'] == 7
    assert row['errors']['center']['mean'] == 0


def test_center_can_be_right_while_boundaries_are_wrong():
    frame = SimpleNamespace(image=np.zeros((30, 40), np.uint8), image_stamp_ns=1)
    result = prediction([[y, 14, 16] for y in range(21)])
    row = metrics.score(frame, label(), result)
    assert row['hits']['2']['center'] == 21
    assert row['hits']['2']['both_boundaries'] == 0
    assert row['errors']['width']['mean'] == 8
    result['predicted_only'] = True
    assert metrics.score(frame, label(), result)['covered_rows'] == 0


@pytest.mark.parametrize('points', [[], [[1, 20, 10], [2, 20, 10]], [[1, 10, 20], [1, 10, 20]], [[1, 10, 20], [2, float('nan'), 20]]])
def test_malformed_valid_prediction_rejected(points):
    frame = SimpleNamespace(image=np.zeros((30, 40), np.uint8), image_stamp_ns=1)
    with pytest.raises(ValueError):
        metrics.score(frame, label(), prediction(points))


def test_missing_frames_reduce_recall_and_unknown_sources_are_rejected(tmp_path):
    records = []
    for stamp in (1, 2):
        records.append(save_sample(tmp_path, synthetic_frame(stamp=stamp), f'f{stamp}', 'fixture', annotation=label()))
    index = tmp_path/'index.json'
    index.write_text(json.dumps(dict(schema='d405_dataset_v1', samples=records)))
    result = prediction([[y, 10, 20] for y in range(21)], source={'image_stamp_ns':'1'})
    output = tmp_path/'results.jsonl'
    output.write_text(json.dumps(result)+'\n')
    summary = metrics.evaluate(index, output)['summary']
    assert summary['covered_fraction'] == 0.5
    assert summary['center_row_recall']['2'] == 0.5
    assert summary['errors_on_covered_rows']['center'] == 0
    assert summary['negative_false_positive_rate'] is None
    output.write_text((json.dumps(result)+'\n')*2)
    with pytest.raises(ValueError, match='duplicate_result'):
        metrics.evaluate(index, output)
    result['source']['image_stamp_ns'] = '3'
    output.write_text(json.dumps(result)+'\n')
    with pytest.raises(ValueError, match='unmatched_prediction'):
        metrics.evaluate(index, output)


def test_explicit_negative_contributes_false_positive_rate(tmp_path):
    annotation = dict(status='approved', origin='human', author='test_fixture', no_seam=True)
    rec = save_sample(tmp_path, synthetic_frame(stamp=1), 'negative', 'fixture', annotation=annotation)
    index = tmp_path/'index.json'
    index.write_text(json.dumps(dict(schema='d405_dataset_v1', samples=[rec])))
    output = tmp_path/'results.jsonl'
    output.write_text(json.dumps(prediction([[0, 10, 20], [1, 10, 20]], source={'image_stamp_ns':'1'}))+'\n')
    summary = metrics.evaluate(index, output)['summary']
    assert summary['negative_false_positive_rate'] == 1
    assert summary['center_row_recall']['5'] is None


@pytest.fixture
def region():
    directory = Path(__file__).resolve().parents[2] / 'experiments/vision/scripts'
    sys.path.insert(0, str(directory))
    try:
        spec = importlib.util.spec_from_file_location('region_experiment', directory/'experiment_d405_region.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


def test_region_requires_evidence_and_preserves_large_gap(region):
    probability = np.zeros((120, 160), np.float32)
    assert region.region_path(probability) == []
    probability[:, 70:90] = 0.95
    points = np.asarray(region.region_path(probability))
    assert len(points) == 120
    assert np.max(abs((points[:, 1]+points[:, 2])/2-159.5)) < 3
    assert np.median(points[:, 2]-points[:, 1]) == pytest.approx(40, abs=4)
    probability[45:75] = 0
    points = np.asarray(region.region_path(probability))
    assert not ((points[:, 0] > 100) & (points[:, 0] < 140)).any()


def test_training_ignores_unknown_y_and_ambiguous_boundaries(region):
    frame = SimpleNamespace(image=np.zeros((30, 40, 3), np.uint8))
    yy, xx = np.mgrid[:15, :20]
    feat = np.stack((2*yy+0.5, 2*xx+0.5), axis=-1).astype(np.float32)
    samples, labels = region.training_pixels(frame, label(), feat, np.random.default_rng(1))
    assert (samples[:, 0] <= 20).all()
    assert ((samples[labels == 1, 1] > 12) & (samples[labels == 1, 1] < 18)).all()
    assert ((samples[labels == 0, 1] < 8) | (samples[labels == 0, 1] > 22)).all()
