"""Check experiment-specific filtering without training or touching hardware."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'experiments/vision/scripts'))
import d405_central_window_ab as ab


@pytest.fixture
def config():
    return json.loads((ROOT/'.runtime/s100p/analysis/d405_unet_round1_20261102/visual_config.json').read_text())


def decode(prob, config):
    observer, _ = ab.expose_paths()
    bounds = ab.geometry.roi_bounds(prob.shape, config)
    rows, confidence, layers = observer(prob, config, bounds)
    paths = ab.paths_from_layers(layers, prob.shape[1])
    selected = next((p for p in paths if p['inside']), None)
    return paths, ab.fit_path(selected, prob, config, bounds)


@pytest.mark.parametrize('x,inside', [(191,False),(192,True),(447,True),(448,False)])
def test_original_coordinates_half_open_window(config, x, inside):
    prob = np.zeros((480,640), np.float32)
    prob[168:456, x-4:x+5] = .9
    paths, result = decode(prob, config)
    assert paths[0]['median_x'] == x
    assert paths[0]['inside'] is inside
    assert result['valid'] is inside


def test_lower_ranked_central_candidate_can_replace_outer_winner(config):
    prob = np.zeros((480,640), np.float32)
    prob[168:456, 160:169] = .99
    prob[168:456, 320:329] = .8
    paths, result = decode(prob, config)
    assert paths[0]['median_x'] == 164 and not paths[0]['inside']
    assert result['valid']
    np.testing.assert_allclose(np.asarray(result['points'])[:,1:].mean(1), 324)


def test_window_does_not_relax_original_coverage_gate(config):
    prob = np.zeros((480,640), np.float32)
    prob[168:456, 160:169] = .99
    prob[168:210, 320:329] = .8
    _, result = decode(prob, config)
    assert not result['valid']
    assert result['reasons'] == ['insufficient_continuous_coverage']


def test_median_uses_own_measured_rows_including_off_center_rows():
    # A path can have some outside rows and still pass: filtering is on median,
    # not per-row clipping, annotation y ranges, or a central image crop.
    centers = [180, 220, 225]
    layers = [dict(y=168+3*i, left=np.array([x-4]), right=np.array([x+4]),
                   scores=np.array([i+1.]), confidence=np.array([.9]),
                   parents=np.array([[i-1, 0] if i else [-1,-1]]))
              for i,x in enumerate(centers)]
    path = ab.paths_from_layers(layers, 640)[0]
    assert path['median_x'] == 220 and path['inside']
    assert len(path['rows']) == 3 and path['rows'][0][1] == 176


def test_tie_order_matches_original_argmax():
    layers = [dict(y=168, left=np.array([200,300]), right=np.array([208,308]),
                   scores=np.array([2.,2.]), confidence=np.array([1.,1.]),
                   parents=np.array([[-1,-1],[-1,-1]]))]
    assert ab.paths_from_layers(layers, 640)[0]['median_x'] == 204
