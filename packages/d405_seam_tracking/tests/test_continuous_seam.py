"""Measured stripe continuity, clutter rejection and explicit task appearance."""
import numpy as np
import pytest
import yaml

from inspection_runtime.paths import project_root
from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.contracts import load_config
from d405_seam_tracking.geometry import candidate, classical_probability, roi_bounds
from d405_seam_tracking.pipeline import Pipeline

CONFIG = project_root() / 'config/d405/vertical_seam.yaml'


@pytest.mark.parametrize('center', [205, 320, 430])
def test_coherent_stripe_beats_brighter_scanline_distractors(center):
    config = load_config(CONFIG)
    probability = np.zeros((480, 640), np.float32)
    probability[168:405, center-4:center+5] = .8
    # Every row has a stronger, unrelated patch alternating far left/right.
    for i, y in enumerate(range(168, 456, 3)):
        x = 150 if i % 2 else 485
        probability[y, x:x+9] = 1.
    result = candidate(probability, config, roi_bounds(probability.shape, config))
    assert result['valid'] and result['coverage'] > .75
    points = np.array(result['points'])
    np.testing.assert_allclose(points[:, 1:].mean(1), center, atol=.5)
    assert points[:, 0].max() < 405  # Do not extend through the bottom occluder.


def test_random_bright_patches_do_not_make_a_path():
    config = load_config(CONFIG)
    probability = np.zeros((480, 640), np.float32)
    rng = np.random.default_rng(42)
    for y in range(168, 456, 3):
        for x in rng.integers(135, 495, 3):
            probability[y, x:x+5] = 1.
    assert not candidate(probability, config, roi_bounds(probability.shape, config))['valid']


def test_long_gap_cannot_join_two_short_segments():
    config = load_config(CONFIG)
    probability = np.zeros((480, 640), np.float32)
    probability[168:270, 330:340] = 1.
    probability[345:456, 330:340] = 1.
    result = candidate(probability, config, roi_bounds(probability.shape, config))
    assert not result['valid']
    rows = np.array(result['rows'])
    assert np.all(np.diff(rows[:, 0]) <= 9)


def test_short_missing_rows_are_not_reported_as_measurements():
    config = load_config(CONFIG)
    probability = np.zeros((480, 640), np.float32)
    probability[:, 330:340] = .9
    probability[240:246] = 0
    result = candidate(probability, config, roi_bounds(probability.shape, config))
    assert result['valid']
    assert not any(240 <= p[0] < 246 for p in result['points'])
    assert not any(result['interpolated'])


@pytest.mark.parametrize('center', [230, 330, 420])
def test_dark_seam_detection_moves_with_image_not_roi_center(center):
    config = dict(load_config(CONFIG), mode='vision_only')
    frame = synthetic_frame('none', shape=(480, 640))
    frame.image[:] = 160
    frame.image[:, center-5:center+6] = 50
    result = Pipeline(config).process(frame)
    assert result['state'] == 'VISION_2D' and not result['metric_valid']
    points = np.array(result['points_2d_y_left_right'])
    np.testing.assert_allclose(points[:, 1:].mean(1), center, atol=1.)


@pytest.mark.parametrize('kind', ['blank', 'step', 'bright'])
def test_task_profile_rejects_flat_surface_single_edge_and_wrong_polarity(kind):
    config = load_config(CONFIG)
    image = np.full((480, 640), 100, np.uint8)
    if kind == 'step': image[:, 330:] = 200
    if kind == 'bright': image[:, 325:336] = 230
    probability = classical_probability(image, config)
    assert not candidate(probability, config, roi_bounds(image.shape, config))['valid']


@pytest.mark.parametrize('key,value', [('visual_polarity', 'auto'), ('path_max_gap_rows', -1),
                                      ('path_max_gap_rows', 1.5), ('path_max_slope', float('nan')),
                                      ('visual_stripe_width_fraction', 0)])
def test_invalid_profile_fails_before_running(key, value, tmp_path):
    config = load_config(CONFIG)
    config[key] = value
    path = tmp_path / 'invalid.yaml'
    path.write_text(yaml.safe_dump(config))
    with pytest.raises(ValueError): load_config(path)
