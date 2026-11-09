"""Annotation import must reject mismatched provenance and invalid boundaries."""
import importlib.util
import json
from pathlib import Path

import pytest

from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.contracts import sha256
from d405_seam_tracking.dataset import read_sample, save_sample


spec = importlib.util.spec_from_file_location('d405_review', Path(__file__).resolve().parents[2] / 'tools/vision/prepare_d405_review.py')
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)


def annotation():
    return {'status': 'approved', 'origin': 'human', 'author': 'test_fixture_only',
            'no_seam': False, 'left_xy': [[60, y] for y in range(10, 70, 10)],
            'right_xy': [[80, y] for y in range(10, 70, 10)]}


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path / 'source'
    root.mkdir()
    frame = synthetic_frame()
    rec = save_sample(root, frame, 'test_frame', 'isolated_synthetic', annotation=None)
    index = root / 'index.json'
    index.write_text(json.dumps({'schema': 'd405_dataset_v1', 'synthetic': True, 'samples': [rec]}))
    labels = tmp_path / 'labels.json'
    value = {'schema': 'd405_review_labels_v1', 'target': 'visible_weld_boundaries',
             'source_index_sha256': sha256(index),
             'samples': [{'sample': rec['sample'], 'raw_sha256': rec['sha256'], 'annotation': annotation()}]}
    labels.write_text(json.dumps(value))
    return index, labels, value, tmp_path / 'output'


def test_import_copies_only_verified_labels_and_preserves_source(bundle):
    index, labels, _, out = bundle
    before = index.read_bytes()
    result = review.import_labels(index, labels, out)
    assert result['labelled'] == 1 and index.read_bytes() == before
    imported = json.loads((out / 'index.json').read_text())['samples'][0]
    assert imported['split'] == 'unassigned'
    assert imported['annotation']['origin'] == 'human'
    assert read_sample(out, imported).image.shape == synthetic_frame().image.shape


@pytest.mark.parametrize('mutation,error', [
    ('index_hash', 'different_dataset'), ('raw_hash', 'hash_mismatch'),
    ('duplicate', 'duplicate_label'), ('automatic', 'human_annotation'),
    ('crossing', 'crossing_boundaries'), ('missing_author', 'author_required'),
    ('out_of_bounds', 'annotation_outside_image'),
])
def test_reject_bad_annotation_before_creating_output(bundle, mutation, error):
    index, labels, value, out = bundle
    item = value['samples'][0]
    if mutation == 'index_hash': value['source_index_sha256'] = 'wrong'
    if mutation == 'raw_hash': item['raw_sha256'] = 'wrong'
    if mutation == 'duplicate': value['samples'].append(item)
    if mutation == 'automatic': item['annotation']['origin'] = 'algorithm'
    if mutation == 'crossing': item['annotation']['left_xy'][2][0] = 90
    if mutation == 'missing_author': item['annotation']['author'] = ''
    if mutation == 'out_of_bounds': item['annotation']['left_xy'][0][0] = -1
    labels.write_text(json.dumps(value))
    with pytest.raises(ValueError, match=error):
        review.import_labels(index, labels, out)
    assert not out.exists()


def test_sync_keeps_immutable_revisions_and_rejects_bad_data(bundle):
    index, _, value, out = bundle
    store = review.ReviewLabelStore(index, out)
    first = store.receive(value)
    original = (out / first['file']).read_bytes()
    assert first['count'] == first['total'] == 1
    assert store.receive(value)['file'] == first['file']
    value['samples'][0]['annotation']['left_xy'][0][0] = 61
    second = store.receive(value)
    assert first['file'] != second['file']
    assert (out / first['file']).read_bytes() == original
    pointer = (out / 'latest_received.json').read_bytes()
    value['samples'][0]['raw_sha256'] = 'bad'
    with pytest.raises(ValueError, match='hash_mismatch'):
        store.receive(value)
    assert (out / 'latest_received.json').read_bytes() == pointer
    assert json.loads(index.read_text())['samples'][0]['annotation'] is None
