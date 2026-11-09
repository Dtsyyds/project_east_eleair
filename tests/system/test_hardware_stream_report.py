"""Acceptance must fail for late startup, stale stamps and malformed frames."""
import pytest

from tools.system.check_hardware_streams import summarize_streams


def observation():
    return {'count': 590, 'first': 1., 'last': 59.9, 'max_gap_s': .2,
            'sampled_stamp_reversals': 0,
            'samples': [{'stamp_ns': 123, 'age_s': .1, 'valid_layout': True}]}


def test_good_stream_and_optional_missing_device():
    rows = {'camera': observation()}
    assert summarize_streams(rows, ['camera'], 60)
    assert rows['camera']['observed_hz'] == pytest.approx(10)
    assert not summarize_streams(rows, ['camera', 'missing'], 60)
    assert not summarize_streams(rows, [], 60)


@pytest.mark.parametrize('changes', [
    {'first': 58., 'count': 20}, {'last': 50.}, {'max_gap_s': 3.},
    {'sampled_stamp_reversals': 1},
    {'samples': [{'stamp_ns': 123, 'age_s': 10., 'valid_layout': True}]},
    {'samples': [{'stamp_ns': 123, 'age_s': .1, 'valid_layout': False}]},
    {'samples': []},
])
def test_faults_cannot_pass(changes):
    row = observation()
    row.update(changes)
    assert not summarize_streams({'camera': row}, ['camera'], 60)
