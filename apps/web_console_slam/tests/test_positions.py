import json
import threading
from types import SimpleNamespace as NS

import pytest

from slam_console.ros_bridge import RosBridge


@pytest.fixture
def bridge(monkeypatch):
    clock = {'wall': 100.0, 'mono': 10.0}
    monkeypatch.setattr('slam_console.ros_bridge.time.time', lambda: clock['wall'])
    monkeypatch.setattr('slam_console.ros_bridge.time.monotonic', lambda: clock['mono'])
    b = RosBridge.__new__(RosBridge)
    b.config = {'input_max_age': 2.0, 'stamp_tolerance': 3.0}
    b.lock = threading.RLock()
    b.observed, b.clouds, b.last_cloud = {}, {}, {}
    b.begin(100, 'site_map')
    return b, clock


def pose(xyz=(1, 2, 3), timestamp=100.0, frame='site_map'):
    sec = int(timestamp)
    return NS(header=NS(frame_id=frame, stamp=NS(sec=sec, nanosec=round((timestamp-sec)*1e9))),
              pose=NS(pose=NS(position=NS(x=xyz[0], y=xyz[1], z=xyz[2]))))


def test_current_updates_but_gicp_initial_is_fixed(bridge):
    b, clock = bridge
    assert b.position_status('mapping')['initial']['xyz'] == [0, 0, 0]
    assert b.position_status('relocation')['initial'] is None
    b.position('initial', pose((10, 20, 30)))
    b.position('current', pose((11, 22, 33)))
    clock.update(wall=101, mono=11)
    b.position('initial', pose((99, 99, 99), 101))
    b.position('current', pose((12, 23, 34), 101))
    status = b.position_status('relocation')
    assert status['initial']['xyz'] == [10, 20, 30]
    assert status['current']['xyz'] == [12, 23, 34]
    assert status['frame_id'] == 'site_map' and status['fresh']
    json.dumps(status, allow_nan=False)


@pytest.mark.parametrize('message', [pose(timestamp=99), pose(timestamp=104),
                                    pose(frame='camera_init'), pose((float('nan'), 2, 3)),
                                    pose((1, float('inf'), 3))])
def test_invalid_or_previous_session_pose_is_not_displayed(bridge, message):
    b, _ = bridge
    for key in ('initial', 'current'):
        b.position(key, message)
    status = b.position_status('relocation')
    assert status['current'] is None and status['initial'] is None
    assert not status['fresh']


def test_stale_duplicate_and_new_session_reset(bridge):
    b, clock = bridge
    b.position('initial', pose())
    b.position('current', pose())
    clock.update(wall=102.5, mono=12.5)
    b.position('current', pose())  # Repeated stamps must not refresh data age.
    status = b.position_status('relocation')
    assert status['age'] == 2.5 and not status['fresh']
    assert status['initial']['xyz'] == [1, 2, 3]
    b.begin(102.5, 'site_map')
    b.position('initial', pose())  # Old transient GICP result is ignored.
    status = b.position_status('relocation')
    assert status['current'] is None and status['initial'] is None


def test_current_out_of_order_and_clock_jump(bridge):
    b, clock = bridge
    clock.update(wall=101, mono=11)
    b.position('current', pose((4, 5, 6), 101))
    b.position('current', pose((1, 2, 3), 100))
    assert b.position_status('mapping')['current']['xyz'] == [4, 5, 6]
    clock['wall'] = 90
    assert not b.position_status('mapping')['fresh']
