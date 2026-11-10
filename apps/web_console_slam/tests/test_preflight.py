"""ROS input checks with observations only; no drivers or DDS required."""
from collections import deque
from types import SimpleNamespace
import threading
import time

import pytest

from slam_console.ros_bridge import RosBridge
from slam_console.__main__ import load_config


def observer():
    bridge = RosBridge.__new__(RosBridge)
    bridge.lock = threading.RLock()
    bridge.config = dict(lidar_topic='/livox/lidar', imu_topic='/livox/imu', input_max_age=2, stamp_tolerance=3)
    now = time.monotonic()
    bridge.inputs = {key: dict(received=now, stamp=time.time(), advancing=True) for key in ('lidar', 'imu')}
    bridge.imu = deque([(now, 9.81, 0.01)] * 20)
    bridge.node = SimpleNamespace(
        get_topic_names_and_types=lambda: [('/livox/lidar', ['livox_ros_driver2/msg/CustomMsg']),
                                          ('/livox/imu', ['sensor_msgs/msg/Imu'])],
        count_publishers=lambda topic: int(topic in ('/livox/lidar', '/livox/imu')),
        get_node_names_and_namespaces=lambda: [])
    return bridge


def test_fresh_si_input_from_any_upstream():
    assert observer().preflight() == []


@pytest.mark.parametrize('case, expected', [
    ('type', '类型'), ('stale', '新鲜'), ('stamp', '时间戳'),
    ('g_units', '米制'), ('moving', '静止'), ('duplicate', '仅有一个'), ('algorithm', '其他定位建图')])
def test_input_errors(case, expected):
    bridge = observer()
    if case == 'type':
        bridge.node.get_topic_names_and_types = lambda: [('/livox/lidar', ['sensor_msgs/msg/PointCloud2'])]
    elif case == 'stale':
        bridge.inputs['lidar']['received'] -= 10
    elif case == 'stamp':
        bridge.inputs['imu']['stamp'] -= 10
    elif case in ('g_units', 'moving'):
        bridge.imu = deque([(time.monotonic(), 1 if case == 'g_units' else 9.81, 1 if case == 'moving' else 0)] * 20)
    elif case == 'duplicate':
        bridge.node.count_publishers = lambda topic: 2
    else:
        bridge.node.get_node_names_and_namespaces = lambda: [('laserMapping', '/')]
    assert any(expected in error for error in bridge.preflight())


def test_config_paths_resolve_from_config_directory(tmp_path, monkeypatch):
    directory = tmp_path / 'config'
    directory.mkdir()
    (directory / 'base.yaml').write_text('/**:\n  ros__parameters: {}\n')
    path = directory / 'slam.yaml'
    path.write_text('algorithm_root: ../algorithm\nbase_config: base.yaml\nruntime_directory: ../runtime/{platform}\n'
                    'lidar_topic: /livox/lidar\nimu_topic: /livox/imu\nimu_acceleration_unit: m/s2\n')
    monkeypatch.chdir('/')
    config = load_config(path, 'x86')
    assert config['algorithm_root'] == tmp_path / 'algorithm'
    assert config['base_config'] == directory / 'base.yaml'
    assert config['runtime_directory'] == tmp_path / 'runtime/x86'
    path.write_text(path.read_text().replace('m/s2', 'g'))
    with pytest.raises(ValueError, match='米制'):
        load_config(path, 'x86')


def test_structured_checks_keep_unknown_stationary_and_stable_ids():
    bridge = observer()
    bridge.imu.clear()
    checks = {c['id']: c for c in bridge.preflight_checks()}
    assert checks['lidar.type']['status'] == 'pass'
    assert checks['imu.samples']['status'] == 'fail'
    assert checks['imu.stationary']['status'] == 'unknown'
    assert checks['imu.stationary']['suggestion']


def test_health_session_evidence_and_monotonic_freshness(monkeypatch):
    bridge = observer()
    bridge.observed, bridge.clouds, bridge.last_cloud = {}, {}, {}
    wall, mono = time.time(), time.monotonic()
    monkeypatch.setattr('slam_console.ros_bridge.time.time', lambda: wall)
    monkeypatch.setattr('slam_console.ros_bridge.time.monotonic', lambda: mono)
    bridge.begin(wall)
    bridge.observe('tracking', True)
    bridge.observe('odom', wall)
    assert bridge.health('mapping')['state'] == 'valid'
    assert not bridge.ready('relocation')
    bridge.observe('initial', wall - 10)  # Old transient evidence must not satisfy this session.
    assert not bridge.ready('relocation')
    bridge.observe('initial', wall)
    assert bridge.ready('relocation')
    bridge.observe('tracking', False)
    assert bridge.health('mapping')['state'] == 'tracking_invalid'
    bridge.observe('tracking', True)
    wall += 20  # Clock step affects stamp validation, not receipt age.
    health = bridge.health('mapping')
    assert health['odom_age'] == 0
    assert health['state'] == 'odom_clock_invalid'
    assert all(i['status'] == 'pass' for i in health['inputs'])
    wall -= 20
    mono += 3
    assert bridge.health('mapping')['state'] == 'odom_stale'
    bridge.observe('odom', wall)
    assert bridge.health('mapping')['state'] == 'tracking_stale'
    bridge.observe('tracking', True)
    assert bridge.ready('relocation')  # Prior initialization is one-time evidence.
    bridge.begin(wall)
    assert bridge.health('mapping')['state'] == 'not_ready'


def test_preflight_receipt_age_does_not_use_sensor_wall_clock(monkeypatch):
    bridge = observer()
    wall = time.time()
    monkeypatch.setattr('slam_console.ros_bridge.time.time', lambda: wall + 100)
    checks = {c['id']: c for c in bridge.preflight_checks()}
    assert checks['imu.freshness']['status'] == 'pass'
    assert checks['imu.stamp']['status'] == 'fail'
    assert checks['imu.stationary']['status'] == 'pass'
