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
    now = time.time()
    bridge.inputs = {key: dict(received=now, stamp=now, advancing=True) for key in ('lidar', 'imu')}
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
        bridge.imu = deque([(time.time(), 1 if case == 'g_units' else 9.81, 1 if case == 'moving' else 0)] * 20)
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
