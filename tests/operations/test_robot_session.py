"""Integrated sensor selection must survive the temporary runtime config."""
import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from inspection_runtime.paths import project_root
from inspection_runtime.supervisor import configure_bridge

ROOT = project_root()
sys.path.insert(0, str(ROOT / 'tools'))
import robot_session


@pytest.mark.parametrize('real', [False, True])
def test_default_session_enables_sensor_streams_and_keeps_candidate_tracking(real):
    args = robot_session.arguments(['--real'] if real else [])
    config = json.loads(robot_session.camera_config(args))
    assert args.camera_mode == 'all'
    assert all(config['hardware']['start_' + sensor]
               for sensor in ('mindvision', 'fisheye', 'd405', 'mid360', 'eddy'))
    assert config['vision']['runtime_mode'] == 'candidate'
    assert Path(config['vision']['algorithm_config']).is_file()
    assert config['websocket_port'] == args.camera_port
    assert config['http_port'] == args.camera_http_port
    for topic in ('/camera/front/image_raw', '/camera/fisheye/image_raw',
                  '/livox/lidar', '/livox/imu', '/eddy/data', '/eddy/channel_mean'):
        assert topic in config['required_topics']
        assert topic in config['record_topics']
    assert '/weld_tracking/error' in config['record_topics']
    assert set(config['required_topics']) <= set(config['record_topics'])
    assert len(config['record_topics']) == len(set(config['record_topics']))
    base = yaml.safe_load((ROOT / 'ros2_ws/src/functions/communication/agv_bridge/config/tank_inspection.yaml').read_text())['websocket_bridge_node']['ros__parameters']
    configure_bridge(base, config)
    for name in ('mindvision_topic', 'fisheye_topic', 'lidar_topic', 'eddy_raw_topic', 'eddy_mean_topic'):
        assert base['tank_inspection.' + name]
    plan = dict(robot_session.launch_plan(args, Path('/tmp/robot-session/bridge.json')))
    assert not any('fastdds_d405_127.xml' in part for part in plan['camera'])
    assert f'tracking_dry_run:={str(not real).lower()}' in plan['robot']


@pytest.mark.parametrize('mode', ['d405', 'cameras'])
def test_explicit_camera_profiles_remain_available(mode):
    args = robot_session.arguments(['--camera-mode', mode])
    config = json.loads(robot_session.camera_config(args))
    assert not config['hardware']['start_mid360']
    assert not config['hardware']['start_eddy']
    if mode == 'd405':
        assert not config['hardware']['start_mindvision']
        assert not config['hardware']['start_fisheye']
        assert config['vision']['runtime_mode'] == 'candidate'


def test_legacy_entrypoint_forwards_arguments_without_starting_hardware():
    result = subprocess.run(['bash', str(ROOT / 'system/run_robot.sh'), '--real', '--plan'],
                            capture_output=True, text=True, check=True)
    assert '传感器模式：all' in result.stdout
    assert 'tracking_dry_run:=false' in result.stdout
    assert str(ROOT / 'apps/web_console_chassis_tracking') in result.stdout


def test_preflight_rejects_missing_camera_backend_before_launch(monkeypatch):
    args = robot_session.arguments(['--web-root', str(ROOT / 'apps/web_console_chassis_tracking')])
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        assert command[3] == 'camera-preflight'
        return subprocess.CompletedProcess(command, 1, '', "Package 'inspection_operations' not found")
    class AvailablePort:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def setsockopt(self, *args): pass
        def bind(self, *args): pass
    monkeypatch.setattr(robot_session.subprocess, 'run', run)
    monkeypatch.setattr(robot_session.socket, 'socket', AvailablePort)
    errors = robot_session.preflight(args)
    assert any('相机后台 ROS 安装未加载' in error and 'inspection_operations' in error for error in errors)
    assert len(calls) == 1
