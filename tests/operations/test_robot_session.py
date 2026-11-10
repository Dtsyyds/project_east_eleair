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


def test_slam_opt_in_plan_only_adds_independent_service():
    base=robot_session.arguments([])
    enabled=robot_session.arguments(['--slam','--domain','81','--no-camera','--host','robot.local','--slam-config','/tmp/slam.yaml'])
    ordinary=robot_session.launch_plan(base,Path('/tmp/bridge.json'))
    assert [name for name,_ in ordinary]==['robot','camera','gateway']
    plan=robot_session.launch_plan(enabled,Path('/tmp/bridge.json'))
    assert [name for name,_ in plan]==['robot','gateway','slam']
    command=dict(plan)['slam']
    assert command[:2]==['bash',str(ROOT/'system_websocket/run_slam.sh')]
    assert command[command.index('--domain')+1]=='81'
    assert command[command.index('--config')+1]=='/tmp/slam.yaml'
    assert 'robot.local' in command
    assert not any('chassis' in arg or 'hardware.launch' in arg for arg in command)


def test_slam_failure_isolated_once_and_critical_still_stops():
    class Exited:
        def poll(self):return 1
    seen=set()
    assert robot_session.child_failure('slam',Exited(),seen)=='isolated'
    assert robot_session.child_failure('slam',Exited(),seen) is None
    assert robot_session.child_failure('robot',Exited(),seen)=='critical'


def test_shutdown_preserves_150_second_slam_budget(monkeypatch):
    now=[0.0];signals=[]
    class Process:
        def __init__(self,pid,finish):self.pid=pid;self.finish=finish
        def poll(self):return 0 if now[0]>=self.finish else None
        def wait(self):self.finish=now[0]
    regular=Process(100,1000);slam=Process(200,20)
    monkeypatch.setattr(robot_session.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(robot_session.time,'sleep',lambda n:now.__setitem__(0,now[0]+n))
    monkeypatch.setattr(robot_session.os,'killpg',lambda pid,sig:signals.append((now[0],pid,sig)))
    robot_session.stop_children([regular,slam],[slam])
    assert any(pid==100 and sig==robot_session.signal.SIGTERM and t>=12 for t,pid,sig in signals)
    assert not any(pid==200 and sig!=robot_session.signal.SIGINT for _,pid,sig in signals)
    assert now[0]>=20


def test_slam_hung_service_escalates_only_after_150_seconds(monkeypatch):
    now=[0.0];signals=[]
    class Process:
        pid=200
        finished=False
        def poll(self):return 0 if self.finished else None
        def wait(self):self.finished=True
    process=Process()
    monkeypatch.setattr(robot_session.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(robot_session.time,'sleep',lambda n:now.__setitem__(0,now[0]+n))
    monkeypatch.setattr(robot_session.os,'killpg',lambda pid,sig:signals.append((now[0],sig)))
    robot_session.stop_children([process],[process])
    assert next(t for t,s in signals if s==robot_session.signal.SIGTERM)>=150
