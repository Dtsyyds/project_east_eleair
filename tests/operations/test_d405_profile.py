"""Recording scope and guards; subprocesses here are doubles, never hardware."""
import json
from pathlib import Path
import subprocess
import time

import pytest
import yaml

from inspection_runtime.operations import Operations, ROOT
from inspection_runtime.supervisor import configure_bridge
from test_operations import request, wait_result


def configuration(name):
    return yaml.safe_load((ROOT / f'config/communication/{name}.yaml').read_text())


@pytest.mark.parametrize('name', ['d405_experiment', 'cameras', 'system', 'robot_all'])
def test_field_profiles_accept_both_d405_serials(name):
    serials = configuration(name)['hardware']['d405_serial_number']
    assert isinstance(serials, str)
    assert set(serials.split(',')) == {'260322276866', '352122271190'}


@pytest.fixture
def d405(tmp_path):
    config = configuration('d405_experiment')
    config['data_root'] = str(tmp_path)
    live = {topic: time.monotonic() for topic in config['required_topics']}
    events = []
    ops = Operations(config, events.append, lambda: dict(live))
    ops.storage = lambda: {'mounted': True, 'free_bytes': 100 * 1024**3}
    yield ops, events, live
    ops.recorder = None
    if ops.recorder_log:
        ops.recorder_log.close()
    ops.close()


def test_only_d405_required_and_missing_stream_still_fails(d405):
    ops, _, live = d405
    assert len(live) == 4 and all(t.startswith('/d405/camera/') for t in live)
    assert ops.snapshot()['capabilities']['collection']['profiles'] == [
        {'id': 'd405_experiment', 'label': 'D405 算法实验采集'}]
    ops.preflight()  # No industrial camera, fisheye, radar, EMAT or eddy input.
    for topic in list(live):
        stamp = live.pop(topic)
        with pytest.raises(RuntimeError, match=topic):
            ops.preflight()
        live[topic] = stamp
    ops.storage = lambda: {'mounted': False, 'free_bytes': 100 * 1024**3}
    with pytest.raises(RuntimeError, match='采集盘'):
        ops.preflight()


@pytest.mark.parametrize('name', ['d405_experiment', 'cameras', 'system', 'robot_all'])
def test_exact_recorder_scope_and_metadata(d405, monkeypatch, name):
    ops, events, live = d405
    config = configuration(name)
    config['data_root'] = ops.config['data_root']
    ops.config = config
    ops.collection_profile = config.get('collection_profile', {'id': 'full_system', 'label': '整机采集'})
    live.clear()
    live.update({topic: time.monotonic() for topic in config['required_topics']})
    commands = []
    def recorder(command, **kwargs):
        commands.append(command)
        bag = Path(command[command.index('--output') + 1])
        bag.mkdir()
        (bag / 'fixture.mcap').touch()
        return type('Recorder', (), {'poll': lambda self: None})()
    monkeypatch.setattr('inspection_runtime.operations.subprocess.Popen', recorder)
    # An unsolicited browser topic list must not widen the configured allowlist.
    ops.handle(request('collection.start', 'record', profile=ops.collection_profile['id'],
                       record_topics=['/camera/front/image_raw', '/inspection/emat/result']))
    assert wait_result(events, 'record')['status'] == 'completed'
    command = commands[0]
    actual = command[command.index('--qos-profile-overrides-path') + 2:]
    if name == 'd405_experiment':
        expected = config['record_topics']
        assert len(actual) == 14
        assert all(t.startswith(('/d405/camera/', '/vision/d405_seam/')) for t in actual)
    elif name == 'robot_all':
        expected = config['record_topics']
        assert '/livox/imu' in actual and '/eddy/data' in actual
        assert '/camera/front/image_raw' in actual and '/camera/fisheye/image_raw' in actual
        assert '/weld_tracking/error' in actual
    else:
        expected = yaml.safe_load((ROOT / 'ros2_ws/src/bringup/inspection_bringup/config/record_topics.yaml').read_text())['topics'] + ['/inspection/emat/result']
    assert actual == expected
    record = json.loads((ops.run_dir / 'run.json').read_text())
    assert record['record_topics'] == expected
    assert record['required_topics'] == config['required_topics']
    assert record['camera_settings'] is None


@pytest.mark.parametrize('invalid', [[], None, '/d405/camera/*', ['/d405/camera/*'],
                                   ['/camera/front/image_raw'], [12], ['--all']])
def test_invalid_scope_fails_before_creating_recorder(d405, invalid):
    ops, events, _ = d405
    ops.config['record_topics'] = invalid
    ops.handle(request('collection.start'))
    assert wait_result(events, 'test')['status'] == 'failed'
    assert ops.run_dir is None and ops.recorder is None


def test_disabled_camera_bridge_inputs_and_old_profile_unchanged():
    base = yaml.safe_load((ROOT / 'ros2_ws/src/functions/communication/agv_bridge/config/tank_inspection.yaml').read_text())['websocket_bridge_node']['ros__parameters']
    for name in ('d405_experiment', 'cameras'):
        params = dict(base)
        config = configuration(name)
        configure_bridge(params, config)
        assert params['tank_inspection.depth_camera_topic'] == '/d405/camera/color/image_raw'
        for key in ('mindvision_topic', 'fisheye_topic'):
            assert params['tank_inspection.' + key] == ('' if name == 'd405_experiment' else base['tank_inspection.' + key])
        if name == 'd405_experiment':
            hardware = config['hardware']
            assert hardware['start_d405'] is True
            assert all(hardware[k] is False for k in ('start_mindvision', 'start_fisheye', 'start_mid360', 'start_eddy', 'start_sync', 'publish_rough_extrinsics'))


def test_dedicated_launcher_selects_explicit_config_without_hardware(tmp_path):
    wrapper = tmp_path / 'system_websocket/run_d405.sh'
    wrapper.parent.mkdir()
    wrapper.write_text((ROOT / 'system_websocket/run_d405.sh').read_text())
    stub = tmp_path / 'apps/web_console/run.sh'
    stub.parent.mkdir(parents=True)
    stub.write_text('#!/usr/bin/env bash\nprintf "%s\\n" "$@"\n')
    result = subprocess.run(['bash', str(wrapper)], text=True, capture_output=True, check=True)
    assert result.stdout.splitlines() == ['--config', str(tmp_path / 'config/communication/d405_a.yaml')]
    legacy = subprocess.run(['bash', str(wrapper), '--legacy'], text=True, capture_output=True, check=True)
    assert legacy.stdout.splitlines() == ['--config', str(tmp_path / 'config/communication/d405_experiment.yaml')]
