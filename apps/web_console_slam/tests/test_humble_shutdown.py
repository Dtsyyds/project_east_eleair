"""No sensor/algorithm execution: validate installed Humble launch signal semantics."""
import importlib.util
import asyncio
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

import pytest


@pytest.mark.skipif(not shutil.which('ros2'), reason='requires ROS Humble environment')
def test_real_service_starts_idle_and_does_not_take_over_port(tmp_path):
    """Only the read-only observer runs in an isolated domain, never algorithms."""
    pytest.importorskip('rclpy')
    from ament_index_python.packages import get_package_share_directory
    from websockets.asyncio.client import connect
    try:
        get_package_share_directory('fast_livo')
    except LookupError:
        pytest.skip('requires built fast_livo overlay')
    root = Path(__file__).resolve().parents[3]
    command = ['bash', str(root / 'system_websocket/run_slam.sh'), '--domain', '83', '--host', '127.0.0.1']
    with (tmp_path / 'service.log').open('w') as log:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            for _ in range(200):
                if 'SLAM 空闲' in (tmp_path / 'service.log').read_text() or process.poll() is not None:
                    break
                time.sleep(.05)
            assert process.poll() is None, (tmp_path / 'service.log').read_text()
            assert 'SLAM 空闲' in (tmp_path / 'service.log').read_text()
            async def status():
                async with connect('ws://127.0.0.1:8082/ws', origin='http://127.0.0.1:8082', proxy=None) as ws:
                    await ws.send(json.dumps({'op': 'status', 'request_id': 'smoke'}))
                    while True:
                        response = json.loads(await ws.recv())
                        if response.get('request_id') == 'smoke':
                            assert response['result']['state'] == 'idle'
                            assert response['session_id'] is None
                            return
            asyncio.run(status())
            check = subprocess.run([*command, '--check'], capture_output=True, text=True, timeout=20)
            assert check.returncode == 2 and '8082' in check.stderr
            assert process.poll() is None
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=20) == 0, (tmp_path / 'service.log').read_text()
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()


@pytest.mark.skipif(not shutil.which('ros2'),reason='requires ROS Humble environment')
def test_humble_internal_process_accepts_120_second_timeout(tmp_path):
    pytest.importorskip('launch')
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch.utilities import perform_substitutions
    from ament_index_python.packages import get_package_share_directory
    # Inspect actual installed mapping and relocalization Node objects, without executing them.
    try:
        base=Path(get_package_share_directory('fast_livo'))/'config/mapping_relocation.yaml'
    except LookupError:
        pytest.skip('requires built fast_livo overlay: source deploy/env.sh')
    context=LaunchContext()
    context.launch_configurations.update(config_file=str(base),republish_compressed='false',enable_rviz='false',
                                        camera_file='',sigterm_timeout='120',sigkill_timeout='5')
    for package,filename,setup in [('fast_livo','mapping_relocation.launch.py','_setup'),('relocalization','relocalization.launch.py','_launch_setup')]:
        path=Path(get_package_share_directory(package))/'launch'/filename
        spec=importlib.util.spec_from_file_location('launch_under_test',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        context.launch_configurations['start_delay']='0';context.launch_configurations['algorithm']='teaser_gicp'
        actions=getattr(module,setup)(context)
        def nodes(actions):
            for action in actions:
                if isinstance(action,ExecuteProcess):yield action
                elif hasattr(action,'actions'):yield from nodes(action.actions)
        checked=list(nodes(actions));assert checked
        for action in checked:
            assert perform_substitutions(context,action._ExecuteLocal__sigterm_timeout)=='120'
            assert perform_substitutions(context,action._ExecuteLocal__sigkill_timeout)=='5'
    # Exercise the same ExecuteLocal implementation through ros2 launch CLI.
    ready=tmp_path/'ready';saved=tmp_path/'saved'
    worker=tmp_path/'worker.py'
    worker.write_text(f'''import pathlib,signal,time

def stop(*_):
 time.sleep(13)
 pathlib.Path({str(saved)!r}).write_text('saved after 13 seconds')
 raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
pathlib.Path({str(ready)!r}).write_text('ready')
while True: time.sleep(.1)
''')
    launch=tmp_path/'save.launch.py'
    launch.write_text(f'''from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
 return LaunchDescription([ExecuteProcess(cmd=[{sys.executable!r},{str(worker)!r}],output='screen')])
''')
    env={**os.environ,'ROS_LOG_DIR':str(tmp_path/'ros_logs')}
    with (tmp_path/'launch.log').open('w') as log:
        process=subprocess.Popen(['ros2','launch',str(launch),'sigterm_timeout:=120','sigkill_timeout:=5'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(200):
                if ready.exists() or process.poll() is not None:break
                time.sleep(.05)
            assert ready.exists(),(tmp_path/'launch.log').read_text()
            start=time.monotonic();process.send_signal(signal.SIGINT);process.wait(timeout=25)
            assert saved.exists(),(tmp_path/'launch.log').read_text()
            assert time.monotonic()-start>=13
            assert process.returncode==0
        finally:
            if process.poll() is None:os.killpg(process.pid,signal.SIGKILL);process.wait()
