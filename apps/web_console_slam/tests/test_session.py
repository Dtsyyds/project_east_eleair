import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import pytest
import yaml

from slam_console.session import SessionManager, pcd_valid
from slam_console.ros_bridge import RosBridge
from slam_console.server import Server

APP = Path(__file__).resolve().parents[1]
ROOT = APP.parents[1]
PCD = b'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n1 2 3\n'


class Bridge:
    def preflight_checks(self): return []
    def health(self, mode): return {"state": "not_ready", "checks": [], "inputs": []}
    def begin(self, started, map_frame='map'): self.started=started
    def position_status(self, mode): return {'current': None, 'initial': None, 'fresh': False, 'age': None}
    def ready(self, mode): return False
    def previews(self): return {}


@pytest.fixture
def manager(tmp_path, monkeypatch):
    base = tmp_path/'base.yaml'
    base.write_text(yaml.safe_dump({'/**':{'ros__parameters':{'time_check':{'enabled':True},'pcd_save':{'type':-1}}}}))
    config = dict(algorithm_root=tmp_path/'algorithm', prior_directory=tmp_path/'maps', runtime_directory=tmp_path/'runtime',
                  base_config=base, lidar_topic='/livox/lidar', imu_topic='/livox/imu', mapping_timeout=60,
                  relocation_timeout=180, shutdown_timeout=130, input_max_age=2, stamp_tolerance=3)
    config['prior_directory'].mkdir()
    (config['prior_directory']/'prior.pcd').write_bytes(PCD)
    monkeypatch.setattr('slam_console.__main__.static_check',lambda *a,**kw: [])
    m=SessionManager(config,Bridge())
    return m


def test_maps_reject_escape_invalid_and_symlink(manager, tmp_path):
    assert manager.maps()[0]['path']==str(tmp_path/'maps/prior.pcd')
    outside=tmp_path/'outside.pcd';outside.write_bytes(PCD)
    (tmp_path/'maps/escape.pcd').symlink_to(outside)
    assert len(manager.maps())==1
    for name in ('../outside.pcd','escape.pcd','missing.pcd'):
        with pytest.raises(ValueError): manager.options(dict(mode='relocation',map=name,save=True))
    assert pcd_valid(outside)
    outside.write_text('POINTS 0\nDATA binary\n')
    assert not pcd_valid(outside)


@pytest.mark.parametrize('payload', [b'garbage\n', b'1 2\n', b'nan nan nan\n'])
def test_pcd_rejects_malformed_ascii(tmp_path, payload):
    path = tmp_path / 'bad.pcd'
    path.write_bytes(PCD.rsplit(b'1 2 3\n', 1)[0] + payload)
    assert not pcd_valid(path)


def install_fake_ros(tmp_path, monkeypatch, delay=0):
    executable=tmp_path/'ros2'
    executable.write_text('''#!/usr/bin/env python3
import pathlib, sys, signal, time, yaml
config=pathlib.Path(next(x.split(':=',1)[1] for x in sys.argv if x.startswith('config_file:=')))
p=yaml.safe_load(config.read_text())['/**']['ros__parameters']
def stop(*args):
    print('停止末尾 stdout',flush=True)
    print('停止末尾 stderr',file=sys.stderr,flush=True)
    time.sleep(DELAY)
    if p['pcd_save']['pcd_save_en']:
        pathlib.Path(p['pcd_save']['output_dir'],'result.pcd').write_bytes(PCD)
    raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
print('READY',flush=True)
for i in range(350): print('中文日志',i,flush=True)
while True: time.sleep(.1)
'''.replace('DELAY',repr(delay)).replace('write_bytes(PCD)',f'write_bytes({PCD!r})'))
    executable.chmod(0o755)
    monkeypatch.setenv('PATH', str(tmp_path)+os.pathsep+os.environ['PATH'])
    monkeypatch.setenv('PYTHONPATH', str(APP)+os.pathsep+os.environ.get('PYTHONPATH',''))


async def wait_ready(manager):
    for _ in range(100):
        await asyncio.sleep(.05)
        if (manager.directory/'launch.log').exists() and b'READY' in (manager.directory/'launch.log').read_bytes(): return
    raise AssertionError('fake launch did not become ready')


@pytest.mark.parametrize('mode,save', [('mapping',True),('mapping',False),('relocation',True)])
def test_session_config_logs_and_save(manager,tmp_path,monkeypatch,mode,save):
    install_fake_ros(tmp_path,monkeypatch)
    original=manager.config['base_config'].read_bytes()
    async def scenario():
        result=await manager.start(dict(mode=mode,map='prior.pcd',save=save),'request-one')
        sid=result['session_id']
        try:
            await wait_ready(manager)
            p=yaml.safe_load((manager.directory/'config.yaml').read_text())['/**']['ros__parameters']
            assert p['locate_in_prior_map']==(mode=='relocation')
            assert p['prior_map_update_en']==(mode=='relocation')
            assert p['time_check']['enabled'] is True
            assert p['common']['img_en']==0 and p['preprocess']['lidar_type']==1
            assert p['publish']['pub_effect_point_en']
            assert p['pcd_save']['pcd_save_en']==p['pcd_save']['pcd_save_en_relocate']==save
            assert p['pcd_save']['interval']==-1 and p['pcd_save']['type']==-1
            with pytest.raises(ValueError): await manager.stop('another-session')
            with pytest.raises(ValueError): await manager.start(dict(mode=mode,map='prior.pcd',save=save))
            await manager.stop(sid)
        finally:
            await manager.close()
        assert manager.current['save_result']==('success' if save else 'disabled')
        assert manager.current['state']=='stopped'
        text=manager.logs(sid)['text']
        assert '中文日志 349' in text and '停止末尾 stdout' in text and '停止末尾 stderr' in text
        assert 'mapping/stdout' in text
        assert 'mapping/stderr' in text
        assert set(manager.current['processes']) == {'mapping'}
        command = manager.current['processes']['mapping']['command']
        assert command[:4] == ['ros2', 'launch', 'relocalization', 'slam.launch.py']
        assert manager.config['base_config'].read_bytes()==original
        assert (manager.directory/'session.json').is_file()
        return sid
    sid=asyncio.run(scenario())
    # Same-minute runs remain distinct.
    second=asyncio.run(scenario())
    assert sid!=second


def test_utf8_log_paging_over_64k(manager):
    sid='20261110_120000_abcdef123456'
    directory=manager.config['runtime_directory']/sid;directory.mkdir(parents=True)
    text='中文测试\x1b[31m正文\x1b[0m\n'*10000
    (directory/'launch.log').write_text(text)
    cursor=0;parts=[]
    while True:
        page=manager.logs(sid,cursor);parts.append(page['text'])
        assert page['next_cursor']-cursor<=65536
        cursor=page['next_cursor']
        if cursor==page['size']:break
    assert ''.join(parts)==text
    with pytest.raises(ValueError):manager.logs('../escape')


def test_fresh_readiness_ignores_old_transient_state():
    b=RosBridge.__new__(RosBridge);b.lock=threading.RLock();b.config={'input_max_age':2,'stamp_tolerance':3}
    b.inputs={};b.observed={};b.clouds={};b.last_cloud={};now=time.time();b.begin(now)
    b.observe('tracking',True);b.observe('odom',now-10);b.observe('initial',now-10)
    assert not b.ready('mapping')
    b.observe('odom',time.time());assert b.ready('mapping')
    assert not b.ready('relocation')
    b.observe('initial',time.time());assert b.ready('relocation')
    b.observe('tracking',False);assert not b.ready('relocation')


def test_protocol_dedup_stop_target_and_fixed_operations(manager, monkeypatch):
    calls=[]
    async def start(params,request_id=None):
        calls.append(params);return {'session_id':'one','state':'starting'}
    monkeypatch.setattr(manager,'start',start)
    server=Server(manager,['localhost'])
    async def scenario():
        request={'op':'start','request_id':'one','params':{'mode':'mapping','save':True}}
        a,b=await asyncio.gather(server.dispatch(request),server.dispatch(request))
        assert a==b and len(calls)==1
        with pytest.raises(ValueError):await server.dispatch({**request,'params':{'save':False}})
        assert not (await server.dispatch({'op':'shell','request_id':'bad'}))['ok']
        assert not (await server.dispatch({'op':'stop','request_id':'stop','session_id':'foreign'}))['ok']
        for i in range(300):await server.dispatch({'op':'stop','request_id':str(i),'session_id':'foreign'})
        assert len(server.cache)==256
    asyncio.run(scenario())


def test_initialization_timeout_keeps_service_and_logs(manager,tmp_path,monkeypatch):
    install_fake_ros(tmp_path,monkeypatch)
    manager.config['mapping_timeout']=.8
    async def scenario():
        await manager.start(dict(mode='mapping',save=False))
        await asyncio.sleep(1.5)
        await manager.close()
        assert manager.current['state']=='failed'
        assert '初始化超时' in manager.current['error']
        assert manager.logs(manager.current['session_id'])['size']>0
    asyncio.run(scenario())


@pytest.mark.parametrize('outcome', ['no_valid_map', 'failed', 'node_failure'])
def test_save_inspects_artifacts_and_internal_node_exit(manager, tmp_path, monkeypatch, outcome):
    install_fake_ros(tmp_path, monkeypatch)
    executable = tmp_path / 'ros2'
    script = executable.read_text()
    if outcome == 'no_valid_map':
        script = script.replace("if p['pcd_save']['pcd_save_en']:", 'if False:')
    elif outcome == 'failed':
        script = script.replace(f'write_bytes({PCD!r})', "write_bytes(b'truncated')")
    else:
        # ros2 launch may report success even though a node failed during saving.
        script = script.replace('raise SystemExit(0)', '''print('[ERROR] process has died [pid 1, exit code 1, cmd mapping]', file=sys.stderr, flush=True)
    print('Final PCD save failed: disk full', file=sys.stderr, flush=True)
    raise SystemExit(0)''')
    executable.write_text(script)
    async def scenario():
        await manager.start(dict(mode='mapping', save=True))
        await wait_ready(manager)
        await manager.close()
        assert manager.current['returncodes']['mapping'] == 0
        assert manager.current['save_result'] == ('failed' if outcome == 'node_failure' else outcome)
        if outcome == 'node_failure':
            assert manager.current['node_failures']['mapping'] == 1
            assert manager.current['state'] == 'failed'
    asyncio.run(scenario())


@pytest.mark.parametrize('code', [0, 1])
def test_unified_launch_unexpected_exit_fails_session(manager, tmp_path, monkeypatch, code):
    install_fake_ros(tmp_path, monkeypatch)
    executable = tmp_path / 'ros2'
    executable.write_text(executable.read_text().replace(
        'while True: time.sleep(.1)', f'time.sleep(.5); raise SystemExit({code})'))
    manager.bridge.ready = lambda mode: True
    async def scenario():
        await manager.start(dict(mode='relocation', map='prior.pcd', save=False))
        try:
            await wait_ready(manager)
            for _ in range(100):
                if manager.current['state'] == 'failed':
                    break
                await asyncio.sleep(.05)
            assert manager.current['state'] == 'failed'
            assert '异常退出' in manager.current['error']
            assert set(manager.processes) == {'mapping'}
        finally:
            await manager.close()
    asyncio.run(scenario())


def test_log_creation_failure_still_drains_and_finishes(manager, tmp_path, monkeypatch):
    install_fake_ros(tmp_path, monkeypatch)
    original_open = Path.open
    def failing_open(path, *args, **kwargs):
        if path.name == 'mapping.stdout.log':
            raise OSError('disk full')
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'open', failing_open)
    async def scenario():
        await manager.start(dict(mode='mapping', save=False))
        await asyncio.wait_for(manager.close(), 10)
        assert manager.current['state'] == 'failed'
        assert '日志创建失败' in manager.current['error']
    asyncio.run(scenario())


def test_unified_core_failure_does_not_interrupt_launch_save_twice(manager, tmp_path, monkeypatch):
    install_fake_ros(tmp_path, monkeypatch)
    executable = tmp_path / 'ros2'
    script = executable.read_text().replace('while True: time.sleep(.1)', '''
signal.signal(signal.SIGINT, lambda *_: print('DUPLICATE SIGINT', flush=True))
print('[ERROR] [transform_publisher-1]: process has died [pid 1, exit code 1]', flush=True)
# The unified launch is already saving its surviving mapping node.
time.sleep(.6)
pathlib.Path(p['pcd_save']['output_dir'], 'result.pcd').write_bytes(PCD)
'''.replace('write_bytes(PCD)', f'write_bytes({PCD!r})'))
    executable.write_text(script)
    async def scenario():
        await manager.start(dict(mode='mapping', save=True))
        try:
            await wait_ready(manager)
            for _ in range(100):
                if manager.current['state'] == 'failed':
                    break
                await asyncio.sleep(.05)
            assert manager.current['state'] == 'failed'
            assert manager.current['save_result'] == 'failed'
            assert manager.current['maps']
            assert 'DUPLICATE SIGINT' not in manager.logs(manager.current['session_id'])['text']
        finally:
            await manager.close()
    asyncio.run(scenario())


def test_guardian_service_sigkill_reaps_algorithm(tmp_path):
    env={**os.environ,'PYTHONPATH':str(APP)}
    marker=tmp_path/'finished'
    child=tmp_path/'child.py'
    child.write_text(f'''import signal,time,pathlib,os
pathlib.Path({str(tmp_path/'pid')!r}).write_text(str(os.getpid()))
def stop(*_):
 pathlib.Path({str(marker)!r}).write_text('saved')
 raise SystemExit(0)
signal.signal(signal.SIGINT,stop)
while True:time.sleep(.1)
''')
    parent=tmp_path/'parent.py'
    parent.write_text('''import subprocess,sys,time,json
p=subprocess.Popen([sys.executable,'-m','slam_console.session','--guard','3',json.dumps([sys.executable,sys.argv[1]])],stdin=subprocess.PIPE)
while True:time.sleep(.1)
''')
    process=subprocess.Popen([sys.executable,str(parent),str(child)],env=env,start_new_session=True,stdout=subprocess.DEVNULL)
    try:
        for _ in range(100):
            if (tmp_path/'pid').exists():break
            time.sleep(.05)
        assert (tmp_path/'pid').exists()
        pid=int((tmp_path/'pid').read_text())
        process.kill();process.wait()
        for _ in range(100):
            if marker.exists() and not Path(f'/proc/{pid}').exists():break
            time.sleep(.05)
        assert marker.read_text()=='saved'
        assert not Path(f'/proc/{pid}').exists()
    finally:
        if process.poll() is None:process.kill();process.wait()


def test_preflight_reports_all_groups_and_blocks_active_probes(manager, monkeypatch):
    result = manager.preflight(dict(mode='relocation', map='../escape', save=True))
    checks = {c['id']: c for c in result['checks']}
    assert not result['ok'] and result['checked_at'] <= time.time()
    assert checks['options']['status'] == 'fail'
    assert checks['dependencies']['status'] == 'pass'
    assert checks['output_directory']['status'] == 'unknown'
    assert checks['runtime_directory']['status'] == 'pass'
    manager.current['state'] = 'running'
    monkeypatch.setattr(manager.bridge, 'preflight_checks', lambda: pytest.fail('active preflight must not scan conflicts'))
    assert manager.preflight(dict(mode='mapping', save=True))['checks'][0]['id'] == 'session'


def test_runtime_health_does_not_change_lifecycle_or_call_preflight(manager, monkeypatch):
    manager.current.update(state='running', mode='mapping', session_id='one')
    monkeypatch.setattr(manager.bridge, 'preflight_checks', lambda: pytest.fail('status must not run full preflight'))
    for state in ('valid', 'tracking_invalid', 'odom_stale'):
        monkeypatch.setattr(manager.bridge, 'health', lambda mode: {'state': state, 'checks': [], 'inputs': []})
        result = manager.status()
        assert result['state'] == 'running'
        assert result['health']['state'] == state
    manager.current['state'] = 'starting'
    manager.started_mono = time.monotonic() - 5
    result = manager.status()
    assert 5 <= result['initialization']['elapsed'] < 6
    assert result['initialization']['timeout'] == 60
    manager.current['state'] = 'stopped'
    assert manager.status()['health']['state'] == 'unknown'


def test_output_roots_follow_backend_configuration(manager, tmp_path):
    for root in (tmp_path / 'algorithm', tmp_path / 'other_algorithm'):
        manager.config['algorithm_root'] = root
        status = manager.status()
        assert status['output_roots'] == {
            'mapping': str(root / 'PCD/priori'),
            'relocation': str(root / 'PCD/relocate'),
        }
