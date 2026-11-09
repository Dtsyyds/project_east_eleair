"""Passive experiment controls: no ROS, devices or robot commands."""
import copy
import hashlib
import json
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
from inspection_runtime.vision import VisionController, INPUTS, tracking_payload
from d405_seam_tracking.writer import EventWriter, SampleCapture
from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.dataset import load_index, read_sample
from test_operations import controller, request, wait_result


def test_missing_or_mismatched_model_stops_before_session_or_process(tmp_path):
    import yaml
    from inspection_runtime.vision import runtime_command
    checkpoint = tmp_path / 'checkpoint.pt'
    cfg = tmp_path / 'algorithm.yaml'
    cfg.write_text(yaml.safe_dump({'algorithm': 'round1_a', 'checkpoint': str(checkpoint),
                                 'checkpoint_sha256': hashlib.sha256(b'expected').hexdigest()}))
    config = {'data_root': str(tmp_path), 'minimum_free_bytes': 1,
              'vision': {'python': sys.executable, 'algorithm_config': str(cfg)}}
    calls=[]
    v=VisionController(config, lambda:{'mounted':True,'free_bytes':100},
                       lambda:{t:time.monotonic() for t in INPUTS},
                       lambda _: {'available':False}, popen=lambda *a,**k:calls.append(a))
    with pytest.raises(RuntimeError,match='模型权重缺失'):v.start()
    assert not calls and not (tmp_path/'vision_experiments').exists()
    assert v.snapshot()['status']=='failed' and 'SHA256' in v.snapshot()['message']
    checkpoint.write_bytes(b'wrong')
    with pytest.raises(RuntimeError,match='权重校验失败'):v.start()
    assert not calls
    checkpoint.write_bytes(b'expected')
    path,command=runtime_command(config)
    assert path==cfg and command[0]==sys.executable


def test_sample_waits_for_actual_file_and_preserves_unlabelled_raw(tmp_path):
    writer = EventWriter(tmp_path/'samples')
    capture = SampleCapture(writer)
    frame = synthetic_frame(); frame.source['session'] = 'experiment-A'
    try:
        queued = capture.request(); assert queued['status']=='queued'
        capture.capture(frame, {'state':'LOST'}, None)
        deadline=time.monotonic()+5
        while capture.poll()['status']=='writing' and time.monotonic()<deadline: time.sleep(.01)
        done=capture.poll();assert done['status']=='completed' and done['request_id']==queued['request_id']
        root=writer.root/done['path'];meta=json.loads((root/'metadata.json').read_text())
        raw=read_sample(root,meta['sample'])
        assert (raw.image==frame.image).all() and (raw.depth==frame.depth).all()
        assert meta['sample']['split']=='unassigned' and meta['sample']['annotation'] is None
        capture.request(); capture.capture(frame,{'state':'LOST'},None)
        assert capture.poll()['status']=='failed'  # Same source stamp is rate-limited, not reported saved.
    finally: writer.close()


def test_no_input_timeout_and_writer_failure(tmp_path):
    clock=[0.];writer=EventWriter(tmp_path/'samples');capture=SampleCapture(writer,lambda:clock[0])
    try:
        capture.request();clock[0]=9
        assert capture.poll()['status']=='failed'
        capture.capture(synthetic_frame(),{},None)
        assert writer.worker.accepted==0
    finally:writer.close()


def test_controller_guards_ownership_storage_and_inputs(tmp_path):
    called=[]
    storage={'mounted':False,'free_bytes':1000};live={}
    v=VisionController({'data_root':str(tmp_path),'minimum_free_bytes':10},lambda:storage,lambda:live,
                       lambda name:{'available':True},popen=lambda *a,**k:called.append(a))
    with pytest.raises(RuntimeError,match='数据盘'):v.start()
    storage['mounted']=True
    with pytest.raises(RuntimeError,match='D405'):v.start()
    live.update({t:time.monotonic() for t in INPUTS})
    with pytest.raises(RuntimeError,match='独立巡迹'):v.start()
    assert not called


@pytest.mark.parametrize('selected', [None, 'd405/vertical_seam.yaml'])
def test_controller_freezes_selected_algorithm_config(tmp_path, monkeypatch, selected):
    from inspection_runtime.paths import config_path
    relative = selected or 'd405/experimental.yaml'
    source = Path(config_path(relative)).read_bytes()
    commands = []
    def start(command, **kwargs):
        commands.append(command)
        v.diagnostic({'session_id': v.state['session_id'], 'processed': 1})
        return SimpleNamespace(poll=lambda: None)
    config = {'data_root': str(tmp_path), 'minimum_free_bytes': 10}
    if selected:
        config['vision'] = {'algorithm_config': selected}
    v = VisionController(config, lambda: {'mounted': True, 'free_bytes': 1000},
                         lambda: {t: time.monotonic() for t in INPUTS},
                         lambda name: {'available': False}, popen=start)
    try:
        assert v.start()['status'] == 'running'
        frozen = Path(commands[0][commands[0].index('--config') + 1])
        assert frozen.parent == v.session and frozen.read_bytes() == source
        # Editing the configured source would not change this running session.
        assert frozen.resolve() != Path(config_path(relative)).resolve()
    finally:
        if v.log: v.log.close()
        v.process = None


def test_controller_saved_index_and_hash_rejection(tmp_path):
    frame=synthetic_frame();frame.source['session']='test-session'
    writer=EventWriter(tmp_path/'samples');capture=SampleCapture(writer)
    def service(name):
        if name=='save_snapshot':
            value=capture.request();capture.capture(frame,{'state':'LOST'},None);return value
        return capture.poll()
    v=VisionController({'data_root':str(tmp_path),'minimum_free_bytes':10},lambda:{'mounted':True,'free_bytes':1000},
                       lambda:{t:time.monotonic() for t in INPUTS},service)
    v.session=tmp_path;v.state.update(status='running',session_id='test-session');v.last_diagnostic=time.monotonic()
    try:
        saved=v.save_sample('recording-run')
        assert saved['status']=='completed'
        root,index=load_index(tmp_path/'index.json')
        assert len(index['samples'])==1 and read_sample(root,index['samples'][0]).source['session']=='test-session'
        assert json.loads((Path(saved['path'])/'collection_reference.json').read_text())['run_id']=='recording-run'
        # Backend never treats a completion string alone as proof of persisted bytes.
        (Path(saved['path'])/'raw.npz').write_bytes(b'corrupt')
        v.service_call=lambda name:dict(capture.poll())
        with pytest.raises(RuntimeError,match='校验失败'):v.save_sample()
    finally:writer.close()


def test_adapter_frame_identity_resize_and_invalid_geometry():
    result={'source':{'image_width':1280,'image_height':720,'image_stamp_ns':'1234567890123456789',
                    'source':{'stream':'color','image_frame_id':'d405_color_optical_frame'}},
            'points_2d_y_left_right':[[100,400,600]],'valid_2d':True,'state':'VISION_2D','metric_valid':False,
            'mode':'fused','seam_identity_confirmed':False,'predicted_only':False,'reasons':[],
            'confidence':.8,'timing_ms':{'total':20}}
    value=tracking_payload(result,640)
    assert value['centerline']==[[249.75,49.75]]
    assert value['frame_ref']['stamp_ns']=='1234567890123456789'
    assert [value['image_width'],value['image_height']]==[640,360]
    result['valid_2d']=False;result['state']='LOST'
    assert tracking_payload(result,640)['centerline']==[]
    result.update(candidate_set_id='set-1',candidates=[{'candidate_id':'1','rank':1,'score':.4,'coverage':.7,
        'confidence':.4,'points_2d_y_center':[[100,500],[200,520]]}])
    mapped=tracking_payload(result,640)
    assert mapped['candidates'][0]['centerline']==[[249.75,49.75],[259.75,99.75]]
    assert mapped['candidate_set_id']=='set-1' and mapped['vision_2d_valid'] is False
    result['source']['source']['stream']='infra1'
    with pytest.raises(ValueError,match='来源'):tracking_payload(result,640)


class FakeVision:
    def __init__(self):self.state={'status':'idle','diagnostics_fresh':True};self.calls=[]
    def snapshot(self):return copy.deepcopy(self.state)
    def start(self):self.calls.append('start');self.state['status']='running'
    def stop(self):self.calls.append('stop');self.state['status']='stopped'
    def save_sample(self,run_id=None):self.calls.append(('save',run_id))
    def select_candidate(self,parameters,clear=False):self.calls.append(('select',parameters,clear))
    def close(self):pass


def test_operations_idempotency_recording_independence_and_replay_gate(controller):
    ops,events,live=controller;ops.vision=FakeVision()
    ops.task={'status':'recording','run_id':'continuous-run'}
    req=request('vision.start');ops.handle(req)
    assert wait_result(events,'test')['status']=='completed'
    ops.handle(req);assert ops.vision.calls==['start']
    ops.handle(request('vision.save_sample','save'))
    assert wait_result(events,'save')['status']=='completed'
    assert ops.vision.calls[-1]==('save','continuous-run')
    ops.handle(request('replay.start','replay'))
    assert wait_result(events,'replay')['status']=='failed'
    ops.handle(request('vision.stop','stop'))
    assert wait_result(events,'stop')['status']=='completed'
    assert ops.task=={'status':'recording','run_id':'continuous-run'}
    ops.handle(request('vision.start','bad',path='/tmp/injected'))
    assert wait_result(events,'bad')['status']=='failed'
    ops.task={'status':'idle'}


def test_capabilities_absent_without_real_adapter(controller):
    ops,events,_=controller
    assert 'vision.start' not in ops.snapshot()['capabilities']['operations']
    ops.handle(request('vision.start'))
    assert wait_result(events,'test')['status']=='failed'


def test_selection_operations_route_parameters_without_starting_chassis(controller):
    ops,events,_=controller;ops.vision=FakeVision()
    ops.config['vision']={'runtime_mode':'candidate'}
    parameters=dict(session_id='test-session',candidate_set_id='test-set',candidate_id='2')
    assert 'vision.select_candidate' in ops.snapshot()['capabilities']['operations']
    ops.handle(request('vision.select_candidate','select',**parameters))
    assert wait_result(events,'select')['status']=='completed'
    assert ops.vision.calls==[('select',parameters,False)]
    ops.handle(request('vision.clear_selection','clear',session_id='test-session'))
    assert wait_result(events,'clear')['status']=='completed'
    assert ops.vision.calls[-1]==('select',{'session_id':'test-session'},True)


def test_selection_controller_checks_live_session_and_awaits_ack(tmp_path):
    calls=[]
    def service(name,request):
        calls.append((name,request));return {'selection':{'state':'ACQUIRING'}}
    v=VisionController({},lambda:None,lambda:{},service)
    v.state.update(status='running',session_id='current',diagnostics={'candidate_selection_supported':True})
    v.last_diagnostic=time.monotonic()
    parameters=dict(session_id='current',candidate_set_id='set',candidate_id='2')
    v.select_candidate(parameters)
    assert calls[0][0]=='select_candidate' and calls[0][1]['candidate_id']=='2'
    assert v.state['selection']['state']=='ACQUIRING'
    with pytest.raises(RuntimeError,match='会话'):
        v.select_candidate(dict(parameters,session_id='old'))
    with pytest.raises(ValueError):v.select_candidate(dict(parameters,path='/tmp/untrusted'))


def test_async_write_error_never_reports_sample_completed(tmp_path):
    def fail(item): raise OSError('test disk error')
    writer=EventWriter(tmp_path/'samples',process=fail);capture=SampleCapture(writer)
    try:
        capture.request();capture.capture(synthetic_frame(),{},None)
        deadline=time.monotonic()+5
        while capture.poll()['status']=='writing' and time.monotonic()<deadline:time.sleep(.01)
        assert capture.poll()['status']=='failed'
        assert not list(writer.root.glob('*/metadata.json'))
    finally:
        try:writer.close()
        except RuntimeError:pass


def test_stop_timeout_keeps_owned_process_for_retry(tmp_path,monkeypatch):
    import subprocess
    v=VisionController({'data_root':str(tmp_path)},lambda:{},lambda:{},lambda n:{})
    def timeout(**kwargs):raise subprocess.TimeoutExpired('owned-fixture',kwargs['timeout'])
    process=SimpleNamespace(pid=123456,poll=lambda:None,wait=timeout)
    v.process=process
    signals=[];monkeypatch.setattr('inspection_runtime.vision.os.killpg',lambda pid,sig:signals.append((pid,sig)))
    with pytest.raises(subprocess.TimeoutExpired):v.stop()
    assert v.snapshot()['status']=='failed' and v.process is process
    assert len(signals)==2 and all(pid==123456 for pid,sig in signals)
