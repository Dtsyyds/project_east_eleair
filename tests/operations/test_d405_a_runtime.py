"""A integration is frame-local; legacy temporal/fused behavior stays explicit."""
import json
import hashlib
import sys
import yaml
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import time
import numpy as np

from d405_seam_tracking.contracts import load_config
from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking import round1_a
from d405_seam_tracking.geometry import candidate, roi_bounds
from inspection_runtime.vision import VisionController, INPUTS, tracking_payload
from inspection_runtime.paths import project_root

ROOT=project_root()


def test_a_config_preserves_every_geometry_parameter():
    config=load_config(ROOT/'config/d405/round1_a.yaml')
    frozen=json.loads((ROOT/'.runtime/s100p/analysis/d405_unet_round1_20261102/visual_config.json').read_text())
    for key,value in frozen.items():
        if key not in ('mode','visual_backend','checkpoint'):
            assert config[key]==value
    assert config['mode']=='vision_only'
    legacy=round1_a.create_pipeline(load_config(ROOT/'config/d405/vertical_seam.yaml'))
    assert type(legacy) is round1_a.Pipeline
    legacy.close()


def test_a_no_smoothing_no_stale_prediction_with_close_source_times(monkeypatch):
    class Visual:
        model_sha256='fixture';last_transform=None
        def __init__(self,config):pass
        def infer(self,image):return image[:,:,0].astype(np.float32)/255,'fixture',None
        def close(self):pass
    monkeypatch.setattr(round1_a,'Round1AVisual',Visual)
    config=load_config(ROOT/'config/d405/round1_a.yaml')
    pipeline=round1_a.create_pipeline(config)
    frame=synthetic_frame(shape=(480,640))
    for i,x in enumerate((300,310,None,330,None)):
        image=np.zeros((480,640,3),np.uint8)
        if x is not None:image[:,x:x+12]=255
        current=replace(frame,image=image,image_stamp_ns=frame.image_stamp_ns+i*50_000_000,
                        source=dict(frame.source,stream='color',image_frame_id='fixture'))
        result=pipeline.process(current)
        expected=candidate(image[:,:,0]/255,config,roi_bounds(image.shape,config))
        assert result['valid_2d']==expected['valid']
        assert result['points_2d_y_left_right']==expected.get('points',[])
        assert not result['predicted_only'] and pipeline.last_measurement is None
        result['source'].update(image_width=640,image_height=480)
        if x is None:
            assert result['state']=='LOST'
            payload=tracking_payload(result,640)
            assert payload['centerline']==payload['left_boundary']==payload['right_boundary']==[]


def test_controller_a_uses_original_environment_passive_node_and_no_recorder(tmp_path):
    # Process construction is a unit test; do not depend on an ignored field checkpoint.
    checkpoint=tmp_path/'checkpoint.pt';checkpoint.write_bytes(b'fixture checkpoint')
    algorithm=yaml.safe_load((ROOT/'config/d405/round1_a.yaml').read_text())
    algorithm.update(checkpoint=str(checkpoint),checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest())
    algorithm_path=tmp_path/'algorithm.yaml';algorithm_path.write_text(yaml.safe_dump(algorithm))
    config={'data_root':str(tmp_path),'minimum_free_bytes':1,
            'vision':{'algorithm_config':str(algorithm_path),'python':sys.executable}}
    commands=[]
    def start(command,**kw):
        commands.append(command)
        v.diagnostic({'session_id':v.state['session_id']})
        return SimpleNamespace(poll=lambda:None)
    v=VisionController(config,lambda:{'mounted':True,'free_bytes':100},
                       lambda:{t:time.monotonic() for t in INPUTS},lambda _: {'available':False},popen=start)
    try:
        v.start()
        assert commands[0][:2]==[str(ROOT/config['vision']['python']),str(ROOT/'tools/vision/run_d405_node.py')]
        assert 'record' not in commands[0] and 'launch' not in commands[0]
        assert '--output' not in commands[0]  # Preview does not record frame results either.
    finally:
        if v.log:v.log.close()
        v.process=None
