from dataclasses import replace
from pathlib import Path
import numpy as np
import pytest
from d405_seam_tracking.single_boundary import SingleBoundary
from d405_seam_tracking.contracts import load_config
from d405_seam_tracking.geometry import candidate, roi_bounds
from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.tracking_error import tracking_error, ErrorReference

ROOT = Path(__file__).resolve().parents[3]


def scene(side='right'):
    p = np.zeros((480, 640), np.float32)
    for i, y in enumerate(range(168, 456)):
        wobble = 14 if (i//3) % 2 else -14
        left, end = (300, 321+wobble) if side == 'right' else (300+wobble, 321)
        p[y, left:end] = .9
    return p


@pytest.mark.parametrize('worn,good', [('right','left'),('left','right')])
def test_fixed_width_recovers_current_reliable_side(worn, good):
    cfg=load_config(ROOT/'config/d405/round1_a.yaml')
    cfg['single_boundary_width_px']=20.
    decoder=SingleBoundary(cfg);p=scene(worn)
    result=decoder.infer(p,roi_bounds(p.shape,cfg),1)
    assert result and result['side']==good
    assert np.allclose(np.asarray(result['points'])[:,1],310)
    wire=dict(state='CENTER_2D',center_2d_valid=True,valid_2d=True,predicted_only=False,
              coordinate_2d='original_image_pixel_centers',confidence=result['confidence'],
              points_2d_y_center=result['points'])
    error,reason=tracking_error(wire,640,480,ErrorReference())
    assert reason is None and error['valid']
    assert error['lateral_error_norm']==pytest.approx(-10/320)
    assert abs(error['heading_error_rad'])<1e-10


def test_width_requires_three_consistent_full_measurements_and_expires():
    cfg=load_config(ROOT/'config/d405/round1_a.yaml')
    decoder=SingleBoundary(cfg);p=scene();bounds=roi_bounds(p.shape,cfg)
    assert decoder.infer(p,bounds,1_000_000_000) is None
    full=np.zeros_like(p);full[168:456,300:321]=.9
    a=candidate(full,cfg,bounds)
    result=dict(state='VISION_2D',visual=a,points_2d_y_left_right=a['points'])
    for i in range(3):
        decoder.learn(result,1_000_000_000+i*100_000_000)
        if i<2: assert decoder.infer(p,bounds,1_200_000_000) is None
    recovered=decoder.infer(p,bounds,1_300_000_000)
    assert recovered and np.allclose(np.array(recovered['points'])[:,1],310)
    other=[[y,420.] for y in range(168,456,3)]
    assert decoder.infer(p,bounds,1_300_000_000,other) is None
    assert decoder.infer(p,bounds,3_300_000_000) is None
    assert decoder.learn(dict(state='CENTER_2D'),3_000_000_000) is None
    assert decoder.learned_at==1_200_000_000
    frame=synthetic_frame()
    decoder.begin(frame)
    decoder.begin(replace(frame,image_stamp_ns=frame.image_stamp_ns-1))
    assert decoder.samples==0


def test_blank_ambiguous_edges_and_other_target_are_rejected():
    cfg=load_config(ROOT/'config/d405/round1_a.yaml');cfg['single_boundary_width_px']=20.
    decoder=SingleBoundary(cfg);p=np.zeros((480,640),np.float32);bounds=roi_bounds(p.shape,cfg)
    assert decoder.infer(p,bounds,1) is None
    p[168:456,300:361]=.9 # Both strong edges conflict with configured width.
    assert decoder.infer(p,bounds,1) is None
    anchor=[[y,420.] for y in range(168,456,3)]
    assert decoder.infer(scene(),bounds,1,anchor) is None


def test_candidate_pipeline_learns_then_outputs_single_edge_and_error(monkeypatch):
    from d405_seam_tracking import round1_a, candidate_pipeline
    class Visual:
        model_sha256='fixture';last_transform=None
        def __init__(self,config): pass
        def infer(self,image): return image[:,:,0].astype(np.float32)/255.,'fixture',None
        def close(self): pass
    monkeypatch.setattr(round1_a,'Round1AVisual',Visual)
    monkeypatch.setattr(candidate_pipeline,'CandidateVisual',lambda v:v)
    cfg=load_config(ROOT/'config/d405/round1_a.yaml')
    pipe=candidate_pipeline.CandidatePipeline(cfg)
    base=synthetic_frame(shape=(480,640));full=np.zeros((480,640),np.float32)
    full[168:456,300:321]=.9
    for i,p in enumerate([full,full,full,scene()]):
        frame=replace(base,image=np.repeat(np.uint8(p*255)[:,:,None],3,axis=2),
                      image_stamp_ns=1_000_000_000+i*100_000_000,sequence=i,
                      source=dict(base.source,session='single-edge'))
        result=pipe.process(frame)
    assert result['measurement_kind']=='SINGLE_BOUNDARY_2D'
    assert result['points_2d_y_left_right']==[] and result['single_boundary']['side']=='left'
    assert tracking_error(result,640,480,ErrorReference())[0]['valid']
    pipe.recovery.manual=True;pipe.recovery.anchor=result['points_2d_y_center']
    pipe.recovery.last_measurement_at=pipe.recovery.clock()
    selected=pipe.process(replace(frame,image_stamp_ns=1_400_000_000,sequence=4))
    assert selected['measurement_kind']=='SELECTED_SINGLE_BOUNDARY_2D'
    assert tracking_error(selected,640,480,ErrorReference())[0]['valid']
