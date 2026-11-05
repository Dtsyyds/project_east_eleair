from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.contracts import load_config
from d405_seam_tracking.geometry import roi_bounds
from d405_seam_tracking.recovery import RecoveryTracker, propose
from d405_seam_tracking.tracking_error import ErrorReference, tracking_error


ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture
def config():
    return load_config(ROOT/'config/d405/round1_a.yaml')


def probability(stripes):
    p = np.zeros((480,640), np.float32)
    for x, score, end in stripes:
        p[168:end, x-5:x+6] = score
    return p


def test_three_distinct_ranked_current_image_proposals_and_no_padding(config):
    p=probability([(220,.45,456),(320,.40,456),(420,.35,456)])
    values=propose(p,config,roi_bounds(p.shape,config))
    assert len(values)==3
    assert [round(np.mean(np.array(v['points_2d_y_center'])[:,1])) for v in values]==[220,320,420]
    assert [v['rank'] for v in values]==[1,2,3]
    assert all(v['measurement_supported'] for v in values)
    assert all(values[i]['score']>values[i+1]['score'] for i in range(2))
    assert len(propose(probability([(320,.4,456)]),config,roi_bounds(p.shape,config)))==1
    assert not propose(np.zeros_like(p),config,roi_bounds(p.shape,config))
    assert not propose(np.full_like(p,np.nan),config,roi_bounds(p.shape,config))


def pipeline_fixture(monkeypatch,config):
    from d405_seam_tracking import round1_a, candidate_pipeline
    class Visual:
        model_sha256='fixture';last_transform=None
        def __init__(self,config):pass
        def infer(self,image):return image[:,:,0].astype(np.float32)/255.,'fixture',None
        def close(self):pass
    monkeypatch.setattr(round1_a,'Round1AVisual',Visual)
    monkeypatch.setattr(candidate_pipeline,'CandidateVisual',lambda original:original)
    pipe=candidate_pipeline.CandidatePipeline(config)
    clock=[0.]
    pipe.recovery=RecoveryTracker(lambda:clock[0])
    base=synthetic_frame(shape=(480,640))
    def process(seq,stripes):
        image=np.repeat(np.uint8(probability(stripes)*255)[:,:,None],3,axis=2)
        frame=replace(base,image=image,sequence=seq,image_stamp_ns=1_000_000_000+seq*100_000_000,
                      source=dict(base.source,session='test',stream='color',image_frame_id='test_optical'))
        return pipe.process(frame)
    return pipe,clock,process


def request(result,ident='choose',candidate='1'):
    return dict(request_id=ident,session_id='test',action='select',
                candidate_set_id=result['candidate_set_id'],candidate_id=candidate)


def test_choice_tracks_new_measurement_not_new_highest_rank_or_old_frame(monkeypatch,config):
    pipe,clock,process=pipeline_fixture(monkeypatch,config)
    initial=process(1,[(220,.45,456),(320,.4,456),(420,.35,456)])
    assert initial['state']=='LOST' and len(initial['candidates'])==3
    assert not tracking_error(initial,640,480,ErrorReference())[0]['valid']
    command=request(initial)
    ack=pipe.recovery.command(command)
    assert ack['status']=='completed' and ack['selection']['state']=='ACQUIRING'
    assert pipe.recovery.command(command)==ack  # Lost ack/retry never selects twice.
    clock[0]=.1
    current=process(2,[(228,.4,456),(420,.95,456)])
    assert current['state']=='CENTER_2D' and current['selection']['state']=='TRACKING'
    assert current['measurement_kind']=='SELECTED_CENTER_2D'
    assert np.allclose(np.array(current['points_2d_y_center'])[:,1],228)
    assert not current['predicted_only'] and not current['points_2d_y_left_right']
    error,_=tracking_error(current,640,480,ErrorReference())
    assert error['valid'] and error['lateral_error_norm']==pytest.approx((228-320)/320)
    # An absent selected target must not silently switch to the strong other seam.
    clock[0]=.2
    lost=process(3,[(420,.95,456)])
    assert lost['state']=='LOST' and not lost['points_2d_y_center']
    assert not tracking_error(lost,640,480,ErrorReference())[0]['valid']
    clock[0]=.8
    still_lost=process(4,[(228,.9,456),(420,.95,456)])
    assert not still_lost['valid_2d'] and still_lost['selection']['state']=='RESELECT_REQUIRED'
    assert pipe.recovery.command(dict(request_id='clear',session_id='test',action='clear'))['status']=='completed'
    automatic=process(5,[(420,.95,456)])
    assert automatic['state']=='VISION_2D' and automatic['selection']['state']=='AUTO'


def test_stale_wrong_session_invalid_id_and_source_timeout(monkeypatch,config):
    pipe,clock,process=pipeline_fixture(monkeypatch,config)
    result=process(1,[(220,.4,456)])
    bad=dict(request(result),session_id='previous-run')
    assert pipe.recovery.command(bad)['status']=='failed'
    assert pipe.recovery.command(request(result,'bad-id','4'))['status']=='failed'
    clock[0]=6.
    assert pipe.recovery.command(request(result,'expired'))['error']=='candidate_set_expired'
    fresh=process(2,[(220,.4,456)])
    assert pipe.recovery.command(request(fresh,'fresh'))['status']=='completed'
    pipe.recovery.expire('source_timeout')
    assert pipe.recovery.status()['state']=='RESELECT_REQUIRED'
    assert not process(3,[(220,.9,456)])['valid_2d']


def test_short_support_choice_does_not_create_valid_measurement(monkeypatch,config):
    pipe,clock,process=pipeline_fixture(monkeypatch,config)
    result=process(1,[(220,.4,270)])
    assert len(result['candidates'])==1 and not result['candidates'][0]['measurement_supported']
    assert pipe.recovery.command(request(result))['status']=='completed'
    clock[0]=.1
    result=process(2,[(220,.4,270)])
    assert not result['valid_2d'] and result['points_2d_y_center']==[]


def test_ambiguous_branches_and_resolution_change_require_reselection(monkeypatch,config):
    pipe,clock,process=pipeline_fixture(monkeypatch,config)
    first=process(1,[(320,.4,456)])
    pipe.recovery.command(request(first))
    clock[0]=.1
    ambiguous=process(2,[(304,.4,456),(336,.4,456)])
    assert not ambiguous['valid_2d'] and ambiguous['selection']['state']=='LOST'
    # Regressed acquisition time invalidates target identity and pending offers.
    result=process(1,[(320,.9,456)])
    assert result['selection']['state']=='RESELECT_REQUIRED' and not result['valid_2d']
