"""Ordering ablation: fixed aim point, old ties, unchanged post-selection gate."""
import json
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'experiments/vision/scripts'))
import d405_candidate_ranking_ab as ab


def path(i,x,score):
    return dict(id=i,median_x=x,dp_score=score)


def test_empty_and_single_keep_original_behavior_even_far_outside_old_window():
    assert ab.ranking([],640)==[]
    p=path(1,150,10)
    assert ab.ranking([p],640)==[p]


def test_fixed_half_width_not_pixel_index_midpoint():
    left=path(1,319,100)
    right=path(2,320,1)
    assert ab.ranking([left,right],640)[0] is right
    assert ab.ranking([path(1,200,10),path(2,400,1)],800)[0]['id']==2


def test_distance_tie_uses_original_score_then_original_scan_order():
    candidates=[path(3,330,8),path(2,310,9),path(1,330,9)]
    assert [p['id'] for p in ab.ranking(candidates,640)]==[1,2,3]


def test_no_hard_window_and_no_input_mutation():
    candidates=[path(1,180,10),path(2,490,20)]
    original=[dict(p) for p in candidates]
    assert ab.ranking(candidates,640)[0]['id']==1
    assert candidates==original


def test_closest_short_prefix_is_not_replaced_after_coverage_failure():
    config=json.loads((ROOT/'.runtime/s100p/analysis/d405_unet_round1_20261102/visual_config.json').read_text())
    prob=np.zeros((480,640),np.float32)
    prob[168:456,350:359]=.9
    prob[168:174,316:325]=.8
    bounds=ab.geometry.roi_bounds(prob.shape,config)
    observer,_=ab.expose_paths()
    _,_,layers=observer(prob,config,bounds)
    candidates=ab.paths_from_layers(layers,640)
    original=candidates[0]
    chosen=ab.ranking(candidates,640)[0]
    assert original['median_x']==354 and chosen['median_x']==320
    assert ab.fit_path(original,prob,config,bounds)['valid']
    result=ab.fit_path(chosen,prob,config,bounds)
    assert not result['valid'] and result['reasons']==['insufficient_continuous_coverage']
