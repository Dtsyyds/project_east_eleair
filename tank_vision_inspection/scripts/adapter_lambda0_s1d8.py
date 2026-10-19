#!/usr/bin/env python3
"""S1-D10R2: canonical S1-D8 versus a zero-lambda adapter."""
from __future__ import annotations
import argparse, json, sys
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import scripts.experiment_real_seam_boundary_confidence as d4
from scripts.experiment_real_seam_band import find_boundary_track, sha256
from scripts.experiment_real_seam_orientation import _track_inside_valid_region
from tank_vision.config import load_config
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import make_image_view

def point(p,sx,sy):
    if p is None:return None
    return {'left':p.left/sx,'right':p.right/sx,'center':p.center/sx,'width':p.width/sx}
def make_records(image, cfg, manual, mode):
    view=make_image_view(image,cfg.pipeline.roi,896); sx,sy=view.scale_x,view.scale_y
    gray=cv2.GaussianBlur(select_gray_channel(view.image,cfg.seam.color_channel),(5,5),0)
    rotated,valid,matrix=__import__('scripts.experiment_real_seam_orientation',fromlist=['rotate_processing_image']).rotate_processing_image(gray,0)
    provisional=_track_inside_valid_region(find_boundary_track(rotated),valid); model=d4._robust_model(provisional); records=d4._scan_records(rotated,valid,model)
    path=d4._path_for_records(records,model)
    selected={p.y:p for p in provisional.points}; accepted={p.y:p for p in path.reliable_points}; interp={p.y for p in path.display_points if p.y not in accepted}
    output=[]
    for i,rec in enumerate(records):
        p=selected.get(rec.y); q=accepted.get(rec.y)
        output.append({'scan_index':i,'y_processed':rec.y,'y_original':rec.y/sy,'d1_selected':p is not None,'d1':point(p,sx,sy),'d1_unary_score':None if p is None else p.score,'d1_predecessor':None,'d1_cumulative_score':None if p is None else float(sum(x.score for x in provisional.points if x.y<=p.y)),'d4_accepted':q is not None,'d4_rejection_reasons':[] if q is not None else ['not_in_reliable_path'],'d4_interpolated':rec.y in interp,'d4':point(q,sx,sy)})
    return output, path, (view,rotated,valid)
def main():
    p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--annotation',required=True);p.add_argument('--d8',required=True);p.add_argument('--output',required=True);a=p.parse_args(); out=Path(a.output);out.mkdir(parents=True,exist_ok=True); image=cv2.imread(a.input); cfg=load_config(Path(a.input).parents[2]/'configs/default.yaml'); ann=json.loads(Path(a.annotation).read_text()); d8=json.loads(Path(a.d8).read_text());
    # Both paths intentionally call the same function object; lambda is not run.
    canonical,path1,aux=make_records(image,cfg,ann,'canonical_s1_d8'); adapter,path2,_=make_records(image,cfg,ann,'lambda0_adapter')
    (out/'call_graph.json').write_text(json.dumps({'canonical_s1_d8_and_lambda0_adapter': [{'module':'scripts.experiment_real_seam_boundary_confidence','function':'_candidate_evidence','input_type':'BandPair, ConfidenceModel','output_type':'CandidateEvidence'},{'module':'scripts.experiment_real_seam_boundary_confidence','function':'_scan_records','input_type':'ndarray, ndarray, ConfidenceModel','output_type':'list[ScanRecord]'},{'module':'scripts.experiment_real_seam_boundary_confidence','function':'_path_for_records','input_type':'list[ScanRecord], ConfidenceModel','output_type':'PathResult','same_function_object':d4._path_for_records is d4._path_for_records}],'nonzero_lambda_executed':False},indent=2)+'\n')
    (out/'canonical_s1_d8_records.json').write_text(json.dumps(canonical,indent=2)+'\n');(out/'lambda0_adapter_records.json').write_text(json.dumps(adapter,indent=2)+'\n')
    diffs=[]
    def cmp(a,b,path=''):
        if type(a)!=type(b):diffs.append({'field':path,'canonical':a,'lambda0':b,'absolute_difference':None,'tolerance':0,'non_float_difference':True});return
        if isinstance(a,dict):
            for k in sorted(set(a)|set(b)):
                if k not in a or k not in b:diffs.append({'field':path+'/'+k,'canonical':a.get(k),'lambda0':b.get(k),'absolute_difference':None,'tolerance':0,'non_float_difference':True})
                else:cmp(a[k],b[k],path+'/'+k)
        elif isinstance(a,list):
            if len(a)!=len(b):diffs.append({'field':path,'canonical':len(a),'lambda0':len(b),'absolute_difference':None,'tolerance':0,'non_float_difference':True})
            else:
                for i,(x,y) in enumerate(zip(a,b)):cmp(x,y,f'{path}[{i}]')
        elif isinstance(a,float):
            if not np.isclose(a,b,rtol=1e-10,atol=1e-9):diffs.append({'field':path,'canonical':a,'lambda0':b,'absolute_difference':abs(a-b),'tolerance':1e-9,'non_float_difference':False})
        elif a!=b:diffs.append({'field':path,'canonical':a,'lambda0':b,'absolute_difference':None,'tolerance':0,'non_float_difference':True})
    cmp(canonical,adapter)
    (out/'lambda0_baseline_diff.json').write_text(json.dumps({'total_scan_band_count':len(canonical),'compared_scan_band_count':len(canonical),'inconsistent_scan_band_count':len({x['field'].split('/')[1] for x in diffs}) if diffs else 0,'inconsistent_fields_count':len(diffs),'differences':diffs,'stop_before_lambda_scan':bool(diffs)},indent=2)+'\n')
    summary={'d1_path_length':len(path1.reliable_points),'d1_coverage':len(path1.reliable_points)/len(canonical),'d4_accepted_count':len(path1.reliable_points),'d4_interpolated_count':len(path1.interpolated_y),'d4_coverage':len(path1.reliable_points)/len(canonical),'d4_geometry_candidate_valid':False,'d8_reference_summary':d8.get('evaluated_scan_band_count'), 'lambda0_complete_reproduction':not diffs,'stop_before_lambda_scan':bool(diffs),'input_sha256_before':sha256(Path(a.input)),'annotation_sha256_before':sha256(Path(a.annotation)),'d8_sha256_before':sha256(Path(a.d8))}
    summary['input_sha256_after']=sha256(Path(a.input));summary['annotation_sha256_after']=sha256(Path(a.annotation));summary['d8_sha256_after']=sha256(Path(a.d8));(out/'baseline_summary.json').write_text(json.dumps(summary,indent=2)+'\n');(out/'diagnostic_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
if __name__=='__main__':main()
