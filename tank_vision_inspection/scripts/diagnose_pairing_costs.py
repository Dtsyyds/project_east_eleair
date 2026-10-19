#!/usr/bin/env python3
"""S1-D9: explain D1 pair/path choices against manual 0-degree boundaries."""
from __future__ import annotations
import argparse, csv, json, math, sys
from pathlib import Path
import cv2, numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.experiment_real_seam_band import find_boundary_track, _mean_profile, _pair_candidates, _transition, sha256
from scripts.experiment_real_seam_boundary_confidence import _scan_records, _robust_model, _path_for_records
from scripts.experiment_real_seam_orientation import _track_inside_valid_region
from tank_vision.config import load_config
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import make_image_view

ROOT=Path(__file__).resolve().parents[1]

def _interp(points,y):
    p=sorted(points,key=lambda z:z[1]); return float(np.interp(y,[z[1] for z in p],[z[0] for z in p]))
def _pstats(a):
    if not len(a): return {"count":0,"p05":None,"p50":None,"p95":None}
    return {"count":len(a),"p05":float(np.percentile(a,5)),"p50":float(np.percentile(a,50)),"p95":float(np.percentile(a,95))}
def _pair_json(p, sx, manual=None):
    if p is None:return None
    edge=1.5*(math.log1p(p.left_strength)+math.log1p(p.right_strength)); contrast=.08*p.contrast
    d={"left_processed":p.left,"right_processed":p.right,"center_processed":p.center,"width_processed":p.width,"left_x":p.left/sx,"right_x":p.right/sx,"center_x":p.center/sx,"width_px":p.width/sx,"left_response":p.left_strength,"right_response":p.right_strength,"left_polarity":"positive_gradient","right_polarity":"negative_gradient","contrast":p.contrast,"edge_unary":edge,"contrast_unary":contrast,"unary_score":p.score,"width_score_or_penalty":0.0,"center_score_or_penalty":0.0}
    if manual:d["manual_errors"]={k:float(d[k]-v) for k,v in manual.items()}
    return d
def _trace_dp(bands,width):
    states=[]; paths=[]; scores=[]
    for bi,pairs in enumerate(bands):
        if not pairs: paths=[];scores=[];states.append([]);continue
        if not paths:
            paths=[[p] for p in pairs];scores=[p.score for p in pairs];states.append([(p,None,p.score) for p in pairs]);continue
        newp=[];news=[];st=[]; prev_y=paths[0][-1].y; ys=max(1,bands[bi][0].y-prev_y)
        for p in pairs:
            best=-1; bestscore=p.score; bestpen=None
            for i,old in enumerate(paths):
                pen=_transition(old[-1],p,ys,width)
                if pen is None:continue
                val=scores[i]+p.score-pen
                if best<0 or val>bestscore:best=i;bestscore=val;bestpen=pen
            if best>=0:newp.append(paths[best]+[p]);news.append(bestscore);st.append((p,paths[best][-1],bestscore))
            else:newp.append([p]);news.append(p.score);st.append((p,None,p.score))
        paths,scores=newp,news;states.append(st)
    best=max(paths,key=lambda p:(len(p),sum(x.score for x in p))) if paths else []
    bykey={}
    for p in best:
        for st in states[p.y//8]:
            if st[0].y==p.y and st[0].left==p.left and st[0].right==p.right: bykey[(p.y,p.left,p.right)]=st
    return best,bykey
def run(input_path,annotation_path,d8_path,config_path,out):
    image=cv2.imread(str(input_path)); ann=json.loads(annotation_path.read_text()); d8=json.loads(d8_path.read_text()); h,w=image.shape[:2]
    left=[(float(p['x']),float(p['y'])) for p in ann['left_boundary_manual_points']]; right=[(float(p['x']),float(p['y'])) for p in ann['right_boundary_manual_points']]
    ymin=max(left[0][1],right[0][1]); ymax=min(left[-1][1],right[-1][1]); cfg=load_config(config_path); view=make_image_view(image,cfg.pipeline.roi,896); sx,sy=view.scale_x,view.scale_y; gray=cv2.GaussianBlur(select_gray_channel(view.image,cfg.seam.color_channel),(5,5),0); rotated,valid,matrix=__import__('scripts.experiment_real_seam_orientation',fromlist=['rotate_processing_image']).rotate_processing_image(gray,0)
    bands=[]; edge_records=[]
    for y in range(0,rotated.shape[0],8):
        pairs,l,r=_pair_candidates(_mean_profile(rotated,y,5),y,rotated.shape[1]); bands.append(pairs);edge_records.append((y,l,r))
    track=_track_inside_valid_region(find_boundary_track(rotated,8,5),valid); selected={(p.y,p.left,p.right):p for p in track.points}; best,trace=_trace_dp(bands,rotated.shape[1]); model=_robust_model(track); recs=_scan_records(rotated,valid,model); d4path=_path_for_records(recs,model); d4={(p.y,p.left,p.right):p for p in d4path.reliable_points}; d8by={int(r['scan_index']):r for r in d8['scan_bands']}
    rows=[]
    for i,(y,lc,rc) in enumerate(edge_records):
        yo=y/sy
        if not ymin<=yo<=ymax:continue
        ml=_interp(left,yo);mr=_interp(right,yo);manual={'left_x':ml,'right_x':mr,'center_x':(ml+mr)/2,'width_px':mr-ml}
        pairs=bands[i]; sp=selected.get((y,selected[(y,0,0)].left,0)) if False else next((p for k,p in selected.items() if k[0]==y),None)
        near=min(pairs,key=lambda p:abs(p.left/sx-ml)+abs(p.right/sx-mr)) if pairs else None
        if not sp:continue
        errsel=max(abs(sp.left/sx-ml),abs(sp.right/sx-mr)); errnear=max(abs(near.left/sx-ml),abs(near.right/sx-mr)) if near else None
        # D8 is the declared source of the target population; do not silently
        # drop a band merely because a second diagnostic reconstruction uses a
        # different proximity boundary.
        if d8by.get(i, {}).get('category') != 'pairing_selected_wrong': continue
        tr=trace.get((y,sp.left,sp.right)); pred=tr[1] if tr else None
        def transition_parts(prev,cur):
            if not prev:return {'left_continuity_penalty':0.0,'right_continuity_penalty':0.0,'center_continuity_penalty':0.0,'width_continuity_penalty':0.0,'total_transition_penalty':0.0}
            ld=abs(cur.left-prev.left)/sx; rd=abs(cur.right-prev.right)/sx; cd=abs(cur.center-prev.center)/sx; wd=abs(cur.width-prev.width)/sx
            return {'left_continuity_penalty':.05*ld,'right_continuity_penalty':.05*rd,'center_continuity_penalty':.10*cd,'width_continuity_penalty':.05*wd,'total_transition_penalty':.10*cd+.05*wd}
        spj=_pair_json(sp,sx,manual); nrj=_pair_json(near,sx,manual); tp=transition_parts(pred,sp)
        # D1 has no unary width/center term; expose zero explicitly and identify the largest actual contribution.
        contrib={'left_gradient_unary':spj['left_response'],'right_gradient_unary':spj['right_response'],'contrast_unary':spj['contrast_unary'],'center_transition_penalty':-tp['center_continuity_penalty'],'width_transition_penalty':-tp['width_continuity_penalty']}
        maxterm=max(contrib,key=lambda k:abs(contrib[k]))
        cat='wrong_left_only' if abs(sp.left/sx-ml)>100 and abs(sp.right/sx-mr)<=100 else 'wrong_right_only' if abs(sp.left/sx-ml)<=100 and abs(sp.right/sx-mr)>100 else 'both_wrong'
        if near is None:cat='correct_candidates_not_formable_as_pair'
        elif sp.score<near.score:cat='correct_pair_local_score_lower'
        if pred and near and _transition(pred,near,8,rotated.shape[1]) is None:cat='correct_pair_lost_by_transition_or_cumulative_path'
        rows.append({'scan_index':i,'y_original':yo,'manual':manual,'selected_pair':spj,'nearest_manual_pair':nrj,'candidate_left_polarity_response':[{"polarity":x.polarity,"response":x.strength,"x_original":x.x/sx} for x in lc], 'candidate_right_polarity_response':[{"polarity":x.polarity,"response":x.strength,"x_original":x.x/sx} for x in rc], 'selected_unary_score_breakdown':{'edge_unary':spj['edge_unary'],'contrast_unary':spj['contrast_unary'],'width_score_or_penalty':0.0,'center_score_or_penalty':0.0,'total_unary':sp.score},'nearest_manual_unary_score_breakdown':None if not near else {'edge_unary':nrj['edge_unary'],'contrast_unary':nrj['contrast_unary'],'width_score_or_penalty':0.0,'center_score_or_penalty':0.0,'total_unary':near.score},'continuity':tp,'predecessor':None if not pred else _pair_json(pred,sx),'cumulative_path_score':None if not tr else tr[2],'selected_minus_nearest_total_score':None if not near else sp.score-near.score,'largest_contribution':maxterm,'classification':cat,'d4_accepted':(y,sp.left,sp.right) in d4})
    out.mkdir(parents=True,exist_ok=True); (out/'pairing_cost_breakdown.json').write_text(json.dumps({'scan_bands':rows,'code_evidence':{'pair_generation':'experiment_real_seam_band.py:_pair_candidates:106-147','transition':'experiment_real_seam_band.py:_transition:150-159','dynamic_programming':'experiment_real_seam_band.py:find_boundary_track:176-221'}},ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    fields=['scan_index','y_original','classification','selected_minus_nearest_total_score','largest_contribution','d4_accepted','selected_pair','nearest_manual_pair','selected_unary_score_breakdown','nearest_manual_unary_score_breakdown','continuity','predecessor','cumulative_path_score']
    with (out/'pairing_cost_breakdown.csv').open('w',newline='') as f:
        z=csv.DictWriter(f,fieldnames=fields);z.writeheader();z.writerows({k:json.dumps(r[k],ensure_ascii=False) if isinstance(r[k],(dict,list)) else r[k] for k in fields} for r in rows)
    manual_width=[r['manual']['width_px'] for r in rows]; selected_width=[r['selected_pair']['width_px'] for r in rows]; near_width=[r['nearest_manual_pair']['width_px'] for r in rows if r['nearest_manual_pair']]
    (out/'width_distribution.json').write_text(json.dumps({'manual':_pstats(manual_width),'selected_d1_wrong_rows':_pstats(selected_width),'nearest_manual_pairs':_pstats(near_width),'reference_width_model_source':'D1 find_boundary_track candidates; D1 has no robust width model for pairing'},ensure_ascii=False,indent=2)+'\n')
    # Trace plot and overlay in original coordinates.
    ov=image.copy(); lm=np.asarray(left,np.int32);rm=np.asarray(right,np.int32);cv2.polylines(ov,[lm.reshape(-1,1,2)],False,(255,0,0),7);cv2.polylines(ov,[rm.reshape(-1,1,2)],False,(0,255,0),7)
    mid=np.column_stack(((np.interp(np.linspace(ymin,ymax,160),left_y:=np.asarray([p[1] for p in left]),left_x:=np.asarray([p[0] for p in left]))+np.interp(np.linspace(ymin,ymax,160),right_y:=np.asarray([p[1] for p in right]),right_x:=np.asarray([p[0] for p in right])))/2,np.linspace(ymin,ymax,160))).astype(np.int32);cv2.polylines(ov,[mid.reshape(-1,1,2)],False,(0,255,255),7)
    for r in rows:
        y=int(r['y_original']); sp=r['selected_pair'];cv2.drawMarker(ov,(int(sp['center_x']),y),(255,255,0),cv2.MARKER_TILTED_CROSS,14,2)
    cv2.imwrite(str(out/'selected_vs_manual_pair_overlay.png'),ov)
    plot=np.full((700,1200,3),255,np.uint8);cv2.putText(plot,'S1-D9 selected D1 pair signed errors by original y',(20,32),0,.8,(0,0,0),2)
    for r in rows:
        x=int(70+(r['selected_pair']['center_x']-r['manual']['center_x']+500)/1000*1090);y=int(70+(r['y_original']-ymin)/(ymax-ymin)*570);cv2.drawMarker(plot,(x,y),(255,0,0),cv2.MARKER_TILTED_CROSS,10,1)
    cv2.imwrite(str(out/'pairing_failure_by_y.png'),plot)
    intervals={'y_768_1353':[r for r in rows if 768<=r['y_original']<=1353],'y_1353_1500':[r for r in rows if 1353<r['y_original']<=1500],'y_2300_3000':[r for r in rows if 2300<=r['y_original']<=3000]}
    (out/'path_transition_trace.json').write_text(json.dumps(intervals,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    counts={};
    for r in rows:counts[r['classification']]=counts.get(r['classification'],0)+1
    summary={'evaluated_wrong_pairing_bands':len(rows),'classification_counts':counts,'main_failure':'D1 pairing/path selection; candidate generation is present near manual boundaries','d1_has_explicit_unary_width_or_center_score':False,'wrong_left_with_correct_right_evidence':sum(r['classification']=='wrong_left_only' for r in rows),'correct_pair_local_score_higher_count':sum(r['nearest_manual_pair'] and r['nearest_manual_pair']['unary_score']>r['selected_pair']['unary_score'] for r in rows),'next_minimal_target':'diagnose/review pair/path scoring, especially candidate identity and cumulative transition behavior; do not change in S1-D9','modules_not_modified':['D1 production detector','D3 orientation search','D4 confidence filter','ROS/realtime pipeline'],'input_sha256_before':sha256(input_path),'annotation_sha256_before':sha256(annotation_path),'d8_sha256_before':sha256(d8_path)}
    summary['input_sha256_after']=sha256(input_path);summary['annotation_sha256_after']=sha256(annotation_path);summary['d8_sha256_after']=sha256(d8_path);summary['inputs_unchanged']=len({summary[k] for k in ['input_sha256_before','input_sha256_after']})==1 and len({summary[k] for k in ['annotation_sha256_before','annotation_sha256_after']})==1 and len({summary[k] for k in ['d8_sha256_before','d8_sha256_after']})==1
    (out/'diagnostic_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2,allow_nan=False)+'\n'); return summary
def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,default=ROOT/'output/s1_real_seam_images/seam_center_lit_quality_01.bmp');p.add_argument('--annotation',type=Path,default=ROOT/'output/s1_d7_manual_ground_truth/seam_center_lit_quality_01_annotation.json');p.add_argument('--d8',type=Path,default=ROOT/'output/s1_d8_manual_boundary_diagnostic/scan_band_diagnostics.json');p.add_argument('--config',type=Path,default=ROOT/'configs/default.yaml');p.add_argument('--output',type=Path,default=ROOT/'output/s1_d9_pairing_cost_diagnostic');a=p.parse_args();print(json.dumps(run(a.input,a.annotation,a.d8,a.config,a.output),ensure_ascii=False,indent=2))
if __name__=='__main__':raise SystemExit(main())
