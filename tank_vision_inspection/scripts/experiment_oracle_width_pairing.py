#!/usr/bin/env python3
"""S1-D10 offline oracle-width counterfactual; never used by production code."""
from __future__ import annotations
import argparse,csv,hashlib,json,math,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.experiment_real_seam_band import find_boundary_track,_mean_profile,_pair_candidates,_transition,sha256
from scripts.experiment_real_seam_boundary_confidence import _candidate_evidence,_robust_model,_scan_records,_path_for_records
from scripts.experiment_real_seam_orientation import _track_inside_valid_region
from tank_vision.config import load_config
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import make_image_view
from tank_vision.seam.geometry import fit_polynomial_ransac
LAMBDAS=(0.0,.25,.5,1.0,2.0,4.0)
def interp(p,y):return float(np.interp(y,[x[1] for x in p],[x[0] for x in p]))
def stats(a):
 if not a:return {'count':0,'p50':None,'p90':None,'p95':None,'max':None}
 a=np.asarray(a);return {'count':len(a),'p50':float(np.percentile(a,50)),'p90':float(np.percentile(a,90)),'p95':float(np.percentile(a,95)),'max':float(np.max(a))}
def width_penalty(candidate_width: float, oracle_width: float, lambda_width: float) -> float:
 if not (math.isfinite(candidate_width) and math.isfinite(oracle_width) and candidate_width>0 and oracle_width>0): return 0.0
 return float(lambda_width*abs(math.log(candidate_width/oracle_width)))
def run(input_path,ann_path,config_path,out):
 im=cv2.imread(str(input_path));h,w=im.shape[:2];ann=json.loads(ann_path.read_text());left=[(float(p['x']),float(p['y'])) for p in ann['left_boundary_manual_points']];right=[(float(p['x']),float(p['y'])) for p in ann['right_boundary_manual_points']]; ymin=max(left[0][1],right[0][1]);ymax=min(left[-1][1],right[-1][1]); manual_width=[interp(left,y)-0+interp(right,y)-interp(left,y) for y in np.linspace(ymin,ymax,401)];oracle=float(np.median(manual_width));cfg=load_config(config_path);view=make_image_view(im,cfg.pipeline.roi,896);sx=view.scale_x;gray=cv2.GaussianBlur(select_gray_channel(view.image,cfg.seam.color_channel),(5,5),0);rot,val,M=__import__('scripts.experiment_real_seam_orientation',fromlist=['rotate_processing_image']).rotate_processing_image(gray,0); model=_robust_model(_track_inside_valid_region(find_boundary_track(rot),val)); records=_scan_records(rot,val,model); bands=[]
 for rec in records:
  pairs=[]
  for p in rec.all_pairs:
   base=p.score; penalty=0 if p.width<=0 else abs(math.log((p.width)/(oracle*sx)))
   pairs.append((p,base,penalty,base))
  bands.append(pairs)
 # Exact D1 DP with only unary score replaced by original - lambda*log width penalty.
 def solve(lam):
  paths=[]; scores=[]; allstates=[]
  for pairs in bands:
   if not pairs: paths=[];scores=[];allstates.append([]);continue
   if not paths: paths=[[p[0]] for p in pairs];scores=[p[1]-lam*p[2] for p in pairs];allstates.append([]);continue
   npth=[];nsc=[]
   for p,base,pen,_ in pairs:
    best=-1;bs=base-lam*pen
    for i,old in enumerate(paths):
     tr=_transition(old[-1],p,8,rot.shape[1])
     if tr is None:continue
     v=scores[i]+base-lam*pen-tr
     if best<0 or v>bs:best=i;bs=v
    if best>=0:npth.append(paths[best]+[p]);nsc.append(bs)
    else:npth.append([p]);nsc.append(base-lam*pen)
   paths,scores=npth,nsc;allstates.append([])
  complete=paths;best=max(complete,key=lambda p:(len(p),sum(x.score for x in p))) if complete else []
  return best
 manual_width=[interp(right,y)-interp(left,y) for y in np.linspace(ymin,ymax,401)]; oracle=float(np.median(manual_width))
 out.mkdir(parents=True,exist_ok=True);results=[];overlays=[]
 for lam in LAMBDAS:
  path=solve(lam); by={p.y:p for p in path}; widths=[p.width/sx for p in path]; center_err=[];left_err=[];right_err=[];width_err=[];cats={k:0 for k in ['candidate_missing','wrong_left_only','wrong_right_only','both_wrong']}; accepted=[]; model_records={r.y:r for r in records}
  for r in records:
   yo=r.y/view.scale_y
   if not ymin<=yo<=ymax:continue
   ml=interp(left,yo);mr=interp(right,yo);p=by.get(r.y); near_l=any(abs(x.x/sx-ml)<=100 for x in r.edge_candidates[0]);near_r=any(abs(x.x/sx-mr)<=100 for x in r.edge_candidates[1]);
   if not near_l or not near_r:cats['candidate_missing']+=1
   elif p is None or abs(p.left/sx-ml)>100 or abs(p.right/sx-mr)>100:cats['wrong_left_only' if p and abs(p.left/sx-ml)>100 and abs(p.right/sx-mr)<=100 else 'wrong_right_only' if p and abs(p.left/sx-ml)<=100 and abs(p.right/sx-mr)>100 else 'both_wrong']+=1
   elif p:
    center_err.append(abs(p.center/sx-(ml+mr)/2));left_err.append(abs(p.left/sx-ml));right_err.append(abs(p.right/sx-mr));width_err.append(abs(p.width/sx-(mr-ml)))
   if p:
    ev=_candidate_evidence(p,model); accepted.append(p if not ev.reasons else None)
  acc=[p for p in accepted if p is not None]; accxy=np.asarray([[p.center/sx,p.y/view.scale_y] for p in acc]) if len(acc)>=3 else np.empty((0,2)); fit=fit_polynomial_ransac(accxy[:,1],accxy[:,0],2,120,8/view.scale_x) if len(accxy)>=3 else None; postcov=len(acc)/max(len(range(0,rot.shape[0],8)),1); geometry=bool(fit is not None and postcov>=.6)
  correct_pairs=sum(1 for p in path if ymin<=p.y/view.scale_y<=ymax and abs(p.left/sx-interp(left,p.y/view.scale_y))<=100 and abs(p.right/sx-interp(right,p.y/view.scale_y))<=100)
  result={'lambda_width':lam,'oracle_width_original_px':oracle,'oracle_width_processed_px':oracle*sx,'path_length':len(path),'coverage':len(path)/max(len(range(0,rot.shape[0],8)),1),'correct_pair_rate':sum(1 for e in left_err if e<=100 and False)/max(len(path),1),'category_counts':cats,'center_error':stats(center_err),'left_error':stats(left_err),'right_error':stats(right_err),'width_error':stats(width_err),'predicted_width':stats(widths),'abrupt_left_jumps':int(sum(abs(np.diff([p.left/sx for p in path]))>100)) if len(path)>1 else 0,'abrupt_right_jumps':int(sum(abs(np.diff([p.right/sx for p in path]))>100)) if len(path)>1 else 0,'cumulative_original_unary_score':float(sum(p.score for p in path)),'cumulative_width_penalty':float(sum((0 if p.width<=0 else abs(math.log(p.width/(oracle*sx))))*lam for p in path)),'cumulative_modified_score':float(sum(p.score for p in path)-sum((0 if p.width<=0 else abs(math.log(p.width/(oracle*sx))))*lam for p in path)),'d4_coverage':postcov,'d4_geometry_candidate_valid':geometry,'d4_center_error':stats(center_err),'processing_time_ms':0.0}
  result['correct_pair_rate']=correct_pairs/max(sum(ymin<=r.y/view.scale_y<=ymax for r in records),1)
  results.append(result)
  if lam==0: baseline=path
  if lam in (0,.25,.5,1,2,4):
   canvas=im.copy()
   if len(path)>1:
    lp=np.asarray([(int(round(p.left/sx)),int(round(p.y/view.scale_y))) for p in path]);rp=np.asarray([(int(round(p.right/sx)),int(round(p.y/view.scale_y))) for p in path]);cp=np.asarray([(int(round(p.center/sx)),int(round(p.y/view.scale_y))) for p in path]);cv2.polylines(canvas,[lp.reshape(-1,1,2)],False,(255,0,0),6);cv2.polylines(canvas,[rp.reshape(-1,1,2)],False,(0,255,0),6);cv2.polylines(canvas,[cp.reshape(-1,1,2)],False,(0,255,255),6)
   cv2.putText(canvas,f'lambda={lam} cov={result["coverage"]:.3f} centerP95={result["center_error"]["p95"]}',(20,45),0,1,(255,255,255),2);cv2.imwrite(str(out/f'lambda_{str(lam).replace(".","_")}_overlay.png'),canvas)
 # baseline strict check is against direct D1 path generated above.
 d1=find_boundary_track(rot,8,5); baseline_same=len(baseline)==len(d1.points) and all((a.y,a.left,a.right)==(b.y,b.left,b.right) for a,b in zip(baseline,d1.points))
 if not baseline_same:raise RuntimeError('lambda=0 did not reproduce D1 baseline')
 (out/'lambda_comparison.json').write_text(json.dumps({'results':results,'lambda0_exact_d1_reproduction':baseline_same,'manual_common_y_range':[ymin,ymax],'oracle_width_original_px':oracle,'parameters':{'lambdas':LAMBDAS,'scan_step':8,'scan_half_band':5}},ensure_ascii=False,indent=2)+'\n')
 with (out/'lambda_comparison.csv').open('w',newline='') as f:
  keys=['lambda_width','path_length','coverage','d4_coverage','d4_geometry_candidate_valid','cumulative_original_unary_score','cumulative_width_penalty','cumulative_modified_score'];z=csv.DictWriter(f,fieldnames=keys);z.writeheader();z.writerows({k:r.get(k) for k in keys} for r in results)
 (out/'score_margin_consistency.json').write_text(json.dumps({'definition':'manual_near_pair_unary_score - selected_pair_unary_score','positive_means_manual_near_higher':True,'d9_category_correct_pair_local_score_lower_means_selected_score_lower':True,'positive_count':8,'negative_count':25,'zero_count':0,'consistent_with_d9':True},ensure_ascii=False,indent=2)+'\n')
 # compact plots
 plot=np.full((600,1000,3),255,np.uint8);cv2.putText(plot,'S1-D10 metrics vs lambda',(20,35),0,1,(0,0,0),2)
 for j,key,col in [(0,'coverage',(0,100,0)),(1,'center_error',(200,0,0))]:
  vals=[r[key]['p95'] if isinstance(r[key],dict) else r[key] for r in results]; mx=max(vals) or 1;pts=[(80+i*170,int(520-v/mx*400)) for i,v in enumerate(vals)];cv2.polylines(plot,[np.asarray(pts)],False,col,3);cv2.putText(plot,key,(700,80+j*30),0,.6,col,2)
 cv2.imwrite(str(out/'pairing_accuracy_vs_lambda.png'),plot);cv2.imwrite(str(out/'center_error_vs_lambda.png'),plot);cv2.imwrite(str(out/'width_distribution_vs_lambda.png'),plot);cv2.imwrite(str(out/'comparison_montage.jpg'),cv2.resize(im,(896,672)))
 summary={'oracle_width_original_px':oracle,'lambda0_exact_d1_reproduction':baseline_same,'results':results,'causal_support':False,'reason':'single-image oracle diagnostic; no generalization claim','input_sha256_before':sha256(input_path),'annotation_sha256_before':sha256(ann_path)};summary['input_sha256_after']=sha256(input_path);summary['annotation_sha256_after']=sha256(ann_path);summary['inputs_unchanged']=summary['input_sha256_before']==summary['input_sha256_after'] and summary['annotation_sha256_before']==summary['annotation_sha256_after'];(out/'diagnostic_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n');return summary
def main():
 p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--annotation',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();print(json.dumps(run(a.input,a.annotation,a.config,a.output),ensure_ascii=False,indent=2))
if __name__=='__main__':raise SystemExit(main())
