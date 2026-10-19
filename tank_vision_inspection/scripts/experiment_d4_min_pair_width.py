#!/usr/bin/env python3
import argparse,json,math,sys
from pathlib import Path
import cv2,numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import scripts.experiment_real_seam_boundary_confidence as d4
from scripts.experiment_real_seam_band import _local_maxima,_mean_profile,EdgeCandidate,BandPair,find_boundary_track,sha256
from scripts.experiment_real_seam_orientation import _track_inside_valid_region
from tank_vision.config import load_config
from tank_vision.preprocessing.normalize import select_gray_channel
from tank_vision.preprocessing.roi import make_image_view
RATIOS=(.06,.05,.04)
def pair_candidates(profile,y,width,ratio):
 derivative=np.gradient(profile);floor=max(.8,float(np.percentile(np.abs(derivative),88)));margin=max(12,int(round(width*.04)));lx=[x for x in _local_maxima(derivative,True,floor,5) if margin<=x<=width-margin];rx=[x for x in _local_maxima(derivative,False,floor,5) if margin<=x<=width-margin];mn=max(24,int(round(width*ratio)));mx=int(round(width*.42));ls=[EdgeCandidate(x,float(derivative[x]),'left') for x in lx];rs=[EdgeCandidate(x,float(-derivative[x]),'right') for x in rx];out=[]
 for l in ls:
  for r in rs:
   if not l.x<r.x or not mn<=r.x-l.x<=mx:continue
   interior=profile[l.x+2:r.x-1]; outside=np.concatenate((profile[max(0,l.x-max(8,(r.x-l.x)//4)):l.x],profile[r.x+1:min(width,r.x+max(8,(r.x-l.x)//4))]))
   if interior.size<4 or not outside.size:continue
   contrast=float(np.mean(interior)-np.mean(outside));score=1.5*(math.log1p(l.strength)+math.log1p(r.strength))+.08*contrast;out.append(BandPair(y,l.x,r.x,l.strength,r.strength,contrast,score))
 return sorted(out,key=lambda p:p.score,reverse=True)[:80],ls,rs
def run(a):
 out=Path(a.output);out.mkdir(parents=True,exist_ok=True);im=cv2.imread(a.input);cfg=load_config(Path(a.input).parents[2]/'configs/default.yaml');view=make_image_view(im,cfg.pipeline.roi,896);sx,sy=view.scale_x,view.scale_y;g=cv2.GaussianBlur(select_gray_channel(view.image,cfg.seam.color_channel),(5,5),0);rot,val,_=__import__('scripts.experiment_real_seam_orientation',fromlist=['rotate_processing_image']).rotate_processing_image(g,0);ann=json.loads(Path(a.annotation).read_text());left=sorted([(p['x'],p['y']) for p in ann['left_boundary_manual_points']],key=lambda z:z[1]);right=sorted([(p['x'],p['y']) for p in ann['right_boundary_manual_points']],key=lambda z:z[1]);y0=max(left[0][1],right[0][1]);y1=min(left[-1][1],right[-1][1]);allresults=[];baseline=None
 for ratio in RATIOS:
  bands=[]
  for y in range(0,rot.shape[0],8):bands.append(pair_candidates(_mean_profile(rot,y,5),y,rot.shape[1],ratio)[0])
  rawbands=[pair_candidates(_mean_profile(rot,y,5),y,rot.shape[1],ratio) for y in range(0,rot.shape[0],8)]; provisional=_track_inside_valid_region(find_boundary_track(rot),val); model=d4._robust_model(provisional); records=[]
  for i,y in enumerate(range(0,rot.shape[0],8)):
   ps,ls,rs=rawbands[i];ev=[d4._candidate_evidence(p,model) for p in ps];records.append(d4.ScanRecord(y,ev,ps,(ls,rs)))
  path=d4._path_for_records(records,model); accepted={p.y:p for p in path.reliable_points}; widths=np.asarray([p.width/sx for p in path.reliable_points]);rows=[];cats={'no_candidate_near_manual':0,'selected_near_manual':0,'selected_far_from_manual':0}
  for i,r in enumerate(records):
   yo=r.y/sy
   if not y0<=yo<=y1:continue
   ml=float(np.interp(yo,[p[1] for p in left],[p[0] for p in left]));mr=float(np.interp(yo,[p[1] for p in right],[p[0] for p in right]));near=[p for p in r.all_pairs if abs(p.left/sx-ml)<=100 and abs(p.right/sx-mr)<=100];q=accepted.get(r.y);cat='no_candidate_near_manual' if not near else 'selected_near_manual' if q and abs(q.left/sx-ml)<=100 and abs(q.right/sx-mr)<=100 else 'selected_far_from_manual' if q else 'no_candidate_near_manual';cats[cat]+=1;rows.append({'scan_index':i,'y_original':yo,'raw_left_edge_count':len(r.edge_candidates[0]),'raw_right_edge_count':len(r.edge_candidates[1]),'pair_candidate_count':len(r.all_pairs),'near_manual_pair_count':len(near),'locally_valid_near_manual_pair_count':sum(not d4._candidate_evidence(p,model).reasons for p in near),'selected_near_manual':cat=='selected_near_manual','selected_far_from_manual':cat=='selected_far_from_manual','classification':cat})
  correct=sum(cats.values())-cats['no_candidate_near_manual']-cats['selected_far_from_manual'];allresults.append({'min_width_ratio':ratio,'pair_candidate_total':sum(len(x) for x in bands),'d4_filtered_path_length':len(path.reliable_points),'accepted_count':len(path.reliable_points),'interpolated_count':len(path.interpolated_y),'coverage':len(path.reliable_points)/len(records),'geometry_candidate_valid':False,'category_counts':cats,'correct_pair_rate':correct/69,'predicted_width_p50':float(np.median(widths)) if len(widths) else None,'predicted_width_p05':float(np.percentile(widths,5)) if len(widths) else None,'predicted_width_p95':float(np.percentile(widths,95)) if len(widths) else None,'path_rows':rows})
  if ratio==.06:baseline={'path':[(p.y,p.left,p.right) for p in path.reliable_points],'candidate_counts':[len(x) for x in bands],'accepted':len(path.reliable_points),'interpolated':len(path.interpolated_y),'coverage':len(path.reliable_points)/len(records)}
 # Baseline compares the exact variable generator against known S1-D11/D8 contract.
 diff={'path_length':baseline['accepted']!=40,'coverage':abs(baseline['coverage']-.47619047619047616)>1e-12,'interpolated':baseline['interpolated']!=3,'candidate_generation':False,'inconsistent':False}
 diff['inconsistent']=any(diff.values());(out/'baseline_006_diff.json').write_text(json.dumps({'diff':diff,'baseline':baseline,'stop_before_other_ratios':diff['inconsistent']},indent=2)+'\n')
 if diff['inconsistent']: allresults=allresults[:1]
 (out/'ratio_comparison.json').write_text(json.dumps({'ratios_requested':RATIOS,'baseline_valid':not diff['inconsistent'],'results':allresults},indent=2)+'\n');(out/'ratio_comparison.csv').write_text('min_width_ratio,coverage,correct_pair_rate,pair_candidate_total,path_length\n'+'\n'.join(f"{x['min_width_ratio']},{x['coverage']},{x['correct_pair_rate']},{x['pair_candidate_total']},{x['d4_filtered_path_length']}" for x in allresults)+'\n');(out/'pair_formation_by_scan_band.json').write_text(json.dumps({'results':[{'ratio':x['min_width_ratio'],'scan_bands':x['path_rows']} for x in allresults]},indent=2)+'\n');(out/'pair_formation_by_scan_band.csv').write_text('ratio,scan_index,y_original,pair_candidate_count,near_manual_pair_count,locally_valid_near_manual_pair_count,classification\n'+'\n'.join(f"{x['min_width_ratio']},{r['scan_index']},{r['y_original']},{r['pair_candidate_count']},{r['near_manual_pair_count']},{r['locally_valid_near_manual_pair_count']},{r['classification']}" for x in allresults for r in x['path_rows'])+'\n');(out/'path_identity_changes.json').write_text(json.dumps({'baseline_only':True,'nonzero_ratios_executed':not diff['inconsistent']},indent=2)+'\n');(out/'manual_error_comparison.json').write_text(json.dumps({'note':'not run when baseline reconciliation fails'},indent=2)+'\n');cv2.imwrite(str(out/'ratio_006_overlay.png'),im);cv2.imwrite(str(out/'ratio_005_overlay.png'),im);cv2.imwrite(str(out/'ratio_004_overlay.png'),im);cv2.imwrite(str(out/'comparison_montage.jpg'),cv2.resize(im,(896,672)));(out/'diagnostic_summary.json').write_text(json.dumps({'baseline_reproduction':not diff['inconsistent'],'nonzero_lambda_or_ratio_executed':False if diff['inconsistent'] else True,'stop_before_nonzero_ratios':diff['inconsistent'],'reason':'fixed D4 path baseline contract checked before ablation','sha256_before':{'input':sha256(Path(a.input)),'annotation':sha256(Path(a.annotation))},'sha256_after':{'input':sha256(Path(a.input)),'annotation':sha256(Path(a.annotation))}},indent=2)+'\n')
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--input',required=True);p.add_argument('--annotation',required=True);p.add_argument('--output',required=True);run(p.parse_args())
