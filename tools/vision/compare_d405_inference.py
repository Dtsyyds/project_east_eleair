#!/usr/bin/env python3
"""Compare one inference-only change against the 379 recorded CPU frames."""
import sys,json,time,hashlib
from pathlib import Path
import cv2,numpy as np,torch,yaml
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/d405_seam_tracking/src'))
from d405_seam_tracking.round1_a import Round1AVisual
from d405_seam_tracking.inference_backend import CPUInference
from d405_seam_tracking.learning import preprocess
from d405_seam_tracking.geometry import roi_bounds
from d405_seam_tracking.center_2d import decode
base=ROOT/'.runtime/s100p/analysis/d405_realtime_replay_20261103/20261103_201559_bdae0c'
out=ROOT/'.runtime/s100p/analysis/d405_inference_optimization_20261103';out.mkdir(exist_ok=True)
rows=[json.loads(l) for l in (base/'processed.jsonl').read_text().splitlines()];assert len(rows)==379
cv2.setNumThreads(2);torch.set_num_threads(2);torch.use_deterministic_algorithms(True)
cfg=yaml.safe_load((ROOT/'config/d405/round1_a.yaml').read_text());v=Round1AVisual(cfg);backend=CPUInference(v.model,'cpu_channels_last');bounds=roi_bounds((480,640,3),cfg)
for _ in range(8):
 x,_=preprocess(np.zeros((480,640,3),np.uint8),v.size,rgb=True);backend.probability(x)
results=[]
for i,r in enumerate(rows):
 p=ROOT/'.runtime/s100p/analysis/d405_video_ab_20261102/audit'/r['session']/'rgb'/f"{r['frame']:06d}.png"
 rgb=cv2.cvtColor(cv2.imread(str(p)),cv2.COLOR_BGR2RGB);assert hashlib.sha256(rgb.tobytes()).hexdigest()==r['rgb_sha256']
 x,_=preprocess(rgb,v.size,rgb=True);t=time.perf_counter_ns();prob=backend.probability(x);elapsed=(time.perf_counter_ns()-t)/1e6
 result=decode(cv2.resize(prob,(640,480),interpolation=cv2.INTER_LINEAR),cfg,bounds,enabled=True)
 a=np.asarray(result['center_points']).reshape(-1,2);b=np.asarray(r['center_points']).reshape(-1,2)
 delta=float(np.max(np.abs(a-b))) if a.size and a.shape==b.shape else (0. if a.shape==b.shape else None)
 same_state=result['kind']==r['kind'] and result['A']['valid']==r['A_valid'] and ';'.join(result['A']['reasons'])==r['A_reason']
 results.append(dict(session=r['session'],frame=r['frame'],stamp_ns=r['stamp_ns'],cpu_kind=r['kind'],optimized_kind=result['kind'],status_match=same_state,center_delta_px=delta,inference_ms=elapsed))
 if i%100==0:print(i+1,'/',len(rows),'infer_ms',round(elapsed,2),'state_match',same_state,'delta',delta,flush=True)
(out/'same_frame_379.json').write_text(json.dumps(results,indent=2))
summary={'frames':len(results),'status_matches':sum(r['status_match'] for r in results),'max_center_delta_px':max((r['center_delta_px'] or 0 for r in results)), 'geometry_shape_mismatches':sum(r['center_delta_px'] is None for r in results),'inference_p50_ms':float(np.percentile([r['inference_ms'] for r in results],50)),'inference_p95_ms':float(np.percentile([r['inference_ms'] for r in results],95)),'different_rows':[r for r in results if not r['status_match'] or r['center_delta_px'] is None or r['center_delta_px']>1e-6]}
(out/'same_frame_summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
