#!/usr/bin/env python3
"""Export frozen RGB fold_1 and validate float ONNX before any quantization.
Uses isolated local wheel dependencies; does not edit the production environment.
"""
import sys,json,csv,hashlib,argparse,platform,subprocess,ctypes
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'.runtime/s100p/onnx_export_deps'),str(ROOT/'packages/d405_seam_tracking/src')]
import numpy as np,cv2,torch,yaml,onnx,onnxruntime as ort
from d405_seam_tracking.round1_a import Round1AVisual
from d405_seam_tracking.learning import preprocess
from d405_seam_tracking.geometry import roi_bounds
from d405_seam_tracking.center_2d import decode
EXPECTED='15dbdde88dc26eba86adda212a146786b624a23bd57929f0bfb1a0152ca4327f'
ANA=ROOT/'.runtime/s100p/analysis';SESS=['WS-20261102_214559-d94ac71b','WS-20261102_214902-f634c958'];ROI_SESSION='VISION-20261103_163515-50ef375a'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,default=ANA/'d405_nash_m_first_hbm_20261103');ap.add_argument('--no-fold',action='store_true');ap.add_argument('--ort-disable-optimizations',action='store_true');ap.add_argument('--diagnostic-continue',action='store_true');ap.add_argument('--existing-onnx',type=Path);ap.add_argument('--probability-atol',type=float,default=1e-5);ap.add_argument('--geometry-atol',type=float,default=.01);args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
 model_file=out/'fold_1_rgb_float.onnx'
 if model_file.exists():raise FileExistsError(model_file)
 cfg=yaml.safe_load((ROOT/'config/d405/round1_a.yaml').read_text());assert cfg['checkpoint_sha256']==EXPECTED
 weight=ROOT/cfg['checkpoint'];assert sha(weight)==EXPECTED
 cv2.setNumThreads(2);torch.set_num_threads(2);torch.use_deterministic_algorithms(True)
 visual=Round1AVisual(cfg);bounds=roi_bounds((480,640,3),cfg);assert bounds==(128,168,512,456)
 frames=[]
 for session in SESS:
  aud=ANA/'d405_video_ab_20261102/audit'/session;src=json.loads((aud/'frames.json').read_text())
  src=sorted([r for r in src if session==SESS[1] or 1090<=int(r['frame'])<=1495],key=lambda r:int(r['stamp_ns']))
  for r in src:frames.append({'session':session,'frame':int(r['frame']),'stamp_ns':str(r['stamp_ns']),'rgb_sha256':r['rgb_sha256'],'path':str(aud/'rgb'/f"{int(r['frame']):06d}.png"),'format':'png'})
 index=json.loads((ANA/'d405_session_review_20261103/annotation_index.json').read_text())
 for r in index['samples']:
  if int(r['source_frame']) in [507,610,785]:frames.append({'session':ROI_SESSION,'frame':int(r['source_frame']),'stamp_ns':str(r['source_timestamp_ns']),'rgb_sha256':r['source_rgb_sha256'],'path':str(ANA/'d405_session_review_20261103'/r['sample']),'format':'npz'})
 assert len(frames)==912
 def read(r):
  if r['format']=='png':rgb=cv2.cvtColor(cv2.imread(r['path']),cv2.COLOR_BGR2RGB)
  else:
   with np.load(r['path']) as z:rgb=z['image'].copy()
  assert hashlib.sha256(rgb.tobytes()).hexdigest()==r['rgb_sha256']
  return rgb
 # Fix validation tolerances and session grouping BEFORE inspecting ONNX results.
 calibration_candidates=[f for f in frames if f['session']==SESS[0] and f['frame']!=1284]
 calibration=[calibration_candidates[i] for i in np.linspace(0,len(calibration_candidates)-1,64,dtype=int)]
 acceptance=[f for f in frames if f['session'] in [SESS[1],ROI_SESSION]]
 (out/'protocol.json').write_text(json.dumps({'constant_folding':not args.no_fold,'ort_graph_optimizations':not args.ort_disable_optimizations,'checkpoint_sha256':EXPECTED,'input_shape':[1,3,192,256],'input_dtype':'float32','probability_max_abs_tolerance':args.probability_atol,'state_mismatches_allowed':0,'geometry_shape_mismatches_allowed':0,'geometry_max_abs_tolerance_px':args.geometry_atol,'calibration_session':SESS[0],'acceptance_sessions':[SESS[1],ROI_SESSION],'calibration_frames':calibration,'acceptance_frames':acceptance,'frame_1284_role':'diagnostic regression; from calibration session, not independent acceptance','no_training':True,'quantization_authorized_only_after_float_pass':True},ensure_ascii=False,indent=2))
 x,_=preprocess(read(frames[0]),visual.size,rgb=True)
 if args.existing_onnx:
  import shutil
  shutil.copyfile(args.existing_onnx,model_file)
 else:
  torch.onnx.export(visual.model,torch.from_numpy(x),str(model_file),opset_version=13,input_names=['rgb_imagenet_nchw'],output_names=['seam_logits'],do_constant_folding=not args.no_fold)
 model=onnx.load(str(model_file));onnx.checker.check_model(model)
 opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
 if args.ort_disable_optimizations:opts.graph_optimization_level=ort.GraphOptimizationLevel.ORT_DISABLE_ALL
 session=ort.InferenceSession(str(model_file),sess_options=opts,providers=['CPUExecutionProvider'])
 details=[]
 for i,f in enumerate(frames):
  rgb=read(f);x,_=preprocess(rgb,visual.size,rgb=True)
  with torch.inference_mode():pt_logits=visual.model(torch.from_numpy(x)).numpy()
  ox_logits=session.run(['seam_logits'],{'rgb_imagenet_nchw':x})[0]
  # Same single sigmoid and same float32 bilinear output-to-source resize.
  p0=torch.sigmoid(torch.from_numpy(pt_logits)).numpy()[0,0];p1=torch.sigmoid(torch.from_numpy(ox_logits)).numpy()[0,0]
  full0=cv2.resize(p0,(640,480),interpolation=cv2.INTER_LINEAR);full1=cv2.resize(p1,(640,480),interpolation=cv2.INTER_LINEAR)
  a=decode(full0,cfg,bounds,enabled=True);b=decode(full1,cfg,bounds,enabled=True)
  aa=np.array(a['center_points']).reshape(-1,2);bb=np.array(b['center_points']).reshape(-1,2)
  delta=float(np.max(np.abs(aa-bb))) if aa.size and aa.shape==bb.shape else (0. if aa.shape==bb.shape else None)
  la=np.array(a['A'].get('points',[])).reshape(-1,3);lb=np.array(b['A'].get('points',[])).reshape(-1,3)
  ad=float(np.max(np.abs(la-lb))) if la.size and la.shape==lb.shape else (0. if la.shape==lb.shape else None)
  err=float(np.max(np.abs(full0-full1)));match=(a['kind']==b['kind'] and a['A']['reasons']==b['A']['reasons'] and a['A']['valid']==b['A']['valid'])
  row={k:f[k] for k in ['session','frame','stamp_ns','rgb_sha256']};row.update(probability_max_abs=err,probability_mae=float(np.mean(np.abs(full0-full1))),logit_max_abs=float(np.max(np.abs(pt_logits-ox_logits))),pytorch_kind=a['kind'],onnx_kind=b['kind'],status_match=match,center_max_abs_px=delta,A_geometry_max_abs_px=ad,A_valid_before=a['A']['valid'],A_valid_after=b['A']['valid'])
  row['pass']=err<=args.probability_atol and match and delta is not None and delta<=args.geometry_atol and ad is not None and ad<=args.geometry_atol;details.append(row)
  if f['session']==SESS[0] and f['frame']==1284:(out/'frame1284_float_geometry.json').write_text(json.dumps({'source':f,'pytorch':a,'onnx':b},ensure_ascii=False,indent=2))
  if i%100==0:print('float check',i+1,'/',len(frames),'prob_max',err,'state',match,flush=True)
  if not row['pass']:
   if not (out/'first_float_mismatch.npz').exists():
    np.savez_compressed(out/'first_float_mismatch.npz',rgb=rgb,pytorch_probability=full0,onnx_probability=full1)
    (out/'first_mismatch_geometry.json').write_text(json.dumps({'source':f,'pytorch':a,'onnx':b},indent=2))
   if not args.diagnostic_continue:
    print('FLOAT MISMATCH; STOP',json.dumps(row),flush=True);break
 with (out/'float_per_frame.csv').open('w',newline='') as fp:
  w=csv.DictWriter(fp,fieldnames=list(details[0]));w.writeheader();w.writerows(details)
 passed=len(details)==len(frames) and all(r['pass'] for r in details)
 report={'passed':passed,'probability_atol':args.probability_atol,'geometry_atol_px':args.geometry_atol,'frames_exceeding_original_1e5_budget':sum(r['probability_max_abs']>1e-5 for r in details),'frames_checked':len(details),'frames_planned':len(frames),'geometry_shape_mismatches':sum(r['center_max_abs_px'] is None or r['A_geometry_max_abs_px'] is None for r in details),'state_matches':sum(r['status_match'] for r in details),'max_probability_error':max(r['probability_max_abs'] for r in details),'max_center_delta_px':max((r['center_max_abs_px'] or 0 for r in details)),'A_valid_count':sum(r['A_valid_before'] for r in details),'A_valid_regressions':sum(r['A_valid_before'] and not r['A_valid_after'] for r in details),'roi_out_results':[r for r in details if r['session']==ROI_SESSION],'onnx_sha256':sha(model_file),'checkpoint_sha256':EXPECTED,'onnx_version':onnx.__version__,'onnxruntime_version':ort.__version__,'torch_version':torch.__version__,'HBM_compiled':False,'BPU_deployed':False}
 (out/'float_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 if passed:
  caldir=out/'calibration';caldir.mkdir(exist_ok=True);calmanifest=[]
  for i,f in enumerate(calibration):
   x,_=preprocess(read(f),visual.size,rgb=True);p=caldir/f'{i:03d}.npy';np.save(p,x)
   calmanifest.append(dict(f,tensor_file=str(p.relative_to(out)),tensor_sha256=sha(p)))
  (out/'calibration_manifest.json').write_text(json.dumps(calmanifest,ensure_ascii=False,indent=2))
  contract={'target':'nash-m','input_name':'rgb_imagenet_nchw','input_shape':[1,3,192,256],'input_dtype':'float32','logical_layout':'NCHW','color':'RGB','resize':'OpenCV INTER_AREA from 640x480 to 256x192','normalization':'uint8 / 255 then per-channel (x-[0.485,0.456,0.406])/[0.229,0.224,0.225]','output_name':'seam_logits','output_shape':[1,1,192,256],'output_processing':'sigmoid exactly once then OpenCV INTER_LINEAR to original RGB width/height; unchanged A / CENTER_2D decoder','roi':[128,168,512,456],'calibration_input':'already preprocessed float32 .npy; do not apply another normalization or NV12 conversion','checkpoint_sha256':EXPECTED,'onnx_sha256':sha(model_file),'float_validation':'passed; see float_validation.json','BPU_CPU_operator_assignment':'unknown until actual compiler report is available'}
  (out/'input_output_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2))
 print(json.dumps(report,ensure_ascii=False),flush=True)
if __name__=='__main__':main()
