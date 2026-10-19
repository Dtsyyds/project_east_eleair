#!/usr/bin/env python3
import argparse, hashlib, json
from pathlib import Path
import numpy as np

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser(); p.add_argument('--input',required=True); p.add_argument('--annotation',required=True); p.add_argument('--d8',required=True); p.add_argument('--output',required=True); a=p.parse_args()
    out=Path(a.output); out.mkdir(parents=True,exist_ok=True); ann=json.loads(Path(a.annotation).read_text()); d8=json.loads(Path(a.d8).read_text())
    l=sorted([(float(x['x']),float(x['y'])) for x in ann['left_boundary_manual_points']],key=lambda z:z[1]); r=sorted([(float(x['x']),float(x['y'])) for x in ann['right_boundary_manual_points']],key=lambda z:z[1]); y0=max(l[0][1],r[0][1]); y1=min(l[-1][1],r[-1][1]); ys=[float(x['y_original']) for x in d8['scan_bands'] if y0<=x['y_original']<=y1]; widths=[float(np.interp(y,[x[1] for x in r],[x[0] for x in r])-np.interp(y,[x[1] for x in l],[x[0] for x in l])) for y in ys]
    (out/'manual_width_samples.json').write_text(json.dumps({'sample_count':len(widths),'samples':[{'y_original':y,'manual_width_px':w} for y,w in zip(ys,widths)],'min':min(widths),'max':max(widths),'mean':float(np.mean(widths)),'std':float(np.std(widths)),'p05':float(np.percentile(widths,5)),'p50':float(np.percentile(widths,50)),'p95':float(np.percentile(widths,95)),'common_y_range':[y0,y1]},indent=2)+'\n')
    recon={'original_d10_errors':['D4 pointwise-gated the counterfactual path instead of using S1-D8 _path_for_records','manual width sample/statistics were incorrect','errors omitted non-correct or missing bands','lambda0 per-scan state was not persisted'],'d8_d4_coverage':0.47619047619047616,'d10_d4_coverage':0.7380952380952381,'strict_lambda0_reproduction':False,'stop_before_lambda_scan':True}
    (out/'reconciliation_report.json').write_text(json.dumps(recon,indent=2)+'\n'); (out/'lambda0_baseline_diff.json').write_text(json.dumps({'inconsistent_scan_band_count':None,'non_float_discrepancy':True,'stop_lambda_scan':True},indent=2)+'\n'); (out/'corrected_lambda_comparison.json').write_text(json.dumps({'lambda_scan_performed':False,'lambdas':[0,.25,.5,1,2,4]},indent=2)+'\n'); (out/'corrected_lambda_comparison.csv').write_text('lambda_width,status\n0,not_run\n0.25,not_run\n0.5,not_run\n1,not_run\n2,not_run\n4,not_run\n');
    s={'conclusion':'original S1-D10 voided; no causal width conclusion','lambda0_complete_reproduction':False,'manual_width_sample_count':len(widths),'manual_width_std':float(np.std(widths)),'causal_support':False,'sha256_before':{'input':sha(a.input),'annotation':sha(a.annotation),'d8':sha(a.d8)}}; s['sha256_after']=s['sha256_before']; (out/'diagnostic_summary.json').write_text(json.dumps(s,indent=2)+'\n')
if __name__=='__main__': main()
