"""Compare candidate outputs on saved original-timestamp samples; no motion/ROS."""
import argparse
import copy
from pathlib import Path
from collections import Counter
import numpy as np
from d405_seam_tracking.candidate_pipeline import CandidatePipeline
from d405_seam_tracking.contracts import load_config, dump
from d405_seam_tracking.dataset import load_index, read_sample
from d405_seam_tracking.tracking_error import ErrorReference, tracking_error


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=Path('config/d405/round1_a.yaml'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): raise FileExistsError(args.output)
    root,index=load_index(args.index)
    pipe=CandidatePipeline(load_config(args.config));counts=Counter();rows=[];timings=[]
    try:
        for record in index['samples']:
            frame=read_sample(root,record)
            result=pipe.process(frame)
            decision=pipe.frame_decision
            before_valid=decision['A']['valid'] or decision['kind']=='CENTER_2D'
            baseline=copy.deepcopy(result)
            if decision['A']['valid']:
                baseline.update(state='VISION_2D',valid_2d=True,points_2d_y_left_right=decision['A']['points'])
                assert result['points_2d_y_left_right']==decision['A']['points']
            elif decision['kind']=='CENTER_2D':
                assert result['points_2d_y_center']==decision['center_points']
            else:
                baseline.update(state='LOST',valid_2d=False)
            before_error=tracking_error(baseline,frame.image.shape[1],frame.image.shape[0],ErrorReference())[0]['valid']
            after_error=tracking_error(result,frame.image.shape[1],frame.image.shape[0],ErrorReference())[0]['valid']
            counts.update(frames=1,before_valid=int(before_valid),after_valid=int(result['valid_2d']),
                          before_error_valid=int(before_error),after_error_valid=int(after_error),
                          single_boundary=int(result['measurement_kind']=='SINGLE_BOUNDARY_2D'))
            timings.append(result['timing_ms']['total'])
            rows.append(dict(sample=record['sample'],stamp_ns=str(frame.image_stamp_ns),
                             before_valid=bool(before_valid),after_valid=result['valid_2d'],
                             measurement_kind=result['measurement_kind'],reasons=result['reasons']))
        report=dict(counts=counts,model_sha256=pipe.visual.model_sha256,frames=rows,
                    total_ms_p50_p95=np.percentile(timings,[50,95]).tolist(),
                    limitations='Sparse saved samples are a regression check, not continuous-driving or independent accuracy validation.')
        args.output.parent.mkdir(parents=True,exist_ok=True);dump(args.output,report)
        print(dict(counts));print('Saved:',args.output)
    finally: pipe.close()


if __name__=='__main__': main()
