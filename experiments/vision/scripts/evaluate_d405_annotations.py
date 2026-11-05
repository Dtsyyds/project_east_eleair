#!/usr/bin/env python3
"""Score labelled sparse D405 snapshots without hiding missing predictions.

No temporal/jitter or metric claims. Only common annotated y ranges are scored;
prediction interpolation never bridges a gap larger than max_gap_rows pixels.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from d405_seam_tracking.contracts import dump, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample


def score(frame, annotation, result, max_gap_rows=9):
    _, truth = annotation_mask(annotation, frame.image.shape)
    predicted = bool(result and result.get('valid_2d') and not result.get('predicted_only'))
    row = {'source_stamp_ns': str(frame.image_stamp_ns), 'prediction': predicted,
           'negative': bool(annotation.get('no_seam')), 'annotated_rows': len(truth),
           'covered_rows': 0, 'errors': {}, 'hits': {str(t): {'center': 0, 'both_boundaries': 0} for t in (2, 5, 10)}}
    if not truth or not predicted:
        return row
    target = np.asarray(truth, dtype=float)
    points = np.asarray(result.get('points_2d_y_left_right', []), dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all():
        raise ValueError('invalid_prediction_geometry')
    if np.any(np.diff(points[:, 0]) <= 0) or np.any(points[:, 1] >= points[:, 2]):
        raise ValueError('unordered_or_crossing_prediction')
    ys = target[:, 0]
    upper = np.searchsorted(points[:, 0], ys, side='left')
    safe = np.clip(upper, 0, len(points)-1)
    exact = points[safe, 0] == ys
    lower = np.clip(upper-1, 0, len(points)-1)
    bracket = (upper > 0) & (upper < len(points)) & (points[safe, 0]-points[lower, 0] <= max_gap_rows)
    covered = exact | bracket
    if not covered.any():
        return row
    y = ys[covered]
    left = np.interp(y, points[:, 0], points[:, 1]); right = np.interp(y, points[:, 0], points[:, 2])
    target = target[covered]
    err = {'left': abs(left-target[:, 1]), 'right': abs(right-target[:, 2]),
           'center': abs((left+right-target[:, 1]-target[:, 2])/2),
           'width': abs((right-left)-(target[:, 2]-target[:, 1]))}
    row['covered_rows'] = len(y)
    row['errors'] = {k: {'mean': float(v.mean()), 'p95': float(np.percentile(v,95)),
                         'max': float(v.max()), 'sum': float(v.sum())} for k,v in err.items()}
    row['predicted_width_median'] = float(np.median(right-left))
    row['label_width_median'] = float(np.median(target[:,2]-target[:,1]))
    for t in (2,5,10):
        row['hits'][str(t)] = {'center': int((err['center'] <= t).sum()),
                              'both_boundaries': int(((err['left'] <= t)&(err['right'] <= t)).sum())}
    return row


def evaluate(index_path, result_path):
    root, index = load_index(index_path)
    results = {}
    for line in Path(result_path).read_text().splitlines():
        result = json.loads(line)
        stamp = str(result['source']['image_stamp_ns'])
        if stamp in results:
            raise ValueError('duplicate_result_source_stamp')
        results[stamp] = result
    rows=[]
    for i,rec in enumerate(index['samples'],1):
        frame=read_sample(root,rec)
        if not rec.get('annotation'):
            raise ValueError('all_samples_must_be_annotated')
        row=score(frame,rec['annotation'],results.pop(str(frame.image_stamp_ns),None))
        row.update(id=i,sample=rec['sample'])
        rows.append(row)
    if results:
        raise ValueError('unmatched_prediction_frames')
    positive=[r for r in rows if not r['negative']]
    total=sum(r['annotated_rows'] for r in positive)
    covered=sum(r['covered_rows'] for r in positive)
    summary={'frames':len(rows),'positive_frames':len(positive),'negative_frames':len(rows)-len(positive),
             'predicted_frames':sum(r['prediction'] for r in rows),'annotated_rows':total,'covered_rows':covered,
             'covered_fraction':covered/total if total else None,
             'errors_on_covered_rows':{k:sum(r['errors'].get(k,{}).get('sum',0) for r in positive)/covered
                                       if covered else None for k in ('left','right','center','width')},
             'center_row_recall':{str(t):sum(r['hits'][str(t)]['center'] for r in positive)/total if total else None for t in (2,5,10)},
             'both_boundaries_row_recall':{str(t):sum(r['hits'][str(t)]['both_boundaries'] for r in positive)/total if total else None for t in (2,5,10)},
             'negative_false_positive_rate':sum(r['prediction'] for r in rows if r['negative'])/(len(rows)-len(positive)) if len(rows)>len(positive) else None,
             'index_sha256':sha256(index_path),'results_sha256':sha256(result_path),
             'scope':'sparse annotated snapshots; error only on covered rows; row recall includes all missing rows',
             'metric_accuracy':'unavailable','independent_generalization_validated':False}
    return {'summary':summary,'samples':rows}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--index',required=True);p.add_argument('--results',required=True);p.add_argument('--output',required=True)
    a=p.parse_args()
    if Path(a.output).exists():raise FileExistsError(a.output)
    value=evaluate(a.index,a.results);dump(a.output,value);print(json.dumps(value['summary'],indent=2))


if __name__=='__main__':main()
