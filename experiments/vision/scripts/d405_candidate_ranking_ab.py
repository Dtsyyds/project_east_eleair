#!/usr/bin/env python3
"""Rank the unchanged round-1 DP endpoint paths by fixed W/2 proximity.

Only the global path ordering changes. Prefixes remain candidates because they
already compete in the original DP global argmax. Geometry gates stay AFTER
selection. Never prefilter by fit quality or annotations, or retry a failed fit.
"""
import argparse
import csv
import json
import platform
import shutil
import sys
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO), str(REPO/'packages/d405_seam_tracking/src')]
import cv2
import numpy as np
from d405_central_window_ab import expose_paths, paths_from_layers, fit_path, as_result, poly, center, caption
from d405_seam_tracking import geometry
from d405_seam_tracking.contracts import dump, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample
from d405_unet_round1 import classify, verify_protocol, result_from_probability, write_lines
from evaluate_d405_annotations import evaluate, score


def ranking(paths, width):
    """No labels/config/window; exact ties use old score then original scan order."""
    if len(paths) < 2:
        return list(paths)
    x0 = width / 2.
    return sorted(paths, key=lambda p: (abs(p['median_x']-x0), -p['dp_score'], p['id']))


def csv_write(path, rows):
    with path.open('x') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def state_summary(value, rules):
    states = [classify(row, rules) for row in value['samples']]
    return dict(value['summary'], frame_states=dict(Counter(states)),
                correct_frames=[i for i,s in enumerate(states,1) if s=='correct_seam'],
                center_hits_5px=sum(r['hits']['5']['center'] for r in value['samples']))


def draw(frame, annotation, prob, paths, outputs, selections, states, frame_id):
    _, truth = annotation_mask(annotation, frame.image.shape)
    truth = np.asarray(truth)
    raw = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
    bounds = geometry.roi_bounds(frame.image.shape, CONFIG)
    def decorate(im):
        cv2.line(im, (320,0), (320,479), (255,255,0), 1)
        x0,y0,x1,y1=bounds
        cv2.rectangle(im,(x0,y0),(x1-1,y1-1),(160,160,160),1)
        for side in (1,2): poly(im,truth[:,[side,0]],(0,255,0))
        poly(im,center(truth),(0,180,0),1)
    candidates=raw.copy()
    for p in paths: poly(candidates,center(p['rows']),(255,140,0))
    decorate(candidates)
    tiles=[caption(candidates,[f'Frame {frame_id:02d}: all {len(paths)} original DP paths',
                               'Green=human; blue=paths; cyan=x0=320', 'Paths include prefixes; original ROI in gray'])]
    heat=cv2.addWeighted(raw,.25,cv2.applyColorMap(np.uint8(prob*255),cv2.COLORMAP_TURBO),.75,0)
    decorate(heat)
    tiles.append(caption(heat,['Saved held-out probability (blue=0, red=1)',
                              'No new inference / no crop / no hard window', 'Human labels used only after A/B selection']))
    for method in ('A','B'):
        im=raw.copy();decorate(im)
        p=selections[method]; d=outputs[method]
        if d['valid']: poly(im,center(d['points']),(0,0,255),2)
        elif p:
            poly(im,center(p['rows']),(255,0,255),2)
            for row in p['rows']:
                cv2.circle(im,(round((row[1]+row[2])/2),row[0]),2,(255,0,255),-1)
        details=f'#{p["id"]} median={p["median_x"]:.2f} dist={abs(p["median_x"]-320):.2f} score={p["dp_score"]:.2f}' if p else 'no candidate'
        tiles.append(caption(im,[f'{method}: {states[method]}',details,
                                 f'Gate coverage={d["coverage"]:.2%}; red=output, magenta=rejected']))
    return np.hstack(tiles)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(); out=args.output.resolve()
    old=REPO/'.runtime/s100p/analysis/d405_unet_round1_20261102'
    previous=REPO/'.runtime/s100p/analysis/d405_central_window_ab_20261102_final'
    if out.exists() or out.is_relative_to(old) or out.is_relative_to(previous):
        raise FileExistsError('Requires a new output directory outside existing experiments')
    cv2.setNumThreads(2)
    protocol=verify_protocol(old)
    preserved=[old,previous,REPO/'.runtime/s100p/analysis/d405_central_window_ab_20261102']
    def snapshot():
        return {str(p.relative_to(REPO)):sha256(p) for folder in preserved for p in sorted(folder.rglob('*')) if p.is_file()}
    before=snapshot()
    for name in ('inputs_manifest.json','delivery_manifest.json'):
        for e in json.loads((old/name).read_text())['files']:
            assert sha256(old/e['path'])==e['sha256']
    audit=json.loads((old/'audit.json').read_text())
    assert sha256(REPO/'d405-human-labels.json')==audit['labels_sha256']
    root,index=load_index(old/'dataset/index.json')
    _,common=load_index(root/'index_common_roi.json')
    global CONFIG
    CONFIG=json.loads((old/'visual_config.json').read_text())
    archived=[json.loads(s) for s in (old/'model.jsonl').read_text().splitlines()]
    import torch
    model_manifest=json.loads((old/'model_manifest.json').read_text())
    held={}
    for fold in model_manifest['models']:
        frozen=next(f for f in protocol['folds'] if f['fold']==fold['fold'])
        checkpoint=torch.load(old/fold['weights'],map_location='cpu',weights_only=True)
        assert sha256(old/fold['weights'])==fold['sha256']
        assert checkpoint['dataset_sha256']==frozen['index_sha256']
        assert checkpoint['training_samples']==[index['samples'][i-1]['sample'] for i in frozen['train_ids']]
        assert fold['held_ids']==frozen['held_ids'] and fold['train_ids']==frozen['train_ids']
        assert not set(fold['held_ids']) & set(fold['train_ids'])
        for i in fold['held_ids']:
            assert i not in held
            held[i]=fold
    assert sorted(held)==list(range(1,31))
    out.mkdir(parents=True);(out/'candidates').mkdir();(out/'comparisons').mkdir()
    # Freeze the alignment convention and exact ordering before any B selection.
    dump(out/'protocol.json',dict(x0='W/2',x0_for_640=320.,
         independent_alignment_calibration='not found in task configs/reports; no label-derived alignment',
         candidate_pool='unchanged DP endpoint best backtraces, including prefixes',
         A='max original DP score, stable scan-order tie',
         B='if count > 1: min (abs(median_measured_x-W/2), -original_score, original_scan_id)',
         geometry_gates='unchanged AFTER selection; no retry, no prefilter',
         hard_window=False,original_config_sha256=sha256(old/'visual_config.json'),
         original_protocol_sha256=sha256(old/'protocol.json'),code_sha256=sha256(__file__)))
    observer,source=expose_paths(); (out/'continuous_rows_observer.py').write_text(source)
    frames=[]; probs=[]; pools=[]; inventory=[]
    for i,rec in enumerate(index['samples'],1):
        frame=read_sample(root,rec);prob=np.load(old/'probabilities'/f'{i:02d}.npy',allow_pickle=False)
        assert prob.shape==frame.image.shape[:2]==(480,640) and np.isfinite(prob).all()
        rows,conf,layers=observer(prob,CONFIG,geometry.roi_bounds(prob.shape,CONFIG))
        paths=paths_from_layers(layers,frame.image.shape[1])
        for p in paths: p.pop('inside')  # No hard-window criterion exists in this experiment.
        assert (paths[0]['rows'] if paths else [])==rows
        assert (paths[0]['confidence'] if paths else [])==conf
        # Verify exact reuse of the prior exposure, not a new candidate generator.
        prior=json.loads((previous/'candidates'/f'{i:02d}.json').read_text())['paths']
        assert len(paths)==len(prior)
        for p,q in zip(paths,prior): assert all(p[k]==q[k] for k in p)
        frames.append(frame);probs.append(prob);pools.append(paths)
        inventory.append(dict(id=i,fold=held[i]['fold'],candidate_count=len(paths)))
    counts=dict(multiple=sum(len(p)>1 for p in pools),single=sum(len(p)==1 for p in pools),none=sum(not p for p in pools))
    print('Candidate inventory BEFORE B:',json.dumps(counts),flush=True)
    dump(out/'candidate_inventory_before_selection.json',dict(counts=counts,frames=inventory))
    results={'A':[],'B':[]};selections=[];detections=[]
    for i,(frame,prob,paths) in enumerate(zip(frames,probs,pools),1):
        sa=paths[0] if paths else None
        rb=ranking(paths,frame.image.shape[1]);sb=rb[0] if rb else None
        selected={'A':sa,'B':sb}
        bounds=geometry.roi_bounds(frame.image.shape,CONFIG)
        detected={m:fit_path(p,prob,CONFIG,bounds) for m,p in selected.items()}
        for m in ('A','B'): results[m].append(as_result(detected[m],frame.image_stamp_ns))
        expected=result_from_probability(frame.image,frame.image_stamp_ns,prob,CONFIG)
        assert results['A'][-1]==expected
        assert all(expected[k]==archived[i-1][k] for k in expected)
        assert archived[i-1]['held_out'] and archived[i-1]['checkpoint_sha256']==held[i]['sha256']
        selections.append(selected);detections.append(detected)
    # ALL predictions are completed before any annotation scoring.
    metrics={};summary={}
    for m in ('A','B'):
        write_lines(out/f'{m}.jsonl',results[m]);metrics[m]={}
        for scope,filename in [('common_roi','index_common_roi.json'),('full_visible','index.json')]:
            value=evaluate(root/filename,out/f'{m}.jsonl')
            if m=='A': assert value['samples']==json.loads((old/f'model_metrics_{scope}.json').read_text())['samples']
            metrics[m][scope]=value;dump(out/f'{m}_metrics_{scope}.json',value)
            summary.setdefault(scope,{})[m]=state_summary(value,protocol['judgement'])
    assert len(summary['common_roi']['A']['correct_frames'])==17
    assert round(summary['common_roi']['A']['center_row_recall']['5']*100,2)==54.48
    table=[];candidate_tables={}
    for i,(frame,prob,paths,selected,detected) in enumerate(zip(frames,probs,pools,selections,detections),1):
        row=dict(inventory[i-1]);candidate_rows=[]
        b_rank={p['id']:n for n,p in enumerate(ranking(paths,640),1)}
        for rank,p in enumerate(paths,1):
            fitted=fit_path(p,prob,CONFIG,geometry.roi_bounds(prob.shape,CONFIG))
            scored=score(frame,common['samples'][i-1]['annotation'],as_result(fitted,frame.image_stamp_ns))
            candidate_rows.append(dict(candidate_id=p['id'],median_x=p['median_x'],distance_to_x0=abs(p['median_x']-320),
                 original_score=p['dp_score'],A_rank=rank,B_rank=b_rank[p['id']],
                 A_selected=p is selected['A'],B_selected=p is selected['B'],measured_rows=len(p['rows']),
                 gate_valid=fitted['valid'],gate_coverage=fitted['coverage'],gate_reasons=';'.join(fitted['reasons']),
                 posthoc_state=classify(scored,protocol['judgement']),
                 posthoc_center_mae=scored['errors'].get('center',{}).get('mean')))
        candidate_tables[i]=candidate_rows
        csv_write(out/'candidates'/f'{i:02d}.csv',candidate_rows)
        dump(out/'candidates'/f'{i:02d}.json',dict(paths=paths,ranking=candidate_rows))
        row['posthoc_gate_valid_count']=sum(c['gate_valid'] for c in candidate_rows)
        row['selection_changed']=selected['A']['id']!=selected['B']['id'] if paths else False
        row['measurement_changed']=any(results['A'][i-1][k]!=results['B'][i-1][k] for k in ('valid_2d','points_2d_y_left_right'))
        _,truth=annotation_mask(index['samples'][i-1]['annotation'],frame.image.shape)
        row['posthoc_truth_median_x']=float(np.median(center(truth)[:,0]))
        row['posthoc_truth_distance_to_x0']=abs(row['posthoc_truth_median_x']-320)
        for m in ('A','B'):
            p=selected[m]
            row.update({m+'_candidate':p['id'] if p else None,m+'_median_x':p['median_x'] if p else None,
                        m+'_distance_to_x0':abs(p['median_x']-320) if p else None,m+'_score':p['dp_score'] if p else None,
                        m+'_measured_rows':len(p['rows']) if p else 0,m+'_gate_coverage':detected[m]['coverage'],
                        m+'_reasons':';'.join(detected[m]['reasons'])})
            for scope in ('common_roi','full_visible'):
                scored=metrics[m][scope]['samples'][i-1];prefix=m+'_'+scope+'_'
                row.update({prefix+'state':classify(scored,protocol['judgement']),
                     prefix+'annotated_rows':scored['annotated_rows'],prefix+'covered_rows':scored['covered_rows'],
                     prefix+'coverage':scored['covered_rows']/scored['annotated_rows'],
                     prefix+'center_mae':scored['errors'].get('center',{}).get('mean'),
                     prefix+'center_p95':scored['errors'].get('center',{}).get('p95'),
                     prefix+'hits_5px':scored['hits']['5']['center'],
                     prefix+'recall_5px':scored['hits']['5']['center']/scored['annotated_rows']})
        table.append(row)
        states={m:row[m+'_common_roi_state'] for m in ('A','B')}
        picture=draw(frame,index['samples'][i-1]['annotation'],prob,paths,detected,selected,states,i)
        cv2.imwrite(str(out/'comparisons'/f'{i:02d}.jpg'),picture,[cv2.IMWRITE_JPEG_QUALITY,92])
    csv_write(out/'per_frame_ab.csv',table);dump(out/'per_frame_ab.json',table)
    csv_write(out/'frame23_all_candidates.csv',candidate_tables[23])
    detail=['# 第23帧全部原候选（含重叠前缀）','',
            'x0=320 px；距离升序、原分数降序；门槛检查保持在选择之后。后两列只作事后诊断。','',
            '|ID|x中位数|距x0|原分数|B排名|A选中|B选中|原门槛通过|事后类别|',
            '|---:|---:|---:|---:|---:|---|---|---|---|']
    for c in candidate_tables[23]:
        detail.append(f'|{c["candidate_id"]}|{c["median_x"]:.2f}|{c["distance_to_x0"]:.2f}|{c["original_score"]:.8f}|{c["B_rank"]}|{c["A_selected"]}|{c["B_selected"]}|{c["gate_valid"]}|{c["posthoc_state"]}|')
    (out/'frame23_all_candidates.md').write_text('\n'.join(detail)+'\n')
    summary['candidate_inventory']=dict(counts,posthoc_gate_valid_multiple=sum(r['posthoc_gate_valid_count']>1 for r in table),
         posthoc_gate_valid_single=sum(r['posthoc_gate_valid_count']==1 for r in table),
         posthoc_gate_valid_none=sum(r['posthoc_gate_valid_count']==0 for r in table))
    summary['changed_selection_frames']=[r['id'] for r in table if r['selection_changed']]
    summary['changed_measurement_frames']=[r['id'] for r in table if r['measurement_changed']]
    summary['changed_category_frames']=[r['id'] for r in table if r['A_common_roi_state']!=r['B_common_roi_state']]
    summary['original_correct_harmed']=[r['id'] for r in table if r['A_common_roi_state']=='correct_seam' and r['B_common_roi_state']!='correct_seam']
    summary['recovered_correct']=[r['id'] for r in table if r['A_common_roi_state']!='correct_seam' and r['B_common_roi_state']=='correct_seam']
    dump(out/'summary.json',summary)
    assert snapshot()==before
    verify_protocol(old)
    dump(out/'verification.json',dict(original_and_previous_experiments_unchanged=True,preserved_sha256=before,
         all_30_A_predictions_and_scores_exact=True,all_30_candidate_pools_exact=True,
         held_out_membership_verified=True,labels_used_only_after_all_predictions=True,
         executable=sys.executable,platform=platform.platform(),torch=torch.__version__,numpy=np.__version__,
         original_protected=protocol['protected'],protocol_sha256=sha256(out/'protocol.json')))
    shutil.copy2(__file__,out/Path(__file__).name)
    for filename in ('d405_central_window_ab.py','d405_unet_round1.py','evaluate_d405_annotations.py'):
        shutil.copy2(Path(__file__).parent/filename,out/filename)
    print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':
    main()
