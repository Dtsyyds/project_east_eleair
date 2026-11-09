#!/usr/bin/env python3
"""Audited, segment-held-out RGB U-Net prototype; no ROS or production-config writes.

prepare freezes protocol before training. run uses existing learning.train and
geometry.candidate. report uses the existing sparse-annotation evaluator.
"""
import argparse
import copy
import csv
import json
import os
import shutil
from pathlib import Path

import cv2
import numpy as np

from d405_seam_tracking.contracts import dump, load_config, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample, safe_path
from d405_seam_tracking.geometry import candidate, classical_probability, roi_bounds
from d405_seam_tracking.learning import build_model, preprocess, train, training_labels
from tools.vision.prepare_d405_review import import_labels
from evaluate_d405_annotations import evaluate


SEGMENTS = [(1, 3), (4, 7), (8, 10), (11, 21), (22, 25), (26, 30)]
FOLDS = [('segment_01', 'segment_04'), ('segment_02', 'segment_05'), ('segment_03', 'segment_06')]
TRAINING = dict(epochs=24, size=[192, 256], seed=20261102, rgb=True,
                split_group='capture_segment', ignore_boundary_pixels=2,
                freeze_encoder_epochs=4, learning_rate=3e-4, encoder_learning_rate=3e-5,
                horizontal_shift_fraction=.10, log_progress=True)
JUDGEMENT = dict(correct_min_coverage=.50, correct_max_center_mae_px=10.,
                 correct_max_center_p95_px=20., wrong_min_coverage=.25,
                 wrong_min_center_mae_px=20.)
PROTECTED = ['config/d405/vertical_seam.yaml',
             'ros2_ws/src/functions/perception/d405_seam_ros/d405_seam_ros/node.py']


def write_lines(path, values):
    with Path(path).open('x') as stream:
        for value in values:
            stream.write(json.dumps(value, allow_nan=False)+'\n')


def capture_segment(number, segments=SEGMENTS):
    return next(f'segment_{i:02d}' for i, (first, last) in enumerate(segments, 1) if first <= number <= last)


def validate_split_spec(spec, sample_count=30):
    """Require exhaustive disjoint segments and exactly one held-out fold per segment."""
    segments, folds = spec['segments'], spec['folds']
    if not isinstance(spec.get('basis'), str) or not spec['basis'].strip():
        raise ValueError('split_basis_required')
    frames = []
    for segment in segments:
        if (len(segment) != 2 or any(type(x) is not int for x in segment)
                or not 1 <= segment[0] <= segment[1] <= sample_count):
            raise ValueError('invalid_capture_segment_range')
        frames.extend(range(segment[0], segment[1] + 1))
    if sorted(frames) != list(range(1, sample_count + 1)):
        raise ValueError('segments_must_cover_each_frame_once')
    names = [f'segment_{i:02d}' for i in range(1, len(segments) + 1)]
    if len(folds) < 2 or any(not fold or len(fold) >= len(names) for fold in folds):
        raise ValueError('folds_require_nonempty_train_and_test')
    if sorted(name for fold in folds for name in fold) != names:
        raise ValueError('each_segment_must_be_held_out_once')
    return segments, folds


def clip_annotation_y(annotation, first, last):
    """Evaluation-only intersection. Never called by inference or training."""
    result = copy.deepcopy(annotation)
    if result.get('no_seam'):
        return result
    left = np.asarray(result['left_xy']); right = np.asarray(result['right_xy'])
    lo = max(first, left[0, 1], right[0, 1]); hi = min(last, left[-1, 1], right[-1, 1])
    if hi <= lo:
        raise ValueError('no_labelled_rows_in_common_roi')
    for key in ('left_xy', 'right_xy'):
        points = np.asarray(result[key])
        ys = np.unique(np.r_[lo, points[(points[:, 1] > lo) & (points[:, 1] < hi), 1], hi])
        result[key] = np.column_stack((np.interp(ys, points[:, 1], points[:, 0]), ys)).tolist()
    return result


def prepare(source_index, labels, output, split_spec=None):
    spec = (json.loads(Path(split_spec).read_text()) if split_spec else
            dict(segments=SEGMENTS, folds=FOLDS,
                 basis='consecutive capture-order blocks at visible scene/pose changes; defined without using annotation coordinates'))
    segments, fold_segments = validate_split_spec(spec)
    out = Path(output).resolve()
    if (out/'protocol.json').exists() or (out/'dataset').exists():
        raise FileExistsError('prepare_requires_new_dataset_and_protocol')
    out.mkdir(parents=True, exist_ok=True)
    source, original = load_index(source_index)
    label_data = json.loads(Path(labels).read_text())
    if len(original['samples']) != 30 or len(label_data['samples']) != 30:
        raise ValueError('round1_requires_all_30_samples')
    if [r['sample'] for r in original['samples']] != [r['sample'] for r in label_data['samples']]:
        raise ValueError('labels_must_follow_capture_order')
    weights = out/'assets/mobilenet_v2.imagenet.safetensors'
    tree = json.loads((out/'assets/hf_tree.json').read_text())
    expected_weights_sha = next(x['lfs']['oid'] for x in tree if x['path'] == 'model.safetensors')
    if sha256(weights) != expected_weights_sha:
        raise ValueError('pretrained_weight_publisher_sha_mismatch')
    protected = {p:sha256(p) for p in PROTECTED}
    import_labels(source_index, labels, out/'dataset')
    root, dataset = load_index(out/'dataset/index.json')
    mask_dir = out/'masks'; mask_dir.mkdir()
    audit = []
    last_stamp = None
    for i, (raw, record) in enumerate(zip(original['samples'], dataset['samples']), 1):
        frame = read_sample(source, raw)
        imported = read_sample(root, record)
        if frame.image.dtype != np.uint8 or frame.image.shape != (480, 640, 3):
            raise ValueError('expected_original_640x480_rgb_uint8')
        with np.load(safe_path(source, raw['sample']), allow_pickle=False) as data:
            metadata = json.loads(str(data['metadata']))
        if metadata['image_encoding'] != 'rgb8' or frame.source.get('stream') != 'color':
            raise ValueError('rgb_channel_provenance_required')
        np.testing.assert_array_equal(frame.image, imported.image)
        np.testing.assert_array_equal(frame.depth, imported.depth)
        if last_stamp is not None and frame.image_stamp_ns <= last_stamp:
            raise ValueError('non_increasing_capture_order')
        last_stamp = frame.image_stamp_ns
        event = safe_path(source, raw['sample']).parent
        event_metadata = json.loads((event/'metadata.json').read_text())
        if not event_metadata.get('complete'):
            raise ValueError('incomplete_raw_sample')
        for file in event_metadata['files']:
            if sha256(safe_path(event, file['path'])) != file['sha256']:
                raise ValueError('event_file_hash_mismatch')
        target = training_labels(record['annotation'], frame.image.shape, 2)
        np.save(mask_dir/f'{i:02d}.npy', target, allow_pickle=False)
        cv2.imwrite(str(mask_dir/f'{i:02d}.png'), np.where(target < 0, 128, target*255).astype(np.uint8))
        _, rows = annotation_mask(record['annotation'], frame.image.shape)
        record['capture_segment'] = capture_segment(i, segments)
        audit.append(dict(id=i, sample=record['sample'], source_sample=raw['sample'], sha256=raw['sha256'],
                          encoding=metadata['image_encoding'], shape=list(frame.image.shape), dtype=str(frame.image.dtype),
                          channel_mean_rgb=frame.image.mean(axis=(0,1)).tolist(),
                          r_b_equal_fraction=float(np.mean(frame.image[:,:,0] == frame.image[:,:,2])),
                          stamp_ns=str(frame.image_stamp_ns), capture_segment=record['capture_segment'],
                          left_points=len(record['annotation']['left_xy']), right_points=len(record['annotation']['right_xy']),
                          labelled_y=[rows[0][0], rows[-1][0]], positive_pixels=int((target == 1).sum()),
                          negative_pixels=int((target == 0).sum()), ignore_pixels=int((target < 0).sum()),
                          target_sha256=sha256(mask_dir/f'{i:02d}.npy')))
    dump(root/'index.json', dataset)
    cfg = load_config(source/'config.yaml')
    dump(out/'visual_config.json', cfg)
    shutil.copy2(source/'config.yaml', out/'source_visual_config.yaml')
    _, y0, _, y1 = roi_bounds((480,640), cfg)
    common = copy.deepcopy(dataset)
    for record in common['samples']:
        record['annotation'] = clip_annotation_y(record['annotation'], y0, y1-1)
    dump(root/'index_common_roi.json', common)
    folds = []
    for f, held_segments in enumerate(fold_segments, 1):
        fold = copy.deepcopy(dataset)
        training_ids, held_ids = [], []
        for i, record in enumerate(fold['samples'], 1):
            held = record['capture_segment'] in held_segments
            record['split'] = 'test' if held else 'train'
            if held:
                record['annotation'] = None
                held_ids.append(i)
            else:
                training_ids.append(i)
        filename = f'index_fold_{f}.json'
        dump(root/filename, fold)
        folds.append(dict(fold=f, held_segments=list(held_segments), train_ids=training_ids, held_ids=held_ids,
                          index=filename, index_sha256=sha256(root/filename)))
    held_ids = [i for fold in folds for i in fold['held_ids']]
    if sorted(held_ids) != list(range(1,31)):
        raise ValueError('each_frame_must_be_held_out_once')
    dump(out/'audit.json', dict(samples=audit, source_index=str(Path(source_index).resolve()),
                               source_index_sha256=sha256(source_index), labels_sha256=sha256(labels),
                               dataset_index_sha256=sha256(root/'index.json'),
                               sample_count=30, rgb_verified=True, coordinates='original pixels; never rescaled on import',
                               source_files_unchanged=True, protected=protected))
    protocol = dict(schema='d405_rgb_unet_round1_v1', training=TRAINING, judgement=JUDGEMENT, folds=folds,
                    segment_basis=spec['basis'],
                    split_spec_sha256=sha256(split_spec) if split_spec else None,
                    segments=segments, real_session_count=len({r['session'] for r in dataset['samples']}),
                    same_physical_part_may_recur_across_segments=True, independent_generalization=False,
                    pretrained_file=str(weights.relative_to(out)), pretrained_sha256=expected_weights_sha,
                    pretrained_repo='smp-hub/mobilenet_v2.imagenet', pretrained_revision='877efe26185b998af32b04bd7b3ca77e9496eafc',
                    checkpoint_selection='fixed final epoch; no held-out labels used for epoch or threshold selection',
                    fixed_center='x=(width-1)/2; one-pixel proxy for center scoring only; no width estimate',
                    common_eval_y=[y0,y1-1], decoder='existing geometry.candidate with frozen source visual config',
                    production_ready=False, protected=protected,
                    code_sha256={p:sha256(p) for p in [__file__, 'packages/d405_seam_tracking/src/d405_seam_tracking/learning.py',
                                                      'packages/d405_seam_tracking/src/d405_seam_tracking/geometry.py',
                                                      'experiments/vision/scripts/evaluate_d405_annotations.py']})
    dump(out/'protocol.json', protocol)
    print(json.dumps(dict(prepared=str(out), samples=30, folds=folds)), flush=True)


def result_from_probability(image, stamp, probability, config):
    detected = candidate(probability, config, roi_bounds(image.shape, config))
    return dict(source={'image_stamp_ns':str(stamp)}, valid_2d=detected['valid'], predicted_only=False,
                metric_valid=False, points_2d_y_left_right=detected.get('points', []),
                reasons=detected['reasons'], coverage=detected['coverage'])


def infer_rgb(model, image, input_size):
    """Inference receives image and model only; no annotations, labels, or target ROI."""
    import torch
    x, _ = preprocess(image, input_size, rgb=True)
    with torch.inference_mode():
        probability = torch.sigmoid(model(torch.from_numpy(x))).numpy()[0,0]
    return cv2.resize(probability, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR)


def verify_protocol(out):
    protocol = json.loads((out/'protocol.json').read_text())
    for path, digest in protocol['protected'].items():
        if sha256(path) != digest:
            raise ValueError('production_file_changed:'+path)
    if sha256(out/protocol['pretrained_file']) != protocol['pretrained_sha256']:
        raise ValueError('pretrained_weights_changed')
    for path, digest in protocol['code_sha256'].items():
        if sha256(path) != digest:
            raise ValueError('frozen_experiment_code_changed:'+path)
    for fold in protocol['folds']:
        if sha256(out/'dataset'/fold['index']) != fold['index_sha256']:
            raise ValueError('frozen_fold_changed')
    return protocol


def run(output):
    import torch
    out = Path(output).resolve()
    protocol = verify_protocol(out)
    torch.set_num_threads(2); cv2.setNumThreads(2)
    torch.use_deterministic_algorithms(True)
    config = json.loads((out/'visual_config.json').read_text())
    root, index = load_index(out/'dataset/index.json')
    if (out/'model.jsonl').exists():
        raise FileExistsError('run_already_completed')
    (out/'models').mkdir(exist_ok=True)
    (out/'probabilities').mkdir(exist_ok=True)
    predictions = {}
    model_manifest = []
    for fold in protocol['folds']:
        folder = out/'models'/f'fold_{fold["fold"]}'
        weights = folder/'checkpoint.pt'
        if not weights.exists():
            if folder.exists():
                raise RuntimeError('incomplete_fold_directory; preserve logs and use a fresh run')
            train(root/fold['index'], folder, encoder_weights_path=out/protocol['pretrained_file'], **protocol['training'])
        checkpoint = torch.load(weights, map_location='cpu', weights_only=True)
        if checkpoint['dataset_sha256'] != fold['index_sha256'] or checkpoint['encoder_weights'] != protocol['pretrained_sha256']:
            raise ValueError('checkpoint_provenance_mismatch')
        expected_train = [index['samples'][i-1]['sample'] for i in fold['train_ids']]
        if checkpoint['training_samples'] != expected_train:
            raise ValueError('checkpoint_training_membership_mismatch')
        model = build_model(in_channels=3)
        model.load_state_dict(checkpoint['model'], strict=True); model.eval()
        held_results = []
        for i in fold['held_ids']:
            frame = read_sample(root, index['samples'][i-1])
            probability = infer_rgb(model, frame.image, checkpoint['input_size'])
            np.save(out/'probabilities'/f'{i:02d}.npy', probability, allow_pickle=False)
            result = result_from_probability(frame.image, frame.image_stamp_ns, probability, config)
            result.update(fold=fold['fold'], held_out=True, checkpoint_sha256=sha256(weights))
            predictions[i] = result; held_results.append(result)
        result_path = folder/'held_out.jsonl'
        if not result_path.exists():
            write_lines(result_path, held_results)
        model_manifest.append(dict(fold=fold['fold'], weights=str(weights.relative_to(out)),
                                   sha256=sha256(weights), train_ids=fold['train_ids'], held_ids=fold['held_ids']))
    legacy, fixed = [], []
    for i, record in enumerate(index['samples'], 1):
        frame = read_sample(root, record)
        legacy.append(result_from_probability(frame.image, frame.image_stamp_ns, classical_probability(frame.image, config), config))
        h, w = frame.image.shape[:2]; center=(w-1)/2
        fixed.append(dict(source={'image_stamp_ns':str(frame.image_stamp_ns)}, valid_2d=True, predicted_only=False,
                          metric_valid=False, center_only=True,
                          points_2d_y_left_right=[[y, center-.5, center+.5] for y in range(h)]))
    write_lines(out/'model.jsonl', [predictions[i] for i in range(1,31)])
    write_lines(out/'legacy.jsonl', legacy); write_lines(out/'fixed_center.jsonl', fixed)
    dump(out/'model_manifest.json', dict(models=model_manifest, protocol_sha256=sha256(out/'protocol.json'),
                                       all_predictions_out_of_segment=True, test_labels_used_for_prediction=False))
    verify_protocol(out)
    print('All held-out predictions complete; labels have not been used for model selection.', flush=True)


def classify(row, rules):
    if not row['prediction']:
        return 'no_output'
    if not row['covered_rows']:
        return 'no_label_overlap'
    coverage = row['covered_rows']/row['annotated_rows']
    error = row['errors']['center']
    if coverage >= rules['correct_min_coverage'] and error['mean'] <= rules['correct_max_center_mae_px'] and error['p95'] <= rules['correct_max_center_p95_px']:
        return 'correct_seam'
    if coverage >= rules['wrong_min_coverage'] and error['mean'] > rules['wrong_min_center_mae_px']:
        return 'wrong_seam'
    return 'partial_or_inaccurate'


def draw_comparison(frame, annotation, results, states, target):
    _, truth = annotation_mask(annotation, frame.image.shape)
    truth = np.asarray(truth)
    tiles=[]
    for method in ('fixed_center', 'legacy', 'model'):
        image = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
        for side in (1,2):
            cv2.polylines(image,[np.rint(truth[:,[side,0]]).astype(np.int32)],False,(0,255,0),1)
        cv2.polylines(image,[np.rint(np.column_stack(((truth[:,1]+truth[:,2])/2,truth[:,0]))).astype(np.int32)],False,(0,180,0),1)
        result = results[method]
        if result['valid_2d'] and result['points_2d_y_left_right']:
            points=np.asarray(result['points_2d_y_left_right'])
            for part in np.split(points, np.flatnonzero(np.diff(points[:,0])>9)+1):
                if method != 'fixed_center':
                    for side in (1,2):
                        cv2.polylines(image,[np.rint(part[:,[side,0]]).astype(np.int32)],False,(255,0,255),1)
                center=np.column_stack(((part[:,1]+part[:,2])/2,part[:,0]))
                cv2.polylines(image,[np.rint(center).astype(np.int32)],False,(0,0,255),2)
        image=cv2.copyMakeBorder(image,48,0,0,0,cv2.BORDER_CONSTANT)
        cv2.putText(image,f'{method}: {states[method]}',(8,19),cv2.FONT_HERSHEY_SIMPLEX,.52,(255,255,255),1)
        cv2.putText(image,'human=green / predicted center=red / edges=magenta',(8,39),cv2.FONT_HERSHEY_SIMPLEX,.43,(255,255,255),1)
        tiles.append(image)
    cv2.imwrite(str(target),np.hstack(tiles))


def report(output):
    out=Path(output).resolve(); protocol=verify_protocol(out)
    root,index=load_index(out/'dataset/index.json')
    methods=('fixed_center','legacy','model')
    metrics={}; results={}
    for method in methods:
        metrics[method]=evaluate(root/'index_common_roi.json',out/(method+'.jsonl'))
        full=evaluate(root/'index.json',out/(method+'.jsonl'))
        for evaluated in (metrics[method],full):
            if method=='fixed_center':
                evaluated['summary']['both_boundaries_row_recall']=None
                evaluated['summary']['errors_on_covered_rows']={k:(v if k=='center' else None) for k,v in evaluated['summary']['errors_on_covered_rows'].items()}
                for row in evaluated['samples']:
                    row['errors']={k:v for k,v in row['errors'].items() if k=='center'}
                    row.pop('predicted_width_median',None)
                    for hits in row['hits'].values():hits['both_boundaries']=None
        dump(out/(method+'_metrics_common_roi.json'),metrics[method])
        dump(out/(method+'_metrics_full_visible.json'),full)
        result_list=[json.loads(x) for x in (out/(method+'.jsonl')).read_text().splitlines()]
        results[method]={str(r['source']['image_stamp_ns']):r for r in result_list}
    comparison=[]; plot_dir=out/'comparisons'; plot_dir.mkdir(exist_ok=True)
    for i,record in enumerate(index['samples'],1):
        frame=read_sample(root,record)
        row=dict(id=i,sample=record['sample'],capture_segment=record['capture_segment'],
                 fold=next(f['fold'] for f in protocol['folds'] if i in f['held_ids']),image_stamp_ns=str(frame.image_stamp_ns))
        states={}
        for method in methods:
            scored=metrics[method]['samples'][i-1]
            states[method]=classify(scored,protocol['judgement'])
            row[method+'_state']=states[method]
            row[method+'_coverage']=scored['covered_rows']/scored['annotated_rows']
            row[method+'_center_mae_px']=scored['errors'].get('center',{}).get('mean')
            row[method+'_center_p95_px']=scored['errors'].get('center',{}).get('p95')
            row[method+'_center_recall_5px']=scored['hits']['5']['center']/scored['annotated_rows']
            row[method+'_left_mae_px']=scored['errors'].get('left',{}).get('mean')
            row[method+'_right_mae_px']=scored['errors'].get('right',{}).get('mean')
        row['model_fixes_old_wrong']=states['legacy']=='wrong_seam' and states['model']=='correct_seam'
        row['model_recovers_old_missing']=states['legacy']=='no_output' and states['model']=='correct_seam'
        row['model_breaks_old_correct']=states['legacy']=='correct_seam' and states['model']!='correct_seam'
        row['model_new_wrong_after_old_missing']=states['legacy']=='no_output' and states['model']=='wrong_seam'
        comparison.append(row)
        draw_comparison(frame,record['annotation'],{m:results[m][str(frame.image_stamp_ns)] for m in methods},states,plot_dir/f'{i:02d}.jpg')
    with (out/'per_frame_results.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
    transitions={key:[r['id'] for r in comparison if r[key]] for key in
                 ('model_fixes_old_wrong','model_recovers_old_missing','model_breaks_old_correct','model_new_wrong_after_old_missing')}
    transitions['model_wrong_frames']=[r['id'] for r in comparison if r['model_state']=='wrong_seam']
    transitions['model_correct_frames']=[r['id'] for r in comparison if r['model_state']=='correct_seam']
    transitions['model_fixes_old_partial']=[r['id'] for r in comparison if r['legacy_state']=='partial_or_inaccurate' and r['model_state']=='correct_seam']
    summary={m:metrics[m]['summary'] for m in methods}
    for m in methods:
        summary[m]['frame_states']={state:sum(r[m+'_state']==state for r in comparison) for state in
                                   ('correct_seam','wrong_seam','partial_or_inaccurate','no_output','no_label_overlap')}
    dump(out/'summary.json',dict(methods=summary,transitions=transitions,protocol_sha256=sha256(out/'protocol.json')))
    dump(out/'per_frame_results.json',comparison)
    print(json.dumps(dict(methods=summary,transitions=transitions),indent=2),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['prepare','run','report'])
    parser.add_argument('--output',required=True)
    parser.add_argument('--source-index');parser.add_argument('--labels')
    parser.add_argument('--split-spec', help='JSON with segments, folds and basis for a new acquisition')
    args=parser.parse_args()
    if args.action=='prepare':
        if not args.source_index or not args.labels:parser.error('prepare requires --source-index and --labels')
        prepare(args.source_index,args.labels,args.output,args.split_spec)
    elif args.action=='run':run(args.output)
    else:report(args.output)
