#!/usr/bin/env python3
"""Offline central-window ablation of frozen round-1 DP path selection.

All DP states already compete for the global maximum in continuous_rows. Expose
their existing backtraces (including prefixes), filter by median measured center
in original pixels, then take the same stable maximum. Do not rerun DP in a crop,
change rewards, refit to labels, or fall back after a failed geometry gate.
"""
import argparse
import csv
import inspect
import json
import platform
import shutil
import sys
import types
from collections import Counter
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO), str(REPO/'packages/d405_seam_tracking/src')]
import cv2
import numpy as np
from d405_seam_tracking import geometry
from d405_seam_tracking.contracts import dump, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample
from d405_unet_round1 import classify, result_from_probability, verify_protocol, write_lines
from evaluate_d405_annotations import evaluate, score


def expose_paths():
    source = inspect.getsource(geometry.continuous_rows)
    old = 'return rows[::-1], scores[::-1]'
    assert source.count(old) == 1
    source = source.replace(old, 'return rows[::-1], scores[::-1], layers')
    namespace = dict(geometry.continuous_rows.__globals__)
    exec(compile(source, '<round1-continuous-rows-observer>', 'exec'), namespace)
    return namespace['continuous_rows'], source


def paths_from_layers(layers, width):
    paths = []
    for yi, layer in enumerate(layers):
        for ni, total in enumerate(layer['scores']):
            pointer = (yi, ni)
            rows, confidence = [], []
            while pointer is not None:
                li, j = pointer
                item = layers[li]
                rows.append([int(item['y']), int(item['left'][j]), int(item['right'][j])])
                confidence.append(float(item['confidence'][j]))
                parent = item['parents'][j]
                pointer = tuple(parent) if parent[0] >= 0 else None
            rows.reverse(); confidence.reverse()
            median = float(np.median([(r[1]+r[2])/2 for r in rows]))
            paths.append(dict(id=len(paths)+1, endpoint=[yi, ni], dp_score=float(total),
                              rows=rows, confidence=confidence, median_x=median,
                              inside=bool(.30*width <= median < .70*width)))
    # Python's stable sort preserves original layer order and first argmax tie.
    return sorted(paths, key=lambda p: -p['dp_score'])


def fit_path(path, probability, config, bounds):
    namespace = dict(geometry.candidate.__globals__)
    namespace['continuous_rows'] = lambda *_: (path['rows'], path['confidence']) if path else ([], [])
    fit = types.FunctionType(geometry.candidate.__code__, namespace,
                             argdefs=geometry.candidate.__defaults__)
    fit.__kwdefaults__ = dict(geometry.candidate.__kwdefaults__ or {})
    return fit(probability, config, bounds)


def as_result(detected, stamp):
    return dict(source={'image_stamp_ns': str(stamp)}, valid_2d=detected['valid'],
                predicted_only=False, metric_valid=False,
                points_2d_y_left_right=detected.get('points', []),
                reasons=detected['reasons'], coverage=detected['coverage'])


def poly(image, points, color, thickness=1):
    if len(points) < 2:
        return
    points = np.asarray(points)
    for part in np.split(points, np.flatnonzero(np.diff(points[:, 1]) > 9)+1):
        cv2.polylines(image, [np.rint(part).astype(np.int32)], False, color, thickness)


def center(points):
    p = np.asarray(points)
    return np.column_stack(((p[:, 1]+p[:, 2])/2, p[:, 0])) if len(p) else []


def decorate(image, truth, bounds):
    h, w = image.shape[:2]
    for x in (int(.30*w), int(.70*w)):
        cv2.line(image, (x, 0), (x, h-1), (255, 255, 0), 1)
    x0, y0, x1, y1 = bounds
    cv2.rectangle(image, (x0, y0), (x1-1, y1-1), (160, 160, 160), 1)
    for side in (1, 2):
        poly(image, truth[:, [side, 0]], (0, 255, 0))
    poly(image, center(truth), (0, 180, 0), 2)


def caption(image, lines):
    result = cv2.copyMakeBorder(image, 80, 0, 0, 0, cv2.BORDER_CONSTANT)
    for n, line in enumerate(lines):
        cv2.putText(result, line, (8, 19+22*n), cv2.FONT_HERSHEY_SIMPLEX, .47, (255,255,255), 1, cv2.LINE_AA)
    return result


def draw_focus(item):
    frame, annotation, prob, paths, da, db, sa, sb, bounds = item
    _, truth = annotation_mask(annotation, frame.image.shape)
    truth = np.asarray(truth)
    raw = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
    tiles = []
    original = raw.copy(); decorate(original, truth, bounds)
    tiles.append(caption(original, ['Original RGB + human edges/center (green)',
                                    'Cyan: [192,448) window; gray: original ROI', 'All original visible annotation rows retained']))
    heat = cv2.applyColorMap(np.uint8(np.clip(prob, 0, 1)*255), cv2.COLORMAP_TURBO)
    heat = cv2.addWeighted(raw, .25, heat, .75, 0)
    decorate(heat, truth, bounds)
    tiles.append(caption(heat, ['Full-resolution saved probability (0 blue -> 1 red)',
                               'Threshold unchanged: 0.55', 'RGB inference: whole image 192x256 -> 640x480']))
    all_paths = raw.copy()
    for path in reversed(paths):
        poly(all_paths, center(path['rows']), (255,180,0) if path['inside'] else (0,140,255))
    decorate(all_paths, truth, bounds)
    correct = next((p for p in paths if p.get('posthoc', {}).get('state') == 'correct_seam'), None)
    if correct:
        poly(all_paths, center(correct['rows']), (0,255,255), 2)
    tiles.append(caption(all_paths, [f'ALL {len(paths)} DP endpoint paths (prefixes overlap)',
                                    f'Blue: median inside ({sum(p["inside"] for p in paths)}); orange: outside',
                                    f'Yellow: post-hoc correct #{correct["id"]}, score={correct["dp_score"]:.2f}' if correct else 'No post-hoc correct path']))
    for name, detected, selected in [('A', da, sa), ('B', db, sb)]:
        picture = raw.copy(); decorate(picture, truth, bounds)
        if detected['valid']:
            poly(picture, center(detected['points']), (0,0,255), 2)
        if selected and not detected['valid']:
            poly(picture, center(selected['rows']), (255,0,255), 1)
        detail = f'path={selected["id"]}, median={selected["median_x"]:.2f}, score={selected["dp_score"]:.2f}' if selected else 'no eligible path'
        tiles.append(caption(picture, [f'{name}: {"OUTPUT (red)" if detected["valid"] else "NO MEASUREMENT"}', detail,
                                       f'Gate coverage={detected["coverage"]:.2%}; rejected rows=magenta']))
    return np.hstack(tiles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--round1', type=Path, default=REPO/'.runtime/s100p/analysis/d405_unet_round1_20261102')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    old, out = args.round1.resolve(), args.output.resolve()
    if out.exists() or out.is_relative_to(old):
        raise FileExistsError('Use a NEW output directory outside round1')
    cv2.setNumThreads(2)
    protocol = verify_protocol(old)
    # Hash the entire first-round delivery before AND after this experiment.
    before = {str(p.relative_to(old)): sha256(p) for p in sorted(old.rglob('*')) if p.is_file()}
    for manifest_name in ('delivery_manifest.json', 'inputs_manifest.json'):
        for entry in json.loads((old/manifest_name).read_text())['files']:
            assert before[entry['path']] == entry['sha256'], (manifest_name, entry['path'])
    audit = json.loads((old/'audit.json').read_text())
    assert sha256(REPO/'d405-human-labels.json') == audit['labels_sha256']
    assert sha256(audit['source_index']) == audit['source_index_sha256']
    root, index = load_index(old/'dataset/index.json')
    _, common = load_index(old/'dataset/index_common_roi.json')
    config = json.loads((old/'visual_config.json').read_text())
    assert config['candidate_selection'] == 'continuous'
    archived = [json.loads(s) for s in (old/'model.jsonl').read_text().splitlines()]
    manifest = json.loads((old/'model_manifest.json').read_text())
    import torch
    held = {}
    for fold in manifest['models']:
        checkpoint = torch.load(old/fold['weights'], map_location='cpu', weights_only=True)
        frozen_fold = next(f for f in protocol['folds'] if f['fold'] == fold['fold'])
        assert fold['train_ids'] == frozen_fold['train_ids'] and fold['held_ids'] == frozen_fold['held_ids']
        assert sha256(old/fold['weights']) == fold['sha256']
        assert checkpoint['dataset_sha256'] == frozen_fold['index_sha256']
        assert checkpoint['training_samples'] == [index['samples'][i-1]['sample'] for i in fold['train_ids']]
        assert not set(fold['train_ids']) & set(fold['held_ids'])
        for i in fold['held_ids']:
            assert i not in held
            held[i] = fold
    assert sorted(held) == list(range(1, 31))
    out.mkdir(parents=True)
    (out/'candidates').mkdir()
    observer, source = expose_paths()
    (out/'continuous_rows_observer.py').write_text(source)
    shutil.copy2(__file__, out/Path(__file__).name)
    frames, probabilities, a_results = [], [], []
    for i, rec in enumerate(index['samples'], 1):
        frame = read_sample(root, rec)
        probability = np.load(old/'probabilities'/f'{i:02d}.npy', allow_pickle=False)
        assert probability.shape == frame.image.shape[:2] == (480, 640)
        assert np.isfinite(probability).all()
        a = result_from_probability(frame.image, frame.image_stamp_ns, probability, config)
        assert all(a[key] == archived[i-1][key] for key in a), f'A differs on {i}'
        assert archived[i-1]['fold'] == held[i]['fold'] and archived[i-1]['held_out']
        assert archived[i-1]['checkpoint_sha256'] == held[i]['sha256']
        a_results.append(a); frames.append(frame); probabilities.append(probability)
    write_lines(out/'A.jsonl', a_results)
    metrics = {'A': {}}
    for scope, filename in [('common_roi', 'index_common_roi.json'), ('full_visible', 'index.json')]:
        metrics['A'][scope] = evaluate(root/filename, out/'A.jsonl')
        reference = json.loads((old/f'model_metrics_{scope}.json').read_text())
        assert metrics['A'][scope]['samples'] == reference['samples'], f'A score mismatch: {scope}'
    baseline = metrics['A']['common_roi']
    assert sum(classify(r, protocol['judgement']) == 'correct_seam' for r in baseline['samples']) == 17
    assert round(baseline['summary']['center_row_recall']['5']*100, 2) == 54.48
    print('A verified: 30/30 exact geometry and scores; 17/30, 54.48%.', flush=True)
    b_results, selections, focus, diagnostic = [], [], [], []
    for i, (frame, prob, rec) in enumerate(zip(frames, probabilities, index['samples']), 1):
        bounds = geometry.roi_bounds(frame.image.shape, config)
        rows, confidence, layers = observer(prob, config, bounds)
        paths = paths_from_layers(layers, frame.image.shape[1])
        sa = paths[0] if paths else None
        assert (sa['rows'] if sa else []) == rows
        assert (sa['confidence'] if sa else []) == confidence
        eligible = [p for p in paths if p['inside']]
        sb = eligible[0] if eligible else None
        da, db = [fit_path(p, prob, config, bounds) for p in (sa, sb)]
        assert as_result(da, frame.image_stamp_ns) == a_results[i-1]
        b_results.append(as_result(db, frame.image_stamp_ns))
        selection = dict(id=i, fold=held[i]['fold'], all_paths=len(paths), eligible_paths=len(eligible),
                         A_path=sa['id'] if sa else None, B_path=sb['id'] if sb else None,
                         A_median_x=sa['median_x'] if sa else None, B_median_x=sb['median_x'] if sb else None,
                         A_gate_coverage=da['coverage'], B_gate_coverage=db['coverage'],
                         A_reasons=da['reasons'], B_reasons=db['reasons'])
        selections.append(selection)
        # Post-hoc diagnostic ONLY, after both selections. Labels cannot affect B.
        if i in (23, 26, 27, 8):
            for path in paths:
                fit = fit_path(path, prob, config, bounds)
                scored = score(frame, common['samples'][i-1]['annotation'], as_result(fit, frame.image_stamp_ns))
                raw = dict(source={'image_stamp_ns':str(frame.image_stamp_ns)}, valid_2d=len(path['rows']) >= 2,
                           points_2d_y_left_right=path['rows'])
                raw_score = score(frame, common['samples'][i-1]['annotation'], raw)
                path['posthoc'] = dict(gate_valid=fit['valid'], gate_coverage=fit['coverage'],
                                       gate_reasons=fit['reasons'], state=classify(scored, protocol['judgement']),
                                       fitted_center_median_x=float(np.median(np.asarray(fit['points'])[:,1:].mean(1))) if fit['valid'] else None,
                                       evaluation_coverage=scored['covered_rows']/scored['annotated_rows'],
                                       center_mae=scored['errors'].get('center', {}).get('mean'),
                                       center_p95=scored['errors'].get('center', {}).get('p95'),
                                       raw_center_mae=raw_score['errors'].get('center', {}).get('mean'),
                                       raw_covered_rows=raw_score['covered_rows'])
            diagnostic.append(dict(selection, valid_paths=sum(p['posthoc']['gate_valid'] for p in paths),
                                   correct_paths=[p['id'] for p in paths if p['posthoc']['state']=='correct_seam'],
                                   closest_paths=sorted([dict(id=p['id'], median_x=p['median_x'], inside=p['inside'],
                                                             dp_score=p['dp_score'], **p['posthoc']) for p in paths
                                                         if p['posthoc']['raw_covered_rows'] > 0],
                                                        key=lambda p: p['raw_center_mae'])[:5]))
            focus.append((i, (frame, rec['annotation'], prob, paths, da, db, sa, sb, bounds)))
        dump(out/'candidates'/f'{i:02d}.json', dict(selection=selection, paths=paths))
        print(f'{i:02d}: {len(paths)} paths / {len(eligible)} inside; A={da["valid"]} B={db["valid"]}', flush=True)
    write_lines(out/'B.jsonl', b_results)
    metrics['B'] = {}
    summary = {}
    for scope, filename in [('common_roi', 'index_common_roi.json'), ('full_visible', 'index.json')]:
        metrics['B'][scope] = evaluate(root/filename, out/'B.jsonl')
        summary[scope] = {}
        for method in ('A', 'B'):
            value = metrics[method][scope]
            dump(out/f'{method}_metrics_{scope}.json', value)
            states = [classify(r, protocol['judgement']) for r in value['samples']]
            summary[scope][method] = dict(value['summary'], frame_states=dict(Counter(states)),
                                          correct_frames=[i for i, s in enumerate(states, 1) if s=='correct_seam'])
    table = []
    for i, selection in enumerate(selections, 1):
        row = dict(selection)
        for scope in ('common_roi', 'full_visible'):
            for method in ('A', 'B'):
                scored = metrics[method][scope]['samples'][i-1]
                prefix = method+'_'+scope+'_'
                row.update({prefix+'state':classify(scored, protocol['judgement']),
                            prefix+'annotated_rows':scored['annotated_rows'],
                            prefix+'covered_rows':scored['covered_rows'],
                            prefix+'coverage':scored['covered_rows']/scored['annotated_rows'],
                            prefix+'center_mae_px':scored['errors'].get('center', {}).get('mean'),
                            prefix+'center_p95_px':scored['errors'].get('center', {}).get('p95'),
                            prefix+'center_hits_5px':scored['hits']['5']['center'],
                            prefix+'center_recall_5px':scored['hits']['5']['center']/scored['annotated_rows']})
        row['measurement_identical'] = all(a_results[i-1][k] == b_results[i-1][k]
                                           for k in ('valid_2d', 'points_2d_y_left_right'))
        row['diagnostics_identical'] = a_results[i-1] == b_results[i-1]
        table.append(row)
    with (out/'per_frame_ab.csv').open('x') as f:
        writer = csv.DictWriter(f, fieldnames=list(table[0])); writer.writeheader(); writer.writerows(table)
    dump(out/'per_frame_ab.json', table)
    summary['changed_measurement_frames'] = [r['id'] for r in table if not r['measurement_identical']]
    summary['changed_diagnostic_frames'] = [r['id'] for r in table if not r['diagnostics_identical']]
    summary['original_correct_harmed'] = [r['id'] for r in table if r['A_common_roi_state']=='correct_seam' and r['B_common_roi_state']!='correct_seam']
    summary['recovered_correct'] = [r['id'] for r in table if r['A_common_roi_state']!='correct_seam' and r['B_common_roi_state']=='correct_seam']
    dump(out/'summary.json', summary); dump(out/'focus_diagnostics.json', diagnostic)
    panels = []
    for i, data in sorted(focus, key=lambda p: (23,26,27,8).index(p[0])):
        panel = draw_focus(data)
        cv2.putText(panel, f'FRAME {i:02d}', (8,75), cv2.FONT_HERSHEY_SIMPLEX,.55,(255,255,255),1)
        panels.append(panel)
    cv2.imwrite(str(out/'focus_comparison.jpg'), np.vstack(panels), [cv2.IMWRITE_JPEG_QUALITY, 95])
    after = {str(p.relative_to(old)): sha256(p) for p in sorted(old.rglob('*')) if p.is_file()}
    assert before == after
    verify_protocol(old)
    dump(out/'verification.json', dict(round1_all_files_unchanged=True, round1_hashes=before,
                                      all_30_A_geometry_and_scores_exact=True,
                                      all_30_unfiltered_dp_paths_exact=True,
                                      held_out_checkpoint_membership_verified=True,
                                      original_labels_unchanged=True, protected=protocol['protected'],
                                      executable=sys.executable, python=sys.version, platform=platform.platform(),
                                      torch=torch.__version__, numpy=np.__version__, cv2=cv2.__version__,
                                      code_sha256=sha256(__file__), no_inference_or_training_needed=True,
                                      candidate_semantics='all original DP endpoint backtraces, including overlapping prefixes; median over own measured rows in original pixels; stable original score max before unchanged geometry gate; no fallback after gate failure'))
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


if __name__ == '__main__':
    main()
