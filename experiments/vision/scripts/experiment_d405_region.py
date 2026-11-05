#!/usr/bin/env python3
"""Offline exploratory RGB region classifier with leave-appearance-group-out predictions.

This is not a deployable tracker or an independent-session accuracy evaluation.
Held-out labels are used only AFTER prediction, for scoring and visualization.
"""
import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

from d405_seam_tracking.contracts import dump, sha256
from d405_seam_tracking.dataset import annotation_mask, load_index, read_sample
from evaluate_d405_annotations import evaluate


CONFIG = dict(seed=20261102, trees=40, max_depth=14, min_samples=15,
              pixels_per_class_per_image=1200, ignore_boundary_px=2,
              scale=0.5, roi_x=[0.2, 0.8], width_original_px=[16, 64],
              width_step_original_px=4, max_center_step_original_px=6,
              transition_cost=0.08, minimum_region_probability=0.50,
              minimum_region_contrast=0.08, minimum_valid_rows=24,
              features='RGB,Lab,gray,gradient,local_mean_std_contrast_scales_3_9_21_41; no image coordinates')


def features(image):
    if image.ndim == 2:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    image = cv2.resize(image, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    rgb = image.astype(np.float32) / 255
    lab = cv2.cvtColor(image, cv2.COLOR_RGB2LAB).astype(np.float32) / 255
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY).astype(np.float32) / 255
    channels = [*cv2.split(rgb), *cv2.split(lab), gray]
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    channels += [abs(gx), abs(gy)]
    for size in (3, 9, 21, 41):
        mean = cv2.blur(gray, (size, size))
        std = np.sqrt(np.maximum(cv2.blur(gray * gray, (size, size)) - mean * mean, 0))
        channels += [mean, std, gray - mean]
    return np.stack(channels, axis=-1)


def training_pixels(frame, annotation, feat, rng):
    _, rows = annotation_mask(annotation, frame.image.shape)
    if not rows:
        raise ValueError('this_experiment_requires_positive_frames')
    rows = np.asarray(rows)
    h, w = feat.shape[:2]
    # OpenCV half-resolution pixel centers map to 2*x+0.5 in the original image.
    yy, xx = np.mgrid[:h, :w]
    yy = 2 * yy + 0.5; xx = 2 * xx + 0.5
    left = np.interp(yy[:, 0], rows[:, 0], rows[:, 1])[:, None]
    right = np.interp(yy[:, 0], rows[:, 0], rows[:, 2])[:, None]
    valid = (yy >= rows[0, 0]) & (yy <= rows[-1, 0])
    margin = CONFIG['ignore_boundary_px']
    pos = valid & (xx > left + margin) & (xx < right - margin)
    neg = valid & ((xx < left - margin) | (xx > right + margin))
    selected, labels = [], []
    for label, mask in ((1, pos), (0, neg)):
        candidates = np.flatnonzero(mask)
        if not len(candidates):
            raise ValueError('missing_training_class')
        pick = rng.choice(candidates, min(len(candidates), CONFIG['pixels_per_class_per_image']), replace=False)
        selected.append(feat.reshape(-1, feat.shape[-1])[pick])
        labels.append(np.full(len(pick), label, np.int32))
    return np.concatenate(selected), np.concatenate(labels)


def region_path(probability):
    """Paired-region evidence followed by bounded center continuity; no label input."""
    h, w = probability.shape
    p = cv2.GaussianBlur(probability, (3, 7), 0)
    cs = np.pad(np.cumsum(p, axis=1), ((0, 0), (1, 0)))
    widths = np.arange(CONFIG['width_original_px'][0]//2, CONFIG['width_original_px'][1]//2+1,
                       CONFIG['width_step_original_px']//2)
    margin = int(widths.max()//2)+6
    centers = np.arange(max(margin, int(w * CONFIG['roi_x'][0])),
                        min(w-margin, int(w * CONFIG['roi_x'][1])))
    if not len(centers):
        raise ValueError('image_too_narrow_for_region_search')
    candidates = []
    for width in widths:
        left = centers - width // 2
        right = left + width
        inside = (cs[:, right] - cs[:, left]) / width
        outside = ((cs[:, left] - cs[:, left-6]) + (cs[:, right+6] - cs[:, right])) / 12
        candidates.append(inside - outside)
    candidate_scores = np.stack(candidates, axis=-1)
    choice = candidate_scores.argmax(axis=-1)
    unary = candidate_scores.max(axis=-1)
    dp = unary[0].copy()
    parents = np.zeros((h, len(centers)), np.int32)
    xs = np.arange(len(centers))
    for y in range(1, h):
        alternatives = []
        step = CONFIG['max_center_step_original_px']//2
        for delta in range(-step, step+1):
            previous = xs + delta
            value = dp[np.clip(previous, 0, len(xs)-1)] - CONFIG['transition_cost'] * abs(delta)
            value[(previous < 0) | (previous >= len(xs))] = -np.inf
            alternatives.append(value)
        options = np.stack(alternatives)
        best = options.argmax(axis=0)
        parents[y] = xs + best - step
        dp = unary[y] + options[best, xs]
    path = np.empty(h, np.int32); path[-1] = dp.argmax()
    for y in range(h-1, 0, -1):
        path[y-1] = parents[y, path[y]]
    center = centers[path]
    width = widths[choice[np.arange(h), path]].astype(np.float32)
    width = cv2.GaussianBlur(width[:, None], (1, 7), 0).ravel()
    left = np.rint(center - width / 2).astype(int)
    right = np.rint(center + width / 2).astype(int)
    inside = (cs[np.arange(h), right] - cs[np.arange(h), left]) / (right-left)
    contrast = unary[np.arange(h), path]
    keep = (inside >= CONFIG['minimum_region_probability']) & (contrast >= CONFIG['minimum_region_contrast'])
    # Require a local run; weak rows remain missing, never fill them using annotations.
    runs = np.split(np.flatnonzero(keep), np.flatnonzero(np.diff(np.flatnonzero(keep)) > 1)+1)
    keep[:] = False
    for run in runs:
        if len(run) >= 4:
            keep[run] = True
    points = np.column_stack((2*np.arange(h)[keep]+0.5, 2*left[keep]-0.5, 2*right[keep]-0.5))
    return points.tolist() if keep.sum() >= CONFIG['minimum_valid_rows'] else []


def draw(frame, annotation, result, target):
    image = cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
    _, truth = annotation_mask(annotation, frame.image.shape)
    for data, color in ((truth, (0, 255, 0)), (result['points_2d_y_left_right'], (255, 0, 255))):
        if not data:
            continue
        data = np.asarray(data)
        parts = np.split(data, np.flatnonzero(np.diff(data[:, 0]) > 9)+1)
        for part in parts:
            for side in (1, 2):
                cv2.polylines(image, [np.rint(part[:, [side, 0]]).astype(np.int32)], False, color, 1)
    cv2.putText(image, 'human=green held-out=magenta', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
    cv2.imwrite(str(target), image)


def run(index_path, groups_path, output):
    cv2.setNumThreads(2)
    cv2.setRNGSeed(CONFIG['seed'])
    root, index = load_index(index_path)
    groups = json.loads(Path(groups_path).read_text())
    records = index['samples']
    if set(groups['samples']) != {r['sample'] for r in records}:
        raise ValueError('group_manifest_must_cover_dataset_exactly')
    out = Path(output); out.mkdir(parents=True, exist_ok=False)
    dump(out/'config.json', CONFIG)
    dump(out/'groups.json', groups)
    frames = [read_sample(root, r) for r in records]
    cached = [features(f.image) for f in frames]
    rng = np.random.default_rng(CONFIG['seed'])
    training = [training_pixels(f, r['annotation'], feat, rng) for f, r, feat in zip(frames, records, cached)]
    all_results = [None]*len(records)
    folds = []
    for group in sorted(set(groups['samples'].values())):
        train_ids = [i for i, r in enumerate(records) if groups['samples'][r['sample']] != group]
        test_ids = [i for i, r in enumerate(records) if groups['samples'][r['sample']] == group]
        if not train_ids:
            raise ValueError('at_least_two_groups_required')
        train_x = np.concatenate([training[i][0] for i in train_ids])
        train_y = np.concatenate([training[i][1] for i in train_ids])
        forest = cv2.ml.RTrees_create()
        forest.setMaxDepth(CONFIG['max_depth']); forest.setMinSampleCount(CONFIG['min_samples'])
        forest.setMaxCategories(2); forest.setActiveVarCount(5)
        forest.setTermCriteria((cv2.TERM_CRITERIA_MAX_ITER, CONFIG['trees'], 0))
        started = time.monotonic()
        forest.train(train_x, cv2.ml.ROW_SAMPLE, train_y)
        train_seconds = time.monotonic()-started
        forest.save(str(out/(group+'.xml')))
        elapsed = []
        for i in test_ids:
            started = time.monotonic()
            feat = cached[i]
            votes = forest.getVotes(feat.reshape(-1, feat.shape[-1]), 0)
            positive_column = np.flatnonzero(votes[0] == 1).item()
            probability = (votes[1:, positive_column] / votes[1:].sum(axis=1)).astype(np.float32).reshape(feat.shape[:2])
            points = region_path(probability)
            elapsed.append(time.monotonic()-started)
            result = dict(source={'image_stamp_ns':str(frames[i].image_stamp_ns)}, valid_2d=bool(points),
                          predicted_only=False, valid_metric=False, points_2d_y_left_right=points,
                          appearance_holdout=group, experimental=True)
            all_results[i] = result
            np.save(out/f'probability_{i+1:02d}.npy', probability, allow_pickle=False)
            draw(frames[i], records[i]['annotation'], result, out/f'{i+1:02d}.jpg')
        fold = dict(held_out_group=group, train_samples=[records[i]['sample'] for i in train_ids],
                    held_out_samples=[records[i]['sample'] for i in test_ids], train_seconds=train_seconds,
                    prediction_seconds_excludes_features=elapsed)
        folds.append(fold)
        print(json.dumps(fold), flush=True)
    with (out/'results.jsonl').open('x') as stream:
        for result in all_results:
            stream.write(json.dumps(result, allow_nan=False)+'\n')
    report = evaluate(index_path, out/'results.jsonl')
    dump(out/'metrics.json', report)
    dump(out/'manifest.json', dict(index_sha256=sha256(index_path), groups_sha256=sha256(groups_path),
                                 script_sha256=sha256(__file__), config=CONFIG, folds=folds,
                                 source_session_count=len({r['session'] for r in records}),
                                 scope='exploratory appearance holdout within one acquisition session; not independent physical-part validation',
                                 production_ready=False))
    print(json.dumps(report['summary']), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', required=True); parser.add_argument('--groups', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--roi-x', type=float, nargs=2, default=[0.2, 0.8],
                        help='Optional center-location prior, not inferred from held-out annotations.')
    args = parser.parse_args()
    if not 0 <= args.roi_x[0] < args.roi_x[1] <= 1:
        parser.error('roi-x must satisfy 0 <= left < right <= 1')
    CONFIG['roi_x'] = args.roi_x
    run(args.index, args.groups, args.output)
