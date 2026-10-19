"""Every frame contributes availability/detection; missing ground truth never becomes zero."""
import time
import resource
import numpy as np
from .dataset import annotation_mask, label_valid_region


def percentile(values):
    if not values: return 'unavailable'
    p = np.percentile(values, [50, 95, 100])
    return {'p50': float(p[0]), 'p95': float(p[1]), 'max': float(p[2]), 'count': len(values), 'algorithm': 'exact_numpy_percentile'}


class Evaluator:
    def __init__(self):
        self.count = self.measured = self.predicted = self.lost = self.labelled = self.positive = self.negative = self.tp = self.fp = self.crossings = 0
        self.iou, self.dice, self.timings, self.jitter, self.outages, self.reacquisition = [], [], [], [], [], []
        self.errors = {layer: {key: [] for key in ('left', 'right', 'center', 'width')} for layer in ('visual_candidate', 'depth_candidate', 'visual_fit', 'depth_fit', 'final_temporal')}
        self.last, self.failure_start = None, None
        self.confirmed_tp = self.confirmed_fp = 0
        self.failures, self.stage_times = {}, {}
        self.start = time.perf_counter(); self.cpu_start = time.process_time()

    def add(self, frame, result, annotation, probability):
        self.count += 1
        measured = result['valid_2d'] and not result['predicted_only']
        self.measured += measured; self.predicted += result['predicted_only']; self.lost += not result['valid_2d']
        stamp = frame.image_stamp_ns / 1e9
        for reason in result['reasons']: self.failures[reason] = self.failures.get(reason, 0) + 1
        self.timings.append(result['timing_ms']['total'])
        for key, value in result['timing_ms'].items(): self.stage_times.setdefault(key, []).append(value)
        if measured:
            points = np.asarray(result['points_2d_y_left_right'])
            self.crossings += bool((points[:, 1] >= points[:, 2]).any())
            if self.last is not None:
                old = self.last; common = np.intersect1d(old[:, 0], points[:, 0])
                if len(common): self.jitter += abs(old[np.isin(old[:, 0], common), 1:].mean(1) - points[np.isin(points[:, 0], common), 1:].mean(1)).tolist()
            self.last = points
            if self.failure_start is not None:
                gap = max(0, stamp - self.failure_start); self.outages.append(gap); self.reacquisition.append(gap); self.failure_start = None
        else:
            if self.failure_start is None: self.failure_start = stamp
        self.last_stamp = stamp
        if not annotation or annotation.get('status') != 'approved': return
        mask, truth = annotation_mask(annotation, frame.image.shape, allow_synthetic=frame.source.get('synthetic', False))
        self.labelled += 1
        positive = bool(mask.any()); self.positive += positive; self.negative += not positive
        self.tp += bool(measured and positive); self.fp += bool(measured and not positive)
        self.confirmed_tp += bool(measured and result['seam_identity_confirmed'] and positive)
        self.confirmed_fp += bool(measured and result['seam_identity_confirmed'] and not positive)
        valid = label_valid_region(frame.image.shape, truth)
        pred = (probability >= result['thresholds']['probability_threshold']) & valid
        intersection, union = int((pred & mask.astype(bool)).sum()), int(((pred | mask.astype(bool)) & valid).sum())
        self.iou.append(intersection / union if union else 1.)
        denominator = int(pred.sum() + mask.sum()); self.dice.append(2 * intersection / denominator if denominator else 1.)
        if not truth: return
        truth = np.array(truth)
        layers = {'visual_candidate': result['visual'].get('rows', []), 'depth_candidate': result['depth'].get('rows', []),
                  'visual_fit': result['visual'].get('points', []), 'depth_fit': result['depth'].get('points', []),
                  'final_temporal': result['points_2d_y_left_right'] if measured else []}
        for layer, data in layers.items():
            p = np.asarray(data)
            if not len(p): continue
            inside = (p[:, 0] >= truth[0, 0]) & (p[:, 0] <= truth[-1, 0]); p = p[inside]
            l = np.interp(p[:, 0], truth[:, 0], truth[:, 1]); r = np.interp(p[:, 0], truth[:, 0], truth[:, 2])
            for key, error in [('left', p[:, 1] - l), ('right', p[:, 2] - r), ('center', p[:, 1:].mean(1) - (l + r) / 2), ('width', p[:, 2] - p[:, 1] - (r - l))]:
                self.errors[layer][key] += abs(error).tolist()

    def report(self):
        wall = time.perf_counter() - self.start
        if self.failure_start is not None: self.outages.append(max(0., self.last_stamp - self.failure_start)); self.failure_start = None
        return {'frames_total': self.count, 'measured_frames': self.measured, 'predicted_frames': self.predicted, 'lost_frames': self.lost,
                'measurement_detection_rate': self.measured / self.count if self.count else None,
                'prediction_ratio': self.predicted / self.count if self.count else None, 'crossing_frames': self.crossings,
                'labelled_frames': self.labelled, 'positive_frames': self.positive, 'negative_frames': self.negative,
                'trajectory_candidate_recall': self.tp / self.positive if self.positive else 'unavailable',
                'trajectory_candidate_negative_false_positive_rate': self.fp / self.negative if self.negative else 'unavailable',
                'seam_identity_recall': self.confirmed_tp / self.positive if self.positive else 'unavailable',
                'seam_identity_negative_false_positive_rate': self.confirmed_fp / self.negative if self.negative else 'unavailable',
                'mask_iou': percentile(self.iou), 'mask_dice': percentile(self.dice),
                'error_pixels_by_layer': {l: {k: percentile(v) for k, v in m.items()} for l, m in self.errors.items()},
                'millimetre_errors': 'unavailable_no_independent_metric_ground_truth',
                'trajectory_jitter_pixels': percentile(self.jitter), 'failure_duration_sec': percentile(self.outages), 'reacquisition_sec': percentile(self.reacquisition),
                'failure_reasons': self.failures, 'timing_ms': {k: percentile(v) for k, v in self.stage_times.items()},
                'total_wall_sec': wall, 'actual_processing_fps': self.count / wall if wall else None,
                'cpu_seconds': time.process_time() - self.cpu_start, 'peak_rss_mb': resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                'offline_result_drops': 0, 'error_availability_rule': 'errors only where ground truth and measured samples overlap; all failures remain in detection denominators',
                'visual_vs_geometric_boundary_truth': 'pixel labels describe visible boundary; depth boundary comparison is diagnostic, not equal physical footprint truth'}
