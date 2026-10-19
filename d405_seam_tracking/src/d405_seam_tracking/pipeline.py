"""Evidence fusion with source-time prediction, bounded state and separate 2D/3D validity."""
import time
import cv2
import numpy as np
from .contracts import align_depth, rays
from .geometry import roi_bounds, candidate, classical_probability, depth_evidence


def clean(value):
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)): return [clean(v) for v in value]
    if isinstance(value, (float, np.floating)) and not np.isfinite(value): return None
    if isinstance(value, np.generic): return value.item()
    return value


class Pipeline:
    def __init__(self, config):
        self.config = dict(config)
        self.last_stamp = self.signature = self.last_measurement = None
        self.visual = self.accelerator = None
        if config['visual_backend'] in ('smp', 'bpu', 'auto'):
            from .learning import VisualModel
            self.visual = VisualModel(config)
        elif config['visual_backend'] != 'classical':
            raise ValueError('unsupported_visual_backend')
        if config['geometry_backend'] == 'opencl':
            from .acceleration import OpenCLGeometry
            self.accelerator = OpenCLGeometry()
        elif config['geometry_backend'] not in ('cpu', 'auto'):
            raise ValueError('unsupported_geometry_backend')

    def close(self):
        if self.accelerator: self.accelerator.close()
        if self.visual: self.visual.close()

    def process(self, frame, diagnostic=False):
        start = time.perf_counter()
        frame.validate()
        config = self.config
        reasons = []
        reset = None
        if self.last_stamp is not None:
            if frame.image_stamp_ns <= self.last_stamp:
                reset = 'source_time_duplicate_or_regression'
            elif (frame.image_stamp_ns - self.last_stamp) / 1e9 > config['reset_gap_sec']:
                reset = 'source_time_long_gap'
            elif frame.signature != self.signature:
                reset = 'resolution_or_calibration_changed'
        if reset:
            self.last_measurement = None; reasons.append(reset)
        self.last_stamp, self.signature = frame.image_stamp_ns, frame.signature
        bounds = roi_bounds(frame.image.shape, config)
        result = {'schema': 'd405_seam_result_v1', 'source': frame.metadata(), 'mode': config['mode'],
                  'coordinate_2d': 'original_image_pixel_centers', 'unit_2d': 'pixels', 'unit_3d': 'metres',
                  'frame_id_3d': None, 'body_coordinate_valid': False, 'calibration_status': 'factory' if frame.image_calibration else 'unavailable',
                  'sync_delta_ns': None if frame.depth_stamp_ns is None else str(frame.depth_stamp_ns - frame.image_stamp_ns),
                  'valid_2d': False, 'metric_valid': False, 'seam_identity_confirmed': False, 'identity_field_validated': False,
                  'predicted_only': False, 'state': 'LOST', 'confidence': 0., 'evidence_source': [],
                  'points_2d_y_left_right': [], 'points_3d_center': [], 'points_3d_valid': [], 'reasons': reasons,
                  'thresholds': {k: config[k] for k in ('min_coverage', 'conflict_pixels', 'minimum_height_m', 'noise_multiplier', 'probability_threshold', 'sync_tolerance_ms', 'width_fraction')},
                  'actual_backends': {'visual': 'not_executed', 'geometry': 'not_executed', 'temporal': 'cpu'}, 'timing_ms': {}}
        visual = {'valid': False, 'reasons': ['visual_disabled']}
        probability = np.zeros(frame.image.shape[:2], np.float32)
        tick = time.perf_counter()
        if config['mode'] != 'depth_only':
            if self.visual:
                probability, backend, fallback = self.visual.infer(frame.image)
                result['actual_backends']['visual'] = backend
                if fallback: reasons.append(fallback)
            else:
                probability = classical_probability(frame.image, config)
                result['actual_backends']['visual'] = 'classical_cpu'
            visual = candidate(probability, config, bounds)
        result['timing_ms']['visual'] = (time.perf_counter() - tick) * 1000
        depth = {'valid': False, 'reasons': ['depth_disabled']}
        z = measured = height = None
        tick = time.perf_counter()
        if config['mode'] != 'vision_only':
            try:
                if frame.depth_stamp_ns is None or frame.depth_stamp_ns <= 0:
                    raise ValueError('invalid_depth_timestamp')
                if abs(frame.depth_stamp_ns - frame.image_stamp_ns) > config['sync_tolerance_ms'] * 1e6:
                    raise ValueError('sync_tolerance_exceeded')
                z, measured = align_depth(frame, config['depth_range_m'])
                result['timing_ms']['registration'] = (time.perf_counter() - tick) * 1000
                tick2 = time.perf_counter()
                depth, height = depth_evidence(z, measured, frame.image_calibration, visual, config, bounds, self.accelerator)
                result['actual_backends']['geometry'] = depth['actual_backend']
                result['timing_ms']['surface'] = (time.perf_counter() - tick2) * 1000
            except ValueError as exc:
                depth = {'valid': False, 'reasons': [str(exc)]}
        result['timing_ms']['depth'] = (time.perf_counter() - tick) * 1000
        result['visual'], result['depth'] = visual, depth
        if self.visual:
            result['model_sha256'] = self.visual.model_sha256
            result['image_to_model_transform'] = self.visual.last_transform
        chosen, measured_now = None, False
        if visual.get('valid') and depth.get('valid'):
            v, d = np.array(visual['points']), np.array(depth['points'])
            common = np.intersect1d(v[:, 0], d[:, 0])
            vv = v[np.isin(v[:, 0], common)]; dd = d[np.isin(d[:, 0], common)]
            delta = float(np.median(abs(vv[:, 1:].mean(1) - dd[:, 1:].mean(1)))) if len(common) else float('inf')
            result['agreement_pixels'], result['agreement_margin_pixels'] = delta, config['conflict_pixels'] - delta
            if delta > config['conflict_pixels']:
                chosen = visual; result['state'] = 'CONFLICT_2D'; reasons.append('credible_branches_conflict')
            else:
                chosen = visual
                result['state'] = 'FUSED_CANDIDATE'
                result['evidence_source'] = ['visual', 'measured_depth']
                # Means evidence identity within this experimental detector, not validated field truth.
                result['seam_identity_confirmed'] = True
        elif visual.get('valid'):
            chosen = visual; result['state'] = 'VISION_2D'; result['evidence_source'] = ['visual']
            if config['mode'] == 'fused': reasons += depth['reasons']
        elif depth.get('valid'):
            chosen = depth; result['state'] = 'GEOMETRY_CANDIDATE'; result['evidence_source'] = ['measured_depth']
            reasons.append('seam_identity_unconfirmed')
        else:
            reasons += visual['reasons'] + depth['reasons']
        tick = time.perf_counter()
        if chosen:
            points = np.array(chosen['points'], dtype=float)
            if self.last_measurement:
                old_stamp, old = self.last_measurement
                common = np.intersect1d(old[:, 0], points[:, 0])
                oi, ni = np.isin(old[:, 0], common), np.isin(points[:, 0], common)
                if len(common) and np.median(abs(old[oi, 1:].mean(1) - points[ni, 1:].mean(1))) > config['max_jump_pixels']:
                    reasons.append('sudden_jump_reacquisition'); self.last_measurement = None
                elif len(common):
                    points[ni, 1:] = config['temporal_alpha'] * points[ni, 1:] + (1 - config['temporal_alpha']) * old[oi, 1:]
            result['valid_2d'] = measured_now = True
            result['confidence'] = chosen['confidence'] * chosen['coverage']
            result['points_2d_y_left_right'] = points.tolist()
            self.last_measurement = (frame.image_stamp_ns, points.copy())
        elif self.last_measurement:
            stamp, points = self.last_measurement
            if 0 < (frame.image_stamp_ns - stamp) / 1e9 <= config['prediction_lifetime_sec']:
                result.update(state='PREDICTED', predicted_only=True, valid_2d=True, points_2d_y_left_right=points.tolist(), evidence_source=['history_prediction'])
            else:
                self.last_measurement = None; reasons.append('prediction_expired')
        if measured_now and z is not None and depth.get('valid') and result['state'] != 'CONFLICT_2D':
            points = np.array(result['points_2d_y_left_right'])
            uv = np.column_stack((points[:, 1:].mean(1), points[:, 0]))
            pixels = np.rint(uv).astype(int)
            ok = measured[pixels[:, 1], pixels[:, 0]]
            xyz = rays(frame.image_calibration, pixels) * z[pixels[:, 1], pixels[:, 0], None]
            result['points_3d_center'] = [p.tolist() if v else None for p, v in zip(xyz, ok)]
            result['points_3d_valid'] = ok.tolist()
            result['sampled_depth_pixels_xy'] = pixels.tolist()
            result['point_3d_to_point_2d_index'] = list(range(len(points)))
            result['metric_valid'] = bool(ok.mean() >= config['min_coverage'])
            result['frame_id_3d'] = frame.image_calibration.frame_id
            result['metric_coverage'] = float(ok.mean())
            if not ok.all(): reasons.append('trajectory_depth_holes')
            if not result['metric_valid']: reasons.append('trajectory_metric_coverage_low')
        result['timing_ms']['fusion_temporal'] = (time.perf_counter() - tick) * 1000
        result['timing_ms']['total'] = (time.perf_counter() - start) * 1000
        result = clean(result)
        if diagnostic:
            return result, {'probability': probability, 'height_m': height, 'measured_mask': measured, 'aligned_depth_m': z}
        return result


def overlay(frame, result, diagnostic=None):
    image = cv2.cvtColor(frame.image, cv2.COLOR_GRAY2BGR) if frame.image.ndim == 2 else cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR)
    color = (0, 200, 0) if result['seam_identity_confirmed'] else (0, 170, 255)
    if result['predicted_only']: color = (230, 0, 230)
    if result['state'] == 'CONFLICT_2D': color = (0, 0, 255)
    for branch in ('visual', 'depth'):
        rejected = result[branch]
        if not rejected.get('valid'):
            for y, left, right in rejected.get('rows', [])[::3]:
                for x in (left, right): cv2.circle(image, (round(x), round(y)), 1, (0, 0, 255), -1)
    p = np.asarray(result['points_2d_y_left_right'])
    if len(p):
        for x in (p[:, 1], p[:, 2], p[:, 1:].mean(1)):
            cv2.polylines(image, [np.rint(np.column_stack((x, p[:, 0]))).astype(np.int32)], False, color, 2)
    cv2.putText(image, result['state'], (10, 25), cv2.FONT_HERSHEY_SIMPLEX, .6, color, 2)
    cv2.putText(image, ','.join(result['reasons'])[:100], (10, 45), cv2.FONT_HERSHEY_SIMPLEX, .35, color, 1)
    if diagnostic is not None and image.shape[0] >= 120:
        # Display-only allocations; neither result nor raw depth/mask is modified.
        w, h = image.shape[1] // 4, (image.shape[0] - 60) // 3
        probability = diagnostic.get('probability')
        mask, height = diagnostic.get('measured_mask'), diagnostic.get('height_m')
        span = max(.002, result['depth'].get('height_threshold_m', .0005) * 3)
        # All display operations are pointwise and the original reduction is
        # nearest-neighbor. Select those exact pixels first; coloring a full
        # 848x480 array for a small tile needlessly competes with tracking.
        sample = lambda values: None if values is None else cv2.resize(values, (w, h), interpolation=cv2.INTER_NEAREST)
        probability = sample(probability)
        mask = sample(None if mask is None else mask.astype(np.uint8))
        height = sample(height)
        panels = [('probability', None if probability is None else np.uint8(np.clip(probability, 0, 1) * 255)),
                  ('measured mask', None if mask is None else mask * 255),
                  (f'height +/-{span*1000:.1f}mm', None if height is None else np.uint8(np.nan_to_num(np.clip(height / span * 127 + 128, 0, 255), nan=0)))]
        for i, (label, values) in enumerate(panels):
            tile = np.zeros((h, w, 3), np.uint8)
            if values is not None:
                if i == 2:
                    tile = cv2.applyColorMap(values, cv2.COLORMAP_TURBO)
                    tile[~np.isfinite(height) | (False if mask is None else mask == 0)] = 0
                else:
                    tile = cv2.cvtColor(values, cv2.COLOR_GRAY2BGR)
            else: label += ' unavailable'
            cv2.putText(tile, label, (2, min(12, h-1)), cv2.FONT_HERSHEY_SIMPLEX, .3, (0, 220, 220), 1)
            image[60+i*h:60+(i+1)*h, -w:] = tile
    return image
