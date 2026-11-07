"""Current-frame boundary measurement with a bounded, independently learned width.

Width is in original-image pixels (right minus left), optionally configured.
Inferred centers never train the width model and never masquerade as two edges.
"""
import numpy as np
from .geometry import robust_lstsq
from .recovery import longest_support, path_distance


class SingleBoundary:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.samples = 0
        self.width = self.reference = None
        self.learned_at = None
        self.signature = self.last_stamp = None

    def begin(self, frame):
        if self.last_stamp is not None and (frame.signature != self.signature or
                frame.image_stamp_ns <= self.last_stamp or
                (frame.image_stamp_ns-self.last_stamp)/1e9 > self.config['reset_gap_sec']):
            self.reset()
        self.signature, self.last_stamp = frame.signature, frame.image_stamp_ns

    def learn(self, result, stamp, anchor=None):
        if result.get('state') != 'VISION_2D' or not result.get('visual', {}).get('valid'):
            return
        p = np.asarray(result['points_2d_y_left_right'], float)
        center = np.column_stack((p[:, 0], p[:, 1:].mean(axis=1)))
        if anchor is not None and path_distance(anchor, center) > 24:
            return
        widths = p[:, 2]-p[:, 1]
        if self.reference is not None:
            common, old, new = np.intersect1d(self.width[:, 0], p[:, 0], return_indices=True)
            compatible = (len(common) >= 9 and path_distance(self.reference, center) <= 24
                          and np.median(abs(self.width[old, 1]-widths[new])) <=
                          max(2., .2*float(np.median(widths)))
                          and (stamp-self.learned_at)/1e9 <= self.config.get('single_boundary_width_lifetime_sec', 2.))
            if not compatible:
                self.samples = 0
        self.samples += 1
        self.width = np.column_stack((p[:, 0], widths))
        self.reference, self.learned_at = center, stamp

    def infer(self, probability, bounds, stamp, anchor=None):
        cfg = self.config
        fixed = cfg.get('single_boundary_width_px', 0.)
        age = None if self.learned_at is None else (stamp-self.learned_at)/1e9
        if not fixed and (self.samples < 3 or age is None or not 0 <= age <= cfg.get('single_boundary_width_lifetime_sec', 2.)):
            return None
        if not fixed and anchor is not None and path_distance(self.reference, anchor) > 24:
            return None  # Width belongs to a different selected seam.
        x0, y0, x1, y1 = bounds
        minw = max(2, int((x1-x0)*cfg['width_fraction'][0]))
        maxw = int((x1-x0)*cfg['width_fraction'][1])
        reference = anchor if anchor is not None else self.reference
        reference = None if reference is None else np.asarray(reference)
        tracks = [[], []]
        for y in range(y0, y1, 3):
            if fixed:
                width = fixed
            else:
                indices = np.flatnonzero(self.width[:, 0] == y)
                if not len(indices):
                    continue  # Never extrapolate the learned width beyond measured rows.
                width = self.width[indices[0], 1]
            if not minw-1 <= width <= maxw:
                continue
            expected = None
            if reference is not None:
                indices = np.flatnonzero(reference[:, 0] == y)
                if not len(indices):
                    continue
                expected = reference[indices[0], 1]
            values = probability[y, x0:x1]
            hit = np.isfinite(values) & (values >= cfg['probability_threshold'])
            edges = np.diff(np.r_[False, hit, False].astype(int))
            starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)
            for side in (0, 1):
                choices = []
                for left, end in zip(starts, ends):
                    if not minw <= end-left <= maxw or left == 0 or end == len(values):
                        continue
                    edge = left if side == 0 else end-1
                    inside = min(end-1, edge+2) if side == 0 else max(left, edge-2)
                    outside = max(0, edge-2) if side == 0 else min(len(values)-1, edge+2)
                    contrast = float(values[inside]-values[outside])
                    if not np.isfinite(contrast) or contrast < .1:
                        continue
                    center = x0+edge+(width/2 if side == 0 else -width/2)
                    distance = abs(center-expected) if expected is not None else 0.
                    if distance <= 24:
                        choices.append((distance, x0+edge, center, float(values[inside])))
                choices.sort()
                if choices and (len(choices) == 1 or choices[1][0]-choices[0][0] >= 8):
                    _, edge, center, confidence = choices[0]
                    tracks[side].append([y, edge, center, confidence])
        fits = []
        total = len(range(y0, y1, 3))
        for side, rows in enumerate(tracks):
            if len(rows) < max(9, total*cfg['min_coverage']):
                continue
            p = np.asarray(rows)
            t = (p[:, 0]-y0)/(y1-y0)
            design = np.column_stack((np.ones(len(t)), t, t*t))
            try:
                _, residual = robust_lstsq(design, p[:, 1])
                keep = longest_support(p, abs(residual) <= 3., 3*(cfg['path_max_gap_rows']+1))
                if len(keep)/total < cfg['min_coverage']:
                    continue
                coef, residual = robust_lstsq(design[keep], p[keep, 1])
            except ValueError:
                continue
            if np.max(abs(residual)) > 3.:
                continue
            centers = design[keep] @ coef + p[keep, 2]-p[keep, 1]
            if np.any(centers < x0) or np.any(centers >= x1):
                continue
            points = np.column_stack((p[keep, 0], centers))
            fits.append(dict(side=('left', 'right')[side], points=points,
                             confidence=float(np.mean(p[keep, 3]))*len(keep)/total*.8,
                             coverage=len(keep)/total))
        if not fits:
            return None
        if len(fits) == 2:
            if path_distance(fits[0]['points'], fits[1]['points']) > 5.:
                return None  # Two plausible edges imply different centers: do not guess.
        best = max(fits, key=lambda f: f['confidence'])
        return dict(points=best['points'].tolist(), confidence=best['confidence'],
                    side=best['side'], coverage=best['coverage'], width_source='configured' if fixed else 'measured_double_boundary',
                    width_age_sec=age, width_samples=self.samples)
