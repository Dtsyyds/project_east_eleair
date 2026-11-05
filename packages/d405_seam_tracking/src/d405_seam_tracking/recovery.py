"""Current-image recovery proposals and an explicitly selected target.

Proposals never become valid measurements just because an operator chose them.
Selection constrains association in later frames; only new supported geometry
can produce a CENTER_2D measurement. No old points are emitted as measurements.
"""
from collections import OrderedDict
import copy
import math
import time
import uuid

import numpy as np

from .geometry import continuous_rows, robust_lstsq


PROPOSAL_THRESHOLD = .30
MIN_PROPOSAL_COVERAGE = .20
ASSOCIATION_DISTANCE_PX = 24.
ASSOCIATION_MARGIN_PX = 8.
OFFER_LIFETIME_SEC = 5.
TARGET_LOSS_SEC = .5


def longest_support(points, mask, max_gap):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return indices
    breaks = np.flatnonzero(np.diff(points[indices, 0]) > max_gap) + 1
    return max(np.split(indices, breaks), key=len)


def propose(probability, config, bounds):
    """Rank up to three distinct, image-supported paths; never pad the list.

Reuse the scanline DP with a lower probability threshold for proposals only.
Suppress each extracted ribbon before looking for a different alternative.
The unchanged A/CENTER_2D decoder continues to use its original thresholds.
"""
    x0, y0, x1, y1 = bounds
    count = len(range(y0, y1, 3))
    search = np.where(np.isfinite(probability), probability, 0.).copy()
    relaxed = dict(config, probability_threshold=PROPOSAL_THRESHOLD)
    alternatives = []
    for _ in range(6):
        rows, scores = continuous_rows(search, relaxed, bounds)
        if not rows:
            break
        p = np.asarray(rows, dtype=float)
        # Remove the actual ribbon even if its curve cannot be fitted.
        for row, left, right in rows:
            search[row, max(x0, int(left)-6):min(x1, int(right)+7)] = 0.
        if len(p) < max(9, math.ceil(MIN_PROPOSAL_COVERAGE * count)):
            continue
        t = (p[:, 0]-y0) / max(1, y1-y0)
        design = np.column_stack((np.ones(len(t)), t, t*t))
        center = p[:, 1:].mean(axis=1)
        try:
            coefficients, residual = robust_lstsq(design, center)
        except ValueError:
            continue
        # Suppress this whole fitted ribbon so fragments are not offered as
        # additional choices. This suppression is search-only, not output.
        ys = np.arange(y0, y1)
        ty = (ys-y0) / max(1, y1-y0)
        xs = coefficients[0] + coefficients[1]*ty + coefficients[2]*ty*ty
        radius = max(12., float(np.median(p[:, 2]-p[:, 1]))/2+6.)
        for row, col in zip(ys, xs):
            lo, hi = max(x0, int(col-radius)), min(x1, int(col+radius)+1)
            if hi > lo:
                search[row, lo:hi] = 0.
        keep = longest_support(p, np.abs(residual) <= 8., 3*(config['path_max_gap_rows']+1))
        if len(keep) < max(9, math.ceil(MIN_PROPOSAL_COVERAGE * count)):
            continue
        try:
            coefficients, residual = robust_lstsq(design[keep], center[keep])
        except ValueError:
            continue
        fitted = design[keep] @ coefficients
        if np.any(fitted < x0) or np.any(fitted >= x1):
            continue
        points = np.column_stack((p[keep, 0], fitted))
        # NMS across common observed rows, including curved alternatives.
        if any(path_distance(points, a['points_2d_y_center']) < 12. for a in alternatives):
            continue
        coverage = len(keep)/count
        confidence = float(np.mean(np.asarray(scores)[keep]))
        rms = float(np.sqrt(np.mean(residual**2)))
        usable = coverage >= config['min_coverage'] and float(np.max(np.abs(residual))) <= config['max_fit_residual_pixels']
        alternatives.append(dict(points_2d_y_center=points.tolist(), coverage=coverage,
            confidence=confidence*coverage, score=confidence*coverage/(1.+rms/5.),
            residual_px=rms, measurement_supported=bool(usable)))
        if len(alternatives) == 3:
            break
    alternatives.sort(key=lambda a: a['score'], reverse=True)
    return [dict(a, candidate_id=str(i+1), rank=i+1) for i, a in enumerate(alternatives)]


def path_distance(first, second):
    """Distance on common measured support only, without extrapolation."""
    a, b = np.asarray(first, dtype=float), np.asarray(second, dtype=float)
    _, ai, bi = np.intersect1d(a[:, 0], b[:, 0], return_indices=True)
    if len(ai) < 9 or len(ai) < .5*min(len(a), len(b)):
        return math.inf
    return float(np.percentile(np.abs(a[ai, 1]-b[bi, 1]), 90))


class RecoveryTracker:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.epoch = uuid.uuid4().hex
        self.offers = OrderedDict()
        self.replies = OrderedDict()
        self.session_id = None
        self.signature = None
        self.last_stamp = None
        self.manual = False
        self.anchor = None
        self.last_measurement_at = None
        self.selection = dict(state='AUTO', reason='')

    def status(self):
        return copy.deepcopy(self.selection)

    def expire(self, reason):
        self.offers.clear()
        self.anchor = None
        if self.manual:
            self.selection.update(state='RESELECT_REQUIRED', reason=reason)

    def command(self, request):
        """Idempotent command ack. set id identifies the exact displayed frame."""
        if not isinstance(request, dict):
            raise ValueError('selection_request_must_be_object')
        ident = request.get('request_id')
        if not isinstance(ident, str) or not 1 <= len(ident) <= 128:
            raise ValueError('invalid_selection_request_id')
        if ident in self.replies:
            previous, reply = self.replies[ident]
            if previous != request:
                return dict(request_id=ident, status='failed', error='request_id_reused', selection=self.status())
            return copy.deepcopy(reply)
        try:
            if request.get('session_id') != self.session_id or not self.session_id:
                raise ValueError('selection_session_mismatch')
            action = request.get('action')
            fields = {'request_id', 'session_id', 'action'}
            if action == 'select':
                fields |= {'candidate_set_id', 'candidate_id'}
            if set(request) != fields or action not in ('select', 'clear'):
                raise ValueError('invalid_selection_fields')
            if action == 'clear':
                self.manual = False; self.anchor = None
                self.selection = dict(state='AUTO', reason='selection_cleared')
                self.offers.clear()
            else:
                set_id, candidate_id = request['candidate_set_id'], request['candidate_id']
                if not isinstance(set_id, str) or not isinstance(candidate_id, str):
                    raise ValueError('invalid_candidate_id')
                offer = self.offers.get(set_id)
                if offer is None or self.clock()-offer[0] > OFFER_LIFETIME_SEC:
                    raise ValueError('candidate_set_expired')
                selected = next((c for c in offer[1] if c['candidate_id'] == candidate_id), None)
                if selected is None:
                    raise ValueError('candidate_not_found')
                self.manual = True
                self.anchor = copy.deepcopy(selected['points_2d_y_center'])
                self.last_measurement_at = self.clock()
                self.selection = dict(state='ACQUIRING', candidate_set_id=set_id,
                    candidate_id=candidate_id, reason='awaiting_new_image_measurement')
                self.offers.clear()
            reply = dict(request_id=ident, status='completed', selection=self.status())
        except ValueError as error:
            reply = dict(request_id=ident, status='failed', error=str(error), selection=self.status())
        self.replies[ident] = (copy.deepcopy(request), copy.deepcopy(reply))
        while len(self.replies) > 64:
            self.replies.popitem(last=False)
        return reply

    def process(self, frame, result, probability, config, bounds):
        now = self.clock()
        if self.last_stamp is not None and (frame.signature != self.signature or
                frame.image_stamp_ns <= self.last_stamp or
                (frame.image_stamp_ns-self.last_stamp)/1e9 > config['reset_gap_sec']):
            self.expire('source_changed_or_time_gap')
        self.session_id = frame.source.get('session')
        self.signature, self.last_stamp = frame.signature, frame.image_stamp_ns
        result.update(candidate_set_id=None, candidates=[])
        candidates = propose(probability, config, bounds) if self.manual or not result['valid_2d'] else []
        if self.manual:
            # An explicit choice stays in charge, even if another seam becomes
            # brighter and the unconstrained A decoder prefers it.
            result.update(state='LOST', valid_2d=False, center_2d_valid=False,
                points_2d_y_left_right=[], points_2d_y_center=[], confidence=0.,
                metric_valid=False, predicted_only=False, measurement_kind='NONE')
            if self.anchor is not None and now-self.last_measurement_at > TARGET_LOSS_SEC:
                self.expire('selected_target_timeout')
            distances = sorted((path_distance(self.anchor, c['points_2d_y_center']), i)
                               for i, c in enumerate(candidates)) if self.anchor is not None else []
            matched = None
            if distances and distances[0][0] <= ASSOCIATION_DISTANCE_PX:
                if len(distances) == 1 or distances[1][0]-distances[0][0] >= ASSOCIATION_MARGIN_PX:
                    matched = candidates[distances[0][1]]
            if matched is not None and matched['measurement_supported']:
                self.anchor = copy.deepcopy(matched['points_2d_y_center'])
                self.last_measurement_at = now
                self.selection.update(state='TRACKING', reason='current_image_selected_center')
                result.update(state='CENTER_2D', valid_2d=True, center_2d_valid=True,
                    measurement_kind='SELECTED_CENTER_2D', points_2d_y_center=self.anchor,
                    confidence=matched['confidence'], reasons=[],
                    evidence_source=['current_frame_operator_selected_center'])
            else:
                if self.anchor is not None:
                    self.selection.update(state='LOST', reason='selected_target_ambiguous_or_unsupported')
                result['reasons'] = [self.selection['reason']]
        if not result['valid_2d'] and candidates:
            set_id = f'{self.epoch}:{frame.sequence}'
            self.offers[set_id] = (now, copy.deepcopy(candidates))
            result.update(candidate_set_id=set_id, candidates=candidates)
        else:
            self.offers.clear()
        while self.offers and (len(self.offers) > 32 or now-next(iter(self.offers.values()))[0] > OFFER_LIFETIME_SEC):
            self.offers.popitem(last=False)
        result['selection'] = self.status()
        return result
