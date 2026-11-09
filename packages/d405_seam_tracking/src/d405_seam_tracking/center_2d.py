"""Opt-in frame-local center fallback. Never changes a valid A measurement."""
import numpy as np
from .geometry import candidate


def decode(probability, config, bounds, *, enabled=False):
    evidence = {} if enabled else None
    original = candidate(probability, config, bounds, fit_evidence=evidence)
    result = {'A': original, 'kind': 'A_UNCHANGED' if original['valid'] else 'LOST',
              'center_points': [], 'reason': ';'.join(original['reasons'])}
    if original['valid']:
        result['center_points'] = [[float(p[0]), (p[1]+p[2])/2] for p in original['points']]
        return result
    if not enabled or original['reasons'] != ['curve_outliers_or_discontinuous']:
        return result
    p = evidence['rows']
    midpoint = (evidence['fitted_left'] + evidence['fitted_right']) / 2
    indices = np.flatnonzero(np.abs((p[:, 1]+p[:, 2])/2-midpoint) <= 5.0)
    breaks = np.flatnonzero(np.diff(p[indices, 0]) > 3*(config['path_max_gap_rows']+1))+1
    segments = np.split(indices, breaks) if len(indices) else []
    longest = max(segments, key=len) if segments else np.array([], dtype=int)
    result['center_support_rows'] = len(indices)
    result['center_longest_rows'] = len(longest)
    if len(longest) >= 53:
        result.update(kind='CENTER_2D', reason='', center_points=np.column_stack((p[longest, 0], midpoint[longest])).tolist())
    else:
        result['reason'] = 'center_residual_or_longest_below_53'
    return result
