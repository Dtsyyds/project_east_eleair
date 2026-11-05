"""Small local optical-coordinate wall fitting, not global mapping or metrology."""
import cv2
import numpy as np
from .contracts import rays


def roi_bounds(shape, config):
    h, w = shape[:2]
    x0, y0, x1, y1 = np.rint(np.asarray(config['roi']) * [w, h, w, h]).astype(int)
    if x1 - x0 < 16 or y1 - y0 < 16:
        raise ValueError('experimental_roi_too_small')
    return int(x0), int(y0), min(int(x1), w), min(int(y1), h)


def robust_lstsq(a, b, iterations=5):
    if len(b) < a.shape[1] * 3 or np.linalg.matrix_rank(a) < a.shape[1] or np.linalg.cond(a) > 1e6:
        raise ValueError('fit_degenerate')
    coef = np.linalg.lstsq(a, b, rcond=None)[0]
    for _ in range(iterations):
        error = b - a @ coef
        scale = max(float(np.median(abs(error - np.median(error))) * 1.4826), 1e-8)
        weight = np.minimum(1., 1.5 * scale / np.maximum(abs(error), 1e-12))
        q = np.sqrt(weight)
        coef = np.linalg.lstsq(a * q[:, None], b * q, rcond=None)[0]
    return coef, b - a @ coef


def basis(x, y, quadratic):
    columns = [np.ones_like(x), x, y]
    if quadratic:
        columns += [x * x, x * y, y * y]
    return np.column_stack(columns)


def classical_probability(image, config):
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    if config.get('visual_feature', 'row_median') == 'local_stripe':
        x0, _, x1, _ = roi_bounds(gray.shape, config)
        width = max(3, int(round((x1 - x0) * config['visual_stripe_width_fraction'])) | 1)
        vertical = max(3, int(round(gray.shape[0] * config['visual_smoothing_fraction'])) | 1)
        small = cv2.GaussianBlur(gray.astype(np.float32), (3, vertical), 0)
        center = cv2.boxFilter(small, -1, (width, 1))
        # Compare a band with BOTH adjacent shoulders. A single intensity step
        # is not a stripe; never join bright and dark responses into one run.
        left, right = np.roll(center, width, axis=1), np.roll(center, -width, axis=1)
        dark = np.minimum(left - center, right - center)
        bright = np.minimum(center - left, center - right)
        response = dark if config['visual_polarity'] == 'dark' else bright
        probability = np.clip(response / config['visual_contrast_u8'], 0., 1.)
        probability[:, :2 * width] = probability[:, -2 * width:] = 0.
        return probability
    small = cv2.GaussianBlur(gray, (3, 3), 0).astype(np.float32)
    # Robust horizontal background preserves a stripe's interior at different resolutions.
    # A short blur previously split a wide stripe into two edges (848x480 regression).
    background = np.median(small, axis=1, keepdims=True)
    contrast = abs(small - background)
    # A visible stripe is a candidate, never trained seam identity evidence.
    return np.clip(contrast / config['visual_contrast_u8'], 0., 1.)


def continuous_rows(probability, config, bounds):
    """Select a coherent chain of measured runs; missing rows stay missing.

    Each layer is one scanline. Dynamic programming scores all allowed runs,
    with bounded lateral/width changes and gaps, rather than greedily selecting
    one run per row. No hard-coded image center or previous-frame path is used.
    """
    x0, y0, x1, y1 = bounds
    minw = max(2, int((x1 - x0) * config['width_fraction'][0]))
    maxw = int((x1 - x0) * config['width_fraction'][1])
    layers, best = [], (-np.inf, None)
    for yi, y in enumerate(range(y0, y1, 3)):
        values = probability[y, x0:x1]
        hit = np.isfinite(values) & (values >= config['probability_threshold'])
        edges = np.diff(np.r_[False, hit, False].astype(np.int8))
        left, end = np.where(edges == 1)[0], np.where(edges == -1)[0]
        keep = (end - left >= minw) & (end - left <= maxw) & (left > 0) & (end < len(values))
        left, end = left[keep], end[keep]
        centers, widths = (left + end - 1) / 2., end - left
        confidence = np.array([float(np.mean(values[l:r])) for l, r in zip(left, end)])
        reward = 1. + confidence
        scores = reward.copy()
        parents = np.full((len(left), 2), -1, dtype=int)
        for gap in range(1, min(config['path_max_gap_rows'] + 2, yi + 1)):
            prev = layers[yi - gap]
            if not len(left) or not len(prev['left']):
                continue
            dx = abs(prev['centers'][:, None] - centers)
            dw = abs(prev['widths'][:, None] - widths)
            limit = max(3., 3 * gap * config['path_max_slope'])
            allowed = (dx <= limit) & (dw <= 2 * limit)
            costs = prev['scores'][:, None] + reward - config['path_position_penalty'] * dx - config['path_width_penalty'] * dw - config['path_gap_penalty'] * (gap - 1)
            costs[~allowed] = -np.inf
            pi = costs.argmax(axis=0)
            proposed = costs[pi, np.arange(len(left))]
            improve = proposed > scores
            scores[improve] = proposed[improve]
            parents[improve, 0] = yi - gap
            parents[improve, 1] = pi[improve]
        layers.append(dict(y=y, left=left + x0, right=end - 1 + x0, centers=centers,
                           widths=widths, confidence=confidence, scores=scores, parents=parents))
        if len(scores) and float(scores.max()) > best[0]:
            best = (float(scores.max()), (yi, int(scores.argmax())))
    rows, scores = [], []
    pointer = best[1]
    while pointer is not None:
        layer, index = pointer
        item = layers[layer]
        rows.append([item['y'], int(item['left'][index]), int(item['right'][index])])
        scores.append(float(item['confidence'][index]))
        parent = item['parents'][index]
        pointer = tuple(parent) if parent[0] >= 0 else None
    return rows[::-1], scores[::-1]


def candidate(probability, config, bounds, *, fit_evidence=None):
    x0, y0, x1, y1 = bounds
    threshold = config['probability_threshold']
    minw = max(2, int((x1 - x0) * config['width_fraction'][0]))
    maxw = int((x1 - x0) * config['width_fraction'][1])
    if config.get('candidate_selection', 'independent') == 'continuous':
        rows, scores = continuous_rows(probability, config, bounds)
    else:
        rows, scores = [], []
        # No interpolation across missing rows; independent scanline evidence.
        for y in range(y0, y1, 3):
            values = probability[y, x0:x1]
            hit = values >= threshold
            edges = np.diff(np.r_[False, hit, False].astype(np.int8))
            runs = [(l, r) for l, r in zip(np.where(edges == 1)[0], np.where(edges == -1)[0]) if minw <= r - l <= maxw]
            if runs:
                l, r = max(runs, key=lambda pair: float(np.mean(values[pair[0]:pair[1]])) - .12 * abs((pair[0] + pair[1]) / 2 - len(values) / 2) / len(values))
                rows.append([y, x0 + l, x0 + r - 1])
                scores.append(float(np.mean(values[l:r])))
    coverage = len(rows) / max(1, len(range(y0, y1, 3)))
    result = {'valid': False, 'coverage': coverage, 'confidence': float(np.mean(scores)) if scores else 0., 'rows': rows, 'reasons': []}
    if coverage < config['min_coverage'] or len(rows) < 6:
        result['reasons'].append('insufficient_continuous_coverage')
        return result
    p = np.array(rows, dtype=float)
    yn = (p[:, 0] - y0) / max(1, y1 - y0)
    a = np.column_stack((np.ones(len(p)), yn, yn ** 2))
    try:
        left, el = robust_lstsq(a, p[:, 1]); right, er = robust_lstsq(a, p[:, 2])
    except ValueError as exc:
        result['reasons'].append(str(exc)); return result
    if fit_evidence is not None:
        fit_evidence.update(rows=p, fitted_left=a @ left, fitted_right=a @ right)
    keep = (abs(el) <= config['max_fit_residual_pixels']) & (abs(er) <= config['max_fit_residual_pixels'])
    if config.get('candidate_selection', 'independent') == 'continuous':
        indices = np.flatnonzero(keep)
        if len(indices):
            breaks = np.flatnonzero(np.diff(p[indices, 0]) > 3 * (config['path_max_gap_rows'] + 1)) + 1
            segments = np.split(indices, breaks)
            longest = max(segments, key=len)
            keep[:] = False
            keep[longest] = True
    coverage = float(keep.sum()) / max(1, len(range(y0, y1, 3)))
    result['coverage'] = coverage
    if coverage < config['min_coverage']:
        result['reasons'].append('curve_outliers_or_discontinuous'); return result
    p = p[keep]
    yn = (p[:, 0] - y0) / max(1, y1 - y0)
    a = np.column_stack((np.ones(len(p)), yn, yn ** 2))
    try:
        left, _ = robust_lstsq(a, p[:, 1]); right, _ = robust_lstsq(a, p[:, 2])
    except ValueError as exc:
        result['reasons'].append(str(exc)); return result
    l, r = a @ left, a @ right
    if np.any(l >= r) or np.any(r - l > maxw) or np.any(r - l < minw - 1):
        result['reasons'].append('crossing_or_width_invalid'); return result
    result.update(valid=True, points=np.column_stack((p[:, 0], l, r)).tolist(), fit_coefficients={'left': left.tolist(), 'right': right.tolist(), 'y_origin': y0, 'y_scale': y1 - y0}, interpolated=[False] * len(p))
    return result


def depth_evidence(z, valid, calibration, visual, config, bounds, accelerator=None):
    x0, y0, x1, y1 = bounds
    q = {'valid': False, 'reasons': [], 'valid_ratio': float(valid[y0:y1, x0:x1].mean()), 'background_points': 0, 'actual_backend': 'cpu_quality_only'}
    if q['valid_ratio'] < .1:
        q['reasons'].append('depth_valid_ratio_low'); return q, np.zeros(z.shape, np.float32)
    center = np.median([(r[1] + r[2]) / 2 for r in visual.get('points', [])]) if visual.get('valid') else (x0 + x1) / 2
    exclusion = (x1 - x0) * config['background_exclusion_fraction'] / 2
    if visual.get('valid'):
        exclusion = max(exclusion, np.median([r[2] - r[1] for r in visual['points']]) * .8)
    yy, xx = np.mgrid[y0:y1:config['background_stride'], x0:x1:config['background_stride']]
    if visual.get('valid'):
        v = np.asarray(visual['points'])
        local_center = (np.interp(yy, v[:, 0], v[:, 1]) + np.interp(yy, v[:, 0], v[:, 2])) / 2
        local_exclusion = np.maximum(exclusion, (np.interp(yy, v[:, 0], v[:, 2]) - np.interp(yy, v[:, 0], v[:, 1])) * .8)
        keep = valid[yy, xx] & (abs(xx - local_center) > local_exclusion)
    else:
        keep = valid[yy, xx] & (abs(xx - center) > exclusion)
    xx, yy = xx[keep], yy[keep]
    q['background_points'] = len(xx)
    if len(xx) < config['min_background_points']:
        q['reasons'].append('background_points_insufficient'); return q, np.zeros(z.shape, np.float32)
    xyz = rays(calibration, np.column_stack((xx, yy))) * z[yy, xx, None]
    origin, scale = np.mean(xyz[:, :2], axis=0), np.std(xyz[:, :2], axis=0)
    if min(scale) < 1e-5:
        q['reasons'].append('background_fit_degenerate'); return q, np.zeros(z.shape, np.float32)
    xy = (xyz[:, :2] - origin) / scale
    quadratic = config['surface'] == 'quadratic'
    try:
        coef, residual = robust_lstsq(basis(xy[:, 0], xy[:, 1], quadratic), xyz[:, 2])
    except ValueError as exc:
        q['reasons'].append(str(exc)); return q, np.zeros(z.shape, np.float32)
    gx, gy = np.full(len(xy), coef[1] / scale[0]), np.full(len(xy), coef[2] / scale[1])
    if quadratic:
        gx = (coef[1] + 2 * coef[3] * xy[:, 0] + coef[4] * xy[:, 1]) / scale[0]
        gy = (coef[2] + coef[4] * xy[:, 0] + 2 * coef[5] * xy[:, 1]) / scale[1]
    residual = residual / np.sqrt(1 + gx * gx + gy * gy)
    noise = max(float(np.median(abs(residual - np.median(residual))) * 1.4826), 1e-7)
    threshold = max(config['minimum_height_m'], config['noise_multiplier'] * noise)
    height = np.full(z.shape, np.nan, dtype=np.float32)
    if accelerator is not None:
        if calibration.distortion != 'none' and any(calibration.d):
            raise ValueError('opencl_requires_rectified_zero_distortion')
        height[y0:y1, x0:x1] = accelerator.residual(z[y0:y1, x0:x1], valid[y0:y1, x0:x1], calibration, bounds, coef, origin, scale)
    else:
        yy, xx = np.nonzero(valid[y0:y1, x0:x1]); yy += y0; xx += x0
        xyz = rays(calibration, np.column_stack((xx, yy))) * z[yy, xx, None]
        xy = (xyz[:, :2] - origin) / scale
        a = basis(xy[:, 0], xy[:, 1], quadratic)
        # Signed normal distance approximation: positive toward camera = protrusion.
        dx = coef[1] / scale[0]; dy = coef[2] / scale[1]
        if quadratic:
            dx = (coef[1] + 2 * coef[3] * xy[:, 0] + coef[4] * xy[:, 1]) / scale[0]
            dy = (coef[2] + coef[4] * xy[:, 0] + 2 * coef[5] * xy[:, 1]) / scale[1]
        height[yy, xx] = (a @ coef - xyz[:, 2]) / np.sqrt(1 + dx * dx + dy * dy)
    probability = np.nan_to_num(np.clip(abs(height) / threshold, 0, 1), nan=0).astype(np.float32)
    c = candidate(probability, config, bounds)
    q.update(c)
    q['actual_backend'] = 'opencl_gpu' if accelerator is not None else 'numpy_cpu'
    q.update(background_noise_m=noise, fit_residual_p95_m=float(np.percentile(abs(residual), 95)), height_threshold_m=threshold,
             surface=config['surface'], surface_coefficients=coef.tolist(), height_sign='positive_toward_camera',
             geometric_significance=float(np.nanmax(abs(height))) / threshold if np.isfinite(height).any() else 0.,
             signed_height_m=float(np.nanmedian([height[int(r[0]), int((r[1] + r[2]) / 2)] for r in c.get('points', [])])) if c.get('valid') else None,
             fit_origin_xy_m=origin.tolist(), fit_scale_xy_m=scale.tolist())
    return q, height
