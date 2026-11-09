"""Output-only conversion of a measured A centerline to the public image errors.

Never selects candidates, changes A, fills rejected rows, or uses history.
Fit at least six measured points inside the configured direction region.
Partial and one-sided support is allowed; evaluate the fitted curve and tangent
at the fixed reference row, including when that row is outside sampled support.
"""
from dataclasses import dataclass
import math
import numpy as np
import yaml


@dataclass(frozen=True)
class ErrorReference:
    image_width: int = 640
    image_height: int = 480
    reference_x: float = 320.0
    reference_y: float = 360.0
    reference_dx_dy: float = 0.0
    heading_y_min: float = 336.0
    heading_y_max: float = 384.0
    min_local_rows: int = 6
    max_row_gap_px: float = 9.0  # Legacy config compatibility; no coverage/gap gate.
    max_local_residual_px: float = 5.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('nonfinite_error_reference')
        if (type(self.image_width) is not int or type(self.image_height) is not int
                or self.image_width <= 0 or self.image_height <= 0
                or not 0 <= self.reference_x < self.image_width
                or not 0 <= self.heading_y_min < self.reference_y < self.heading_y_max < self.image_height
                or type(self.min_local_rows) is not int or self.min_local_rows < 6
                or self.max_row_gap_px <= 0 or self.max_local_residual_px <= 0):
            raise ValueError('invalid_error_reference')

    @classmethod
    def load(cls, path):
        with open(path) as stream:
            return cls(**yaml.safe_load(stream))


def invalid_error():
    return dict(valid=False, lateral_error_norm=0.0, heading_error_rad=0.0, confidence=0.0)


def tracking_error(result, width, height, reference):
    """Return (wire fields, diagnostic reason); input result is never modified."""
    invalid = invalid_error()
    if (width, height) != (reference.image_width, reference.image_height):
        return invalid, 'processing_image_size_mismatch'
    center_only = result.get('state') == 'CENTER_2D' and result.get('center_2d_valid') is True
    if (result.get('valid_2d') is not True or result.get('predicted_only') is not False
            or (not center_only and (result.get('state') != 'VISION_2D' or result.get('visual', {}).get('valid') is not True))
            or result.get('coordinate_2d') != 'original_image_pixel_centers'):
        return invalid, 'not_complete_measured_visual_output'
    try:
        points = np.asarray(result['points_2d_y_center'] if center_only else result['points_2d_y_left_right'], dtype=float)
        confidence = float(result['confidence'])
    except (KeyError, TypeError, ValueError):
        return invalid, 'malformed_visual_output'
    if not math.isfinite(confidence) or not 0 < confidence <= 1:
        return invalid, 'invalid_joint_confidence'
    if (points.ndim != 2 or points.shape[1] != (2 if center_only else 3) or len(points) < reference.min_local_rows
            or not np.isfinite(points).all()):
        return invalid, 'invalid_centerline_points'
    if center_only:
        y, center = points.T
        if (np.any(np.diff(y) <= 0) or np.any(y < 0) or np.any(y >= height) or np.any(center < 0) or np.any(center >= width)):
            return invalid, 'invalid_centerline_coordinates'
    else:
        y, left, right = points.T
    if not center_only and (np.any(np.diff(y) <= 0) or np.any(y < 0) or np.any(y >= height)
            or np.any(left < 0) or np.any(right >= width) or np.any(left >= right)):
        return invalid, 'invalid_centerline_coordinates'
    # Count actual points inside the region. Neither full endpoint coverage,
    # samples on both sides of reference_y nor bounded row gaps are required.
    local = points[(y >= reference.heading_y_min) & (y <= reference.heading_y_max)]
    if len(local) < reference.min_local_rows:
        return invalid, 'insufficient_local_rows'
    scale = reference.heading_y_max - reference.heading_y_min
    t = (local[:, 0] - reference.reference_y) / scale
    design = np.column_stack((np.ones(len(t)), t, t*t))
    center = local[:, 1] if center_only else local[:, 1:].mean(axis=1)
    try:
        coefficients, _, rank, _ = np.linalg.lstsq(design, center, rcond=None)
    except np.linalg.LinAlgError:
        return invalid, 'local_direction_fit_failed'
    if rank != 3 or not np.isfinite(coefficients).all():
        return invalid, 'local_direction_degenerate'
    if np.max(np.abs(design @ coefficients - center)) > reference.max_local_residual_px:
        return invalid, 'local_direction_residual_high'
    lateral = (float(coefficients[0]) - reference.reference_x) / (width / 2.0)
    # Forward vector is (-dx/dy, -1), never its backward equivalent.
    heading = math.atan(float(coefficients[1]) / scale) - math.atan(reference.reference_dx_dy)
    if not math.isfinite(lateral) or not math.isfinite(heading):
        return invalid, 'nonfinite_errors'
    # Both fields share A's complete-output confidence; not calibrated accuracy.
    return dict(valid=True, lateral_error_norm=lateral, heading_error_rad=heading,
                confidence=confidence), None
