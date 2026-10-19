"""Explicit original-pixel, optical-frame, depth-unit and source-time contracts."""
from dataclasses import dataclass, asdict
import hashlib
import json
from pathlib import Path
import cv2
import numpy as np
import yaml


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def dump(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.partial')
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def load_config(path):
    config = yaml.safe_load(Path(path).read_text())
    if config['mode'] not in ('vision_only', 'depth_only', 'fused'):
        raise ValueError('unsupported fusion mode')
    if config['surface'] not in ('plane', 'quadratic'):
        raise ValueError('unsupported wall surface')
    for key in ('minimum_height_m', 'noise_multiplier', 'visual_contrast_u8', 'min_background_points', 'background_stride', 'conflict_pixels', 'max_fit_residual_pixels', 'reset_gap_sec', 'max_jump_pixels'):
        if not np.isfinite(config[key]) or config[key] <= 0: raise ValueError('invalid_positive_config:' + key)
    for key in ('sync_tolerance_ms', 'prediction_lifetime_sec'):
        if not np.isfinite(config[key]) or config[key] < 0: raise ValueError('invalid_nonnegative_config:' + key)
    for key in ('min_coverage', 'probability_threshold', 'temporal_alpha', 'background_exclusion_fraction'):
        if not 0 < config[key] <= 1: raise ValueError('invalid_fraction_config:' + key)
    lo, hi = config['depth_range_m']
    if not 0 < lo < hi or not np.isfinite([lo, hi]).all(): raise ValueError('invalid_depth_range')
    lo, hi = config['width_fraction']
    if not 0 < lo < hi < 1: raise ValueError('invalid_width_fraction')
    if not isinstance(config['background_stride'], int) or not isinstance(config['min_background_points'], int): raise ValueError('integer_sampling_config_required')
    r = config['roi']
    if len(r) != 4 or not (0 <= r[0] < r[2] <= 1 and 0 <= r[1] < r[3] <= 1):
        raise ValueError('ROI must be normalized x0,y0,x1,y1')
    return config


@dataclass(frozen=True)
class Calibration:
    width: int
    height: int
    k: list
    d: list
    distortion: str
    frame_id: str
    source: str

    def validate(self):
        k = np.asarray(self.k, dtype=float).reshape(3, 3)
        if self.width <= 0 or self.height <= 0 or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            raise ValueError('invalid_intrinsics')
        if not np.allclose(k[2], [0, 0, 1], atol=1e-12) or abs(k[0, 1]) > 1e-12 or abs(k[1, 0]) > 1e-12:
            raise ValueError('unsupported_skew_or_nonpinhole_intrinsics')
        if not self.frame_id or not self.source or not np.isfinite(self.d).all():
            raise ValueError('missing_calibration_provenance')
        if self.distortion not in ('none', 'plumb_bob', 'rational_polynomial', 'equidistant'):
            raise ValueError('unsupported_distortion:' + self.distortion)
        expected = {'plumb_bob': (0, 4, 5), 'rational_polynomial': (8, 12, 14), 'equidistant': (4,), 'none': (0,)}
        if len(self.d) not in expected[self.distortion]:
            raise ValueError('invalid_distortion_coefficients')
        return self

    @property
    def fingerprint(self):
        return hashlib.sha256(json.dumps(asdict(self), sort_keys=True).encode()).hexdigest()


@dataclass
class Frame:
    image: np.ndarray                 # mono8 OR RGB uint8, never implicit BGR/NV12
    depth: np.ndarray | None          # original uint16 OR float32/64, no colorized depth
    raw_valid: np.ndarray | None
    image_stamp_ns: int
    depth_stamp_ns: int | None
    sequence: int
    image_calibration: Calibration | None
    depth_calibration: Calibration | None
    depth_to_image: dict | None        # R row-major, t metres, source/target optical frame
    depth_scale_m: float | None
    source: dict
    receive_stamp_ns: int | None = None

    def validate(self):
        if self.image.dtype != np.uint8 or self.image.ndim not in (2, 3) or (self.image.ndim == 3 and self.image.shape[2] != 3):
            raise ValueError('image must be mono8 or RGB8')
        if self.image_stamp_ns <= 0:
            raise ValueError('invalid_image_timestamp')
        if self.depth is not None:
            if self.depth.ndim != 2 or self.depth.dtype not in (np.uint16, np.float32, np.float64):
                raise ValueError('raw_depth_type_invalid')
            if self.raw_valid is None or self.raw_valid.shape != self.depth.shape or self.raw_valid.dtype != bool:
                raise ValueError('raw_valid_mask_required')
            if self.depth_scale_m is None or not np.isfinite(self.depth_scale_m) or self.depth_scale_m <= 0:
                raise ValueError('explicit_depth_scale_required')
        for c, shape in ((self.image_calibration, self.image.shape), (self.depth_calibration, None if self.depth is None else self.depth.shape)):
            if c is not None:
                c.validate()
                if shape is not None and (c.height, c.width) != tuple(shape[:2]):
                    raise ValueError('calibration_resolution_mismatch')
        return self

    @property
    def signature(self):
        return (self.image.shape, None if self.depth is None else self.depth.shape,
                None if self.image_calibration is None else self.image_calibration.fingerprint,
                None if self.depth_calibration is None else self.depth_calibration.fingerprint,
                json.dumps(self.depth_to_image, sort_keys=True), self.depth_scale_m, self.source.get('session'))

    def metadata(self):
        return {'image_stamp_ns': str(self.image_stamp_ns), 'depth_stamp_ns': None if self.depth_stamp_ns is None else str(self.depth_stamp_ns),
                'receive_stamp_ns': None if self.receive_stamp_ns is None else str(self.receive_stamp_ns),
                'sequence': self.sequence, 'image_calibration': None if self.image_calibration is None else asdict(self.image_calibration),
                'depth_calibration': None if self.depth_calibration is None else asdict(self.depth_calibration),
                'depth_to_image': self.depth_to_image, 'depth_scale_m': self.depth_scale_m, 'source': self.source,
                'image_encoding': 'mono8' if self.image.ndim == 2 else 'rgb8'}


@dataclass(frozen=True)
class PixelTransform:
    """Crop then resize, using OpenCV half-pixel centers; inverse is exact."""
    x0: float
    y0: float
    sx: float
    sy: float

    def forward(self, points):
        p = np.asarray(points, dtype=float)
        return (p - [self.x0, self.y0] + .5) * [self.sx, self.sy] - .5

    def inverse(self, points):
        return (np.asarray(points, dtype=float) + .5) / [self.sx, self.sy] - .5 + [self.x0, self.y0]


def rays(calibration, pixels):
    c = calibration.validate()
    p = np.ascontiguousarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    if not len(p): return np.empty((0, 3), np.float64)
    k = np.asarray(c.k).reshape(3, 3)
    d = np.asarray(c.d, dtype=float)
    if c.distortion == 'equidistant':
        xy = cv2.fisheye.undistortPoints(p, k, d).reshape(-1, 2)
    elif not np.any(d):
        # The D405 depth profile is normally pinhole. Avoid an iterative
        # undistortion call for its exactly zero coefficients, not for fisheye.
        xy = (p.reshape(-1, 2) - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]
    else:
        xy = cv2.undistortPoints(p, k, d if len(d) else None).reshape(-1, 2)
    return np.column_stack((xy, np.ones(len(xy))))


def project(calibration, points):
    c = calibration.validate()
    # OpenCV fisheye bindings can silently ignore a sliced input's row stride.
    p = np.ascontiguousarray(points, dtype=np.float64).reshape(-1, 1, 3)
    if not len(p): return np.empty((0, 2), np.float64)
    k = np.asarray(c.k).reshape(3, 3)
    d = np.asarray(c.d, dtype=float)
    if c.distortion == 'equidistant':
        return cv2.fisheye.projectPoints(p, np.zeros(3), np.zeros(3), k, d)[0].reshape(-1, 2)
    if not np.any(d) or c.distortion == 'plumb_bob':
        # Forward Brown-Conrady projection, equivalent to projectPoints with
        # zero pose. OpenCV also allocates a large, unused per-point Jacobian.
        # Preserve its Z=0 convention for this public geometry helper; the
        # registration caller independently rejects nonpositive target Z.
        xyz = p.reshape(-1, 3)
        z = np.where(xyz[:, 2] == 0, 1., xyz[:, 2])
        x, y = (xyz[:, :2] / z[:, None]).T
        if np.any(d):
            k1, k2, p1, p2 = d[:4]
            k3 = d[4] if len(d) == 5 else 0.
            r2 = x*x + y*y
            radial = 1. + k1*r2 + k2*r2*r2 + k3*r2*r2*r2
            xy = np.column_stack((x*radial + 2*p1*x*y + p2*(r2 + 2*x*x),
                                  y*radial + p1*(r2 + 2*y*y) + 2*p2*x*y))
        else:
            xy = np.column_stack((x, y))
        return xy * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    return cv2.projectPoints(p, np.zeros(3), np.zeros(3), k, d if len(d) else None)[0].reshape(-1, 2)


def align_depth(frame, limits):
    """Measured samples only. Nearest target Z wins; never resize or hole-fill."""
    if frame.depth is None or frame.depth_calibration is None or frame.image_calibration is None or frame.depth_to_image is None:
        raise ValueError('calibration_or_depth_unavailable')
    e = frame.depth_to_image
    if not e.get('source') or e.get('from_frame') != frame.depth_calibration.frame_id or e.get('to_frame') != frame.image_calibration.frame_id:
        raise ValueError('extrinsic_frame_or_provenance_mismatch')
    r, t = np.asarray(e['rotation'], dtype=float).reshape(3, 3), np.asarray(e['translation_m'], dtype=float)
    if not np.isfinite(r).all() or not np.isfinite(t).all() or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(r), 1, atol=1e-5):
        raise ValueError('invalid_rigid_transform')
    z = frame.depth.astype(np.float64) * frame.depth_scale_m
    valid = frame.raw_valid & np.isfinite(z) & (z >= limits[0]) & (z <= limits[1])
    a, b = frame.depth_calibration, frame.image_calibration
    if (a.width, a.height, a.k, a.d, a.distortion) == (b.width, b.height, b.k, b.d, b.distortion) and np.allclose(r, np.eye(3), atol=1e-12) and np.allclose(t, 0, atol=1e-12):
        return np.where(valid, z, np.nan), valid  # Proven calibration + identity transform, not size alone.
    y, x = np.nonzero(valid)
    xyz = rays(frame.depth_calibration, np.column_stack((x, y))) * z[y, x, None]
    xyz = xyz @ r.T + t
    positive = xyz[:, 2] > 0
    xyz = xyz[positive]
    uv = np.rint(project(frame.image_calibration, xyz)).astype(np.int64)
    h, w = frame.image.shape[:2]
    keep = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
    uv, xyz = uv[keep], xyz[keep]
    target = np.full(h * w, np.inf, dtype=np.float64)
    np.minimum.at(target, uv[:, 1] * w + uv[:, 0], xyz[:, 2])
    measured = np.isfinite(target).reshape(h, w)
    target[~np.isfinite(target)] = np.nan
    return target.reshape(h, w), measured
