#!/usr/bin/env python3
"""Convert the R6 assembly STL to a bounded, explicitly estimated web preview.

Only writes the requested JS asset. The CAD reference is a geometry frame, not
livox_frame; the explicit axis assumption and saved IMU observation are retained
in the asset. No ROS TF, device configuration, or calibration is modified.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from export_vehicle_preview import simplify, stl_vertices


def rotation_to_up(up):
    up = up / np.linalg.norm(up)
    target = np.array([0., 0., 1.])
    c = float(up @ target)
    if c <= -.99:
        raise ValueError('Upside-down reference cannot use this leveling estimate')
    x, y, z = np.cross(up, target)
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + skew + skew @ skew / (1 + c)


def export(stl, reference_path, settings_path, observations_path, limit=12000):
    reference = json.loads(reference_path.read_text())
    settings = json.loads(settings_path.read_text())
    if reference['STL_units'] != 'mm' or reference['transform_translation_unit'] != 'm':
        raise ValueError('Expected millimetre mesh and metre transform translation')
    digest = hashlib.sha256(stl.read_bytes()).hexdigest()
    if digest != settings['stl_sha256']:
        raise ValueError('STL does not match the reviewed R6 export')
    cad = stl_vertices(stl) * .001
    if not len(cad) or not np.isfinite(cad).all():
        raise ValueError('Empty or non-finite STL')
    T = np.array(reference['T_assembly_from_lidar'], dtype=float)
    B = np.array(settings['rotation_display_from_assembly'], dtype=float)
    A = np.array(settings['rotation_lidar_cad_from_livox_assumed'], dtype=float)
    if T.shape != (4, 4) or not np.allclose(T[3], [0, 0, 0, 1]):
        raise ValueError('Invalid homogeneous CAD transform')
    for rotation in (T[:3, :3], B, A):
        if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-8) or not np.isclose(np.linalg.det(rotation), 1):
            raise ValueError('Expected right-handed orthonormal rotation')
    # Display origin: horizontal mesh centre, lowest model plane at Z=0.
    # This is deliberately not a physical base_link or a measured ground plane.
    rotated = cad @ B.T
    lo, hi = rotated.min(0), rotated.max(0)
    offset = -(lo + hi) / 2
    offset[2] = -lo[2]
    vertices = rotated + offset
    translation = B @ T[:3, 3] + offset
    cad_rotation = B @ T[:3, :3] @ A
    obs = json.loads(observations_path.read_text())['imu']
    accel = np.array(obs['accel_median'])
    if not (9 < np.linalg.norm(accel) < 11 and max(obs['accel_std']) < .2
            and np.linalg.norm(obs['gyro_median']) < .03):
        raise ValueError('Observation must be approximately stationary')
    up_cad = cad_rotation @ (accel / np.linalg.norm(accel))
    correction = rotation_to_up(up_cad)
    rotation = correction @ cad_rotation
    v, f, cell = simplify(vertices, limit)
    # Rounding and clustering can collapse very small remaining triangles.
    triangles = v[f]
    area = np.linalg.norm(np.cross(triangles[:, 1] - triangles[:, 0],
                                   triangles[:, 2] - triangles[:, 0]), axis=1)
    f = f[area > 1e-10]
    source_paths = (stl, reference_path, settings_path, observations_path)
    return {
        'schema_version': 1, 'model_id': 'r6_full_assembly_20261107',
        'label': 'R6 磁吸附整车', 'estimated': True,
        'source_frame': 'livox_frame', 'display_frame': 'r6_vehicle_preview',
        'translation_m': translation.tolist(), 'rotation': rotation.tolist(),
        'ground_z_m': 0., 'bounds_m': [vertices.min(0).tolist(), vertices.max(0).tolist()],
        'tilt_degrees': float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1)))),
        'stationary_acceleration_m_s2': accel.tolist(),
        'cad_imu_residual_degrees': float(np.degrees(np.arccos(np.clip(up_cad[2], -1, 1)))),
        'rotation_display_from_assembly': B.tolist(),
        'translation_display_from_assembly_m': offset.tolist(),
        'rotation_lidar_cad_from_livox_assumed': A.tolist(),
        'rotation_display_from_livox_cad_estimate': cad_rotation.tolist(),
        'coordinate_reference': reference,
        'assumptions': settings['assumptions'],
        'parts': [{'name': 'r6_full_assembly', 'vertices': v.tolist(), 'faces': f.tolist(),
                   'color': [100, 166, 208], 'cluster_cell_m': cell,
                   'original_triangles': len(cad) // 3}],
        'source_sha256': {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in source_paths},
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for arg in ('stl', 'reference', 'settings', 'observations', 'output'):
        parser.add_argument('--' + arg, required=True, type=Path)
    parser.add_argument('--triangle-limit', type=int, default=12000)
    args = parser.parse_args()
    if args.triangle_limit < 1000:
        parser.error('triangle-limit must be at least 1000')
    model = export(args.stl, args.reference, args.settings, args.observations, args.triangle_limit)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text('/* R6 full assembly; CAD/IMU display estimate, not calibrated extrinsics. */\n'
                          '"use strict";\nwindow.VehiclePreviewModel=Object.freeze('
                          + json.dumps(model, ensure_ascii=False, separators=(',', ':'), allow_nan=False) + ');\n')
    print(json.dumps({key: model[key] for key in ('model_id', 'translation_m', 'bounds_m',
                     'cad_imu_residual_degrees')}, indent=2))
    print('triangles:', len(model['parts'][0]['faces']), 'bytes:', args.output.stat().st_size)


if __name__ == '__main__':
    main()
