#!/usr/bin/env python3
"""Fuse sorted PCD scans with explicit world-frame sensor poses."""
import argparse
from pathlib import Path

def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--poses", type=Path, required=True, help="One x y z qw qx qy qz row per PCD")
    parser.add_argument("--input-frame", choices=["world", "body"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--voxel-size", type=float, default=0.02)
    parser.add_argument("--sdf-trunc", type=float, default=0.1)
    parser.add_argument("--min-weight", type=float, default=0.1)
    parser.add_argument("--preview", action="store_true")
    return parser

def load_poses(path, count):
    import numpy as np
    from scipy.spatial.transform import Rotation
    poses = np.loadtxt(path, ndmin=2)
    if poses.shape != (count, 7) or not np.isfinite(poses).all():
        raise ValueError("Expected one finite x y z qw qx qy qz row per PCD")
    if (np.linalg.norm(poses[:, 3:], axis=1) < 1e-12).any():
        raise ValueError("Pose quaternion cannot be zero")
    return poses[:, :3], Rotation.from_quat(poses[:, [4, 5, 6, 3]]).as_matrix()

def main(argv=None):
    parser = arguments()
    args = parser.parse_args(argv)
    import math
    if any(not math.isfinite(v) or v <= 0 for v in [args.voxel_size, args.sdf_trunc, args.min_weight]):
        parser.error("Volume parameters must be finite and positive")
    files = sorted(args.source.glob("*.pcd"))
    if not files: parser.error("No PCD scans found")
    if not args.poses.is_file(): parser.error("Pose file does not exist")
    import numpy as np
    import open3d as o3d
    import vdbfusion
    from scipy.spatial import cKDTree
    origins, rotations = load_poses(args.poses, len(files))
    volume = vdbfusion.VDBVolume(voxel_size=args.voxel_size, sdf_trunc=args.sdf_trunc)
    colored_points, colors = [], []
    for index, file in enumerate(files):
        cloud = o3d.io.read_point_cloud(str(file))
        points = np.asarray(cloud.points, dtype=np.float64)
        if points.size == 0 or not np.isfinite(points).all():
            raise ValueError(f"Empty/non-finite PCD: {file}")
        if args.input_frame == "body": points = points @ rotations[index].T + origins[index]
        # VDBFusion expects world points and a THREE-dimensional world sensor origin.
        volume.integrate(np.ascontiguousarray(points), np.ascontiguousarray(origins[index]))
        if cloud.has_colors():
            colored_points.append(points.copy()); colors.append(np.asarray(cloud.colors).copy())
    vertices, triangles = volume.extract_triangle_mesh(min_weight=args.min_weight)
    if len(vertices) == 0: raise ValueError("Fusion produced an empty mesh")
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(vertices), o3d.utility.Vector3iVector(triangles))
    if colored_points:
        _, indices = cKDTree(np.vstack(colored_points)).query(vertices)
        mesh.vertex_colors = o3d.utility.Vector3dVector(np.vstack(colors)[indices])
    mesh.compute_vertex_normals()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_triangle_mesh(str(args.output), mesh): raise RuntimeError(f"Cannot write mesh: {args.output}")
    if args.preview: o3d.visualization.draw_geometries([mesh])

if __name__ == "__main__": main()
