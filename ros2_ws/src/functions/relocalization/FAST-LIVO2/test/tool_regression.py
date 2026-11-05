#!/usr/bin/env python3
"""Offline regressions for pose conversion, ROS2 argument handling and output paths."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[2]

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


class Tools(unittest.TestCase):
    def test_ros2_arguments_are_literal(self):
        ros=module("ros_helper",ROOT/"rpg_vikit/vikit_py/src/vikit_py/ros_node.py")
        node=ros.RosNode("package","executable")
        with patch.object(ros.subprocess,"run") as run:
            node.run({"nested":{"enabled":True,"name":"literal $(touch /tmp/not-run)"}})
        command=run.call_args.args[0]
        self.assertEqual(command[:5],["ros2","run","package","executable","--ros-args"])
        self.assertIn("nested.enabled:=true",command)
        self.assertTrue(any("$(touch /tmp/not-run)" in arg for arg in command))
        self.assertNotIn("shell",run.call_args.kwargs)

    def test_cpu_info_is_text(self):
        cpu=module("cpu_helper",ROOT/"rpg_vikit/vikit_py/src/vikit_py/cpu_info.py")
        self.assertTrue(isinstance(cpu.get_cpu_info(),str) and cpu.get_cpu_info())

    def test_colmap_preserves_existing_files(self):
        script=ROOT/"FAST-LIVO2/scripts/colmap_output.sh"
        with tempfile.TemporaryDirectory() as folder:
            output=Path(folder)/"space in path"
            subprocess.run(["bash",str(script),str(output)],check=True,capture_output=True)
            keep=output/"images/keep.txt";keep.write_text("keep")
            subprocess.run(["bash",str(script),str(output)],check=True,capture_output=True)
            self.assertEqual(keep.read_text(),"keep")

    def test_mesh_rejects_missing_pose_rows(self):
        mesh=module("mesh_helper",ROOT/"FAST-LIVO2/scripts/mesh.py")
        with tempfile.TemporaryDirectory() as folder:
            poses=Path(folder)/"poses.txt";poses.write_text("0 0 0 1 0 0 0\n")
            with self.assertRaises(ValueError):mesh.load_poses(poses,2)

    def test_mesh_rejects_zero_quaternion(self):
        mesh=module("mesh_helper",ROOT/"FAST-LIVO2/scripts/mesh.py")
        with tempfile.TemporaryDirectory() as folder:
            poses=Path(folder)/"poses.txt";poses.write_text("0 0 0 0 0 0 0\n")
            with self.assertRaises(ValueError):mesh.load_poses(poses,1)

    def test_mesh_world_points_and_sensor_origin(self):
        # Exercise the complete wrapper with real Open3D I/O and a tiny fake TSDF backend.
        # Actual VDBFusion reconstruction is optional and is not claimed by this test.
        import numpy as np
        import open3d as o3d
        mesh=module("mesh_helper",ROOT/"FAST-LIVO2/scripts/mesh.py")
        calls=[]
        class Volume:
            def __init__(self,**kwargs):pass
            def integrate(self,points,origin):calls.append((points.copy(),origin.copy()))
            def extract_triangle_mesh(self,**kwargs):
                return np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.]]),np.array([[0,1,2]])
        with tempfile.TemporaryDirectory() as folder:
            source=Path(folder)
            for name,point in [("scan_0002.pcd",[0.,1.,0.]),("scan_0001.pcd",[1.,0.,0.])]:
                cloud=o3d.geometry.PointCloud();cloud.points=o3d.utility.Vector3dVector([point])
                self.assertTrue(o3d.io.write_point_cloud(str(source/name),cloud))
            poses=source/"poses.txt";poses.write_text("10 20 30 1 0 0 0\n0 0 0 .7071067811865476 0 0 .7071067811865476\n")
            output=source/"nested/mesh.ply"
            with patch.dict(sys.modules,{"vdbfusion":types.SimpleNamespace(VDBVolume=Volume)}):
                mesh.main([str(source),"--poses",str(poses),"--input-frame","body","--output",str(output)])
            self.assertTrue(output.is_file())
            np.testing.assert_allclose(calls[0][0],[[11,20,30]])
            np.testing.assert_allclose(calls[0][1],[10,20,30])
            np.testing.assert_allclose(calls[1][0],[[-1,0,0]],atol=1e-12)
            self.assertEqual(calls[1][1].shape,(3,))

    def test_depth_errors_are_defined(self):
        depth=module("depth_helper",ROOT/"rpg_vikit/vikit_py/src/vikit_py/depthmap_utils.py")
        with self.assertRaises(depth.MapIOError):depth.load_depthmap("invalid.extension",1,1)


if __name__=="__main__":unittest.main()
