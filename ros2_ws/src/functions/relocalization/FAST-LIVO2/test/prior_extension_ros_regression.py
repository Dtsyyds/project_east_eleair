#!/usr/bin/env python3
"""Real DDS/process checks for prior extension and failure-time saving; synthetic inputs only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import sys
import tempfile

os.environ.setdefault("ROS_DOMAIN_ID", "191")
os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ultra-extension-ros-logs")
sys.dont_write_bytecode = True
import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from ros_regression import Runner


def cluster(index):
    points = []
    x = 1.1 + 2 * index
    for a in range(20):
        for b in range(20):
            points.extend(((x+a*.015, 1.1+b*.015, 1.1),
                           (x+.6, 1.1+a*.015, 1.1+b*.015),
                           (x+a*.015, 1.7, 1.1+b*.015)))
    return points


def saved_points(file):
    data = file.read_bytes()
    header, payload = data.split(b"DATA binary\n", 1)
    fields = re.search(rb"FIELDS (.+)", header).group(1).split()
    sizes = list(map(int, re.search(rb"SIZE (.+)", header).group(1).split()))
    counts = list(map(int, re.search(rb"COUNT (.+)", header).group(1).split()))
    step = sum(a*b for a, b in zip(sizes, counts))
    total = int(re.search(rb"POINTS (\d+)", header).group(1))
    assert len(payload) == total * step
    offset = sum(a*b for a, b in zip(sizes[:fields.index(b"x")], counts[:fields.index(b"x")]))
    return total, [struct.unpack_from("<f", payload, i*step+offset)[0] for i in range(total)]


def scenario(runner, extend):
    directory = runner.temporary / "workspace with spaces" / ("extension" if extend else "fixed")
    directory.mkdir(parents=True)
    source = directory / "prior.pcd"
    runner.write_map(source, cluster(0))
    original = hashlib.sha256(source.read_bytes()).digest()
    parameters = runner.base()
    parameters.update({
        "locate_in_prior_map": True, "prior_map_update_en": extend,
        "workspace_root": str(directory), "prior_map_path": ".", "prior_map_name": source.name,
        "uav.gravity_align_en": False, "publish.dense_map_en": True,
        "preprocess.filter_size_map": .01, "preprocess.filter_size_surf": .01,
        "lio.voxel_size": .5, "lio.min_eigen_value": .0025, "lio.max_layer": 2,
        "lio.beam_err": .05, "lio.dept_err": .02, "lio.sigma_num": 3.,
        "pcd_save.pcd_save_en": True, "pcd_save.pcd_save_en_relocate": True,
        "pcd_save.output_dir_relocate": "output", "pcd_save.interval": -1})
    valid, poses, clouds, initial, global_pose = [], [], [], [], []
    subscriptions = [
        runner.node.create_subscription(Bool, "/fast_livo2/tracking_valid", valid.append, runner.qos),
        runner.node.create_subscription(Odometry, "/aft_mapped_to_init", poses.append, 100),
        runner.node.create_subscription(PointCloud2, "/cloud_registered", clouds.append, 100),
        runner.node.create_subscription(PoseWithCovarianceStamped, "/fast_livo2/initial_map_pose", initial.append, runner.qos),
        runner.node.create_subscription(Odometry, "/aft_mapped_in_map", global_pose.append, 100)]
    transform, _ = runner.start("relocalization", "transform_publisher", {})
    process, log = runner.start("fast_livo", "fastlivo_mapping", parameters)
    try:
        runner.spin(.6)
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.
        runner.check(runner.set_pose(pose).accepted, "extension flag does not change initial-pose service acceptance")
        runner.cloud_points = cluster(0)
        runner.feed(1.5)
        runner.check(process.poll() is None and len(poses)>3 and valid[-1].data, "prior initialization reaches valid tracking")
        runner.check(len(initial)==1 and runner.until(lambda: bool(global_pose)), "final initial alignment and global odometry are delivered")
        runner.cloud_points = cluster(0) + cluster(1)
        runner.feed(.7)
        runner.check(process.poll() is None and valid[-1].data, "a partially unknown scan still tracks against old geometry")
        if extend:
            for step in range(1, 4):
                before = len(poses)
                runner.cloud_points = cluster(step) + cluster(step+1)
                runner.feed(.6)
                runner.check(process.poll() is None and valid[-1].data and len(poses)>before+2,
                             "DDS tracking continues without original prior overlap, stage "+str(step))
            runner.check(len(initial)==1 and global_pose[-1].header.frame_id=="map", "extension retains the original alignment and global frame")
            runner.cloud_points = cluster(100) + cluster(101)
        else:
            runner.cloud_points = cluster(1) + cluster(2)
        runner.feed(.9)
        runner.check(runner.until(lambda: process.poll() is not None), "constraint-loss protection terminates the mapper")
        runner.spin(.2)
        runner.check(process.returncode==1 and valid and not valid[-1].data, "failed tracking remains invalid and returns exit code 1")
        runner.check("lost constraints" in log.read_text() and "effective_points=" in log.read_text(), "failure log retains cause, mode and constraint count")
        maps = list((directory / "output").glob("*.pcd"))
        runner.check(len(maps)==1, "exceptional exit writes the accepted observation map")
        count, x = saved_points(maps[0])
        runner.check(count==sum(cloud.width*cloud.height for cloud in clouds), "saved point count equals only the successfully published scans")
        runner.check(max(x)<20., "the distant invalid scan was not saved")
        if extend:
            runner.check(max(x)>9., "saved observations contain newly explored geometry")
        metadata = yaml.safe_load(maps[0].with_name(maps[0].stem+"_metadata.yaml").read_text())
        runner.check(metadata["coordinate_frame"]=="map" and metadata["prior_map_update_en"]==extend,
                     "saved metadata records original-map coordinates and extension mode")
        runner.check(hashlib.sha256(source.read_bytes()).digest()==original, "input prior file is unchanged")
    finally:
        runner.stop(process)
        runner.stop(transform)
        for subscription in subscriptions:
            runner.node.destroy_subscription(subscription)
        runner.spin(.3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--install", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    rclpy.init()
    with tempfile.TemporaryDirectory(prefix="ultra-extension-ros-") as temporary:
        runner = Runner(args.install, Path(temporary))
        try:
            scenario(runner, False)
            scenario(runner, True)
            report = {"passed": len(runner.results), "checks": runner.results}
            if args.report:
                args.report.write_text(json.dumps(report, indent=2))
            print("PASS", len(runner.results), "prior extension DDS checks")
        except BaseException:
            for _, _, log in runner.processes:
                print(log.read_text(), file=sys.stderr)
            raise
        finally:
            runner.close()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
