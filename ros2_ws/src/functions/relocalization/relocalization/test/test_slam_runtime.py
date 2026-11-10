"""Opt-in installed ROS integration tests: isolated domains, no hardware drivers.

source deploy/env.sh s100p
SLAM_ALGORITHM_TEST=1 python3 -m pytest <this file> -q
"""
import itertools
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import pytest
import yaml

pytestmark = pytest.mark.skipif(os.environ.get('SLAM_ALGORITHM_TEST') != '1',
                                reason='requires built ROS packages and local DDS sockets')
domains = itertools.count(181)


@pytest.fixture
def runtime(tmp_path, monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from tf2_msgs.msg import TFMessage
    monkeypatch.setenv('ROS_DOMAIN_ID', str(next(domains)))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('slam_test_observer', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    transforms = []
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(TFMessage, '/tf_static',
        lambda msg: transforms.extend(msg.transforms), qos)
    children = []

    def start(command):
        path = tmp_path / f'process_{len(children)}.log'
        log = path.open('w')
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append((process, log, path))
        return process, path

    def wait(predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        pytest.fail('condition timeout\n' + '\n'.join(path.read_text() for _, _, path in children))

    def pump(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)

    try:
        yield node, transforms, start, wait, pump
    finally:
        for process, log, _ in reversed(children):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
            log.close()
        node.destroy_subscription(subscription)
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def configuration(tmp_path, prior):
    from ament_index_python.packages import get_package_share_directory
    base = Path(get_package_share_directory('fast_livo')) / 'config/mapping_relocation.yaml'
    data = yaml.safe_load(base.read_text())
    p = data['/**']['ros__parameters']
    p.update(locate_in_prior_map=prior, workspace_root=str(tmp_path),
             prior_map_path=str(tmp_path), prior_map_name='missing.pcd')
    p['common'].update(img_en=0, lid_topic='/test/lidar', imu_topic='/test/imu')
    p['time_check']['enabled'] = False
    p['publish']['map_frame_id'] = 'test_map'
    p['pcd_save'].update(pcd_save_en=False, pcd_save_en_relocate=False)
    path = tmp_path / 'config.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


@pytest.mark.parametrize('stop_kind', ['normal', 'core_failure'])
def test_direct_stack_identity_without_prior_and_owned_shutdown(runtime, tmp_path, stop_kind):
    node, transforms, start, wait, pump = runtime
    config = configuration(tmp_path, False)
    process, log = start(['ros2', 'launch', 'relocalization', 'slam.launch.py',
                          f'config_file:={config}'])
    expected = {'laserMapping', 'teaser_gicp_node', 'transform_publisher'}
    wait(lambda: expected.issubset(node.get_node_names()) and bool(transforms))
    alignment = next(t for t in transforms if t.header.frame_id == 'test_map')
    assert alignment.child_frame_id == 'camera_init'
    p, q = alignment.transform.translation, alignment.transform.rotation
    assert (p.x, p.y, p.z, q.x, q.y, q.z, q.w) == (0, 0, 0, 0, 0, 0, 1)
    # A disabled solver must not load a missing prior, receive clouds or wait
    # for a SetInitialPose service that direct mapping never creates.
    subscriptions = node.get_subscriber_names_and_types_by_node('teaser_gicp_node', '/')
    assert not any(name in ('/livox/lidar', '/pointcloud2', '/initialpose') for name, _ in subscriptions)
    assert not node.get_client_names_and_types_by_node('teaser_gicp_node', '/')
    assert 'registration disabled' in log.read_text()
    if stop_kind == 'normal':
        process.send_signal(signal.SIGINT)
    else:
        pid = re.search(r'\[transform_publisher-\d+\]: process started with pid \[(\d+)\]', log.read_text())
        assert pid
        os.kill(int(pid[1]), signal.SIGKILL)
    wait(lambda: process.poll() is not None)
    # SIGKILL prevents DDS unregister; graph entries may outlive the process
    # until the participant lease expires. Verify actual owned PIDs are reaped.
    pids = re.findall(r'process started with pid \[(\d+)\]', log.read_text())
    assert len(pids) == 3
    wait(lambda: all(not Path(f'/proc/{pid}').exists() for pid in pids))
    if stop_kind == 'normal':
        assert process.returncode == 0
    else:
        assert 'exited (-9)' in log.read_text()


def test_prior_alignment_waits_for_valid_final_pose_and_latches(runtime):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, DurabilityPolicy
    node, transforms, start, wait, pump = runtime
    process, _ = start(['ros2', 'run', 'relocalization', 'transform_publisher',
                        '--ros-args', '-p', 'locate_in_prior_map:=true', '-p', 'map_frame_id:=test_map'])
    wait(lambda: 'transform_publisher_node' in node.get_node_names())
    publisher = node.create_publisher(PoseWithCovarianceStamped, '/fast_livo2/initial_map_pose',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    wait(lambda: publisher.get_subscription_count() == 1)
    pump(.3)
    assert not transforms  # Prior mode must never fall back to identity.
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'test_map'
    msg.pose.pose.orientation.w = 0.0
    publisher.publish(msg)  # Zero quaternion is invalid.
    pump(.3)
    assert not transforms
    msg.pose.pose.position.x = 3.0
    msg.pose.pose.orientation.w = 2.0
    publisher.publish(msg)
    wait(lambda: bool(transforms))
    assert transforms[-1].transform.translation.x == 3.0
    assert transforms[-1].transform.rotation.w == 1.0
    count = len(transforms)
    msg.pose.pose.position.x = 7.0
    publisher.publish(msg)
    pump(.3)
    assert len(transforms) == count
    assert process.poll() is None


def test_prior_failure_stops_entire_stack_without_identity(runtime, tmp_path):
    node, transforms, start, wait, pump = runtime
    config = configuration(tmp_path, True)
    process, log = start(['ros2', 'launch', 'relocalization', 'slam.launch.py',
                          f'config_file:={config}', 'start_delay:=0'])
    wait(lambda: process.poll() is not None)
    assert not transforms
    assert 'process has died' in log.read_text()
    wait(lambda: not {'laserMapping', 'teaser_gicp_node', 'transform_publisher'}.intersection(node.get_node_names()))


def test_prior_success_keeps_stack_alive_through_tracking_and_save(runtime, tmp_path):
    """Real solver/service/IMU/TF path, with synthetic static scans and GICP.

    The identical prior and scan deliberately avoid testing global-registration
    accuracy: this regression exercises the post-registration lifecycle.
    """
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from nav_msgs.msg import Odometry
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from std_msgs.msg import Bool

    node, transforms, start, wait, pump = runtime
    grid = [i * .1 for i in range(-15, 16)]
    points = [(3., a, b) for a in grid for b in grid]
    points += [(a, 4., b) for a in grid for b in grid]
    points += [(a, b, -2.) for a in grid for b in grid]
    prior = tmp_path / 'prior.pcd'
    prior.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                     f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
                     f'POINTS {len(points)}\nDATA ascii\n' +
                     ''.join(f'{x} {y} {z}\n' for x, y, z in points))
    config = configuration(tmp_path, True)
    data = yaml.safe_load(config.read_text())
    params = data['/**']['ros__parameters']
    params['prior_map_name'] = prior.name
    params['common']['lid_topic'] = '/livox/lidar'
    params['extrin_calib']['extrinsic_T'] = [0., 0., 0.]
    params['relocalization'].update(converged_count_thre=3,
        initial_pose_timeout_seconds=2., timeout_seconds=20.)
    output = tmp_path / 'saved'
    params['pcd_save'].update(pcd_save_en=True, pcd_save_en_relocate=True,
        output_dir_relocate=str(output), interval=-1)
    config.write_text(yaml.safe_dump(data))

    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    initial, diagnostic, odometry, valid = [], [], [], []
    node.create_subscription(PoseWithCovarianceStamped, '/fast_livo2/initial_map_pose', initial.append, durable)
    node.create_subscription(PoseWithCovarianceStamped, '/icp_result', diagnostic.append, durable)
    node.create_subscription(Odometry, '/aft_mapped_in_map', odometry.append, qos_profile_sensor_data)
    node.create_subscription(Bool, '/fast_livo2/tracking_valid', valid.append, durable)
    lidar = node.create_publisher(CustomMsg, '/livox/lidar', qos_profile_sensor_data)
    imu = node.create_publisher(Imu, '/test/imu', qos_profile_sensor_data)
    scan = CustomMsg()
    scan.header.frame_id = 'livox_frame'
    scan.point_num = len(points)
    scan.points = [CustomPoint(x=x, y=y, z=z, reflectivity=10, tag=0x10,
        line=i % 4, offset_time=int(i * 100000000 / len(points)))
        for i, (x, y, z) in enumerate(points)]

    def publish_scan():
        stamp = node.get_clock().now().nanoseconds - 100000000
        scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp, 1000000000)
        scan.timebase = stamp
        lidar.publish(scan)

    def publish_imu():
        msg = Imu()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.linear_acceleration.z = 9.80665
        imu.publish(msg)

    scan_timer = node.create_timer(.1, publish_scan)
    imu_timer = node.create_timer(.005, publish_imu)
    process, log = start(['ros2', 'launch', 'relocalization', 'slam.launch.py',
                          f'config_file:={config}', 'start_delay:=0', 'algorithm:=gicp'])
    wait(lambda: 'node remains idle until shutdown' in log.read_text(), timeout=25)
    wait(lambda: bool(initial) and bool(transforms) and len(odometry) >= 3 and
         any(msg.data for msg in valid), timeout=25)
    count = len(odometry)
    # Stay beyond the delivery deadline; neither a retry nor a late queued
    # manual initial pose may restart the solver or tear down the launch.
    manual = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    msg = PoseWithCovarianceStamped()
    msg.pose.pose.position.x = 100.
    msg.pose.pose.orientation.w = 1.
    manual.publish(msg)
    pump(2.5)
    assert process.poll() is None, log.read_text()
    assert len(odometry) > count
    assert len(diagnostic) == 1
    assert len(initial) == 1
    assert log.read_text().count('Mapping confirmed initial pose acceptance:') == 1
    assert not node.get_client_names_and_types_by_node('teaser_gicp_node', '/')
    subscriptions = node.get_subscriber_names_and_types_by_node('teaser_gicp_node', '/')
    assert not any(name in ('/livox/lidar', '/pointcloud2', '/initialpose') for name, _ in subscriptions)
    expected = {'laserMapping', 'teaser_gicp_node', 'transform_publisher'}
    assert expected.issubset(node.get_node_names())
    process.send_signal(signal.SIGINT)
    wait(lambda: process.poll() is not None)
    scan_timer.cancel()
    imu_timer.cancel()
    assert process.returncode == 0, log.read_text()
    pids = re.findall(r'process started with pid \[(\d+)\]', log.read_text())
    assert len(pids) == 3
    wait(lambda: all(not Path(f'/proc/{pid}').exists() for pid in pids))
    saved = list(output.glob('*.pcd'))
    assert saved, log.read_text()
    assert all(int(re.search(rb'POINTS (\d+)', path.read_bytes())[1]) > 0 for path in saved)
