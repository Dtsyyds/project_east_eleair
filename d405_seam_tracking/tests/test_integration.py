"""Real MCAP plugin/CDR and independent DDS recipient, installed adapter, no hardware."""
import json
import os
from pathlib import Path
import subprocess
import time
import numpy as np
import pytest
from d405_seam_tracking.cli import synthetic_frame
from d405_seam_tracking.dataset import extract_mcap, read_sample


def messages(frame):
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics
    image = Image(); image.height, image.width = frame.image.shape
    image.header.frame_id = frame.image_calibration.frame_id
    image.header.stamp.sec, image.header.stamp.nanosec = divmod(frame.image_stamp_ns, 1_000_000_000)
    image.encoding = 'mono8'; image.step = image.width; image.data = frame.image.tobytes()
    depth = Image(); depth.height, depth.width = frame.depth.shape; depth.header = image.header
    depth.encoding = '16UC1'; depth.step = depth.width * 2; depth.data = frame.depth.tobytes()
    info = CameraInfo(); info.header = image.header; info.height, info.width = frame.image.shape
    info.k = [float(v) for v in frame.image_calibration.k]; info.d = frame.image_calibration.d; info.distortion_model = 'none'
    extrinsic = Extrinsics(); extrinsic.rotation = np.eye(3).ravel(order='F').tolist(); extrinsic.translation = [0., 0., 0.]
    return image, depth, info, extrinsic


TOPICS = {'image': '/fixture/camera/infra1/image_rect_raw', 'depth': '/fixture/camera/depth/image_rect_raw',
          'image_info': '/fixture/camera/infra1/camera_info', 'depth_info': '/fixture/camera/depth/camera_info',
          'extrinsics': '/fixture/camera/extrinsics/depth_to_infra1'}


def test_actual_multibag_mcap_roundtrip(tmp_path):
    import rosbag2_py
    from rclpy.serialization import serialize_message, deserialize_message
    from rosidl_runtime_py.utilities import get_message
    paths, original = [], {}
    msgtypes = {'image': 'sensor_msgs/msg/Image', 'depth': 'sensor_msgs/msg/Image', 'image_info': 'sensor_msgs/msg/CameraInfo',
                'depth_info': 'sensor_msgs/msg/CameraInfo', 'extrinsics': 'realsense2_camera_msgs/msg/Extrinsics'}
    for bag in range(2):
        path = tmp_path / f'bag_{bag}'; paths.append(str(path))
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        for role, topic in TOPICS.items(): writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=msgtypes[role], serialization_format='cdr'))
        frame = synthetic_frame(stamp=1_000_000_000 + bag * 100_000_000, sequence=bag)
        image, depth, info, extrinsic = messages(frame)
        for role, message in [('image_info', info), ('depth_info', info), ('extrinsics', extrinsic), ('image', image), ('depth', depth)]:
            raw = serialize_message(message); ts = frame.image_stamp_ns
            writer.write(TOPICS[role], raw, ts); original[(bag, TOPICS[role])] = raw
        del writer
    for bag, path in enumerate(paths):
        reader = rosbag2_py.SequentialReader(); reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
        count = 0
        while reader.has_next():
            import hashlib
            topic, raw, ts = reader.read_next(); assert hashlib.sha256(raw).hexdigest() == hashlib.sha256(original[(bag, topic)]).hexdigest()
            msgtype = next(msgtypes[role] for role, name in TOPICS.items() if name == topic)
            decoded = deserialize_message(raw, get_message(msgtype))
            expected = deserialize_message(original[(bag, topic)], get_message(msgtype))
            # CDR padding is not semantic data; reserialization may differ in padding bytes.
            assert decoded == expected; count += 1
        assert count == 5
    index = extract_mcap(paths, tmp_path / 'extracted', TOPICS, 'fixture_session', .001)
    assert len(index['samples']) == 2 and index['sync_counts']['matched'] == 2
    for r in index['samples']:
        frame = read_sample(tmp_path / 'extracted', r)
        np.testing.assert_array_equal(frame.depth, synthetic_frame().depth)
        assert frame.image_stamp_ns == frame.depth_stamp_ns
    # Source immutable; recompute actual hashes after extraction.
    from d405_seam_tracking.contracts import sha256
    for source in index['sources']: assert sha256(source['path']) == source['sha256']


def test_installed_ros_dds_exact_count_and_exit(tmp_path):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    if os.environ.get('ROS_DOMAIN_ID') != '187': pytest.fail('run fixture with isolated ROS_DOMAIN_ID=187')
    project = Path(__file__).resolve().parents[1]
    executable = project / 'ros2_ws/install/d405_seam_ros/lib/d405_seam_ros/d405_seam_node'
    assert executable.is_file(), 'build installed ROS adapter first'
    rclpy.init(); node = rclpy.create_node('d405_fixture_sender_receiver')
    qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs = {role: node.create_publisher({'image': Image, 'depth': Image, 'image_info': CameraInfo, 'depth_info': CameraInfo, 'extrinsics': Extrinsics}[role], topic, persistent if role == 'extrinsics' else qos) for role, topic in TOPICS.items()}
    results, paths, overlays = [], [], []
    subscriptions = [node.create_subscription(String, '/vision/d405_seam/result', lambda m: results.append(json.loads(m.data)), 20),
                     node.create_subscription(RosPath, '/vision/d405_seam/path_3d', paths.append, 20),
                     node.create_subscription(Image, '/vision/d405_seam/overlay', overlays.append, qos)]
    log = (tmp_path / 'node.log').open('w')
    process = subprocess.Popen([str(executable), '--config', str(project / 'configs/experimental.yaml'), '--camera-prefix', '/fixture/camera',
                                '--depth-scale-m', '.001', '--duration', '6', '--output', str(tmp_path / 'output'), '--overlay'], stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 4
        while not all(p.get_subscription_count() for p in pubs.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert all(p.get_subscription_count() for p in pubs.values())
        time.sleep(.15)
        for i in range(10):
            frame = synthetic_frame(stamp=1_000_000_000 + i * 50_000_000, sequence=i)
            image, depth, info, extrinsic = messages(frame)
            for role, message in [('image_info', info), ('depth_info', info), ('extrinsics', extrinsic), ('image', image), ('depth', depth)]: pubs[role].publish(message)
            until = time.monotonic() + .05
            while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.001)
        deadline = time.monotonic() + 8
        while process.poll() is None and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.02)
        assert process.wait(timeout=1) == 0
        for _ in range(20): rclpy.spin_once(node, timeout_sec=.01)
        assert len(results) == len(paths) == len(overlays) == 10
        assert [r['source']['image_stamp_ns'] for r in results] == [str(1_000_000_000 + i * 50_000_000) for i in range(10)]
        for result, path in zip(results, paths):
            assert result['metric_valid'] and result['valid_2d'] and result['state'] == 'FUSED_CANDIDATE'
            assert result['unit_3d'] == 'metres' and path.header.frame_id == 'synthetic_optical'
            assert len(path.poses) == sum(result['points_3d_valid']) and all(.07 <= p.pose.position.z <= .6 for p in path.poses)
        report = json.loads((tmp_path / 'output/result_manifest.json').read_text())
        assert report['complete'] and report['processed'] == 10 and report['latest_dropped'] == report['invalid'] == 0
        assert report['sync']['matched'] == 10 and report['camera_owners_added'] == 0
    finally:
        if process.poll() is None:
            process.terminate(); process.wait(timeout=3)
        log.close(); node.destroy_node(); rclpy.shutdown()


def test_empty_installed_ros_analysis_never_claims_complete(tmp_path):
    project = Path(__file__).resolve().parents[1]
    executable = project / 'ros2_ws/install/d405_seam_ros/lib/d405_seam_ros/d405_seam_node'
    if os.environ.get('ROS_DOMAIN_ID') != '187': pytest.fail('run fixture with isolated ROS_DOMAIN_ID=187')
    result = subprocess.run([str(executable), '--config', str(project / 'configs/experimental.yaml'), '--camera-prefix', '/empty/camera',
                             '--depth-scale-m', '.001', '--duration', '.1', '--output', str(tmp_path / 'empty')], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    manifest = json.loads((tmp_path / 'empty/result_manifest.json').read_text())
    assert not manifest['complete'] and manifest['processed'] == 0


def test_ros_stride_and_big_endian_raw_contract():
    from sensor_msgs.msg import Image
    from d405_seam_tracking.dataset import image_array
    m = Image(); m.width = 3; m.height = 2; m.step = 4; m.encoding = 'mono8'; m.data = bytes([1, 2, 3, 99, 4, 5, 6, 99])
    np.testing.assert_array_equal(image_array(m), [[1, 2, 3], [4, 5, 6]])
    m.width = 2; m.step = 6; m.encoding = '16UC1'; m.is_bigendian = 1
    m.data = bytes([0, 1, 1, 0, 0, 0, 2, 0, 0, 0, 0, 0])
    values = image_array(m); assert values.dtype == np.uint16
    np.testing.assert_array_equal(values, [[1, 256], [512, 0]])


def test_overlay_owned_payload_exact_bytes_without_pixel_validation():
    from array import array
    from sensor_msgs.msg import Image
    from d405_seam_tracking.ros_node import image_payload
    source = np.arange(12 * 18 * 3, dtype=np.uint8).reshape(12, 18, 3)[:, ::2]
    expected = source.tobytes()
    payload = image_payload(source); assert isinstance(payload, array) and payload.typecode == 'B'
    message = Image(); message.data = payload
    assert message.data is payload and message.data.tobytes() == expected
    source[:] = 0
    assert message.data.tobytes() == expected  # Publication owns immutable frame bytes.


@pytest.mark.parametrize('mode', ['vision_only', 'fused'])
def test_ros_visual_fallback_without_factory_calibration(tmp_path, mode):
    if os.environ.get('ROS_DOMAIN_ID') != '187': pytest.fail('run fixture with isolated ROS_DOMAIN_ID=187')
    import rclpy, yaml
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from rclpy.qos import qos_profile_sensor_data
    project = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((project / 'configs/experimental.yaml').read_text()); config['mode'] = mode
    (tmp_path / 'config.yaml').write_text(yaml.safe_dump(config))
    rclpy.init(); node = rclpy.create_node('uncalibrated_visual_fixture'); received = []
    pub = node.create_publisher(Image, '/uncalibrated/camera/infra1/image_rect_raw', qos_profile_sensor_data)
    dep = node.create_publisher(Image, '/uncalibrated/camera/depth/image_rect_raw', qos_profile_sensor_data)
    sub = node.create_subscription(String, '/vision/d405_seam/result', lambda m: received.append(json.loads(m.data)), 10)
    log = (tmp_path / 'fallback.log').open('w')
    executable = project / 'ros2_ws/install/d405_seam_ros/lib/d405_seam_ros/d405_seam_node'
    process = subprocess.Popen([str(executable), '--config', str(tmp_path / 'config.yaml'), '--depth-scale-m', '.001', '--camera-prefix', '/uncalibrated/camera', '--duration', '4', '--output', str(tmp_path / 'analysis')], stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic() + 2
        while not (pub.get_subscription_count() and node.count_publishers('/vision/d405_seam/result') and dep.get_subscription_count()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        assert pub.get_subscription_count() and node.count_publishers('/vision/d405_seam/result') and dep.get_subscription_count()
        # Wait for both directions of DDS discovery; retain the original 10 Hz rate.
        time.sleep(.15)
        for i in range(3):
            image, depth, _, _ = messages(synthetic_frame(stamp=1_000_000_000 + i * 100_000_000))
            pub.publish(image)
            if mode == 'fused': dep.publish(depth)
            until = time.monotonic() + .1
            while time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.01)
        until = time.monotonic() + 3
        while len(received) < 3 and time.monotonic() < until: rclpy.spin_once(node, timeout_sec=.02)
        assert len(received) == 3
        assert all(r['valid_2d'] and not r['metric_valid'] and r['calibration_status'] == 'unavailable' for r in received)
        assert all(r['state'] == 'VISION_2D' for r in received)
        if mode == 'vision_only': assert all(r['source']['depth_stamp_ns'] is None for r in received)
        else: assert all('calibration_or_depth_unavailable' in r['reasons'] for r in received)
        assert process.wait(timeout=5) == 0
        manifest = json.loads((tmp_path / 'analysis/result_manifest.json').read_text())
        assert manifest['processed'] == 3 and manifest['latest_dropped'] == manifest['invalid'] == 0
    finally:
        if process.poll() is None: process.terminate(); process.wait(timeout=3)
        log.close(); node.destroy_node(); rclpy.shutdown()
