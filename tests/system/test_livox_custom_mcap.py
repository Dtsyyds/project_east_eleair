"""Real ROS recorder round trip; run in an isolated ROS_DOMAIN_ID."""
import json
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
import rosbag2_py
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message, deserialize_message
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from inspection_operations.replay_worker import Playback
from inspection_tools.validate_bag import validate

TOPIC = '/livox/lidar'


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    return reader


def fields(msg):
    # CDR alignment bytes are unspecified; compare every defined field instead.
    # Packing each float also compares signed zero and our NaN/Inf bit patterns.
    return (msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id,
            msg.timebase, msg.point_num, msg.lidar_id, tuple(msg.rsvd),
            b''.join(struct.pack('<IfffBBB', p.offset_time, p.x, p.y, p.z,
                                 p.reflectivity, p.tag, p.line) for p in msg.points))


def test_ros_recorder_preserves_every_custom_field_and_replays(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0'), 'Use an isolated DDS domain'
    bag = tmp_path / 'custom_bag'
    rclpy.init()
    node = rclpy.create_node('custommsg_recording_fixture')
    pub = node.create_publisher(CustomMsg, TOPIC, qos_profile_sensor_data)
    process = None
    expected = {}
    try:
        with (tmp_path / 'recorder.log').open('w') as log:
            process = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'mcap', '-o', str(bag), TOPIC],
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 12
        while not pub.get_subscription_count() and time.monotonic() < deadline:
            assert process.poll() is None, (tmp_path / 'recorder.log').read_text()
            rclpy.spin_once(node, timeout_sec=.05)
        assert pub.get_subscription_count(), (tmp_path / 'recorder.log').read_text()
        time.sleep(.3)
        for frame in range(8):
            msg = CustomMsg(timebase=1800000000000000123 + frame * 100000000, lidar_id=7, rsvd=[1, 2, 3],
                            point_num=6001, points=[CustomPoint(
                                offset_time=i * 1000, x=float(i) / 16, y=-float(i) / 8, z=1.25,
                                reflectivity=i % 256, tag=(i * 3) % 256, line=i % 4) for i in range(6001)])
            msg.points[3].x = float('nan')
            msg.points[6].z = float('inf')
            msg.header.frame_id = 'livox_frame'
            msg.header.stamp.sec = 1800000000
            msg.header.stamp.nanosec = 123 + frame * 100000000
            expected[msg.header.stamp.nanosec] = serialize_message(msg)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=.15)
        time.sleep(.5)
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
        assert (bag / 'metadata.yaml').is_file()
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        node.destroy_node(); rclpy.shutdown()

    reader = read(bag)
    assert {x.name: x.type for x in reader.get_all_topics_and_types()} == {TOPIC: 'livox_ros_driver2/msg/CustomMsg'}
    seen = set()
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg = deserialize_message(data, CustomMsg)
        assert msg.point_num == len(msg.points) == 6001
        assert fields(msg) == fields(deserialize_message(expected[msg.header.stamp.nanosec], CustomMsg))
        seen.add(msg.header.stamp.nanosec)
    assert seen == set(expected)
    report = validate(bag, required_topics=[TOPIC])
    assert report['valid'], report['errors']
    packets = []
    Playback(bag, 'custom-roundtrip', 2., packets.append).run()
    previews = [p['data'] for p in packets if p['kind'] == 'sensor']
    assert previews and all(p['data']['kind'] == 'cloud' for p in previews)
    for packet in previews:
        msg = deserialize_message(expected[int(packet['stamp_ns']) % 10**9], CustomMsg)
        assert packet['data']['timebase'] == str(msg.timebase)
        assert packet['data']['point_count'] == 6001
        assert 0 < packet['data']['displayed_points'] <= 5000
        assert packet['frame_id'] == msg.header.frame_id
    (tmp_path / 'roundtrip_report.json').write_text(json.dumps({
        'frames': len(seen), 'points_per_frame': 6001, 'all_defined_fields_equal': True,
        'validation': report, 'replay_previews': len(previews)}, indent=2))


@pytest.mark.parametrize('kind', ['legacy', 'unsupported', 'invalid_custom'])
def test_mcap_type_dispatch_for_legacy_and_errors(tmp_path, kind):
    bag = tmp_path / kind
    if kind == 'legacy':
        msg = PointCloud2(height=1, width=1, point_step=20, row_step=20,
                         fields=[PointField(name=n, offset=i*4, datatype=7, count=1)
                                 for i, n in enumerate(['x', 'y', 'z', 'intensity', 'time_offset'])],
                         data=struct.pack('<fffff', 1., 2., 3., 40., 0.))
        msg.header.stamp.sec = 1800000000
        type_name = 'sensor_msgs/msg/PointCloud2'
    elif kind == 'invalid_custom':
        msg = CustomMsg(timebase=1, point_num=2, points=[CustomPoint(x=1.)])
        msg.header.stamp.sec = 1800000000
        type_name = 'livox_ros_driver2/msg/CustomMsg'
    else:
        msg = String(data='not a cloud'); type_name = 'std_msgs/msg/String'
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    writer.create_topic(rosbag2_py.TopicMetadata(name=TOPIC, type=type_name, serialization_format='cdr'))
    writer.write(TOPIC, serialize_message(msg), 1800000000000000123)
    del writer
    report = validate(bag, required_topics=[TOPIC])
    assert report['valid'] == (kind == 'legacy'), report['errors']
    packets = []; Playback(bag, kind, 1., packets.append).run()
    preview = next(p['data']['data'] for p in packets if p['kind'] == 'sensor')
    if kind == 'legacy':
        assert preview['xyz'] == [1., 2., 3.]
    else:
        assert preview['kind'] == 'error'
        assert ('unsupported lidar message type' if kind == 'unsupported' else 'point_num') in preview['message']
