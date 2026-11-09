#!/usr/bin/env python3
"""Observe real DDS streams without opening devices or changing their settings."""
import argparse
import json
import math
import os
from pathlib import Path
import time


def summarize_streams(rows, required, elapsed):
    for row in rows.values():
        span = (row['last'] - row['first']) if row['count'] > 1 else 0
        row['observed_hz'] = (row['count'] - 1) / span if span > 0 else 0
        row['fresh_at_end'] = row['last'] is not None and elapsed - row['last'] < 2
        # Include startup and trailing silence: a stream that starts just before
        # the deadline must not pass an otherwise empty observation window.
        row['coverage_fraction'] = span / elapsed if elapsed > 0 else 0
        row['continuous'] = (row['fresh_at_end'] and row['max_gap_s'] < 2
                             and row['count'] > 2 and row['coverage_fraction'] >= .9)
        row['sampled_content_valid'] = bool(row['samples']) and all(
            sample['stamp_ns'] > 0 and abs(sample['age_s']) < 5
            and sample.get('valid_layout', True) for sample in row['samples'])
        row['passed'] = (row['continuous'] and row['sampled_content_valid']
                         and row['sampled_stamp_reversals'] == 0)
    return bool(required) and all(rows.get(t, {}).get('passed', False) for t in required)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, help='Override the system YAML domain for an isolated test')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds < 5:
        parser.error('--seconds must be at least 5')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image
    from eddy_driver.msg import EddyFrame
    import numpy as np
    from inspection_runtime.paths import load_system_config

    config = load_system_config()
    domain = config['ros_domain_id'] if args.domain_id is None else args.domain_id
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    topics = {
        '/camera/front/image_raw': Image,
        '/camera/fisheye/image_raw': Image,
        '/d405/camera/color/image_raw': Image,
        '/d405/camera/depth/image_rect_raw': Image,
        '/d405/camera/infra1/image_rect_raw': Image,
        '/d405/camera/infra2/image_rect_raw': Image,
        '/eddy/data': EddyFrame,
    }
    rclpy.init()
    node = rclpy.create_node('inspection_stream_acceptance')
    started = time.monotonic()
    rows = {topic: {'count': 0, 'bytes': 0, 'first': None, 'last': None,
                    'max_gap_s': 0, 'sampled_stamp_reversals': 0,
                    'last_stamp_ns': None, 'samples': []} for topic in topics}
    subscriptions = []
    for topic, kind in topics.items():
        def receive(raw, topic=topic, kind=kind):
            now = time.monotonic() - started
            row = rows[topic]
            row['count'] += 1
            row['bytes'] += len(raw)
            if row['first'] is None:
                row['first'] = now
            if row['last'] is not None:
                row['max_gap_s'] = max(row['max_gap_s'], now-row['last'])
            row['last'] = now
            # Decode once a second; receiving raw images avoids copying every
            # frame just to count it while the recorder and browser are active.
            if row['samples'] and now-row['samples'][-1]['at_s'] < 1:
                return
            msg = deserialize_message(raw, kind)
            stamp = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            if row['last_stamp_ns'] is not None and stamp < row['last_stamp_ns']:
                row['sampled_stamp_reversals'] += 1
            row['last_stamp_ns'] = stamp
            sample = {'at_s': now, 'stamp_ns': stamp,
                      'age_s': (time.time_ns()-stamp)/1e9}
            if kind is Image:
                pixel_bytes = {'16UC1': 2, 'mono16': 2, 'mono8': 1, '8UC1': 1,
                               'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
                valid_layout = (pixel_bytes is not None and msg.width > 0 and msg.height > 0
                                and msg.step >= msg.width * pixel_bytes
                                and len(msg.data) == msg.step * msg.height)
                dtype = ('>u2' if msg.is_bigendian else '<u2') if msg.encoding in ('16UC1', 'mono16') and len(msg.data) % 2 == 0 else 'u1'
                data = np.frombuffer(msg.data, dtype=dtype)
                sample.update(width=msg.width, height=msg.height,
                              valid_layout=valid_layout,
                              encoding=msg.encoding, payload_bytes=len(msg.data),
                              min=int(data.min()) if data.size else None,
                              max=int(data.max()) if data.size else None,
                              nonzero_fraction=float(np.count_nonzero(data)/data.size) if data.size else 0)
            else:
                sample['channel_mean'] = list(msg.channel_mean)
            row['samples'].append(sample)
        subscriptions.append(node.create_subscription(kind, topic, receive, qos_profile_sensor_data, raw=True))
    try:
        while time.monotonic()-started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        elapsed = time.monotonic()-started
        node.destroy_node()
        rclpy.shutdown()
    required = config['required_topics']
    passed = summarize_streams(rows, required, elapsed)
    report = {'passed': passed, 'duration_s': elapsed, 'ros_domain_id': domain, 'required_topics': required,
              'scope': 'DDS delivery and sampled image content; EMAT and algorithm accuracy are separate',
              'topics': rows}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'passed': passed, 'topics': {t: {k:r[k] for k in ('count','observed_hz','max_gap_s','continuous')} for t,r in rows.items()}}, indent=2))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
