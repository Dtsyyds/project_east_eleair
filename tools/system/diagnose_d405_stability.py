#!/usr/bin/env python3
"""Read-only D405 DDS soak: per-frame timing CSV and periodic health snapshots.

Does not own/reset the camera. Use the normal recording workflow for raw MCAP.
The caller supplies the ROS environment and must not reset devices during a trial.
"""
import argparse
import csv
import gzip
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=int, default=1800)
    parser.add_argument('--streams', choices=('pair', 'four'), default='four')
    parser.add_argument('--max-gap', type=float, default=2.)
    parser.add_argument('--expected-fps', type=float, default=15.)
    args = parser.parse_args()
    if args.seconds < 10 or not 0 < args.max_gap < 30 or not 0 < args.expected_fps <= 120:
        parser.error('seconds >= 10, 0 < max-gap < 30 and 0 < expected-fps <= 120 required')
    args.output.mkdir(parents=True, exist_ok=False)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    topics = {'color': '/d405/camera/color/image_raw',
              'depth': '/d405/camera/depth/image_rect_raw'}
    if args.streams == 'four':
        topics.update(infra1='/d405/camera/infra1/image_rect_raw',
                      infra2='/d405/camera/infra2/image_rect_raw')
    rows = {k: dict(count=0, bytes=0, first=None, last=None, stamp_ns=None,
                    repeats=0, reversals=0, invalid_layout=0, max_gap_s=0.,
                    max_stamp_gap_s=0., max_abs_source_age_s=0.) for k in topics}
    rclpy.init()
    node = rclpy.create_node('d405_stability_soak')
    start = time.monotonic()
    started_wall = time.time_ns()
    subscriptions = []
    reason = 'interrupted'
    frame_file = gzip.open(args.output/'frames.csv.gz', 'wt')
    writer = csv.writer(frame_file)
    writer.writerow(('stream', 'receive_monotonic_s', 'receive_wall_ns', 'stamp_ns',
                     'width', 'height', 'encoding', 'bytes'))
    status_file = (args.output/'health.jsonl').open('w')

    def receive(key):
        def callback(msg):
            now = time.monotonic()-start
            wall = time.time_ns()
            stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            row = rows[key]
            row['count'] += 1
            row['bytes'] += len(msg.data)
            if row['first'] is None:
                row['first'] = now
            if row['last'] is not None:
                row['max_gap_s'] = max(row['max_gap_s'], now-row['last'])
                row['repeats'] += int(stamp == row['stamp_ns'])
                row['reversals'] += int(stamp < row['stamp_ns'])
                row['max_stamp_gap_s'] = max(row['max_stamp_gap_s'], (stamp-row['stamp_ns'])/1e9)
            pixel_bytes = {'rgb8': 3, 'bgr8': 3, 'mono8': 1, '8UC1': 1,
                           '16UC1': 2, 'mono16': 2}.get(msg.encoding)
            valid = (pixel_bytes is not None and msg.width > 0 and msg.height > 0
                     and msg.step >= msg.width*pixel_bytes
                     and len(msg.data) == msg.step*msg.height and stamp > 0)
            row['invalid_layout'] += int(not valid)
            row['max_abs_source_age_s'] = max(row['max_abs_source_age_s'], abs(wall-stamp)/1e9)
            row.update(last=now, stamp_ns=stamp)
            writer.writerow((key, round(now, 6), wall, stamp, msg.width, msg.height,
                             msg.encoding, len(msg.data)))
        return callback

    def snapshot():
        elapsed = time.monotonic()-start
        return {'wall_ns': time.time_ns(), 'elapsed_s': elapsed,
                'streams': {k: dict(r, age_s=None if r['last'] is None else elapsed-r['last'],
                    rate_hz=(r['count']-1)/(r['last']-r['first'])
                    if r['count'] > 1 and r['last'] > r['first'] else 0.) for k, r in rows.items()}}

    def report():
        value = snapshot()
        status_file.write(json.dumps(value)+'\n'); status_file.flush(); frame_file.flush()
        print(json.dumps(value), flush=True)

    for key, topic in topics.items():
        subscriptions.append(node.create_subscription(Image, topic, receive(key), qos_profile_sensor_data))
    next_report = 10
    try:
        while rclpy.ok() and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()-start
            if now >= next_report:
                report(); next_report += 10
            if any(r['last'] is not None and now-r['last'] > args.max_gap for r in rows.values()):
                reason = 'source_stalled'; break
            if now > 15 and any(r['last'] is None for r in rows.values()):
                reason = 'required_stream_missing'; break
            if any(r['repeats'] or r['reversals'] or r['invalid_layout'] for r in rows.values()):
                reason = 'invalid_frame_or_timestamp'; break
        else:
            reason = 'duration_completed' if rclpy.ok() else 'ros_shutdown'
    except KeyboardInterrupt:
        reason = 'operator_interrupted'
    finally:
        report()
        summary = snapshot()
        summary.update(started_wall_ns=started_wall, requested_seconds=args.seconds,
                       stop_reason=reason, topics=topics, expected_fps=args.expected_fps,
                       raw_recording='separate_normal_MCAP_workflow')
        summary['passed'] = (reason == 'duration_completed' and all(
            r['count'] > 1 and r['last']-r['first'] >= args.seconds*.9
            and summary['elapsed_s']-r['last'] < args.max_gap
            and r['max_gap_s'] < args.max_gap and not r['repeats']
            and not r['reversals'] and not r['invalid_layout']
            and r['max_abs_source_age_s'] < 5
            and (r['count']-1)/(r['last']-r['first']) >= args.expected_fps*.8
            for r in rows.values()))
        (args.output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
        frame_file.close(); status_file.close()
        node.destroy_node(); rclpy.shutdown()
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
