#!/usr/bin/env python3
"""Model-free D405 subscriber. Save only explicitly requested, paired raw inputs."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import yaml
from d405_seam_tracking.contracts import Frame
from d405_seam_tracking.dataset import Synchronizer, from_camera_info, image_array, stamp_ns
from d405_seam_tracking.writer import EventWriter, SampleCapture


def capture_result(frame):
    """Review-compatible metadata, explicitly without predictions or labels."""
    return {'state': 'CAPTURE_ONLY', 'source': frame.metadata(),
            'points_2d_y_left_right': [], 'metric_valid': False,
            'visual': {'reasons': ['capture_only_no_inference'], 'coverage': 0},
            'annotation_status': 'unlabelled', 'model_sha256': None}


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--runtime-mode', choices=['capture'], default='capture')
    parser.add_argument('--stream', choices=['color'], default='color')
    parser.add_argument('--depth-scale-m', type=float, required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--events', required=True)
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text())
    tolerance = float(config['sync_tolerance_ms'])
    if config['algorithm'] != 'capture_only' or not 0 < tolerance <= 50:
        parser.error('invalid capture configuration')
    if not np.isfinite(args.depth_scale_m) or args.depth_scale_m <= 0:
        parser.error('positive depth scale required')

    class CaptureNode(Node):
        def __init__(self):
            super().__init__('d405_capture_node')
            self.sync = Synchronizer(int(tolerance * 1e6), 6)
            self.calibration, self.extrinsics = {}, None
            self.sequence, self.last_stamp, self.error = 0, 0, None
            self.writer = EventWriter(args.events)
            self.capture = SampleCapture(self.writer)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            prefix = '/d405/camera'
            self.owned = [
                self.create_subscription(Image, prefix+'/color/image_raw', lambda m: self.image('image', m), qos),
                self.create_subscription(Image, prefix+'/depth/image_rect_raw', lambda m: self.image('depth', m), qos),
                self.create_subscription(CameraInfo, prefix+'/color/camera_info', lambda m: self.info('image', m), qos),
                self.create_subscription(CameraInfo, prefix+'/depth/camera_info', lambda m: self.info('depth', m), qos),
                self.create_subscription(Extrinsics, prefix+'/extrinsics/depth_to_color', self.extrinsic, persistent)]
            self.diagnostic = self.create_publisher(String, '/vision/d405_seam/diagnostics', 10)
            self.create_service(Trigger, '/vision/d405_seam/save_snapshot', self.save)
            self.create_service(Trigger, '/vision/d405_seam/sample_status', self.status)
            self.create_timer(.5, self.report)

        def info(self, kind, message):
            try:
                self.calibration[kind] = from_camera_info(message, 'live_camera_info:'+kind)
            except ValueError as error:
                self.error = str(error)

        def extrinsic(self, message):
            self.extrinsics = {'rotation': np.array(message.rotation).reshape(3, 3, order='F').ravel().tolist(),
                               'translation_m': list(message.translation), 'source': 'live_factory_depth_to_color'}

        def image(self, kind, message):
            try:
                pairs = self.sync.add(kind, message)
                for image, depth in pairs:
                    if self.capture.poll()['status'] != 'queued':
                        continue
                    if stamp_ns(image) <= self.last_stamp:
                        raise ValueError('duplicate_or_reset_source_time')
                    if depth.encoding.lower() != '16uc1' or image.encoding.lower() not in ('rgb8', 'bgr8'):
                        raise ValueError('expected_rgb_and_uint16_mm_depth')
                    for role, msg in [('image', image), ('depth', depth)]:
                        if role in self.calibration and self.calibration[role].frame_id != msg.header.frame_id:
                            raise ValueError('image_header_calibration_frame_mismatch')
                    rgb, raw_depth = image_array(image), image_array(depth)
                    self.sequence += 1
                    extrinsics = None if self.extrinsics is None else dict(self.extrinsics,
                        from_frame=depth.header.frame_id, to_frame=image.header.frame_id)
                    frame = Frame(rgb, raw_depth, raw_depth > 0, stamp_ns(image), stamp_ns(depth), self.sequence,
                                  self.calibration.get('image'), self.calibration.get('depth'), extrinsics,
                                  args.depth_scale_m, {'session': args.session_id, 'stream': 'color',
                                  'image_frame_id': image.header.frame_id, 'depth_frame_id': depth.header.frame_id,
                                  'depth_unit_source': 'explicit_ros_driver_mm_contract'}, time.time_ns()).validate()
                    self.capture.capture(frame, capture_result(frame), None)
                    self.last_stamp = frame.image_stamp_ns
                    self.error = None
            except (ValueError, RuntimeError) as error:
                self.error = str(error)

        def save(self, request, response):
            try:
                value = self.capture.request()
                # Require inputs received after this request, not buffered old frames.
                self.sync.images.clear(); self.sync.depths.clear()
                response.success, response.message = True, json.dumps(value, ensure_ascii=False)
            except RuntimeError as error:
                response.success, response.message = False, str(error)
            return response

        def status(self, request, response):
            response.success, response.message = True, json.dumps(self.capture.poll(), ensure_ascii=False)
            return response

        def report(self):
            value = {'session_id': args.session_id, 'runtime_mode': 'capture', 'model_sha256': None,
                     'candidate_selection_supported': False, 'sync': dict(self.sync.counts),
                     'sample': self.capture.poll(), 'last_error': self.error, 'writer': self.writer.worker.status}
            message = String(); message.data = json.dumps(value, ensure_ascii=False)
            self.diagnostic.publish(message)

    rclpy.init()
    node = None
    try:
        node = CaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            try:
                node.writer.close(5.)
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
