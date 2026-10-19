"""Passive ROS adapter: no camera owner, control, or production launch modifications."""
import argparse
import json
from pathlib import Path
import time
from array import array
import numpy as np
from .contracts import Frame, dump, load_config
from .dataset import Synchronizer, from_camera_info, image_array, stamp_ns
from .pipeline import Pipeline, overlay


def image_payload(view):
    # Humble Image.data validates every element twice when assigned bytes.
    # An owned array('B') keeps the same bytes and uses its constant-time type check.
    payload = array('B')
    payload.frombytes(memoryview(np.ascontiguousarray(view)))
    return payload


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.utilities import remove_ros_args
    from sensor_msgs.msg import Image, CameraInfo
    from realsense2_camera_msgs.msg import Extrinsics
    from std_msgs.msg import String
    from nav_msgs.msg import Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import Trigger
    parser = argparse.ArgumentParser(description='Passive D405 fusion subscriber, experimental output only')
    parser.add_argument('--config', required=True)
    parser.add_argument('--stream', choices=('infra1', 'color'), default='infra1')
    parser.add_argument('--namespace', default='/vision/d405_seam')
    parser.add_argument('--camera-prefix', default='/d405/camera')
    parser.add_argument('--depth-scale-m', type=float, required=True)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--output', help='New analysis directory. Lightweight results only unless save requested.')
    parser.add_argument('--events', help='New event directory, only explicit save_snapshot requests copy raw inputs')
    parser.add_argument('--overlay', action='store_true')
    args = parser.parse_args(remove_ros_args(args=argv)[1:] if argv is not None else remove_ros_args()[1:])
    config = load_config(args.config)
    if args.depth_scale_m <= 0: parser.error('positive explicit depth scale required')
    rclpy.init(args=argv)

    class SeamNode(Node):
        def __init__(self):
            super().__init__('d405_seam_node')
            self.pipeline = Pipeline(config)
            self.sync = Synchronizer(int(config['sync_tolerance_ms'] * 1e6), 6)
            self.calibration, self.extrinsics, self.latest = {}, None, None
            self.sequence = self.processed = self.latest_dropped = self.invalid = self.missing_calibration = 0
            self.abandoned = 0
            self.image_only_work_items = 0
            self.errors, self.states, self.timings, self.ages = {}, {}, [], []
            self.callback_timings, self.overlay_timings = [], []
            self.overlay_published = 0
            self.started = time.monotonic()
            self.save_requested = False
            self.output = None
            if args.output:
                p = Path(args.output); p.mkdir(parents=True, exist_ok=False)
                self.output = (p / 'results.jsonl').open('x')
                dump(p / 'result_manifest.json', {'complete': False, 'schema': 'd405_ros_analysis_v1'})
            self.writer = None
            if args.events:
                from .writer import EventWriter
                self.writer = EventWriter(args.events)
            qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            persistent = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            prefix = args.camera_prefix.rstrip('/')
            image_topic = prefix + ('/color/image_raw' if args.stream == 'color' else '/infra1/image_rect_raw')
            self.subscriptions_owned = [
                self.create_subscription(Image, image_topic, lambda m: self.image('image', m), qos),
                self.create_subscription(Image, prefix + '/depth/image_rect_raw', lambda m: self.image('depth', m), qos),
                self.create_subscription(CameraInfo, prefix + '/' + args.stream + '/camera_info', lambda m: self.info('image', m), qos),
                self.create_subscription(CameraInfo, prefix + '/depth/camera_info', lambda m: self.info('depth', m), qos),
                self.create_subscription(Extrinsics, prefix + '/extrinsics/depth_to_' + args.stream, self.extrinsic, persistent)]
            ns = args.namespace.rstrip('/')
            self.result_pub = self.create_publisher(String, ns + '/result', 10)
            self.path_pub = self.create_publisher(RosPath, ns + '/path_3d', 10)
            self.diagnostic_pub = self.create_publisher(String, ns + '/diagnostics', 10)
            self.overlay_pub = self.create_publisher(Image, ns + '/overlay', 1)
            self.create_service(Trigger, ns + '/save_snapshot', self.save)
            self.create_timer(.005, self.process)
            self.create_timer(1., self.diagnostics)
            self.get_logger().info('Passive D405 fusion ready; actual backend=' + config['visual_backend'] + '/' + config['geometry_backend'])

        def info(self, kind, message):
            try: self.calibration[kind] = from_camera_info(message, 'live_camera_info:' + kind)
            except ValueError as exc: self.reject(str(exc))

        def extrinsic(self, message):
            self.extrinsics = {'rotation': np.array(message.rotation).reshape(3, 3, order='F').ravel().tolist(),
                               'translation_m': list(message.translation), 'source': 'live_factory_depth_to_' + args.stream}

        def reject(self, reason):
            if reason not in self.errors and len(self.errors) >= 64: reason = 'additional_failure_reasons'
            self.invalid += 1; self.errors[reason] = self.errors.get(reason, 0) + 1

        def image(self, kind, message):
            try:
                if config['mode'] == 'vision_only':
                    if kind != 'image': return
                    if stamp_ns(message) <= 0: raise ValueError('invalid_source_stamp')
                    self.sync.counts['image_received'] += 1
                    self.image_only_work_items += 1
                    if self.latest is not None: self.latest_dropped += 1
                    self.latest = (message, None, time.time_ns())
                    return
                # Retain the header and receive time independently; no synthetic synchronization.
                pairs = self.sync.add(kind, message)
                for pair in pairs:
                    if self.latest is not None: self.latest_dropped += 1
                    self.latest = (*pair, time.time_ns())
            except ValueError as exc: self.reject(str(exc))

        def process(self):
            if self.latest is None: return
            callback_started = time.perf_counter()
            image, depth, receive = self.latest; self.latest = None
            if len(self.calibration) != 2 or self.extrinsics is None:
                self.missing_calibration += 1  # Warning, not a dropped frame: 2D fallback is independent.
            previous_processed, previous_invalid = self.processed, self.invalid
            try:
                if ('image' in self.calibration and image.header.frame_id != self.calibration['image'].frame_id) or (depth is not None and 'depth' in self.calibration and depth.header.frame_id != self.calibration['depth'].frame_id):
                    raise ValueError('image_header_calibration_frame_mismatch')
                if depth is not None and depth.encoding.lower() != '16uc1': raise ValueError('live_depth_encoding_requires_explicit_new_unit_contract')
                self.sequence += 1
                a, d = image_array(image), None if depth is None else image_array(depth)
                e = None if self.extrinsics is None or depth is None else dict(self.extrinsics, from_frame=depth.header.frame_id, to_frame=image.header.frame_id)
                frame = Frame(a, d, None if d is None else np.isfinite(d) & (d > 0), stamp_ns(image), None if depth is None else stamp_ns(depth), self.sequence,
                              self.calibration.get('image'), self.calibration.get('depth'), e, args.depth_scale_m,
                              {'session': 'live_read_only', 'stream': args.stream, 'image_frame_id': image.header.frame_id,
                               'depth_frame_id': None if depth is None else depth.header.frame_id, 'depth_unit_source': 'explicit_ros_driver_mm_contract'}, receive)
                result, diagnostic = self.pipeline.process(frame, diagnostic=True)
                result['input_age_receive_ms'] = (receive - frame.image_stamp_ns) / 1e6
                result['processing_completed_host_ns'] = str(time.time_ns())
                result['input_age_output_ms'] = (int(result['processing_completed_host_ns']) - frame.image_stamp_ns) / 1e6
                result['age_clock_comparability_validated'] = False  # Verify stream metadata clock domain separately.
                self.processed += 1
                self.timings.append(result['timing_ms']['total']); self.ages.append(result['input_age_output_ms'])
                # Bounded runtime metric window; output JSONL preserves each result for exact offline statistics.
                if len(self.timings) > 300: self.timings.pop(0); self.ages.pop(0)
                self.states[result['state']] = self.states.get(result['state'], 0) + 1
                text = String(); text.data = json.dumps(result, ensure_ascii=False, allow_nan=False); self.result_pub.publish(text)
                if self.output: self.output.write(text.data + '\n')
                path = RosPath(); path.header = image.header
                path.header.frame_id = result['frame_id_3d'] or image.header.frame_id
                # Empty path clears stale display on invalid/predicted results. Never publish pixels as metres.
                if result['metric_valid']:
                    for point in result['points_3d_center']:
                        if point is None: continue
                        pose = PoseStamped(); pose.header = path.header
                        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
                        pose.pose.orientation.w = 1.; path.poses.append(pose)
                self.path_pub.publish(path)
                if args.overlay and self.overlay_pub.get_subscription_count():
                    overlay_started = time.perf_counter()
                    view = overlay(frame, result, diagnostic); output = Image(); output.header = image.header
                    output.height, output.width = view.shape[:2]; output.encoding = 'bgr8'; output.step = output.width * 3; output.data = image_payload(view)
                    self.overlay_pub.publish(output)
                    self.overlay_published += 1
                    self.overlay_timings.append((time.perf_counter() - overlay_started) * 1000)
                    if len(self.overlay_timings) > 300: self.overlay_timings.pop(0)
                if self.save_requested:
                    self.save_requested = False
                    self.writer.request(frame, result, diagnostic)
            except Exception as exc:
                self.reject(type(exc).__name__ + ':' + str(exc))
                self.get_logger().error(str(exc))
            finally:
                self.callback_timings.append((time.perf_counter() - callback_started) * 1000)
                if len(self.callback_timings) > 300: self.callback_timings.pop(0)
                if self.processed == previous_processed and self.invalid == previous_invalid:
                    self.abandoned += 1  # Interrupted frame, never silently claim it processed.

        def save(self, request, response):
            if self.writer is None:
                response.success = False; response.message = 'events disabled; start with --events NEW_PATH'
            else:
                try:
                    self.writer.worker.check(); self.save_requested = True
                    response.success = True; response.message = 'next matched frame requested; final metadata verifies actual save'
                except Exception as exc: response.success = False; response.message = str(exc)
            return response

        def status(self):
            return {'sync': self.sync.finish(), 'processed': self.processed, 'latest_dropped': self.latest_dropped,
                    'sync_mode': 'image_only_no_depth_match' if config['mode'] == 'vision_only' else 'two_source_tolerance',
                    'image_only_work_items': self.image_only_work_items,
                    'missing_calibration': self.missing_calibration, 'invalid': self.invalid, 'errors': self.errors, 'states': self.states,
                    'pending_matched': int(self.latest is not None), 'camera_owners_added': 0, 'actual_processing_fps': self.processed / max(.001, time.monotonic() - self.started),
                    'interrupted_processing_frames': self.abandoned,
                    'timing_window_count': len(self.timings), 'timing_window_p50_ms': float(np.percentile(self.timings, 50)) if self.timings else None,
                    'timing_window_p95_ms': float(np.percentile(self.timings, 95)) if self.timings else None,
                    'age_window_p95_ms': float(np.percentile(self.ages, 95)) if self.ages else None,
                    'callback_window_p95_ms': float(np.percentile(self.callback_timings, 95)) if self.callback_timings else None,
                    'overlay_window_p95_ms': float(np.percentile(self.overlay_timings, 95)) if self.overlay_timings else None,
                    'overlay_published': self.overlay_published,
                    'writer': None if self.writer is None else self.writer.worker.status,
                    'algorithm_accuracy_validated': False, 'hardware_realtime_validated': False, 'production_ready': False}

        def diagnostics(self):
            text = String(); text.data = json.dumps(self.status(), ensure_ascii=False); self.diagnostic_pub.publish(text)

        def finish(self):
            status = self.status()
            complete = self.abandoned == 0 and self.processed > 0
            try:
                if self.writer: self.writer.close(5.)
                self.pipeline.close()
            except Exception as exc:
                complete = False; status['shutdown_error'] = str(exc)
            if self.output:
                self.output.close()
                from .contracts import sha256
                dump(Path(args.output) / 'result_manifest.json', dict(status, schema='d405_ros_analysis_v1', complete=complete,
                    config_sha256=sha256(args.config), config=config, results_sha256=sha256(Path(args.output) / 'results.jsonl'),
                    source_run_id='unavailable_live_not_formal_run', field_accuracy_verified=False))
            return status

    node = None
    try:
        node = SeamNode()
        while rclpy.ok() and (args.duration <= 0 or time.monotonic() - node.started < args.duration):
            rclpy.spin_once(node, timeout_sec=.02)
    except KeyboardInterrupt: pass
    finally:
        if node:
            print(json.dumps(node.finish(), ensure_ascii=False, allow_nan=False)); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
