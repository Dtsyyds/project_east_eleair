"""ROS observation only; the executor never calls a business runtime or drives hardware."""
from collections import deque
import math
import threading
import time

from .pointcloud import xyz_preview


def check_item(key, label, ok, detail='', suggestion=''):
    return {'id': key, 'label': label, 'status': 'unknown' if ok is None else 'pass' if ok else 'fail',
            'detail': detail, 'suggestion': suggestion if ok is not True else ''}


def stamp(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


class RosBridge:
    def __init__(self, config):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import Imu, PointCloud2
        from livox_ros_driver2.msg import CustomMsg
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from std_msgs.msg import Bool
        self.config = config
        self.lock = threading.RLock()
        self.inputs = {}
        self.imu = deque(maxlen=100)
        self.observed = {}
        self.clouds = {}
        self.last_cloud = {}
        self.positions = {}
        self.map_frame = 'map'
        self.started = float('inf')
        self.started_mono = float('inf')
        self.context = Context()
        rclpy.init(context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('slam_console_observer', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        for cls, key in ((CustomMsg, 'lidar'), (Imu, 'imu')):
            self.node.create_subscription(cls, config[key + '_topic'], lambda msg, k=key: self.input(k, msg), qos_profile_sensor_data)
        transient = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(Bool, '/fast_livo2/tracking_valid', lambda m: self.observe('tracking', bool(m.data)), 1)
        self.node.create_subscription(Odometry, '/aft_mapped_to_init', lambda m: self.observe('odom', stamp(m)), qos_profile_sensor_data)
        self.node.create_subscription(Odometry, '/aft_mapped_in_map', lambda m: self.position('current', m), qos_profile_sensor_data)
        self.node.create_subscription(PoseWithCovarianceStamped, '/icp_result', lambda m: self.position('initial', m), transient)
        self.node.create_subscription(PoseWithCovarianceStamped, '/fast_livo2/initial_map_pose', lambda m: self.observe('initial', stamp(m)), transient)
        for topic in ('/cloud_effected', '/cloud_registered'):
            self.node.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.cloud(t, msg), qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def input(self, key, msg):
        now = time.monotonic()
        with self.lock:
            previous = self.inputs.get(key)
            timestamp = stamp(msg)
            self.inputs[key] = {'received': now, 'stamp': timestamp,
                                'advancing': previous is not None and timestamp > previous['stamp']}
            if key == 'imu':
                a, g = msg.linear_acceleration, msg.angular_velocity
                self.imu.append((now, math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z), math.sqrt(g.x*g.x+g.y*g.y+g.z*g.z)))

    def observe(self, key, value):
        with self.lock:
            self.observed[key] = (time.monotonic(), value)

    def begin(self, started, map_frame='map'):
        with self.lock:
            self.started = started
            self.started_mono = time.monotonic()
            self.observed.clear()
            self.clouds.clear()
            self.last_cloud.clear()
            self.positions = {}
            self.map_frame = map_frame

    def position(self, key, message):
        """Keep current map position and the first GICP initial IMU position."""
        with self.lock:
            timestamp = stamp(message)
            point = message.pose.pose.position
            xyz = [point.x, point.y, point.z]
            if (not math.isfinite(timestamp) or timestamp < self.started or
                    abs(time.time() - timestamp) > self.config['stamp_tolerance'] or
                    message.header.frame_id != self.map_frame or
                    not all(math.isfinite(value) for value in xyz)):
                return
            previous = self.positions.get(key)
            if previous and (key == 'initial' or timestamp <= previous['stamp']):
                return
            self.positions[key] = {'xyz': xyz, 'frame_id': self.map_frame,
                                   'stamp': timestamp, 'received': time.monotonic()}

    def position_status(self, mode):
        with self.lock:
            current = self.positions.get('current')
            initial = self.positions.get('initial') if mode == 'relocation' else {
                'xyz': [0.0, 0.0, 0.0], 'frame_id': self.map_frame, 'stamp': self.started}
            age = max(0, time.monotonic() - current['received']) if current else None
            fresh = bool(current and age <= self.config['input_max_age'] and
                         abs(time.time() - current['stamp']) <= self.config['stamp_tolerance'])
            return {'frame_id': self.map_frame,
                    'current': {k: current[k] for k in ('xyz', 'stamp')} if current else None,
                    'initial': {k: initial[k] for k in ('xyz', 'stamp')} if initial else None,
                    'age': age, 'fresh': fresh}

    def ready(self, mode):
        return self.health(mode)['ready']

    def health(self, mode):
        """Lightweight current-session evidence; no graph scan or writable probes."""
        now, wall = time.monotonic(), time.time()
        with self.lock:
            def observation(key):
                received, value = self.observed.get(key, (None, None))
                age = max(0, now - received) if received is not None else None
                fresh = received is not None and received >= self.started_mono and age <= self.config['input_max_age']
                return value, age, fresh
            tracking, tracking_age, tracking_fresh = observation('tracking')
            odom, odom_age, odom_fresh = observation('odom')
            initial = self.observed.get('initial')
            odom_ok = bool(odom_fresh and odom >= self.started and abs(wall - odom) <= self.config['stamp_tolerance'])
            prior_ok = mode == 'mapping' or bool(initial and initial[0] >= self.started_mono and initial[1] >= self.started)
            checks = [check_item('tracking', '本次有效跟踪', bool(tracking_fresh and tracking)),
                      check_item('odom', '本次新鲜且时钟有效的里程计', odom_ok)]
            if mode == 'relocation':
                checks.append(check_item('initial', '本次先验初始化证据', prior_ok))
            ready = all(c['status'] == 'pass' for c in checks)
            state = ('valid' if ready else 'tracking_invalid' if tracking_fresh and not tracking
                     else 'odom_stale' if odom_age is not None and not odom_fresh
                     else 'odom_clock_invalid' if odom_fresh and not odom_ok
                     else 'tracking_stale' if tracking_age is not None and not tracking_fresh else 'not_ready')
            inputs = []
            for key, label in (('lidar', '雷达输入'), ('imu', 'IMU 输入')):
                sample = self.inputs.get(key, {})
                age = max(0, now - sample['received']) if sample else None
                inputs.append({'id': key, 'label': label, 'age': age,
                               'status': 'unknown' if age is None else 'pass' if age <= self.config['input_max_age'] else 'fail'})
            return {'state': state, 'ready': ready, 'checks': checks, 'inputs': inputs,
                    'tracking_age': tracking_age, 'odom_age': odom_age}

    def preflight(self):
        return [c['detail'] for c in self.preflight_checks() if c['status'] != 'pass']

    def preflight_checks(self):
        checks = []
        now, wall = time.monotonic(), time.time()
        topics = dict(self.node.get_topic_names_and_types())
        with self.lock:
            for key, expected in [('lidar', 'livox_ros_driver2/msg/CustomMsg'), ('imu', 'sensor_msgs/msg/Imu')]:
                topic = self.config[key + '_topic']
                label = '雷达' if key == 'lidar' else 'IMU'
                checks.append(check_item(key + '.type', label + '消息类型', topics.get(topic) == [expected],
                    f'{topic} 类型须为 {expected}，实际 {topics.get(topic, [])}', '检查驱动和话题配置。'))
                sample = self.inputs.get(key, {})
                checks.append(check_item(key + '.freshness', label + '输入新鲜度',
                    bool(sample) and now - sample['received'] <= self.config['input_max_age'],
                    f'{topic} 新鲜输入要求 ≤ {self.config["input_max_age"]:g}s', '确认传感器已启动且持续发布。'))
                checks.append(check_item(key + '.stamp', label + '时间戳推进与偏差',
                    bool(sample.get('advancing')) and abs(wall - sample.get('stamp', 0)) <= self.config['stamp_tolerance'],
                    f'{topic} 时间戳须推进且偏差 ≤ {self.config["stamp_tolerance"]:g}s', '检查系统时间及传感器时钟。'))
                publishers = self.node.count_publishers(topic)
                checks.append(check_item(key + '.publishers', label + '发布者数量', publishers == 1,
                    f'{topic} 须有且仅有一个输入发布者，实际 {publishers}', '检查驱动，结束相同话题的历史回放或重复驱动。'))
            samples = [v for v in self.imu if now - v[0] <= self.config['input_max_age']]
            checks.append(check_item('imu.samples', '静止 IMU 样本数量', len(samples) >= 10,
                f'至少需要 10 个新鲜 IMU 样本，实际 {len(samples)}', '保持静止并等待输入后重试。'))
            reasonable = all(math.isfinite(a) and 7.5 <= a <= 12.0 and math.isfinite(g) and g <= .3 for _, a, g in samples)
            checks.append(check_item('imu.stationary', 'IMU 静止及米制输入合理性', reasonable if len(samples) >= 10 else None,
                '静止米制输入要求：加速度模约 9.8 m/s²、角速度接近 0；此检查不能自动识别设备真实单位',
                '检查单位转换并保持静止后重试。'))
        names = {name for name, _ in self.node.get_node_names_and_namespaces()}
        conflict = bool(names & {'laserMapping', 'teaser_gicp_node', 'transform_publisher'}) or any(
            self.node.count_publishers(t) for t in ('/aft_mapped_to_init', '/fast_livo2/tracking_valid', '/fast_livo2/initial_map_pose'))
        checks.append(check_item('instances', '定位建图实例冲突', not conflict,
            '同 ROS 域不可存在其他定位建图实例', '先正常结束原任务，再启动新任务。'))
        return checks

    def cloud(self, topic, message):
        now = time.time()
        with self.lock:
            if now < self.started or now - self.last_cloud.get(topic, 0) < .2:
                return
            self.last_cloud[topic] = now
            generation = self.started
        payload = {'type': 'cloud', 'topic': topic, 'frame_id': message.header.frame_id,
                   'stamp': stamp(message), 'received': now, 'source_points': message.width * message.height}
        try:
            payload['points'] = xyz_preview(message)
            payload['error'] = None
        except (ValueError, TypeError, BufferError) as error:
            payload.update(points=[], error=str(error))
        with self.lock:
            if generation == self.started:
                self.clouds[topic] = payload

    def previews(self):
        with self.lock:
            return dict(self.clouds)

    def close(self):
        self.executor.shutdown(timeout_sec=5)
        self.thread.join(timeout=5)
        self.node.destroy_node()
        self.context.shutdown()
