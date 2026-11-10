"""ROS observation only; the executor never calls a business runtime or drives hardware."""
from collections import deque
import math
import threading
import time

from .pointcloud import xyz_preview


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
        self.started = float('inf')
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
        self.node.create_subscription(PoseWithCovarianceStamped, '/fast_livo2/initial_map_pose', lambda m: self.observe('initial', stamp(m)), transient)
        for topic in ('/cloud_effected', '/cloud_registered'):
            self.node.create_subscription(PointCloud2, topic, lambda msg, t=topic: self.cloud(t, msg), qos_profile_sensor_data)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def input(self, key, msg):
        now = time.time()
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
            self.observed[key] = (time.time(), value)

    def begin(self, started):
        with self.lock:
            self.started = started
            self.observed.clear()
            self.clouds.clear()
            self.last_cloud.clear()

    def ready(self, mode):
        now = time.time()
        with self.lock:
            def fresh(key):
                received, value = self.observed.get(key, (0, 0))
                return received >= self.started and now - received <= self.config['input_max_age'] and value
            odom = fresh('odom')
            initial = self.observed.get('initial', (0, 0))
            return bool(fresh('tracking') and odom and odom >= self.started and
                        abs(now - odom) <= self.config['stamp_tolerance'] and
                        (mode == 'mapping' or (initial[0] >= self.started and initial[1] >= self.started)))

    def preflight(self):
        errors = []
        now = time.time()
        topics = dict(self.node.get_topic_names_and_types())
        with self.lock:
            for key, expected in [('lidar', 'livox_ros_driver2/msg/CustomMsg'), ('imu', 'sensor_msgs/msg/Imu')]:
                topic = self.config[key + '_topic']
                if topics.get(topic) != [expected]:
                    errors.append(f'{topic} 类型须为 {expected}，实际 {topics.get(topic, [])}')
                sample = self.inputs.get(key, {})
                if now - sample.get('received', 0) > self.config['input_max_age']:
                    errors.append(f'{topic} 没有新鲜输入')
                if abs(now - sample.get('stamp', 0)) > self.config['stamp_tolerance'] or not sample.get('advancing'):
                    errors.append(f'{topic} 时间戳异常/未推进，请检查系统时间及传感器时钟')
                if self.node.count_publishers(topic) != 1:
                    errors.append(f'{topic} 须有且仅有一个输入发布者；不可同时回放')
            samples = [v for v in self.imu if now - v[0] <= self.config['input_max_age']]
            if len(samples) < 10:
                errors.append('至少需要 10 个新鲜 IMU 样本，请保持静止后重试')
            elif any(not math.isfinite(a) or not 7.5 <= a <= 12.0 or not math.isfinite(g) or g > .3 for _, a, g in samples):
                errors.append('IMU 不满足静止米制输入：加速度模应约 9.8 m/s²、角速度应接近 0；检查单位转换并保持静止')
        names = {name for name, _ in self.node.get_node_names_and_namespaces()}
        if names & {'laserMapping', 'teaser_gicp_node', 'transform_publisher'} or any(
                self.node.count_publishers(t) for t in ('/aft_mapped_to_init', '/fast_livo2/tracking_valid', '/fast_livo2/initial_map_pose')):
            errors.append('同 ROS 域存在其他定位建图实例，请先结束原任务')
        return errors

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
