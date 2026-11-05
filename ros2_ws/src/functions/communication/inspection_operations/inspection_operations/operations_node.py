#!/usr/bin/env python3
"""ROS adapter for the one-click inspection operations service."""
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.parameter import Parameter
from eddy_driver.msg import EddyFrame
from inspection_interfaces.msg import UltrasoundFrame
import yaml

from inspection_runtime.operations import Operations, ROOT


class OperationsNode(Node):
    def __init__(self, config=None):
        super().__init__("inspection_operations")
        from inspection_runtime.paths import load_system_config
        config = config or load_system_config()
        self.live, self.live_lock = {}, threading.Lock()
        self.events = self.create_publisher(String, "/inspection/operations/events", 64)
        self.ultrasound = self.create_publisher(UltrasoundFrame, "/inspection/ultrasound/raw", 10)
        self.result_pub = self.create_publisher(String, "/inspection/emat/result",
                            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_result = None
        self.ultrasound_sequence = 0
        self.emat_settings = config["emat"]
        self.vision_clients = {name: self.create_client(Trigger, '/vision/d405_seam/' + name) for name in ('save_snapshot', 'sample_status')}
        self.selection_command = self.create_publisher(String, '/weld_tracking/select_candidate', 10)
        self.selection_pending = {}
        self.selection_lock = threading.Lock()
        bridge = yaml.safe_load((ROOT / 'ros2_ws/src/functions/communication/agv_bridge/config/tank_inspection.yaml').read_text())['websocket_bridge_node']['ros__parameters']
        self.vision_preview_width = int(bridge.get('tank_inspection.preview_width', 640))
        vision_enabled = config.get('hardware', {}).get('start_d405', True) and bridge.get('tank_inspection.depth_camera_topic') == '/d405/camera/color/image_raw'
        self.camera_get = self.create_client(Trigger, '/camera/front/get_settings')
        self.camera_set = self.create_client(SetParametersAtomically, '/camera/front/set_parameters_atomically')
        self.operations = Operations(config, self.emit, self.live_topics, self.publish_ultrasound,
                                     self.camera_operation if config.get('hardware', {}).get('start_mindvision', True) else None,
                                     self.vision_call if vision_enabled else None)
        self.subscriptions_owned = []
        for topic in config["required_topics"]:
            kind = EddyFrame if topic == "/eddy/data" else PointCloud2 if topic == "/livox/lidar" else Imu if topic == "/livox/imu" else Image
            self.subscriptions_owned.append(self.create_subscription(kind, topic, lambda msg, t=topic: self.seen(t, msg), qos_profile_sensor_data))
        for topic in ('/d405/camera/color/image_raw', '/d405/camera/depth/image_rect_raw'):
            if topic not in config['required_topics']:
                self.subscriptions_owned.append(self.create_subscription(Image, topic, lambda msg, t=topic: self.seen(t, msg), qos_profile_sensor_data))
        self.subscriptions_owned.extend([
            self.create_subscription(String, '/vision/d405_seam/result', self.vision_result, 10),
            self.create_subscription(String, '/weld_tracking/selection_status', self.selection_reply, 10),
            self.create_subscription(String, '/vision/d405_seam/diagnostics', self.vision_diagnostic, 10)])
        self.command_sub = self.create_subscription(String, "/inspection/operations/command", self.command, 32)
        self.timer = self.create_timer(.5, self.operations.tick)

    def vision_call(self, name, request=None):
        if name == 'select_candidate':
            done = threading.Event()
            pending = {'done': done, 'reply': None, 'session_id': request['session_id']}
            ident = request['request_id']
            with self.selection_lock: self.selection_pending[ident] = pending
            try:
                if not self.selection_command.get_subscription_count():
                    raise RuntimeError('候选选择节点未连接')
                message = String(); message.data = json.dumps(request, ensure_ascii=False)
                self.selection_command.publish(message)
                if not done.wait(3.):
                    raise RuntimeError('候选选择响应超时，结果未知，请查看跟踪状态')
                reply = pending['reply']
                if reply.get('status') != 'completed':
                    raise RuntimeError(reply.get('error', '候选选择失败'))
                return reply
            finally:
                with self.selection_lock: self.selection_pending.pop(ident, None)
        if name == 'probe':
            return {'available': self.vision_clients['save_snapshot'].wait_for_service(timeout_sec=.2)}
        reply = self.camera_call(self.vision_clients[name], Trigger.Request())
        if not reply.success: raise RuntimeError(reply.message)
        return json.loads(reply.message)

    def selection_reply(self, message):
        try:
            reply = json.loads(message.data)
            if not isinstance(reply, dict): return
            with self.selection_lock:
                pending = self.selection_pending.get(reply.get('request_id'))
                if pending and pending['session_id'] == reply.get('session_id'):
                    pending['reply'] = reply
                    pending['done'].set()
        except (ValueError, TypeError):
            self.get_logger().warning('候选选择响应格式无效')

    def vision_diagnostic(self, msg):
        try:
            if self.operations.vision: self.operations.vision.diagnostic(json.loads(msg.data))
        except (ValueError, TypeError) as error:
            self.get_logger().warning('巡迹诊断无效：' + str(error))

    def vision_result(self, msg):
        from inspection_runtime.vision import tracking_payload
        try:
            if self.operations.vision is None: return
            result = json.loads(msg.data)
            state = self.operations.vision.snapshot()
            if state['status'] not in ('starting', 'running') or result['source']['source']['session'] != state.get('session_id'): return
            self.operations.packet('status_sensor', {'tracking_result': tracking_payload(result, self.vision_preview_width)})
        except (ValueError, TypeError, KeyError, IndexError) as error:
            self.get_logger().warning('巡迹结果无效：' + str(error))

    def camera_call(self, client, request):
        # Called only by Operations' worker, while the ROS executor continues
        # servicing responses. Never spin a second executor for this node.
        if not client.wait_for_service(timeout_sec=2.):
            raise RuntimeError('设备操作服务未连接：' + client.srv_name)
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(8.):
            future.cancel()
            raise RuntimeError('设备操作服务响应超时，结果未知：' + client.srv_name)
        return future.result()

    def camera_operation(self, action, parameters):
        if action != 'camera.settings.get':
            from inspection_runtime.camera_tuning import FIELDS
            values = parameters.get('values')
            serial = parameters.get('serial_number')
            if not isinstance(serial, str) or not serial or not isinstance(values, dict) or not values or set(values) - set(FIELDS):
                raise ValueError('相机参数或序列号无效，请重新读取当前参数')
            revision = parameters.get('expected_revision')
            if not isinstance(revision, str) or len(revision) != 64:
                raise ValueError('缺少当前参数版本，请重新读取')
            args = dict(values, tuning_serial=serial, tuning_revision=revision, save_tuning=action == 'camera.settings.save')
            for name in ('exposure_time_us', 'analog_gain'):
                if name in args and type(args[name]) in (int, float):
                    args[name] = float(args[name])
            request = SetParametersAtomically.Request(parameters=[Parameter(name, value=value).to_parameter_msg() for name, value in args.items()])
            reply = self.camera_call(self.camera_set, request)
            if not reply.result.successful:
                raise RuntimeError(reply.result.reason)
        reply = self.camera_call(self.camera_get, Trigger.Request())
        if not reply.success:
            raise RuntimeError(reply.message)
        return json.loads(reply.message)

    def seen(self, topic, msg):
        if msg.header.stamp.sec <= 0:
            return
        if isinstance(msg, Image) and (not msg.data or msg.width <= 0 or msg.height <= 0):
            return
        if isinstance(msg, PointCloud2) and (not msg.data or msg.width * msg.height <= 0):
            return
        with self.live_lock:
            self.live[topic] = time.monotonic()

    def live_topics(self):
        with self.live_lock:
            return dict(self.live)

    def command(self, msg):
        try:
            self.operations.handle(json.loads(msg.data))
        except (ValueError, TypeError) as error:
            self.get_logger().warning(str(error))

    def emit(self, packet):
        if rclpy.ok():
            self.events.publish(String(data=json.dumps(packet, ensure_ascii=False, allow_nan=False)))
            result = packet["payload"].get("measurement", {})
            if result.get("status") == "completed" and result.get("measurement_id") != self.last_result:
                self.result_pub.publish(String(data=json.dumps(result, ensure_ascii=False)))
                self.last_result = result["measurement_id"]

    def publish_ultrasound(self, samples, sampling_rate):
        if not rclpy.ok():
            return
        msg = UltrasoundFrame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "emat_probe_link"
        msg.channel_count = 1
        msg.sample_count = len(samples)
        msg.sampling_rate_hz = sampling_rate
        self.ultrasound_sequence += 1
        msg.sequence = self.ultrasound_sequence
        msg.sensor_id = "emat_serial"
        msg.calibration_id = "UNASSIGNED"
        msg.sample_encoding = "uint8_adc_in_int16"
        msg.sound_velocity_m_s = float(self.emat_settings["sound_speed_m_s"])
        msg.samples = samples
        self.ultrasound.publish(msg)


def main():
    rclpy.init()
    node = OperationsNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.operations.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
