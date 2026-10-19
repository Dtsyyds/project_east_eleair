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
from eddy_driver.msg import EddyFrame
from inspection_interfaces.msg import UltrasoundFrame
import yaml

from operations import Operations, ROOT


class OperationsNode(Node):
    def __init__(self, config=None):
        super().__init__("inspection_operations")
        config = config or yaml.safe_load((ROOT / "system_websocket/config/system.yaml").read_text())
        self.live, self.live_lock = {}, threading.Lock()
        self.events = self.create_publisher(String, "/inspection/operations/events", 64)
        self.ultrasound = self.create_publisher(UltrasoundFrame, "/inspection/ultrasound/raw", 10)
        self.result_pub = self.create_publisher(String, "/inspection/emat/result",
                            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.last_result = None
        self.ultrasound_sequence = 0
        self.emat_settings = config["emat"]
        self.operations = Operations(config, self.emit, self.live_topics, self.publish_ultrasound)
        self.subscriptions_owned = []
        for topic in config["required_topics"]:
            kind = EddyFrame if topic == "/eddy/data" else PointCloud2 if topic == "/livox/lidar" else Imu if topic == "/livox/imu" else Image
            self.subscriptions_owned.append(self.create_subscription(kind, topic, lambda msg, t=topic: self.seen(t, msg), qos_profile_sensor_data))
        self.command_sub = self.create_subscription(String, "/inspection/operations/command", self.command, 32)
        self.timer = self.create_timer(.5, self.operations.tick)

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
