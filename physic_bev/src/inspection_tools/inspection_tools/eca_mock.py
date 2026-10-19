#!/usr/bin/env python3
"""Deterministic native ECA mock for whole-system transport validation only."""

from __future__ import annotations
import math
import rclpy
from eddy_driver.msg import EddyChannelMean, EddyFrame, EddyStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class EcaMock(Node):
    def __init__(self) -> None:
        super().__init__("eca_mock")
        rate = float(self.declare_parameter("rate_hz", 10.0).value)
        if rate <= 0:
            raise ValueError("rate_hz must be positive")
        self._raw = self.create_publisher(EddyFrame, "/eddy/data", qos_profile_sensor_data)
        self._mean = self.create_publisher(EddyChannelMean, "/eddy/channel_mean", qos_profile_sensor_data)
        self._status = self.create_publisher(EddyStatus, "/eddy/status", 10)
        self._sequence = 0
        self._timer = self.create_timer(1.0 / rate, self._tick)
        self._status_timer = self.create_timer(1.0, self._status_tick)
        self.get_logger().warning("ECA MOCK active: values are synthetic and must not be accepted as hardware")

    def _tick(self) -> None:
        raw = EddyFrame()
        raw.header.stamp = self.get_clock().now().to_msg()
        raw.header.frame_id = "eca_mock_frame"
        raw.sequence = self._sequence
        raw.samples_per_channel = 20
        raw.channel_count = 8
        raw.data = [int(1000 + channel * 25 + 80 * math.sin(
            0.11 * self._sequence + 0.25 * sample + channel))
            for sample in range(20) for channel in range(8)]
        raw.channel_mean = [sum(raw.data[channel::8]) / 20.0 for channel in range(8)]
        raw.excitation_amplitude = 100
        raw.excitation_frequency = 100000
        raw.dropped_frames = 0
        mean = EddyChannelMean()
        mean.header = raw.header
        mean.channel_mean = raw.channel_mean
        self._raw.publish(raw)
        self._mean.publish(mean)
        self._sequence += 1

    def _status_tick(self) -> None:
        status = EddyStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = "eca_mock_frame"
        status.connected = True
        status.current_amplitude = 100
        status.current_frequency = 100000
        status.has_status = True
        status.received_frames = self._sequence
        status.queue_depth = 0
        self._status.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EcaMock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
