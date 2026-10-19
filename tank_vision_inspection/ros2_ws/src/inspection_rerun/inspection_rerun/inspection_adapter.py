"""Adapters for the real inspection_interfaces message fields."""

import numpy as np


def diagnostic_level(value):
    """Normalize Humble uint8 representations (int or one-byte bytes)."""
    return value[0] if isinstance(value, (bytes, bytearray)) else int(value)


def eddy_channels(message):
    channels = int(message.channel_count)
    samples = int(message.sample_count)
    expected = channels * samples
    signal_i = np.asarray(message.signal_i, dtype=np.float32)
    signal_q = np.asarray(message.signal_q, dtype=np.float32)
    if channels <= 0 or samples <= 0 or signal_i.size != expected or signal_q.size != expected:
        raise ValueError("EddyCurrentFrame channel/sample dimensions do not match signal arrays")
    return signal_i.reshape(channels, samples), signal_q.reshape(channels, samples)


def ultrasound_channels(message):
    channels = int(message.channel_count)
    samples = int(message.sample_count)
    values = np.asarray(message.samples, dtype=np.int16)
    if channels <= 0 or samples <= 0 or values.size != channels * samples:
        raise ValueError("UltrasoundFrame channel/sample dimensions do not match samples")
    return values.reshape(channels, samples)


def fusion_scalars(message):
    imu_delta_ns = 0
    if bool(message.imu_valid):
        start = int(message.imu_start_stamp.sec) * 1_000_000_000 + int(message.imu_start_stamp.nanosec)
        end = int(message.imu_end_stamp.sec) * 1_000_000_000 + int(message.imu_end_stamp.nanosec)
        imu_delta_ns = end - start
    return {
        "camera_delta_ms": float(message.camera_time_error_ns) / 1.0e6,
        "lidar_delta_ms": float(message.lidar_time_error_ns) / 1.0e6,
        "imu_window_ms": float(imu_delta_ns) / 1.0e6,
        "camera_valid": bool(message.camera_valid),
        "lidar_valid": bool(message.lidar_valid),
        "imu_valid": bool(message.imu_valid),
        "pose_valid": bool(message.pose_valid),
        "invalid_reason": str(message.invalid_reason),
    }
