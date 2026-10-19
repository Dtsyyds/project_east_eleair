"""Defensive extraction of standard ROS IMU values."""

import math


def imu_values(message):
    acceleration = [float(message.linear_acceleration.x), float(message.linear_acceleration.y), float(message.linear_acceleration.z)]
    angular_velocity = [float(message.angular_velocity.x), float(message.angular_velocity.y), float(message.angular_velocity.z)]
    covariance = list(message.orientation_covariance)
    orientation_valid = not covariance or float(covariance[0]) != -1.0
    orientation = None
    if orientation_valid:
        candidate = [float(message.orientation.x), float(message.orientation.y), float(message.orientation.z), float(message.orientation.w)]
        norm = math.sqrt(sum(value * value for value in candidate))
        if math.isfinite(norm) and norm > 1.0e-12 and all(math.isfinite(value) for value in candidate):
            orientation = [value / norm for value in candidate]
    return acceleration, angular_velocity, orientation
