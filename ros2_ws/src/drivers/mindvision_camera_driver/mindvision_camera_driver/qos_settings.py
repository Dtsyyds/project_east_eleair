"""Bounded sensor QoS validation independent of rclpy."""

from __future__ import annotations


def validate_qos(depth: int, reliability: str) -> tuple[int, str]:
    depth = int(depth)
    reliability = str(reliability).lower()
    if not 1 <= depth <= 4:
        raise ValueError("qos_depth must be between 1 and 4")
    if reliability not in {"best_effort", "reliable"}:
        raise ValueError("qos_reliability must be best_effort or reliable")
    return depth, reliability
