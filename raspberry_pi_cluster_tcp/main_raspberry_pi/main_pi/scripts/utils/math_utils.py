#!/usr/bin/env python3
"""
SAFRS Main Pi - Utility Module
Module: math_utils

Description:
    Math helper functions for angle normalization,
    quaternion conversion, etc.
"""

import math
from geometry_msgs.msg import Quaternion


def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert yaw (radians) → Quaternion
    """
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def normalize_angle(angle: float) -> float:
    """
    Normalize angle to [-pi, pi]
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi
