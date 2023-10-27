#!/usr/bin/env python3

import rclpy
import math as m
from rclpy.node import Node


def get_time_in_secs(some_node: Node) -> float:
    return some_node.get_clock().now().nanoseconds / 1e9


def euler_from_quaternion(x: float, y: float, z: float, w: float) -> tuple:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = m.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = m.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = m.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians
