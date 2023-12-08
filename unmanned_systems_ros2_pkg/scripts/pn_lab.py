#!/usr/bin/env python3

import numpy as np
import rclpy
import matplotlib.pyplot as plt

from rclpy.node import Node
from unmanned_systems_ros2_pkg import ProNav, quaternion_tools
from unmanned_systems_ros2_pkg import TurtleBotNode


def get_mean_target(heading_list: list) -> float:
    average = np.mean(heading_list)
    return average


def global_los(heading_target_rad: float, current_yaw: float):
    LOS = heading_target_rad + current_yaw
    if LOS < 0:
        LOS += 2 * np.pi
    if LOS > 2 * np.pi:
        LOS -= 2 * np.pi
    else:
        LOS = LOS

    return LOS


def main() -> None:
    rclpy.init(args=None)

    turtlebot_pursuer = TurtleBotNode.TurtleBotNode("turtle", "pursuer")
    turtlebot_pursuer.move_turtle(0.0, 0.0)
    pro_nav = ProNav.ProNav(3)
    while rclpy.ok():
        rclpy.spin_once(turtlebot_pursuer)
        dt = 0.1

        target_average = get_mean_target(turtlebot_pursuer.detected_heading_angle_list)
        target_average_rad = np.deg2rad(target_average)
        LOS = global_los(target_average_rad, turtlebot_pursuer.orientation_euler[2])

        flight_path_rate = pro_nav.simple_pro_nav(LOS, dt)
        turtlebot_pursuer.move_turtle(0.15, flight_path_rate)


if __name__ == "__main__":
    main()
