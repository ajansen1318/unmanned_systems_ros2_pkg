#!/usr/bin/env python3

import rclpy
import numpy as np
import datetime
import csv
import os
import matplotlib.pyplot as plt

from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs
from turtle.pid import PIDControl as pid


def main() -> None:
    # initiate and create node
    rclpy.init(args=None)
    turtlebot_node = TurtleBot_Node()

    # set parameters
    time_now = get_time_in_secs(turtlebot_node)
    lin_vel_cmd = 0.15  # m/s
    des_head_angle_rad = np.deg2rad(90)
    heading_error_tol_rad = np.deg2rad(1)
    max_ang_speed_rad = 2.84

    # pid parameters
    kp_ang = 3
    ki_ang = 0.25
    kd_ang = 0.5
    dt_ang = 0.1

    # for plots
    desired_heading = []
    actual_heading = []
    time_diff = get_time_in_secs(turtlebot_node) - time_now

    try:
        while rclpy.ok():
            # move bot
            pid_angular = pid(kp_ang, ki_ang, kd_ang)
            pid_angular.calculate_pid(
                des_head_angle_rad, turtlebot_node.orientation_euler[2], dt_ang
            )
            while pid_angular.error[0] >= heading_error_tol_rad:
                heading_gain, _ = pid_angular.calculate_pid(
                    des_head_angle_rad, turtlebot_node.orientation_euler[2], dt_ang
                )

                if heading_gain > max_ang_speed_rad:
                    heading_gain = max_ang_speed_rad
                elif heading_gain <= -max_ang_speed_rad:
                    heading_gain = -max_ang_speed_rad
                turtlebot_node.move_turtle(lin_vel_cmd, heading_gain)
                desired_heading.append(des_head_angle_rad)
                actual_heading.append(turtlebot_node.orientation_euler[2])

                rclpy.spin_once(turtlebot_node)

            if pid_angular.error[0] <= heading_error_tol_rad:
                turtlebot_node.move_turtle(0.0, 0.0)

            rclpy.spin_once(turtlebot_node)
            # Create the desired vs actual heading plot
            plt.figure(figsize=(8, 6))
            plt.plot(time_diff, desired_heading, label="Desired Heading", color="blue")
            plt.plot(time_diff, actual_heading, label="Actual Heading", color="red")

            plt.xlabel("Time Step")
            plt.ylabel("Heading (radians)")
            plt.title("Desired vs Actual Heading")
            plt.legend()
            plt.grid()
            plt.show()

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)


if __name__ == "__main__":
    main()
