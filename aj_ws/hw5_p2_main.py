#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
import matplotlib.pyplot as plt

from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs
from turtle.pid import PIDControl as pid


def main() -> None:
    # initiate and create node
    rclpy.init(args=None)
    turtlebot_node = TurtleBot_Node()

    # set parameters
    lin_vel_cmd = 0.15  # m/s
    max_ang_speed_rad = 2.84  # rad/s
    time_now = get_time_in_secs(turtlebot_node)
    path_list = [(0, 0), (0, 1), (2, 2), (3, -3)]
    heading_error_tol_rad = np.deg2rad(1)
    waypoint_proximity = 0.1
    theta = 0.0
    waypoint_index = 0

    # for plotting
    x_pos_history = []
    y_pos_history = []

    try:
        while rclpy.ok():
            if waypoint_index < len(path_list):
                target_x, target_y = path_list[waypoint_index]
                dx = target_x - turtlebot_node.current_position[0]
                dy = target_y - turtlebot_node.current_position[1]
                distance_to_waypoint = m.sqrt(dx**2 + dy**2)
                desired_heading = m.atan2(dy, dx)

                # pid
                kp_ang = 3
                ki_ang = 0.0
                kd_ang = 0.0
                dt_ang = 0.1

                pid_angular = pid(kp_ang, ki_ang, kd_ang)
                heading_gain, angle_error = pid_angular.calculate_pid(
                    desired_heading, turtlebot_node.orientation_euler[2], dt_ang, False
                )

                if distance_to_waypoint <= waypoint_proximity:
                    waypoint_index += 1
                    print(f"Reached waypoint {waypoint_index}")

                if waypoint_index < len(path_list):
                    # Calculate heading_move based on heading_gain
                    ang_vel_cmd = max(
                        -max_ang_speed_rad, min(heading_gain, max_ang_speed_rad)
                    )
                    turtlebot_node.move_turtle(lin_vel_cmd, ang_vel_cmd)

                else:
                    turtlebot_node.move_turtle(0.0, 0.0)

            rclpy.spin_once(turtlebot_node)

            x_pos_history.append(turtlebot_node.current_position[0])
            y_pos_history.append(turtlebot_node.current_position[1])

            if waypoint_index == len(path_list):
                break

        # Create the desired vs actual heading plot
        plt.figure(figsize=(8, 6))
        plt.plot(x_pos_history, y_pos_history)

        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title("Turtlebot Following Path")
        plt.legend()
        plt.grid()
        plt.show()
    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)


if __name__ == "__main__":
    main()
