#!/usr/bin/env python3

import rclpy
import csv
from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs


def main() -> None:
    # initiate and create node
    rclpy.init(args=None)
    turtlebot_node = TurtleBot_Node()

    # set parameters
    lin_vel_cmd = 1.5  # m/s
    ang_vel_cmd = 0.15  # rad/s
    vel_dur = 5  # time going forward
    turn_dur = 2  # time while turning
    time_now = get_time_in_secs(turtlebot_node)

    while rclpy.ok():
        time_diff = get_time_in_secs(turtlebot_node) - time_now

        # moving the bot

        rclpy.spin_once(turtlebot_node)


if __name__ == "__main__":
    main()
