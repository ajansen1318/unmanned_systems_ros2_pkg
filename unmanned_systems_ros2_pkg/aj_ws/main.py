#!/usr/bin/env python3

import rclpy
import csv
from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs
from turtle.logger import Logger


# initiate and create node
rclpy.init(args=None)
turtlebot_node = TurtleBot_Node()

# log info
log_info = Logger(
    "/home/aj/ros2_ws/src/unmanned_systems_ros2_pkg/unmanned_systems_ros2_pkg/log/",
    ["time", "x", "y", "yaw", "vel_x", "yaw_vel"],
)

# set parameters
lin_vel_cmd = 1.5  # m/s
ang_vel_cmd = 0.15  # rad/s
vel_dur = 5  # time going forward
turn_dur = 2  # time while turning
time_now = get_time_in_secs(turtlebot_node)


while rclpy.ok():
    time_diff = get_time_in_secs(turtlebot_node) - time_now
    # for logging
    log_data = [
        time_diff,
        turtlebot_node.current_position[0],
        turtlebot_node.current_position[1],
        turtlebot_node.orientation_euler[2],
        turtlebot_node.movement[0],
        turtlebot_node.movement[1],
    ]
    log_info.save_csv(log_data)

    # moving the bot
    if time_diff < vel_dur:
        turtlebot_node.move_turtle(lin_vel_cmd, 0.0)
    elif vel_dur < time_diff < (vel_dur + turn_dur):
        turtlebot_node.move_turtle(0.0, -ang_vel_cmd)
    elif vel_dur + turn_dur < time_diff < 2 * vel_dur + turn_dur:
        turtlebot_node.move_turtle(lin_vel_cmd, 0.0)
    else:
        turtlebot_node.move_turtle(0.0, 0.0)
        rclpy.shutdown()

    rclpy.spin_once(turtlebot_node)
