#!/usr/bin/env python3

import rclpy
import math as m
import numpy as np
import threading
import matplotlib.pyplot as plt
import datetime

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from unmanned_systems_ros2_pkg import ProNav
from pathfinding_library.Obstacle import Obstacle


def get_mean_heading_target(heading_list: list) -> float:
    heading_list = np.array(heading_list)
    mean_heading_target = np.mean(heading_list)
    return mean_heading_target


def compute_global_heading(heading_target_rad: float, curent_yaw_rad: float):
    global_heading_rad = heading_target_rad + curent_yaw_rad

    if global_heading_rad > 2 * np.pi:
        global_heading_rad = global_heading_rad - 2 * np.pi
    elif global_heading_rad < 0:
        global_heading_rad = global_heading_rad + 2 * np.pi

    # print("global heading deg", np.rad2deg(global_heading_rad))

    return global_heading_rad


def get_time_in_secs(some_node: Node) -> float:
    return some_node.get_clock().now().nanoseconds / 1e9


def check_close(
    current_pos: np.ndarray, des_pos: np.ndarray, dist_tolerance: float = 0.5
) -> bool:
    """check if agent is close to waypoint"""
    des_pos = np.array(des_pos)
    current_pos = np.array(current_pos)
    dist_error = np.linalg.norm(des_pos - current_pos)
    if dist_error < dist_tolerance:
        return True
    else:
        return False


def compute_distance(current_pos: np.ndarray, des_pos: np.ndarray) -> float:
    """compute distance between two points"""
    des_pos = np.array(des_pos)
    current_pos = np.array(current_pos)
    return np.linalg.norm(current_pos - des_pos)


def main() -> None:
    rclpy.init(args=None)

    # lidar frequency
    lidar_freq = 5.0  # hz

    turtlebot_2 = TurtleBotNode.TurtleBotNode("turtle", "pursuer")

    time_now = get_time_in_secs(turtlebot_2)

    # since our lidar is super slow we're going to set this node to match our
    # lidar rate to about 3 times its sampling rate
    thread = threading.Thread(target=rclpy.spin, args=(turtlebot_2,), daemon=True)
    thread.start()
    rate = turtlebot_2.create_rate(lidar_freq * 3)

    # 5 works well for my side increase to make it more snappier on turns

    # this value works well with simple pn
    # pro_nav = ProNav.ProNav(1.5)

    # this value works well with true pn
    pro_nav = ProNav.ProNav(3.0)

    dt = 1 / lidar_freq
    old_evader_position = np.array([2, 1])
    obstacle_x = [5, 5, 5, 5, 5, 0, 1, 2, 3, 3]
    obstacle_y = [0, 1, 2, 3, 4, 5, 4, 3, 2, 3]
    obstacle_radius = 0.25

    obstacle_list = [
        Obstacle(x, y, obstacle_radius) for x, y in zip(obstacle_x, obstacle_y)
    ]

    pursuer_position_history = []
    evader_position_history = []

    while rclpy.ok():
        # rclpy.spin_once(turtlebot_pursuer)
        time_diff = get_time_in_secs(turtlebot_2) - time_now
        rate.sleep()

        mean_target = get_mean_heading_target(turtlebot_2.detected_heading_angle_list)

        global_heading_ref = compute_global_heading(
            np.deg2rad(mean_target), turtlebot_2.orientation_euler[2]
        )

        evader_position = np.array(turtlebot_2.evader_position)

        evader_velocity = (evader_position - old_evader_position) / dt

        flight_path_rate, cmd_vel = pro_nav.true_pro_nav(
            np.array(turtlebot_2.current_position),
            evader_position,
            dt,
            evader_velocity,
            np.array(turtlebot_2.current_velocity),
            True,
            global_heading_ref,
        )

        # cmd_vel = 0.20
        # flight_path_rate = pro_nav.simple_pro_nav(
        #     global_heading_ref, dt
        # )

        # do this command for half a second

        pursuer_position = np.array(turtlebot_2.current_position)
        old_evader_position = evader_position
        turtlebot_2.move_turtle(cmd_vel, flight_path_rate)
        pursuer_position_history.append(pursuer_position)
        evader_position_history.append(evader_position)
        # print(pursuer_position_history)
        if check_close(turtlebot_2.current_position, evader_position) == True:
            print("Pursuer caught evader")
            break
        if time_diff > 15:
            break
    pursuer_x = [x[0] for x in pursuer_position_history]
    pursuer_y = [x[1] for x in pursuer_position_history]
    evader_x = [x[0] for x in evader_position_history]
    evader_y = [x[1] for x in evader_position_history]
    # plot
    fig, ax = plt.subplots()
    ax.plot(evader_x, evader_y, "-o", label="evader")
    ax.plot(pursuer_x, pursuer_y, "-x", label="pursuer")
    ax.set_title("True PN")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    main()
