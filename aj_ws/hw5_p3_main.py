#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
import matplotlib.pyplot as plt

from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs
from turtle.pid import PIDControl as pid
from pathfinding_library.Obstacle import Obstacle
from pathfinding_library.Grid import Grid
from pathfinding_library.node import Node
from pathfinding_library.dijkstra_astar import DijkstraAstar
from pathfinding_library.rrt import RRT


def main() -> None:
    # initiate and create node
    rclpy.init(args=None)
    turtlebot_node = TurtleBot_Node()

    # set parameters``
    lin_vel_cmd = 0.15  # m/s
    max_ang_speed_rad = 2.84  # rad/s
    waypoint_proximity = 0.1
    waypoint_index = 0
    start_x = 1
    start_y = 1
    goal_x = 7
    goal_y = 13
    obstacle_radius = 0.25
    obstacle_x = [
        2,
        2,
        2,
        2,
        0,
        1,
        2,
        3,
        4,
        5,
        5,
        5,
        5,
        5,
        8,
        9,
        10,
        11,
        12,
        13,
        8,
        8,
        8,
        8,
        8,
        8,
        8,
        2,
        3,
        4,
        5,
        6,
        7,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        2,
        2,
        2,
        2,
        2,
        2,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        6,
        7,
        8,
        9,
        10,
        11,
        12,
        12,
        12,
        12,
        12,
    ]

    obstacle_y = [
        2,
        3,
        4,
        5,
        5,
        5,
        5,
        5,
        5,
        5,
        2,
        3,
        4,
        5,
        2,
        2,
        2,
        2,
        2,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        7,
        7,
        7,
        7,
        7,
        7,
        6,
        6,
        6,
        6,
        6,
        6,
        6,
        8,
        9,
        10,
        11,
        12,
        13,
        9,
        10,
        11,
        12,
        13,
        14,
        15,
        12,
        12,
        12,
        12,
        12,
        12,
        8,
        9,
        10,
        11,
        12,
    ]
    max_x = 15
    max_y = 15
    # obstacle_list = list(zip(obstacle_x, obstacle_y))
    # Create a list of Obstacle objects
    obstacle_list = [
        Obstacle(x, y, obstacle_radius) for x, y in zip(obstacle_x, obstacle_y)
    ]
    grid = Grid(
        min_x=0,
        min_y=0,
        max_x=max_x,
        max_y=max_y,
        grid_space=0.5,
        obstacles=obstacle_list,
        robot_radius=0.5,
    )
    start_node = Node(start_x, start_y)
    end_node = Node(goal_x, goal_y)
    # dijkstra = DijkstraAstar(
    #     grid=grid, start_node=start_node, end_node=end_node, use_dijkstra=False
    # )
    # path = dijkstra.find_path()
    # path = path[::-1]

    rrt = RRT(grid, start_node, end_node, step_length=0.5)
    path = rrt.find_path()
    path = path[::-1]
    grid.plot(path)

    # for plotting
    x_pos_history = []
    y_pos_history = []

    try:
        while rclpy.ok():
            if waypoint_index < len(path):
                target_x, target_y = path[waypoint_index]
                dx = target_x - turtlebot_node.current_position[0]
                dy = target_y - turtlebot_node.current_position[1]
                distance_to_waypoint = m.sqrt(dx**2 + dy**2)
                desired_heading = m.atan2(dy, dx)

                if desired_heading - turtlebot_node.orientation_euler[2] > m.pi:
                    desired_heading -= 2 * m.pi

                # pid
                kp_ang = 1
                ki_ang = 0.0
                kd_ang = 0.0
                dt_ang = 0.1

                pid_angular = pid(kp_ang, ki_ang, kd_ang)
                heading_gain, angle_error = pid_angular.calculate_pid(
                    desired_heading, turtlebot_node.orientation_euler[2], dt_ang
                )
                # print(desired_heading * 180 / m.pi, angle_error[0] * 180 / m.pi)

                if distance_to_waypoint <= waypoint_proximity:
                    waypoint_index += 1
                    print(f"Reached waypoint {waypoint_index}")

                if waypoint_index < len(path):
                    # Calculate heading_move based on heading_gain
                    if Obstacle.is_obstacle_in_path(
                        turtlebot_node.current_position,
                        (target_x, target_y),
                        obstacle_list,
                    ):
                        turtlebot_node.move_turtle(0.0, 0.0)

                    else:
                        ang_vel_cmd = max(
                            -max_ang_speed_rad, min(heading_gain, max_ang_speed_rad)
                        )
                        turtlebot_node.move_turtle(lin_vel_cmd, ang_vel_cmd)

            rclpy.spin_once(turtlebot_node)

            x_pos_history.append(turtlebot_node.current_position[0])
            y_pos_history.append(turtlebot_node.current_position[1])
            # print(path)
            if waypoint_index == len(path):
                break

        # Create the desired vs actual plot
        plt.figure(figsize=(8, 6))
        plt.plot(x_pos_history, y_pos_history, label="Actual Path", color="blue")
        plt.plot(
            [p[0] for p in path],
            [p[1] for p in path],
            label="Desired Path",
            color="red",
        )
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
