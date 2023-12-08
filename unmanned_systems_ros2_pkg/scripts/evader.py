#!/usr/bin/env python3
import rclpy
import math as m

from random import randint
from rclpy.node import Node
from unmanned_systems_ros2_pkg import TurtleBotNode, quaternion_tools
from pathfinding_library.Obstacle import Obstacle
from pathfinding_library.dijkstra_astar import DijkstraAstar
from pathfinding_library.node import Node
from pathfinding_library.Grid import Grid
from pid import PIDControl as pid


def generate_random_waypoints(n_random_waypoints: int, max_val: int) -> list:
    """generate random waypoints from 1 to 1"""

    random_wp_list = []
    for i in range(0, n_random_waypoints + 1):
        rand_x = randint(0, max_val)
        rand_y = randint(0, max_val)
        random_wp_list.append((rand_x, rand_y))

    return random_wp_list


def compute_desired_heading(current_pos: list, des_pos: list) -> float:
    """compute desired heading based on positions"""
    return m.atan2(des_pos[1] - current_pos[1], des_pos[0] - current_pos[0])


def compute_dist_error(current_pos: list, des_pos: list) -> float:
    """compute distance error"""
    return m.dist(des_pos, current_pos)


def compute_heading_error(current_heading: float, des_heading: float) -> float:
    """compute heading error in radians"""
    return des_heading - current_heading


def gimme_da_loot(turtlebot: TurtleBotNode, waypoint: list) -> list:
    """helper function"""
    desired_heading = compute_desired_heading(turtlebot.current_position, waypoint)

    heading_error = compute_heading_error(
        turtlebot.orientation_euler[2], desired_heading
    )

    dist_error = compute_dist_error(turtlebot.current_position, waypoint)

    return [desired_heading, heading_error, dist_error]


def main() -> None:
    rclpy.init(args=None)

    turtlebot_1 = TurtleBotNode.TurtleBotNode("turtle", "evader")
    turtlebot_1.move_turtle(0.0, 0.0)

    is_done = False
    heading_tol = 0.1
    # radians
    dist_tolerance = 0.25  # meters
    max_turn_speed = 2.84  # rad/speed
    linear_speed = 0.15  # m/s
    stop_speed = 0.0
    waypoint_index = 0
    waypoint_proximity = 0.1
    start_x = 2
    start_y = 1
    goal_x = 7
    goal_y = 2
    obstacle_radius = 0.25
    obstacle_x = [5, 5, 5, 5, 5, 0, 1, 2, 3, 3]
    obstacle_y = [0, 1, 2, 3, 4, 5, 4, 3, 2, 3]
    max_x = 10
    max_y = 10

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
    dijkstra = DijkstraAstar(
        grid=grid, start_node=start_node, end_node=end_node, use_dijkstra=True
    )
    path = dijkstra.find_path()
    path = path[::-1]

    while rclpy.ok():
        if waypoint_index < len(path):
            target_x, target_y = path[waypoint_index]
            dx = target_x - turtlebot_1.current_position[0]
            dy = target_y - turtlebot_1.current_position[1]
            distance_to_waypoint = m.sqrt(dx**2 + dy**2)
            desired_heading = m.atan2(dy, dx)

            if desired_heading - turtlebot_1.orientation_euler[2] > m.pi:
                desired_heading -= 2 * m.pi

            # for pid
            kp_heading = 1
            ki_heading = 0
            kd_heading = 0
            dt_heading = 0.1

            heading_pid = pid(kp_heading, ki_heading, kd_heading)
            heading, heading_error = heading_pid.calculate_pid(
                desired_heading, turtlebot_1.orientation_euler[2], dt_heading
            )

            if distance_to_waypoint <= waypoint_proximity:
                waypoint_index += 1
                print(f"Reached waypoint {waypoint_index}")

            if waypoint_index < len(path):
                if Obstacle.is_obstacle_in_path(
                    turtlebot_1.current_position, (target_x, target_y), obstacle_list
                ):
                    turtlebot_1.move_turtle(0.0, 0.0)
                else:
                    ang_vel_cmd = max(-max_turn_speed, min(heading, max_turn_speed))
                    turtlebot_1.move_turtle(linear_speed, ang_vel_cmd)
        rclpy.spin_once(turtlebot_1)


if __name__ == "__main__":
    main()
