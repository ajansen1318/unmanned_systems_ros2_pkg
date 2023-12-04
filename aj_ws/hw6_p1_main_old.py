#!/usr/bin/env python3

import rclpy
import numpy as np
import math as m
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from turtle.turtlebot_node import TurtleBot_Node
from turtle.utility import get_time_in_secs
from turtle.pid import PIDControl as pid
from turtle.ProNav import ProNav


def compute_heading(current_pos: np.ndarray, des_pos: np.ndarray) -> float:
    """compute desired heading based on positions"""
    return np.arctan2(des_pos[1] - current_pos[1], des_pos[0] - current_pos[0])


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


# def get_mean_target(heading_list: list) -> float:
#     average = np.mean(heading_list)
#     return average


def get_mean_target(heading_list: list) -> float:
    # Check if the list is not empty
    if not heading_list:
        # Return a default value or raise an exception, depending on your use case
        return np.nan  # or any default value you prefer for an empty list

    # Calculate the mean of the non-empty list
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
    # set parameters
    ### time_now = get_time_in_secs(pursuer)
    max_ang_speed_rad = 2.84
    heading_error_tol_rad = np.deg2rad(1)
    dt = 0.1
    waypoint_proximity = 0.1

    # pursuer parameters
    pursuer_velocity = 0.2  # m/s

    # evader parameters
    evader_velocity = 0.15  # m/s

    #### PRO NAVOPTIONS ####
    pro_nav_options = ["Simple ProNav", "True ProNav", "Augmented ProNav"]
    guidance_option = pro_nav_options[1]
    nav_constant = 5.0
    pro_nav = ProNav(nav_constant)

    # pid parameters
    kp_ang = 3
    ki_ang = 0.25
    kd_ang = 0.5
    dt_ang = 0.1

    # initiate and create node
    rclpy.init(args=None)
    pursuer = TurtleBot_Node(
        max_speed=pursuer_velocity,
        max_turn_rate=max_ang_speed_rad,
        node_name="turtle",
        ns="pursuer",
    )
    evader = TurtleBot_Node(
        max_speed=evader_velocity,
        max_turn_rate=max_ang_speed_rad,
        node_name="turtle",
        ns="evader",
    )
    evader.current_position = [2.0, 2.0]
    pursuer_position_history = []
    evader_position_history = []
    pursuer_heading_history = []
    evader_heading_history = []

    try:
        while rclpy.ok():
            RANDOM_VEL = False
            RANDOM_ANGLE = False
            # compute current los
            from_lidar_heading = np.deg2rad(
                get_mean_target(pursuer.detected_heading_angle_list)
            )
            print(from_lidar_heading)
            LOS = global_los(from_lidar_heading, pursuer.orientation_euler[2])

            current_distance = compute_distance(
                pursuer.current_position,
                evader.current_position,
            )

            # use simple pro nav
            if guidance_option == pro_nav_options[0]:
                flight_path_rate = pro_nav.simple_pro_nav(LOS, dt)
                pursuer.move_turtle(pursuer_velocity, flight_path_rate)
            # use true pro nav
            if guidance_option == pro_nav_options[1]:
                target_vel = np.array(
                    [
                        evader_velocity * np.cos(LOS),
                        evader_velocity * np.sin(LOS),
                    ]
                )
                pursuer_velocity_other = np.array(
                    [
                        pursuer_velocity * np.cos(pursuer.orientation_euler[2]),
                        pursuer_velocity * np.sin(pursuer.orientation_euler[2]),
                    ]
                )
                flight_path_rate, vel_cmd = pro_nav.true_pro_nav(
                    pursuer.current_position,
                    evader.current_position,
                    dt,
                    target_vel,
                    pursuer_velocity_other,  # sorry idk what it should be called
                    True,
                    LOS,
                )
                pursuer.move_turtle(vel_cmd, flight_path_rate)
                print(LOS)
            if guidance_option == pro_nav_options[2]:
                target_vel = np.array(
                    [
                        evader_velocity * np.cos(LOS),
                        evader_velocity * np.sin(LOS),
                    ]
                )
                pursuer_velocity_other = np.array(
                    [
                        pursuer_velocity * np.cos(pursuer.orientation_euler[2]),
                        pursuer_velocity * np.sin(pursuer.orientation_euler[2]),
                    ]
                )
                flight_path_rate, vel_cmd = pro_nav.augmented_pro_nav(
                    pursuer.current_position,
                    evader.current_position,
                    dt,
                    target_vel,
                    pursuer_velocity_other,
                )
                pursuer.move_turtle(vel_cmd, flight_path_rate)
            pursuer_position_history.append(pursuer.current_position)
            pursuer_heading_history.append(pursuer.orientation_euler[2])
            ## move the evader
            # if RANDOM_VEL == True and RANDOM_ANGLE == False:
            #     random_vel = np.random.uniform(
            #         evader.current_velocity[0] / 2, evader.current_velocity[0]
            #     )
            #     evader.move_turtle(random_vel, 0)
            # elif RANDOM_VEL == False and RANDOM_ANGLE == True:
            #     random_ang_vel = np.random.uniform(-np.pi / 2, np.pi / 2)
            #     evader.move_turtle(evader.current_velocity[0], random_ang_vel)
            # elif RANDOM_VEL == True and RANDOM_ANGLE == True:
            #     random_vel = np.random.uniform(
            #         evader.current_velocity[0] / 2, evader.current_velocity[0]
            #     )
            #     random_ang_vel = np.random.uniform(-np.pi / 2, np.pi / 2)
            #     evader.move_turtle(random_vel, random_ang_vel)
            # else:
            #     evader.move_turtle(evader_velocity, 0)
            #     # target_x, target_y = (9, 9)
            #     # dx = target_x - evader.current_position[0]
            #     # dy = target_y - evader.current_position[1]
            #     # distance_to_wp = m.sqrt(dx**2 + dy**2)
            #     # print(distance_to_wp)
            #     # # evader heading pid
            #     # if distance_to_wp <= waypoint_proximity:
            #     #     evader_heading_pid = pid(kd_ang, ki_ang, kd_ang)
            #     #     heading_gain, angle_error = evader_heading_pid.calculate_pid(
            #     #         LOS, evader.orientation_euler[2], dt_ang
            #     #     )
            #     #     ang_vel_cmd = max(
            #     #         -max_ang_speed_rad, min(heading_gain, max_ang_speed_rad)
            #     #     )
            #     #     print(ang_vel_cmd)
            #     #     evader.move_turtle(evader_velocity, ang_vel_cmd)
            # # rclpy.spin_once(evader)
            evader_position_history.append(evader.current_position)
            evader_heading_history.append(evader.orientation_euler[2])
            if check_close(pursuer.current_position, evader.current_position) == True:
                print("Pursuer caught evader")
                break
            rclpy.spin_once(pursuer)

        pursuer_x = [x[0] for x in pursuer_position_history]
        pursuer_y = [x[1] for x in pursuer_position_history]
        pursuer_heading_dg = [np.rad2deg(x) for x in pursuer_heading_history]
        evader_x = [x[0] for x in evader_position_history]
        evader_y = [x[1] for x in evader_position_history]
        evader_heading_dg = [np.rad2deg(x) for x in evader_heading_history]
        # plot
        fig, ax = plt.subplots()
        ax.plot(evader_x, evader_y, "-o", label="evader")
        ax.plot(pursuer_x, pursuer_y, "-x", label="pursuer")
        ax.set_title(guidance_option)
        ax.legend()
        # plot heading
        fig1, ax1 = plt.subplots(2, 1)
        ax1[0].plot(pursuer_heading_dg, label="pursuer heading")
        ax1[0].legend()
        plt.show()

    except KeyboardInterrupt:
        pursuer.move_turtle(0.0, 0.0)


if __name__ == "__main__":
    main()
