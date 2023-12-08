#!/usr/bin/env python3

# import modules here
import rclpy
import math as m
from rclpy.node import Node
import datetime
import os
import csv
import numpy as np

# import messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from unmanned_systems_ros2_pkg import pid_ctrl


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


# class node to run
class TurtleBot_Node(Node):
    def __init__(self):
        super().__init__("hw4_bot")

        # publisher info
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # subscriber info
        self.odom_subscriber = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.vel_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.vel_callback, 10
        )

        self.current_position = [0, 0]
        self.orientation_quat = [0, 0, 0, 0]  # x,y,z,w
        self.orientation_euler = [0, 0, 0]  # roll, pitch, yaw
        self.movement = [0, 0]  # linear, angular velocity

    def odom_callback(self, msg: Odometry) -> None:
        # subscribe to odometry
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion(qx, qy, qz, qw)

        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch
        self.orientation_euler[2] = yaw

    def vel_callback(self, msg: Twist) -> None:
        # subscribe to twist?
        self.movement[0] = msg.linear.x
        self.movement[1] = msg.angular.z

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

    def move_turtle(self, linear_vel: float, angular_vel: float) -> None:
        """Moves turtlebot"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)


def main() -> None:
    # initiate and create node
    rclpy.init(args=None)
    turtlebot_node = TurtleBot_Node()

    # PID parameters
    kp_angular = 0.5
    ki_angular = 0
    kd_angular = 0
    dt_angular = 0.1
    MAX_ANG_SPEED_RAD = 2.84

    # wp list to hit
    wp_list = [(0, 1), (2, 2), (3, -3), (0, 0)]

    # run pid for heading
    pid_angular = pid_ctrl.PID(
        kp=kp_angular, ki=ki_angular, kd=kd_angular, dt=dt_angular
    )

    heading_error_tol_rad = np.deg2rad(2)
    distance_error_tol_m = 0.15
    # while running node
    try:
        while rclpy.ok():
            current_wp = wp_list[0]

            dx = current_wp[0] - turtlebot_node.current_position[0]
            dy = current_wp[1] - turtlebot_node.current_position[1]
            desired_heading_rad = m.atan2(dy, dx)  # in radians
            desired_heading_deg = np.rad2deg(desired_heading_rad)

            current_heading_error = pid_angular.compute_error(
                desired_heading_rad, turtlebot_node.orientation_euler[2]
            )

            # add current distance error

            current_distance_error = np.sqrt(dy**2 + dx**2)
            while abs(current_heading_error) >= heading_error_tol_rad:
                current_heading_error = pid_angular.compute_error(
                    desired_heading_rad, turtlebot_node.orientation_euler[2]
                )

                if abs(current_heading_error) <= heading_error_tol_rad:
                    print("i did it")
                    break
                angular_gains = pid_angular.get_gains(
                    desired_heading_rad, turtlebot_node.orientation_euler[2]
                )
                if angular_gains > MAX_ANG_SPEED_RAD:
                    angular_gains = MAX_ANG_SPEED_RAD
                elif angular_gains <= -MAX_ANG_SPEED_RAD:
                    angular_gains = -MAX_ANG_SPEED_RAD
                turtlebot_node.move_turtle(0.0, angular_gains)

                rclpy.spin_once(turtlebot_node)

            while current_distance_error >= distance_error_tol_m:
                dx = current_wp[0] - turtlebot_node.current_position[0]
                dy = current_wp[1] - turtlebot_node.current_position[1]
                current_distance_error = np.sqrt(dy**2 + dx**2)
                if current_distance_error <= distance_error_tol_m:
                    print("converged to wp")
                    turtlebot_node.move_turtle(0.0, 0.0)

                    break

                turtlebot_node.move_turtle(0.215, 0.0)

            else:
                turtlebot_node.move_turtle(0.0, 0.0)
                rclpy.shutdown()

    except KeyboardInterrupt:
        turtlebot_node.move_turtle(0.0, 0.0)


if __name__ == "__main__":
    main()
