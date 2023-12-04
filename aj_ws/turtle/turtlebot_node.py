#!/usr/bin/env python3

# import modules here
import numpy as np
from rclpy.node import Node
import turtle.utility as utility
from turtle.pid import PIDControl


# import messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TurtleBot_Node(Node):
    def __init__(
        self,
        max_speed: float,
        max_turn_rate: float,
        node_name: str,
        ns="",
        controller=None,
    ):
        super().__init__(node_name)

        if ns != "":
            self.ns = ns
        else:
            self.ns = ns

        if controller != None:
            self.pid = PIDControl(
                kp=controller[0], ki=controller[1], kd=controller[2], dt=controller[3]
            )

        # publisher info
        self.vel_publisher = self.create_publisher(Twist, self.ns + "/cmd_vel", 10)

        # subscriber info
        self.odom_subscriber = self.create_subscription(
            Odometry, self.ns + "/odom", self.odom_callback, 10
        )
        self.vel_subscriber = self.create_subscription(
            Twist, self.ns + "/cmd_vel", self.vel_callback, 10
        )
        self.lidar_subscriber = self.create_subscription(
            LaserScan, self.ns + "/scan", self.lidar_track_cb, 1
        )

        self.current_position = [0, 0]
        self.orientation_quat = [0, 0, 0, 0]  # x,y,z,w
        self.orientation_euler = [0, 0, 0]  # roll, pitch, yaw
        self.current_velocity = [0, 0]  # linear x, angular velocity

        self.detected_range_list = []  # depth detected
        self.detected_heading_angle_list = []  # heading detected

        self.max_speed = max_speed
        self.max_turn_rate = max_turn_rate

    def odom_callback(self, msg: Odometry) -> None:
        # subscribe to odometry
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = utility.euler_from_quaternion(qx, qy, qz, qw)

        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch
        self.orientation_euler[2] = yaw

        # if self.orientation_euler[2] < 0:
        #     self.orientation_euler[2] += 2 * np.pi
        # else:
        #     self.orientation_euler[2] = self.orientation_euler[2]

    def vel_callback(self, msg: Twist) -> None:
        # subscribe to twist
        self.current_velocity[0] = msg.linear.x
        self.current_velocity[1] = msg.angular.z

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

    def move_turtle(self, linear_vel: float, angular_vel: float) -> None:
        twist = Twist()

        linear_vel = np.clip(linear_vel, -self.max_speed, self.max_speed)
        angular_vel = np.clip(angular_vel, -self.max_turn_rate, self.max_turn_rate)

        if linear_vel >= 0.23:
            linear_vel = 0.23
        elif linear_vel <= -0.23:
            linear_vel = -0.23

        if angular_vel >= 2.84:
            angular_vel = 2.84
        elif angular_vel <= -2.84:
            angular_vel = -2.84

        twist.linear.x = float(linear_vel)
        twist.angular.z = float(angular_vel)
        self.vel_publisher.publish(twist)

    def lidar_track_cb(self, msg: LaserScan):
        """lidar information remember the msg is an array of 0-> 359"""
        self.detected_range_list = []
        self.detected_heading_angle_list = []
        inf = float("inf")

        lidar_vals = msg.ranges

        # append detections if values not infinity or 0.0
        for i, val in enumerate(lidar_vals):
            if val != inf:
                self.detected_heading_angle_list.append(i)
                self.detected_range_list.append(val)
