#!/usr/bin/env python3

# import modules here
from rclpy.node import Node
import turtle.utility as utility


# import messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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

        self.current_position = [None, None]
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

        roll, pitch, yaw = utility.euler_from_quaternion(qx, qy, qz, qw)

        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch
        self.orientation_euler[2] = yaw

    def vel_callback(self, msg: Twist) -> None:
        # subscribe to twist
        self.movement[0] = msg.linear.x
        self.movement[1] = msg.angular.z

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

    def move_turtle(self, linear_vel: float, angular_vel: float) -> None:
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.vel_publisher.publish(twist)
