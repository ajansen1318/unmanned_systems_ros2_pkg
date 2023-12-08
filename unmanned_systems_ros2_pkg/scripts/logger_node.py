#!/usr/bin/env python3

# import modules
import rclpy
from rclpy.node import Node
import csv
import datetime

# import messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from unmanned_systems_ros2_pkg import quaternion_tools


def get_time_in_secs(some_node: Node) -> float:
    return some_node.get_clock().now().nanoseconds / 1e9


class LoggerNode(Node):
    def __init__(self, namespace: str):
        super().__init__("logger_node")
        self.current_position = [None, None]
        self.orientation_euler = [None, None, None]
        self.movement = [0, 0]

        # subscriptions
        self.odom_subscriber = self.create_subscription(
            Odometry, namespace + "/odom", self.odom_callback, 10
        )
        self.vel_subscriber = self.create_subscription(
            Twist, namespace + "/cmd_vel", self.vel_callback, 10
        )

    def odom_callback(self, msg):
        # subscribe to odometry
        self.current_position[0] = msg.pose.pose.position.x
        self.current_position[1] = msg.pose.pose.position.y

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        roll, pitch, yaw = quaternion_tools.euler_from_quaternion(qx, qy, qz, qw)

        self.orientation_euler[0] = roll
        self.orientation_euler[1] = pitch
        self.orientation_euler[2] = yaw

    def vel_callback(self, msg: Twist) -> None:
        # subscribe to twist?
        self.movement[0] = msg.linear.x
        self.movement[1] = msg.angular.z

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z


def main():
    FILEPATH = (
        "/home/aj/ros2_ws/src/unmanned_systems_ros2_pkg/unmanned_systems_ros2_pkg/log/"
    )

    rclpy.init(args=None)
    logger_node = LoggerNode("logger_node")

    # header
    myData = ["time", "x", "y", "yaw", "vel_x", "yaw_vel"]
    fileNameBase = FILEPATH + datetime.datetime.now().strftime("%b_%d_%H_%M")
    fileNameSuffix = ".csv"
    fileName = fileNameBase + fileNameSuffix

    myFile = open(fileName, "a")
    with myFile:
        writer = csv.writer(myFile)
        writer.writerow(myData)

    time_now = get_time_in_secs(logger_node)

    while rclpy.ok():
        now = get_time_in_secs(logger_node) - time_now

        myData = [
            now,
            logger_node.current_position[0],
            logger_node.current_position[1],
            logger_node.orientation_euler[2],
            logger_node.movement[0],
            logger_node.movement[1],
        ]

        print(myData)

        # stick everything in the file
        myFile = open(fileName, "a")
        with myFile:
            writer = csv.writer(myFile)
            writer.writerow(myData)

        rclpy.spin_once(logger_node)


if __name__ == "__main__":
    main()
