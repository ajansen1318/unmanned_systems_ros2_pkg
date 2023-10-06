#!/usr/bin/env python3

# import modules here
import rclpy
import math as m
from rclpy.node import Node
import datetime
import os
import csv

# import messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


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

    # set parameters
    lin_vel_cmd = 1.5  # m/s
    ang_vel_cmd = 0.15  # rad/s
    vel_dur = 5  # time going forward
    turn_dur = 2  # time while turning
    time_now = get_time_in_secs(turtlebot_node)

    # save file info
    FILEPATH = (
        "/home/aj/ros2_ws/src/unmanned_systems_ros2_pkg/unmanned_systems_ros2_pkg/log/"
    )
    FILENAME = "hw4_p1_data.csv"

    myData = ["time", "x", "y", "yaw", "vel_x", "yaw_vel"]
    fileNameBase = FILEPATH + datetime.datetime.now().strftime("%b_%d_%H_%M")
    fileNameSuffix = ".csv"
    num = 1
    fileName = fileNameBase + fileNameSuffix
    while os.path.isfile(fileName):
        fileName = fileNameBase + "_" + str(num) + "_" + fileNameSuffix
        num = num + 1

    myFile = open(fileName, "a")
    with myFile:
        writer = csv.writer(myFile)
        writer.writerow(myData)

    # while running node
    while rclpy.ok():
        time_diff = get_time_in_secs(turtlebot_node) - time_now
        # for logging
        myData = [
            time_diff,
            turtlebot_node.current_position[0],
            turtlebot_node.current_position[1],
            turtlebot_node.orientation_euler[2],
            turtlebot_node.movement[0],
            turtlebot_node.movement[1],
        ]
        myFile = open(fileName, "a")
        with myFile:
            writer = csv.writer(myFile)
            writer.writerow(myData)

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


if __name__ == "__main__":
    main()
