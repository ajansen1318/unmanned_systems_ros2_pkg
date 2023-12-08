#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# run logger and turtle node
def generate_launch_description():
    turtle_ns = LaunchConfiguration("turtlebot_ns", default="hw5_p1_main")
    turtle_node = Node(
        package="unmanned_systems_ros2_pkg",
        namespace=turtle_ns,
        executable="hw5_p1_main.py",
    )
    logger_ns = LaunchConfiguration("logger_ns", default="logger")
    logger_node = Node(
        package="unmanned_systems_ros2_pkg",
        namespace=logger_ns,
        executable="logger_node.py",
    )
    launch_description = LaunchDescription([logger_node])

    return launch_description
