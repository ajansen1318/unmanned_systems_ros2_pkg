from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# run logger and turtle node
def generate_launch_description():
    pursuer_ns = LaunchConfiguration("pursuer_ns", default="pn")
    pursuer_node = Node(
        package="unmanned_systems_ros2_pkg",
        namespace=pursuer_ns,
        executable="hw6_p1_main.py",
    )
    evader_ns = LaunchConfiguration("evader_ns", default="evader")
    evader_node = Node(
        package="unmanned_systems_ros2_pkg",
        namespace=evader_ns,
        executable="evader.py",
    )
    launch_description = LaunchDescription([evader_node])
    return launch_description
