#!/usr/bin/env python

from ipaddress import ip_address
import pandas as pd
import numpy as np
import rclpy
from rclpy.node import Node

import time
import csv
import os
import datetime

from nav_msgs.msg import Odometry

from unmanned_systems_ros2_pkg import quaternion_tools


class Logger(Node):
    FILEPATH = "/home/aj/ros2_ws/src/unmanned_systems_ros2_pkg/log/"
    FILENAME = "hw4_p1_data.csv"

    print(os.getcwd())
    rclpy.init(args=None)
    odom_node = TurtleBot_Node("odom")
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
