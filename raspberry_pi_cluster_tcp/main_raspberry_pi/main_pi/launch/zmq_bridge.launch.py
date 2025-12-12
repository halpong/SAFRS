#!/usr/bin/env python3
"""
SAFRS Main Pi - ZMQ Bridge Launcher
Runs: camera_zmq_sub, lidar_zmq_sub, motor_zmq_pub, odom_zmq_sub
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    ld = LaunchDescription()

    # -------------------------------
    # ZMQ Parameter YAML
    # -------------------------------
    pkg = get_package_share_directory("main_pi")
    zmq_config = os.path.join(pkg, "config", "zmq_params.yaml")

    # -------------------------------
    # Camera ZMQ Subscriber
    # -------------------------------
    ld.add_action(
        Node(
            package="main_pi",
            executable="camera_zmq_sub",
            name="camera_zmq_sub",
            parameters=[zmq_config],
            output="screen"
        )
    )

    # -------------------------------
    # LiDAR ZMQ Subscriber
    # -------------------------------
    ld.add_action(
        Node(
            package="main_pi",
            executable="lidar_zmq_sub",
            name="lidar_zmq_sub",
            parameters=[zmq_config],
            output="screen"
        )
    )

    # -------------------------------
    # Motor ZMQ Publisher
    # -------------------------------
    ld.add_action(
        Node(
            package="main_pi",
            executable="motor_zmq_pub",
            name="motor_zmq_pub",
            parameters=[zmq_config],
            output="screen"
        )
    )

    # -------------------------------
    # Odom ZMQ Subscriber
    # -------------------------------
    ld.add_action(
        Node(
            package="main_pi",
            executable="odom_zmq_sub",
            name="odom_zmq_sub",
            parameters=[zmq_config],
            output="screen"
        )
    )

    return ld
