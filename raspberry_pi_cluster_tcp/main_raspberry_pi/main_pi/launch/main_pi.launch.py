#!/usr/bin/env python3
"""
SAFRS Main Pi - Full System Launcher
Runs:
  • ZMQ Bridge (camera, lidar, motor, odom)
  • TF (laser ↔ base_link)
  • Navigation (AMCL + planner)
  • RViz
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("main_pi")

    return LaunchDescription([

        # ------------------------------------------------
        # 1) ZMQ Bridge
        # ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "zmq_bridge.launch.py")
            )
        ),

        # ------------------------------------------------
        # 2) Static Transform (base_link → laser)
        # ------------------------------------------------
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_tf",
            arguments=[
                "0.10", "0.0", "0.12",
                "0", "0", "0",
                "base_link", "laser"
            ],
            output="screen"
        ),

        # ------------------------------------------------
        # 3) Navigation System (NAV2)
        # ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "navigation.launch.py")
            )
        ),

        # ------------------------------------------------
        # 4) RViz Viewer
        # ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "rviz_view.launch.py")
            )
        ),
    ])
