#!/usr/bin/env python3
"""
SAFRS Main Pi - RViz Launcher
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("main_pi")
    rviz_file = os.path.join(pkg, "config", "default.rviz")

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_file]
        )
    ])
