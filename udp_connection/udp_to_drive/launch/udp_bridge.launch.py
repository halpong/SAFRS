#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory("udp_to_drive")

    # Correct SAFRS config path
    config_file = os.path.join(pkg_dir, "config", "config.yaml")

    return LaunchDescription([
        Node(
            package="udp_to_drive",
            executable="udp_bridge",
            name="udp_to_drive_bridge",
            output="screen",
            parameters=[config_file],
        )
    ])
