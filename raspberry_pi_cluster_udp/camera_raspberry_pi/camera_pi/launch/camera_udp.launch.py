#!/usr/bin/env python3
"""
SAFRS Camera Raspberry Pi – UDP Camera Client Launch File
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_dir = get_package_share_directory("camera_pi")

    # Load config files
    camera_config = os.path.join(pkg_dir, "config", "camera_params.yaml")
    inference_config = os.path.join(pkg_dir, "config", "inference_params.yaml")
    udp_config = os.path.join(pkg_dir, "config", "udp_params.yaml")

    return LaunchDescription([
        Node(
            package="camera_pi",
            executable="camera_udp_client",
            name="camera_udp_client",
            output="screen",
            parameters=[
                camera_config,
                inference_config,
                udp_config
            ]
        )
    ])
