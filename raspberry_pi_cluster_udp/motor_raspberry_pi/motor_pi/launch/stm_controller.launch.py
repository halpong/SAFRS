#!/usr/bin/env python3
"""
SAFRS Motor Pi
- STM32 Motor / Gimbal / Odometry Controller Launch
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ------------------------------------------------------------
    # Package info
    # ------------------------------------------------------------
    package_name = "motor_pi"
    package_share = get_package_share_directory(package_name)

    # ------------------------------------------------------------
    # Config files
    # ------------------------------------------------------------
    serial_config = os.path.join(
        package_share, "config", "serial_params.yaml"
    )

    odom_config = os.path.join(
        package_share, "config", "odom_params.yaml"
    )

    # ------------------------------------------------------------
    # STM32 Controller Node
    # ------------------------------------------------------------
    stm_controller_node = Node(
        package=package_name,
        executable="stm_controller_node",
        name="stm_controller",
        output="screen",
        parameters=[
            serial_config,
            odom_config,
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    # ------------------------------------------------------------
    # Launch Description
    # ------------------------------------------------------------
    return LaunchDescription([
        stm_controller_node
    ])
