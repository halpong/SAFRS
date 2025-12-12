#!/usr/bin/env python3
"""
SAFRS Main Pi Launch File

- Launches Main Bridge Node (D-Pi)
- Central mode broadcast & cmd_vel relay node
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    main_bridge_node = Node(
        package="main_pi",
        executable="main_bridge_node",
        name="main_bridge",
        output="screen",
    )

    return LaunchDescription([
        main_bridge_node
    ])
