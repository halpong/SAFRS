#!/usr/bin/env python3
"""
SAFRS Main Pi - Navigation Launcher
Loads: map.yaml + nav2_params.yaml + nav2_bringup
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("main_pi")
    nav2_pkg = get_package_share_directory("nav2_bringup")

    map_file = os.path.join(pkg, "config", "map.yaml")
    params_file = os.path.join(pkg, "config", "nav2_params.yaml")

    return LaunchDescription([

        # Map argument
        DeclareLaunchArgument(
            "map",
            default_value=map_file,
            description="Map YAML file"
        ),

        # NAV2 bringup
        Node(
            package="nav2_bringup",
            executable="bringup_launch.py",
            output="screen",
            parameters=[params_file],
            arguments=[
                "--map", LaunchConfiguration("map"),
                "--params-file", params_file,
                "--autostart", "true"
            ]
        ),
    ])
