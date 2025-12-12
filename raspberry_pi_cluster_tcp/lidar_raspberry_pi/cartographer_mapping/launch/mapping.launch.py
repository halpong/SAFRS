#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory("cartographer_mapping")

    lua_config = os.path.join(pkg, "config", "cartographer.lua")
    param_yaml = os.path.join(pkg, "config", "cartographer_params.yaml")

    return LaunchDescription([

        # ================================
        # Cartographer Node
        # ================================
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            output="screen",
            parameters=[param_yaml],
            arguments=["-configuration_directory", os.path.join(pkg, "config"),
                       "-configuration_basename", "cartographer.lua"]
        ),

        # ================================
        # Occupancy Grid Publisher
        # ================================
        Node(
            package="cartographer_ros",
            executable="occupancy_grid_node",
            name="occupancy_grid",
            output="screen",
            parameters=[
                {"resolution": 0.05}
            ]
        ),
    ])
