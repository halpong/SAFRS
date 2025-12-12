#!/usr/bin/env python3
"""
SAFRS Main Pi - Utility Module
Module: param_utils

Description:
    ROS2 parameter declaration + YAML loader helper functions.  
"""

import yaml


def load_yaml(file_path: str) -> dict:
    """
    Load YAML file into Python dictionary.
    """
    with open(file_path, "r") as f:
        return yaml.safe_load(f)


def declare_and_get(node, param_name: str, default_value):
    """
    Declare a ROS2 parameter and return its value.

    Example:
        ip = declare_and_get(self, "lidar_ip", "172.30.1.14")
    """
    node.declare_parameter(param_name, default_value)
    return node.get_parameter(param_name).value
