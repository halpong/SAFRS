#!/bin/bash
set -e

echo "[SAFRS-LiDAR] Starting LiDAR Driver..."

# ------------------------------
# Load ROS2 environment
# ------------------------------
source /opt/ros/humble/setup.bash

# Workspace setup (modify if different)
if [ -f /home/ubuntu/ros2_ws/install/setup.bash ]; then
    source /home/ubuntu/ros2_ws/install/setup.bash
fi

# ------------------------------
# Execute LiDAR Node
# ------------------------------
exec python3 -m lidar_driver.lidar_node
