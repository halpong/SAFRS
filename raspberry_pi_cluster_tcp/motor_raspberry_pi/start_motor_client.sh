#!/bin/bash
# ----------------------------------------
# SAFRS MotorPi ROS2 Node Launcher
# ----------------------------------------

source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash

echo "[MotorPi] Starting ROS2 motor nodes..."

# Run all nodes in background
ros2 run motor_pi motor_serial_node &
ros2 run motor_pi motor_cmd_node &
ros2 run motor_pi motor_odom_node &

# Prevent script from exiting
wait
