#!/bin/bash
# SAFRS Camera Pi Start Script

# --------------------------------------
# 1) ROS2 환경 설정
# --------------------------------------
source /opt/ros/humble/setup.bash

# Camera Pi workspace (필요하면 수정)
if [ -f /home/ubuntu/camera_pi/install/setup.bash ]; then
    source /home/ubuntu/camera_pi/install/setup.bash
fi

# --------------------------------------
# 2) Python venv 로드 (선택)
# --------------------------------------
if [ -f /home/ubuntu/venv/bin/activate ]; then
    source /home/ubuntu/venv/bin/activate
fi

# --------------------------------------
# 3) Camera Node 실행
# --------------------------------------
echo "[CameraPi] Starting camera_node..."
exec ros2 run camera_pi camera_node
