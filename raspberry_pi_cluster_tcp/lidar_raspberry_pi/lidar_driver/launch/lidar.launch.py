#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # 패키지 내부 config 경로 (lidar_params.yaml)
    pkg_share = get_package_share_directory('lidar_driver')
    params_file = os.path.join(pkg_share, 'config', 'lidar_params.yaml')

    return LaunchDescription([
        Node(
            package='lidar_driver',
            executable='lidar_driver_node',   # setup.py에서 등록할 실행 이름
            name='lidar_x4pro_driver',
            output='screen',
            parameters=[params_file]
        )
    ])
