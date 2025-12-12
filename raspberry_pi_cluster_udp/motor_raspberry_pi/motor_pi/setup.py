from setuptools import setup
from glob import glob
import os

package_name = 'motor_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],

    data_files=[
        # ROS2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),

        # config files
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='KAIROS5 JIYUNFARM',
    maintainer_email='ubuntu@kairos5.local',

    description=(
        'SAFRS Motor Pi (C-Pi) node for STM32 motor control, '
        'odometry, IMU, and tracking command handling'
    ),

    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            # Main STM32 controller node
            'stm_controller = motor_pi.stm_controller_node:main',
        ],
    },
)
