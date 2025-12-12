from setuptools import setup

package_name = 'motor_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/config', [
            'config/odom_params.yaml',
            'config/serial_params.yaml',
            'config/zmq_params.yaml'
        ]),

        ('share/' + package_name + '/service', [
            'service/motor_pi.service',
            'service/start_motor_pi.sh'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAFRS',
    maintainer_email='example@example.com',
    description='SAFRS MotorPi ROS2 driver (odometry, ZMQ bridge, serial control).',
    license='MIT',
    tests_require=['pytest'],

    # -----------------------------
    # Entry point: ros2 run motor_pi motor_cmd_node
    # -----------------------------
    entry_points={
        'console_scripts': [
            'motor_cmd_node = motor_pi.motor_cmd_node:main',
            'motor_serial_node = motor_pi.motor_serial_node:main',
            'motor_odom_node = motor_pi.motor_odom_node:main',
        ],
    },
)
