from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'lidar_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAFRS',
    maintainer_email='example@example.com',
    description='SAFRS LiDAR driver package for YDLIDAR X4 / X4 Pro.',
    license='MIT',

    data_files=[
        # Package registration
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],

    entry_points={
        'console_scripts': [
            'lidar_driver_node = lidar_driver.lidar_node:main',
        ],
    },
)
