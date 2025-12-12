from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # model files
        (os.path.join('share', package_name, 'model'),
         glob('model/*')),

        # config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SAFRS',
    maintainer_email='example@example.com',
    description='Camera Pi node for ally/enemy classification and pose-based ROI extraction.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = camera_pi.camera_node:main',
            'color_class_node = camera_pi.color_class_node:main',
        ],
    },
)
