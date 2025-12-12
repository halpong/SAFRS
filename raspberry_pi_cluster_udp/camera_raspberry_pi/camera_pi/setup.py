from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Config files
        (os.path.join('share', package_name, 'config'),
         glob('camera_pi/config/*.yaml')),

        # Model files
        (os.path.join('share', package_name, 'model'),
         glob('camera_pi/model/*')),

        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('camera_pi/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='SAFRS Camera Raspberry Pi UDP Client',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Final A-Pi executable (UDP version)
            'camera_udp_client = camera_pi.camera_udp_client:main',
        ],
    },
)
