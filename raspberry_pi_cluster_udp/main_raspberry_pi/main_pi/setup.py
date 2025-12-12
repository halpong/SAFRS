from setuptools import setup, find_packages

package_name = 'main_pi'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        # ROS2 package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch files
        ('share/' + package_name + '/launch',
         ['launch/main_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='SAFRS',
    maintainer_email='ubuntu@example.com',
    description='SAFRS Main Pi (D-Pi) central bridge node',
    license='MIT',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'main_bridge_node = main_pi.main_bridge_node:main',
        ],
    },
)
