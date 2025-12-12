from setuptools import setup, find_packages
import os
from glob import glob

package_name = "udp_to_drive"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SAFRS",
    maintainer_email="example@example.com",
    description="SAFRS - UDP → ROS2 bridge module for AGV start trigger.",
    license="MIT",

    data_files=[
        # Package index
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),

        # package.xml
        ("share/" + package_name, ["package.xml"]),

        # config files
        (os.path.join("share", package_name, "config"),
         glob("udp_to_drive/config/*.yaml")),

        # launch files
        (os.path.join("share", package_name, "launch"),
         glob("launch/*.launch.py")),
    ],

    entry_points={
        "console_scripts": [
            "udp_bridge = udp_to_drive.udp_bridge:main",
        ],
    },
)
