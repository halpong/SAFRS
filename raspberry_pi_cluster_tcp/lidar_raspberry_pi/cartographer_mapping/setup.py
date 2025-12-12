# ==============================================================
# SAFRS Cartographer Mapping Package - setup.py
# ==============================================================

from setuptools import setup, find_packages

package_name = "cartographer_mapping"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SAFRS",
    maintainer_email="example@example.com",
    description="Cartographer SLAM integration package for SAFRS LiDAR Pi",
    license="MIT",

    # --------------------------------------------------------------
    # Data Files (ament index, package.xml, config, launch)
    # --------------------------------------------------------------
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name]
        ),

        ("share/" + package_name, ["package.xml"]),

        ("share/" + package_name + "/launch", [
            "launch/mapping.launch.py"
        ]),

        ("share/" + package_name + "/config", [
            "config/cartographer.lua",
            "config/cartographer_params.yaml",
        ]),
    ],

    # No entrypoints required for Cartographer package
)
