from setuptools import setup, find_packages
import os
from glob import glob

package_name = "pallet_vision"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Required for ament resource index
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        # Package manifest
        (os.path.join("share", package_name), ["package.xml"]),
        # Launch files
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ahmed",
    maintainer_email="ahmed@todo.todo",
    description="Pallet ArUco detection, Nav2 dispatch, and YOLO fine-alignment",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pallet_pose_node        = pallet_vision.pallet_pose_node:main",
            "pallet_navigator        = pallet_vision.pallet_navigator:main",
            "pallet_front_angle_node = pallet_vision.pallet_front_angle_node:main",
        ],
    },
)
