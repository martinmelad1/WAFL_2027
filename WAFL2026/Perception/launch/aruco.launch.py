"""
aruco.launch.py
================
Launches only the ArUco detection pipeline:
  - pallet_pose_node   (ArUco marker detection + pose estimation)
  - pallet_navigator   (Nav2 dispatcher)

Usage
-----
  ros2 launch pallet_vision aruco.launch.py

Overrides:
  ros2 launch pallet_vision aruco.launch.py marker_length:=0.105
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

KINECT_RGB_MATRIX = [
    526.60717328,   0.0,          318.52510740,
      0.0,        526.60717328,   241.18145973,
      0.0,          0.0,            1.0,
]
KINECT_RGB_DIST = [0.10265184, -0.18646532, 0.0, 0.0, 0.0]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("image_topic",   default_value="/image_raw"),
        DeclareLaunchArgument("camera_frame",  default_value="camera_rgb_optical_frame"),
        DeclareLaunchArgument("marker_length", default_value="0.07"),
    ]

    pose_node = Node(
        package="pallet_vision",
        executable="pallet_pose_node",
        name="pallet_pose_node",
        output="screen",
        parameters=[{
            "image_topic":   LaunchConfiguration("image_topic"),
            "camera_frame":  LaunchConfiguration("camera_frame"),
            "marker_length": LaunchConfiguration("marker_length"),
            "camera_matrix": KINECT_RGB_MATRIX,
            "dist_coeffs":   KINECT_RGB_DIST,
        }],
    )

    navigator_node = Node(
        package="pallet_vision",
        executable="pallet_navigator",
        name="pallet_navigator",
        output="screen",
    )

    return LaunchDescription(args + [pose_node, navigator_node])
