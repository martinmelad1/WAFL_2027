"""
yolo.launch.py
===============
Launches only the YOLO fine-alignment pipeline:
  - pallet_front_angle_node  (TensorRT detection + depth-based angle estimation)

Usage
-----
  ros2 launch pallet_vision yolo.launch.py

Overrides:
  ros2 launch pallet_vision yolo.launch.py keypoint_conf:=0.05
  ros2 launch pallet_vision yolo.launch.py debug_view:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("rgb_topic",       default_value="/image_raw"),
        DeclareLaunchArgument("depth_topic",     default_value="/depth/image_raw"),
        DeclareLaunchArgument("keypoint_conf",   default_value="0.10"),
        DeclareLaunchArgument("debug_view",      default_value="true"),
    ]

    yolo_node = Node(
        package="pallet_vision",
        executable="pallet_front_angle_node",
        name="pallet_front_angle_node",
        output="screen",
        parameters=[{
            "rgb_topic":         LaunchConfiguration("rgb_topic"),
            "depth_topic":       LaunchConfiguration("depth_topic"),
            "camera_info_topic": "/camera_info",
            "fx": 526.60717328,
            "fy": 526.60717328,
            "cx": 318.5251074,
            "cy": 241.18145973,
            "keypoint_conf":     LaunchConfiguration("keypoint_conf"),
            "debug_view":        LaunchConfiguration("debug_view"),
        }],
    )

    return LaunchDescription(args + [yolo_node])
