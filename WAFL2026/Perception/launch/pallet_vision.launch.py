"""
pallet_vision.launch.py
========================
All-in-one launch file for the integrated pallet_vision system.

Launches:
  1. kinect_ros2_node             – Kinect v1 driver, remapped under /kinect/ namespace
  2. pallet_pose_node             – ArUco marker detection + pallet ID lookup
  3. pallet_navigator             – Nav2 dispatcher driven by ArUco detections
  4. pallet_front_angle_node      – YOLO + depth fine-alignment helper (Jetson GPU)

The kinect driver is remapped so it publishes:
  /kinect/image_raw
  /kinect/depth/image_raw
  /kinect/camera_info
This satisfies both nodes:
  - pallet_pose_node reads /kinect/image_raw (override of /image_raw)
  - pallet_front_angle_node reads /kinect/image_raw, /kinect/depth/image_raw

Usage
-----
  ros2 launch pallet_vision pallet_vision.launch.py

Disable the YOLO node (e.g. on a laptop without CUDA):
  ros2 launch pallet_vision pallet_vision.launch.py enable_yolo:=false

Disable the Kinect driver (e.g. if you start it manually):
  ros2 launch pallet_vision pallet_vision.launch.py enable_kinect:=false

Override marker length:
  ros2 launch pallet_vision pallet_vision.launch.py marker_length:=0.105
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Kinect v1 RGB calibration
KINECT_RGB_MATRIX = [
    526.60717328,   0.0,          318.52510740,
      0.0,        526.60717328,   241.18145973,
      0.0,          0.0,            1.0,
]
KINECT_RGB_DIST = [0.10265184, -0.18646532, 0.0, 0.0, 0.0]


def generate_launch_description() -> LaunchDescription:

    # ── Overridable CLI arguments ──────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "enable_kinect", default_value="true",
            description="Start the kinect_ros2 driver (set false if running separately)",
        ),
        DeclareLaunchArgument(
            "enable_yolo", default_value="true",
            description="Start the YOLO/TensorRT angle node (requires CUDA GPU)",
        ),
        DeclareLaunchArgument(
            "image_topic", default_value="/image_raw",
            description="RGB image topic published by kinect_ros2_node",
        ),
        DeclareLaunchArgument(
            "camera_frame", default_value="camera_rgb_optical_frame",
            description="TF frame attached to the camera",
        ),
        DeclareLaunchArgument(
            "marker_length", default_value="0.07",
            description="ArUco marker side length in metres (black border edge to edge)",
        ),
    ]

    # ── Kinect driver, remapped under /kinect/ ────────────────────────
    # The driver natively publishes on /image_raw and /depth/image_raw – we remap
    # everything into the /kinect/ namespace so both downstream nodes work.
    kinect_node = Node(
        package="kinect_ros2",
        executable="kinect_ros2_node",
        name="kinect_ros2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_kinect")),

    )

    # ── ArUco pose node (your code) ───────────────────────────────────
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

    # ── Nav2 dispatcher (your code) ───────────────────────────────────
    navigator_node = Node(
        package="pallet_vision",
        executable="pallet_navigator",
        name="pallet_navigator",
        output="screen",
    )

    # ── YOLO + depth fine-alignment (teammate's code) ─────────────────
    yolo_node = Node(
        package="pallet_vision",
        executable="pallet_front_angle_node",
        name="pallet_front_angle_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_yolo")),
        additional_env={"DISPLAY": os.environ.get("DISPLAY", ":0")},
        parameters=[{
            "rgb_topic":         "/image_raw",
            "depth_topic":       "/depth/image_raw",
            "camera_info_topic": "/camera_info",
            # Intrinsics – matches the Kinect calibration above
            "fx": 526.60717328,
            "fy": 526.60717328,
            "cx": 318.5251074,
            "cy": 241.18145973,
        }],
    )

    return LaunchDescription(args + [kinect_node, pose_node, navigator_node, yolo_node])
