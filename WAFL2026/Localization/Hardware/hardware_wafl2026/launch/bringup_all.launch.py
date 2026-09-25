"""
MASTER BRINGUP — single command launches the whole stack.

Equivalent to running these manually:
  ros2 launch robot_odometry robot_hardware.launch.py
  ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
  ros2 launch hardware_wafl2026 laptop_b_localization.launch.py
  ros2 run rviz2 rviz2 -d <nav2_default_view.rviz>
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 rplidar_link laser

Plus: pose_persistence_node saves/restores AMCL pose between runs.

Usage:
  ros2 launch hardware_wafl2026 bringup_all.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # ── Paths ───────────────────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz',
    )

    # ── 1. Robot odometry stack (encoder + IMU + odom_fusion) ───────────────
    robot_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_odometry'),
            'launch', 'robot_hardware.launch.py')]),
    )

    # ── 2. RPLiDAR driver ───────────────────────────────────────────────────
    # NOTE: using the non-"view_" variant so it doesn't open a second RViz.
    # If your installed rplidar_ros only has view_rplidar_a2m8_launch.py,
    # change this to that name — the extra RViz is harmless, just close it.
    rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rplidar_ros'),
            'launch', 'rplidar_a2m8_launch.py')]),
    )

    # ── 3. Static TF: rplidar_link → laser ──────────────────────────────────
    # RPLiDAR driver publishes scans with frame_id="laser", but the URDF
    # uses "rplidar_link". This bridge connects them so AMCL can transform
    # scans into base_footprint and the map.
    laser_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_bridge',
        arguments=['0', '0', '0', '0', '0', '0', 'rplidar_link', 'laser'],
    )

    # ── 4. Localization stack (RSP, EKF, map_server, AMCL) ──────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('hardware_wafl2026'),
            'launch', 'laptop_b_localization.launch.py')]),
    )

    # ── 5. Pose persistence node ────────────────────────────────────────────
    # Saves /amcl_pose to ~/.ros/amcl_last_pose.yaml every 2s. On next boot,
    # republishes it to /initialpose so the robot remembers where it was.
    # Started AFTER the localization launch's own t=13s initialpose so this
    # one wins when a saved pose exists.
    pose_persistence = TimerAction(
        period=16.0,
        actions=[Node(
            package='hardware_wafl2026',
            executable='pose_persistence_node',
            name='pose_persistence_node',
            output='screen',
            parameters=[{
                'save_period_s':   2.0,
                'restore_delay_s': 0.5,   # this node itself starts late
            }],
        )],
    )

    # ── 6. RViz2 ────────────────────────────────────────────────────────────
    # Delayed so TF chain is up before RViz tries to render.
    rviz = TimerAction(
        period=10.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )],
    )

    return LaunchDescription([
        robot_hardware,
        rplidar,
        laser_frame_bridge,
        localization,
        pose_persistence,
        rviz,
    ])
