"""
LAPTOP B — Localization & visualization launch
Package: hardware_wafl2026

Full TF chain when running:
  map ──(AMCL)──> odom ──(EKF)──> base_footprint ──(RSP/URDF)──> base_link ──> rplidar_link, ...

Receives from Laptop A over WiFi (same ROS_DOMAIN_ID):
  /odom    -> EKF -> /odometry/filtered  +  TF: odom -> base_footprint
  /scan    -> AMCL -> TF: map -> odom
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # ── IMPORTANT: package is hardware_wafl2026, NOT wafl2026 ──────────────
    pkg_share     = get_package_share_directory('hardware_wafl2026')
    urdf_path     = os.path.join(pkg_share, 'urdf', 'wafl2025.urdf')
    map_file_path = os.path.join(pkg_share, 'map', 'my_map.yaml')
    ekf_config    = os.path.join(pkg_share, 'config', 'ekf.yaml')
    amcl_config   = os.path.join(pkg_share, 'config', 'amcl_config.yaml')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # ── 1. Robot State Publisher (t=0) ──────────────────────────────────────
    # Publishes static TF: base_footprint -> base_link -> all sensors/wheels
    # Does NOT touch odom -> base_footprint (EKF owns that edge)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False,
        }],
    )

    # ── 2. EKF (t=0) ────────────────────────────────────────────────────────
    # Subscribes to /odom (arriving from Laptop A over WiFi).
    # Sole publisher of TF: odom -> base_footprint
    # Also publishes /odometry/filtered for AMCL.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False},
        ],
    )

    # ── 3. Map server (t=0, activated at t=3) ───────────────────────────────
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename': map_file_path},
            {'use_sim_time': False},
        ],
    )

    activate_map = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'ros2 lifecycle set /map_server configure && '
                 'sleep 1.0 && '
                 'ros2 lifecycle set /map_server activate'],
            output='screen',
        )],
    )

    # ── 4. AMCL (started t=6, activated t=9) ────────────────────────────────
    # Reads /scan (from RPLiDAR on robot, arriving over WiFi).
    # Reads TF: odom -> base_footprint -> rplidar_link to ray-cast.
    # Publishes TF: map -> odom  (this is the "map frame doesn't exist" fix).
    amcl = TimerAction(
        period=6.0,
        actions=[LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[
                amcl_config,
                {'use_sim_time': False},
            ],
        )],
    )

    activate_amcl = TimerAction(
        period=9.0,
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'ros2 lifecycle set /amcl configure && '
                 'sleep 1.0 && '
                 'ros2 lifecycle set /amcl activate'],
            output='screen',
        )],
    )

    # ── 5. Initial pose (t=13) ───────────────────────────────────────────────
    # Sets AMCL's starting particle distribution on the map.
    # Edit x/y to match where the robot physically starts.
    initial_pose_pub = TimerAction(
        period=13.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 '{header: {frame_id: map}, pose: {pose: {position: '
                 '{x: 0.0, y: 0.0, z: 0.0}, '
                 'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'],
            output='screen',
        )],
    )

    return LaunchDescription([
        robot_state_publisher,  # t=0
        ekf_node,               # t=0 — must be early so TF edge exists before AMCL
        map_server,             # t=0 (lifecycle: unconfigured until t=3)
        activate_map,           # t=3
        amcl,                   # t=6
        activate_amcl,          # t=9
        initial_pose_pub,       # t=13
    ])
