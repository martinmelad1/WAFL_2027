import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    package_name = 'wafl2026'
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'wafl2025.urdf')
    map_file_path = os.path.join(pkg_share, 'map', 'my_map.yaml')
    world_path = os.path.join(pkg_share, 'map', 'my_map.sdf')

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. Gazebo — clock source
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 2. Map server — starts immediately unconfigured
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        output='screen',
        parameters=[
            {'yaml_filename': map_file_path},
            {'use_sim_time': True},
        ],
    )

    # 3. EKF starts at t=0 BEFORE the bridge publishes /clock.
    # When use_sim_time=True and no /clock is available yet, the EKF
    # waits with internal time=0. The first /clock message it receives
    # sets its baseline, so dt on the first predict() call is tiny (~0).
    # This prevents the dt=15s explosion that produces NaN when EKF starts
    # after sim time has already advanced.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    # 4. Bridge at t=3s — /clock starts flowing AFTER EKF is already running
    bridge = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                'castor_cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/kinect/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/kinect/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
                '/kinect/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            parameters=[{}],
            output='screen',
        )]
    )

    # 5. Robot State Publisher at t=5s
    robot_state_publisher = TimerAction(
        period=5.0,
        actions=[Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True,
            }],
        )]
    )

    # 6. Spawn robot at t=6s
    spawn_robot = TimerAction(
        period=6.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'wafl2025',
                '-topic', 'robot_description',
                '-z', '0.62',
            ],
            output='screen',
        )]
    )

    # 7. Sensor fix relay at t=7s
    # /odom  -> /odom_fixed  (injects non-zero covariance)
    # /imu   -> /imu_fixed   (injects non-zero covariance + fixes frame_id)
    sensor_fix = TimerAction(
        period=7.0,
        actions=[Node(
            package='wafl2026',
            executable='odom_covariance_fix',
            name='sensor_fix',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )]
    )

    # 8. Activate map server at t=9s
    activate_map = TimerAction(
        period=9.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'ros2 lifecycle set /map_server configure && '
                     'sleep 1.0 && '
                     'ros2 lifecycle set /map_server activate'],
                output='screen',
            )
        ]
    )

    # 9. AMCL at t=15s
    amcl = TimerAction(
        period=15.0,
        actions=[LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'amcl_config.yaml'),
                {'use_sim_time': True},
            ],
            remappings=[('map', 'map')],
        )]
    )

    # 10. Activate AMCL at t=18s
    activate_amcl = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c',
                     'ros2 lifecycle set /amcl configure && '
                     'sleep 1.0 && '
                     'ros2 lifecycle set /amcl activate'],
                output='screen',
            )
        ]
    )

    # 11. Initial pose for AMCL at t=22s
    initial_pose_pub = TimerAction(
        period=22.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                     'geometry_msgs/msg/PoseWithCovarianceStamped',
                     '{header: {frame_id: map}, pose: {pose: {position: '
                     '{x: 0.0, y: 0.0, z: 0.0}, '
                     'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        ignition_gazebo,
        map_server,
        ekf_node,       # t=0 — MUST be before bridge so first dt ~ 0
        bridge,         # t=3s
        robot_state_publisher,  # t=5s
        spawn_robot,    # t=6s
        sensor_fix,     # t=7s
        activate_map,   # t=9s
        amcl,           # t=15s
        activate_amcl,  # t=18s
        initial_pose_pub,  # t=22s
    ])
