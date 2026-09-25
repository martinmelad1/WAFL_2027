import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2026'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'wafl2025.urdf')
    map_file_path = os.path.join(pkg_share, 'map', 'my_map.yaml')
    world_path = os.path.join(pkg_share, 'map', 'my_map.sdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True
        }]
    )

    # 2. Ignition Gazebo — using ros_gz_sim and modern gz_args
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # 3. Spawn the Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'wafl2025',
            '-topic', 'robot_description',
            '-z', '0.2'
        ],
        output='screen',
    )

    # 4. Bridge — Updated to include castor joint command
    # Topic format: /topic@ros_msg@gz_msg
    # [ = GZ to ROS only
    # ] = ROS to GZ only
    # @ = Bidirectional (or as defined by internal logic)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/wafl2025/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # New line for active castor rotation: ROS -> GZ
            'castor_cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            # Inside the bridge node arguments:
		        'left_steering_cmd@std_msgs/msg/Float64]gz.msgs.Double',
		        'right_steering_cmd@std_msgs/msg/Float64]gz.msgs.Double',
		        'left_wheel_speed@std_msgs/msg/Float64]gz.msgs.Double',
		        'right_wheel_speed@std_msgs/msg/Float64]gz.msgs.Double',
		        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
		        '/kinect/image@sensor_msgs/msg/Image[gz.msgs.Image',
		        '/kinect/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
		        '/kinect/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
		        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
		        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/model/wafl2025/tf', '/tf')
        ],
        output='screen'
    )


    # 1. Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path},
                    {'use_sim_time': True}]
    )

    # 2. AMCL (Localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'amcl_config.yaml')]
    )

    # 3. Lifecycle Manager (Required to activate Nav2 nodes)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]
    )

    return LaunchDescription([
        robot_state_publisher,
        ignition_gazebo,
        spawn_robot,
        bridge,
        map_server,
        amcl,
        lifecycle_manager
    ])
