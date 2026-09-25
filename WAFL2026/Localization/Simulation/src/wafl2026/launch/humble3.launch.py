import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2025'

    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'wafl2025.urdf')

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

    # 2. Ignition Gazebo — use ros_gz_sim (not deprecated ros_ign_gazebo)
    #    and gz_args (not deprecated ign_args)
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
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

    # 4. Bridge — ros_gz_bridge (not deprecated ros_ign_bridge)
    #    Format: /topic@ros_msg_type@gz_msg_type
    #    Use gz.msgs (not ignition.msgs) to avoid deprecation warnings
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # cmd_vel: bidirectional (ROS <-> GZ)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # odom: GZ -> ROS only
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # tf: GZ -> ROS only
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # joint_states: GZ -> ROS only
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        ignition_gazebo,
        spawn_robot,
        bridge,
    ])
