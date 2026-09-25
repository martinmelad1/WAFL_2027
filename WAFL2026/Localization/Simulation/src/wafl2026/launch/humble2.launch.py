import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2025'

    # Get paths
    pkg_share = get_package_share_directory(package_name)
    urdf_path = os.path.join(pkg_share, 'urdf', 'wafl2025.urdf')

    # Read and clean URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()
    robot_desc = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_desc)

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        # 2. Launch Gazebo Server (The core simulator)
        # This explicitly loads the factory plugin required for spawning
        ExecuteProcess(
            cmd=['gzserver', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 3. Launch Gazebo Client (The GUI)
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),

        # 4. Spawn the Robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description', # Using topic is more reliable than -file
                '-entity', 'wafl2025',
                '-z', '0.05'
            ],
            output='screen'
        ),
    ])
