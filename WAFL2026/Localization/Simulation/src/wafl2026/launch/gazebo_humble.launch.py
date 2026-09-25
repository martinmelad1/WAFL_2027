import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2025'
    
    # Path to your URDF file
    urdf_path = os.path.join(get_package_share_directory(package_name), 'urdf', 'wafl2025.urdf')

    # READ THE FILE CONTENT MANUALLY
    # This ensures we pass the XML content, not the file path
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Path to the Gazebo ROS system launch file
    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        # 1. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc, # Use the content, not the path
                'use_sim_time': True
            }]
        ),

        # 2. Start Gazebo Classic
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
            ),
        ),

        # 3. Spawn the robot
        # This looks for the content published by robot_state_publisher on the /robot_description topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'wafl2025', '-z', '0.5'],
            output='screen'
        ),
    ])
