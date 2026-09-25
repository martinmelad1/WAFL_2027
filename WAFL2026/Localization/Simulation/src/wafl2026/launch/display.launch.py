import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2025'
    urdf_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'wafl2025.urdf')

    with open(urdf_file, 'r') as infp:
        robot_description_config = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        # This starts Gazebo Sim (Jazzy default)
        Node(
            package='ros_gz_sim',
            executable='gz_sim',
            arguments=['-r', 'empty.sdf'],
        ),
        # This spawns the robot into Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'wafl2025'],
            output='screen'
        ),
    ])
