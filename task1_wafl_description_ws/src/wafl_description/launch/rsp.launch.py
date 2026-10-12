from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('wafl_description'),
        'urdf',
        'WAFL2025.xacro',
    ])
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
        }]
    )

    return LaunchDescription([
        start_robot_state_publisher_cmd,
    ])