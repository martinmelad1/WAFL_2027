from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_share = FindPackageShare('wafl_description')

    urdf_file = PathJoinSubstitution([
        package_share,
        'urdf',
        'WAFL2025.xacro',
    ])
    rviz_config = PathJoinSubstitution([
        package_share,
        'rviz',
        'wafl_config.rviz',
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

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
    )

    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    return LaunchDescription([
        start_robot_state_publisher_cmd,
        start_rviz_cmd,
        start_joint_state_publisher_gui_cmd,
    ])
