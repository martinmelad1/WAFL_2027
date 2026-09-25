import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'wafl2025'

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        'wafl2025.urdf'
    )

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    # Prevent lxml/spawn_entity error if XML declaration exists
    robot_desc = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', robot_desc)

    gazebo_ros_path = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_desc,
                'use_sim_time': True
            }]
        ),

        IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(
		os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
	    ),
	    launch_arguments={
		'gui': 'false'
	    }.items()
	),

        Node(
	    package='gazebo_ros',
	    executable='spawn_entity.py',
	    arguments=[
		'-file', urdf_path,
		'-entity', 'wafl2025',
		'-z', '0.02'
	    ],
	    output='screen'
	),
    ])
