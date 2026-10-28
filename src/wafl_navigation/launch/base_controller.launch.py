import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('wafl_navigation'), 'config', 'base_controller.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('sim', default_value='true'),
        Node(package='wafl_navigation', executable='base_controller', name='base_controller',
             output='screen',
             parameters=[cfg, {'sim': ParameterValue(LaunchConfiguration('sim'), value_type=bool),
                               'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                              value_type=bool)}]),
    ])