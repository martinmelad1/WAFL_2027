#!/usr/bin/env python3
"""gazebo.launch.py - Gazebo Harmonic + WAFL2025 robot (patched in memory) + ros_gz_bridge.
Arguments: use_sim_time, world, robot_xacro, start_rsp, bridge_config, headless,
           spawn_x/y/z/yaw, spawn_delay   (same meaning as before)."""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from wafl_simulation.urdf_patch import build_robot_description

WORLD_NAME = 'warehouse'
MODEL_NAME = 'wafl'


def _start_rsp(context):
    if LaunchConfiguration('start_rsp').perform(context).lower() != 'true':
        return []
    xacro_path = LaunchConfiguration('robot_xacro').perform(context)
    sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    return [Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
                 parameters=[{'robot_description': build_robot_description(xacro_path),
                              'use_sim_time': sim_time}])]


def generate_launch_description():
    pkg_share = get_package_share_directory('wafl_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'warehouse.world')),
        DeclareLaunchArgument('robot_xacro', default_value=os.path.join(
            pkg_share, 'urdf', 'wafl_sim_robot.urdf.xacro')),
        DeclareLaunchArgument('start_rsp', default_value='true'),
        DeclareLaunchArgument('bridge_config', default_value=os.path.join(
            pkg_share, 'config', 'ros_gz_bridge.yaml')),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('spawn_x', default_value='-5.5'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.02'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('spawn_delay', default_value='4.0'),
    ]

    # model:// (pallets, racks, dock) and package://wafl_description/meshes/... URIs
    resource_paths = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg_share, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(get_package_prefix('wafl_simulation'), 'share')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                                  os.path.join(get_package_prefix('wafl_description'), 'share')),
    ]

    gz_args = PythonExpression([
        "('-r -s --headless-rendering ' if '", headless, "' == 'true' else '-r ') + '", world, "'"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    spawn = TimerAction(
        period=LaunchConfiguration('spawn_delay'),
        actions=[Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=['-world', WORLD_NAME, '-name', MODEL_NAME, '-topic', 'robot_description',
                       '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                       '-z', LaunchConfiguration('spawn_z'), '-Y', LaunchConfiguration('spawn_yaw')])])

    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': LaunchConfiguration('bridge_config')}])

    return LaunchDescription(args + resource_paths + [gz_sim, OpaqueFunction(function=_start_rsp), spawn, bridge])