"""
LAPTOP A — Robot hardware launch
Runs on the robot onboard laptop.

Publishes:
  /wheel_velocity
  /imu_yaw
  /odom

ROS_DOMAIN_ID must match on both laptops.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    encoder_node = Node(
        package='encoder_odometry',
        executable='encoder_velocity_node',
        name='encoder_velocity_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    imu_node = Node(
        package='phone_imu',
        executable='imu_udp_node',
        name='imu_udp_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    odom_fusion = Node(
        package='robot_odometry',
        executable='odom_fusion_node',
        name='odom_fusion_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'publish_rate_hz': 50.0,
            'max_dt_s': 0.2,
            'velocity_timeout_s': 0.5,
        }],
    )

    return LaunchDescription([
        encoder_node,
        imu_node,
        odom_fusion,
    ])