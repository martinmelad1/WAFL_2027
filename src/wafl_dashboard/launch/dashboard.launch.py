from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="8000", description="Dashboard HTTP port"),
        SetEnvironmentVariable("WAFL_DASHBOARD_PORT", port),
        Node(package="wafl_dashboard", executable="dashboard_server", output="screen"),
    ])
