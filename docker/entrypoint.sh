#!/bin/bash
# Docker entrypoint — sources ROS and workspace, then runs the command
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/catkin_ws/src/WAFL2025/meshes:$GAZEBO_MODEL_PATH
exec "$@"
