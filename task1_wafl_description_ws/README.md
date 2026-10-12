# wafl_description (ROS 2)

## Build & run
    #in the workspace directory:
    colcon build --packages-select wafl_description --symlink-install
    source install/setup.bash
    ros2 launch wafl_description view_urdf.launch.py