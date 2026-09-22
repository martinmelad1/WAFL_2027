FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

# Install all WAFL2025 ROS dependencies + tools
RUN apt-get update && apt-get install -y \
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-hector-mapping \
    ros-noetic-rplidar-ros \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-tf \
    ros-noetic-rviz \
    ros-noetic-controller-manager \
    ros-noetic-robot-localization \
    ros-noetic-dwa-local-planner \
    ros-noetic-navfn \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    ros-noetic-can-msgs \
    ros-noetic-teleop-twist-keyboard \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Python deps
RUN pip3 install numpy

# Set up catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copy WAFL2025 package into the workspace
COPY WAFL2025-forklift2425-patch-manual-mapping/WAFL2025/ /catkin_ws/src/WAFL2025/

# Build the workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /catkin_ws && catkin_make"

# Make all scripts executable
RUN chmod +x /catkin_ws/src/WAFL2025/scripts/*.py 2>/dev/null || true

# Source both ROS and workspace on every shell startup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export DISPLAY=\$DISPLAY" >> /root/.bashrc

# Default shell
CMD ["/bin/bash"]
