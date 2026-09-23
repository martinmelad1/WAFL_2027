FROM osrf/ros:noetic-desktop-full

# Avoid interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive

# ─── Install all WAFL2025 ROS + system dependencies ───────────────────────────
RUN apt-get update && apt-get install -y \
    # Navigation stack
    ros-noetic-move-base \
    ros-noetic-amcl \
    ros-noetic-map-server \
    ros-noetic-navfn \
    ros-noetic-dwa-local-planner \
    # SLAM
    ros-noetic-hector-mapping \
    # Sensors
    ros-noetic-rplidar-ros \
    # Robot description & control
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-controller-manager \
    ros-noetic-robot-localization \
    # TF & visualization
    ros-noetic-tf \
    ros-noetic-rviz \
    # Simulation
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-plugins \
    # Messages
    ros-noetic-can-msgs \
    ros-noetic-teleop-twist-keyboard \
    # Build tools
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    # Utilities
    git \
    nano \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ─── Python dependencies ───────────────────────────────────────────────────────
RUN pip3 install numpy

# ─── Catkin workspace setup ───────────────────────────────────────────────────
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# ─── Copy WAFL2025 package ────────────────────────────────────────────────────
COPY WAFL2025-forklift2425-patch-manual-mapping/WAFL2025/ /catkin_ws/src/WAFL2025/

# ─── Build workspace ──────────────────────────────────────────────────────────
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# ─── Make all Python scripts executable ───────────────────────────────────────
RUN find /catkin_ws/src/WAFL2025/scripts -name "*.py" -exec chmod +x {} \;

# ─── Auto-source on every shell ───────────────────────────────────────────────
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc && \
    echo "export GAZEBO_MODEL_PATH=/catkin_ws/src/WAFL2025/meshes:\$GAZEBO_MODEL_PATH" >> /root/.bashrc

# ─── Entrypoint: source ROS then run the provided command ─────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
