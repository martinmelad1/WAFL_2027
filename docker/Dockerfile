# ============================================================
# WAFL 2026 — Docker Image
# Inside the container: Ubuntu 22.04 + ROS 2 Humble + Ignition Gazebo Fortress
# Host machine can be anything that has Docker (Ubuntu 24 + Jazzy is fine)
# ============================================================
FROM osrf/ros:humble-desktop-full

# Prevent interactive prompts during apt
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ── System packages ──────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Ignition Gazebo (Fortress) + ROS bridge
    ros-humble-ros-gz \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    # Navigation stack
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-controller \
    ros-humble-nav2-planner \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-lifecycle-manager \
    # Localization / TF
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # RPLidar driver (kept for hardware compatibility)
    ros-humble-rplidar-ros \
    # Build tooling
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    # GUI forwarding (Gazebo + RViz windows appear on host via X11)
    mesa-utils \
    libgl1-mesa-dri \
    x11-apps \
    # Utilities
    tmux \
    git \
    wget \
    curl \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ── PyQt5 for the manual dashboard ──────────────────────────
RUN pip3 install PyQt5

# ── Bootstrap rosdep ────────────────────────────────────────
RUN rosdep init 2>/dev/null || true && rosdep update

# ── Copy the full repository into the image ─────────────────
WORKDIR /WAFL_2027
COPY . .

# ── Build workspace 1: Localization (wafl2026 package) ──────
RUN bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /WAFL_2027/WAFL2026/Localization/Simulation && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

# ── Build workspace 2: Navigation (wafl_navigation package) ─
RUN bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /WAFL_2027/WAFL2026/Navigation/Simulation && \
    rosdep install --from-paths wafl_navigation --ignore-src -r -y && \
    colcon build --symlink-install"

# ── Source everything in every shell ────────────────────────
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /WAFL_2027/WAFL2026/Localization/Simulation/install/setup.bash" >> /root/.bashrc && \
    echo "source /WAFL_2027/WAFL2026/Navigation/Simulation/install/setup.bash"   >> /root/.bashrc && \
    echo "export REPO=/WAFL_2027" >> /root/.bashrc

# ── Default: drop into bash (user picks which terminal to open) ─
CMD ["/bin/bash"]
