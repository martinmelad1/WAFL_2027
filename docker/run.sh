#!/bin/bash
# =============================================================
#  WAFL 2025 — Docker launcher
#  Usage:  bash run.sh [command]
#
#  Commands:
#    build       — Build the Docker image (run once)
#    gui         — URDF viewer: see robot model + joint sliders
#    gazebo      — Gazebo simulation (spawn robot in empty world)
#    teleop      — Keyboard teleoperation (WASD + QE strafe)
#    slam        — Hector SLAM mapping mode
#    nav         — Full autonomous navigation (AMCL + move_base)
#    shell       — Open a bash shell inside the container
# =============================================================

IMAGE="wafl2025:latest"
PACKAGE_PATH="$(cd "$(dirname "$0")/.." && pwd)/WAFL2025-forklift2425-patch-manual-mapping"

# Allow Docker to use your display
xhost +local:docker > /dev/null 2>&1

# Common Docker flags for GUI support
DOCKER_FLAGS=(
    --rm
    --env DISPLAY="$DISPLAY"
    --env QT_X11_NO_MITSHM=1
    --env LIBGL_ALWAYS_SOFTWARE=0
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
    --volume "$PACKAGE_PATH/WAFL2025:/catkin_ws/src/WAFL2025"
    --network host
    --privileged
)

case "$1" in

  build)
    echo ">>> Building WAFL2025 Docker image (this takes ~5-10 min first time)..."
    REPO25_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    docker build -t "$IMAGE" -f "$(dirname "$0")/Dockerfile" "$REPO25_ROOT"
    echo ">>> Done! Image '$IMAGE' is ready."
    ;;

  gui)
    echo ">>> Launching URDF viewer (joint_state_publisher_gui + RViz)..."
    docker run "${DOCKER_FLAGS[@]}" "$IMAGE" \
      bash -c "source /catkin_ws/devel/setup.bash && \
               roslaunch WAFL2025 display.launch"
    ;;

  gazebo)
    echo ">>> Launching Gazebo simulation..."
    docker run "${DOCKER_FLAGS[@]}" "$IMAGE" \
      bash -c "source /catkin_ws/devel/setup.bash && \
               roslaunch WAFL2025 gazebo.launch"
    ;;

  teleop)
    echo ">>> Launching Gazebo + keyboard teleoperation..."
    echo "    Keys: W=forward  S=backward  A=turn-left  D=turn-right"
    echo "          Q=strafe-left  E=strafe-right  Space=STOP"
    docker run -it "${DOCKER_FLAGS[@]}" "$IMAGE" \
      bash -c "source /catkin_ws/devel/setup.bash && \
               roslaunch WAFL2025 hector_slam.launch &
               sleep 4 &&
               rosrun WAFL2025 teleop_keyboard.py"
    ;;

  slam)
    echo ">>> Launching Hector SLAM mapping mode..."
    docker run "${DOCKER_FLAGS[@]}" "$IMAGE" \
      bash -c "source /catkin_ws/devel/setup.bash && \
               roslaunch WAFL2025 hector_slam.launch"
    ;;

  nav)
    echo ">>> Launching full autonomous navigation stack..."
    docker run "${DOCKER_FLAGS[@]}" "$IMAGE" \
      bash -c "source /catkin_ws/devel/setup.bash && \
               roslaunch WAFL2025 move_base.launch"
    ;;

  shell)
    echo ">>> Opening shell inside WAFL2025 container..."
    docker run -it "${DOCKER_FLAGS[@]}" "$IMAGE" bash
    ;;

  *)
    echo "Usage: bash run.sh [build|gui|gazebo|teleop|slam|nav|shell]"
    echo ""
    echo "  build   — Build Docker image (first time only, ~5-10 min)"
    echo "  gui     — URDF viewer with interactive joint sliders"
    echo "  gazebo  — Gazebo simulation (empty world)"
    echo "  teleop  — Gazebo + keyboard teleoperation (WASD)"
    echo "  slam    — Hector SLAM mapping mode"
    echo "  nav     — Full autonomous navigation (AMCL + move_base)"
    echo "  shell   — Interactive bash shell in the container"
    ;;

esac
