#!/bin/bash
# =============================================================
#  WAFL 2025 — Docker Launcher
#  Usage:  bash docker/run.sh <command>
#
#  Commands:
#    build    Build the Docker image (run once, ~10 min)
#    gui      View robot URDF model + interactive joint sliders
#    gazebo   Spawn robot in Gazebo (empty world, no teleop)
#    teleop   Gazebo + keyboard teleoperation  ← START HERE
#    slam     Hector SLAM mapping (requires real LiDAR)
#    nav      Full autonomous navigation stack
#    shell    Open interactive bash shell in container
# =============================================================

set -e

IMAGE="wafl2025:latest"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PACKAGE_DIR="$REPO_ROOT/WAFL2025-forklift2425-patch-manual-mapping/WAFL2025"

# ── Detect display ────────────────────────────────────────────
if [ -z "$DISPLAY" ]; then
  echo "[WARN] No DISPLAY variable set. GUI windows may not appear."
  echo "       If on a desktop, try: export DISPLAY=:0"
fi

# ── Allow Docker to use host display ──────────────────────────
xhost +local:docker > /dev/null 2>&1 || true

# ── Common Docker run flags ───────────────────────────────────
DOCKER_RUN=(
  docker run --rm
  --env DISPLAY="$DISPLAY"
  --env QT_X11_NO_MITSHM=1
  --env LIBGL_ALWAYS_SOFTWARE=0
  --env ROS_MASTER_URI=http://localhost:11311
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw
  # Live-mount the package so changes on host appear instantly
  --volume "$PACKAGE_DIR:/catkin_ws/src/WAFL2025"
  --network host
  --privileged
)

# ── Commands ──────────────────────────────────────────────────
case "$1" in

  # ── BUILD ────────────────────────────────────────────────────
  build)
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║         Building WAFL2025 Docker image               ║"
    echo "║  This downloads ~2 GB and takes ~10 min first time   ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""
    docker build \
      -t "$IMAGE" \
      -f "$SCRIPT_DIR/Dockerfile" \
      "$REPO_ROOT"
    echo ""
    echo "✅  Build complete! Image: $IMAGE"
    echo ""
    echo "Next steps:"
    echo "  bash docker/run.sh gui     ← see the robot model"
    echo "  bash docker/run.sh teleop  ← drive it in simulation"
    ;;

  # ── GUI — Robot model viewer ──────────────────────────────────
  gui)
    echo "🤖  Launching URDF viewer (RViz + joint sliders)..."
    echo "    Close the RViz window to exit."
    echo ""
    "${DOCKER_RUN[@]}" "$IMAGE" \
      roslaunch WAFL2025 sim_display.launch
    ;;

  # ── GAZEBO — Simulation only ──────────────────────────────────
  gazebo)
    echo "🌍  Launching Gazebo simulation (robot in empty world)..."
    echo "    Close the Gazebo window to exit."
    echo ""
    "${DOCKER_RUN[@]}" "$IMAGE" \
      roslaunch WAFL2025 sim_teleop.launch
    ;;

  # ── TELEOP — Gazebo + keyboard control ───────────────────────
  teleop)
    echo "🕹️   Launching Gazebo + keyboard teleoperation..."
    echo ""
    echo "    Keyboard controls (click the terminal, then type):"
    echo "      W / S  — Forward / Backward"
    echo "      A / D  — Turn Left / Right"
    echo "      Q / E  — Strafe Left / Right"
    echo "      Space  — Emergency STOP"
    echo "      Z      — Zero steering"
    echo "      + / -  — Speed up / slow down"
    echo "      Ctrl-C — Exit"
    echo ""
    echo "    Gazebo window will open. Wait ~5 seconds for it to load."
    echo ""
    "${DOCKER_RUN[@]}" -it "$IMAGE" \
      bash -c "roslaunch WAFL2025 sim_teleop.launch &
               sleep 6
               rosrun WAFL2025 teleop_keyboard.py"
    ;;

  # ── SLAM — Hector SLAM mapping (real hardware) ────────────────
  slam)
    echo "🗺️   Launching Hector SLAM (requires RPLiDAR on /dev/ttyUSB0)..."
    echo "    Connect your LiDAR before running this."
    echo ""
    "${DOCKER_RUN[@]}" \
      --device /dev/ttyUSB0 \
      "$IMAGE" \
      roslaunch WAFL2025 hector_slam.launch
    ;;

  # ── NAV — Full autonomous navigation ─────────────────────────
  nav)
    echo "🧭  Launching full autonomous navigation stack..."
    echo "    (AMCL + move_base + custom Pure Pursuit controller)"
    echo ""
    "${DOCKER_RUN[@]}" "$IMAGE" \
      roslaunch WAFL2025 move_base.launch
    ;;

  # ── SHELL — Interactive bash inside container ─────────────────
  shell)
    echo "💻  Opening bash shell inside WAFL2025 container..."
    echo "    ROS and workspace are already sourced."
    echo "    Type 'exit' to leave."
    echo ""
    "${DOCKER_RUN[@]}" -it "$IMAGE" bash
    ;;

  # ── HELP ──────────────────────────────────────────────────────
  *)
    echo ""
    echo "WAFL 2025 — Docker Launcher"
    echo ""
    echo "Usage:  bash docker/run.sh <command>"
    echo ""
    echo "Commands:"
    echo "  build    Build Docker image (first time only, ~10 min)"
    echo "  gui      View robot URDF + interactive joint sliders"
    echo "  gazebo   Spawn robot in Gazebo simulation"
    echo "  teleop   Gazebo + keyboard control (recommended for testing)"
    echo "  slam     Hector SLAM mapping (needs real LiDAR)"
    echo "  nav      Full autonomous navigation stack"
    echo "  shell    Interactive bash shell in container"
    echo ""
    echo "Quick start:"
    echo "  bash docker/run.sh build    # build once"
    echo "  bash docker/run.sh gui      # see the robot"
    echo "  bash docker/run.sh teleop   # drive it"
    echo ""
    ;;

esac
