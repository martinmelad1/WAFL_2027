# WAFL 2026 — Launch Guide

> **Start here if you just cloned this repo.**  
> Requires ROS 2 Humble + Ignition Gazebo — no Docker needed.

---

## What you need

| Requirement | Notes |
|-------------|-------|
| Linux (Ubuntu 22.04) | Windows/macOS not supported for GUI |
| ROS 2 Humble | Install guide below |
| Ignition Gazebo | Installed via `ros-humble-ros-gz` |
| Nav2 | Installed via `ros-humble-nav2-*` |
| A monitor / desktop | GUI windows won't work on a headless server |

---

## Step 1 — Install ROS 2 Humble (skip if already installed)

```bash
# Set locale
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 apt repo
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# Install
sudo apt update
sudo apt install -y ros-humble-desktop
```

---

## Step 2 — Install simulation dependencies

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-nav2-bringup \
  ros-humble-nav2-map-server \
  ros-humble-nav2-amcl \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-behaviors \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-robot-localization \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rplidar-ros \
  python3-colcon-common-extensions
```

---

## Step 3 — Clone the repo

```bash
git clone https://github.com/martinmelad1/WAFL_2027.git
cd WAFL_2027
```

---

## 🚀 NEW: Run via Docker (Recommended for Ubuntu 24.04 / Non-Humble users)

If you don't have Ubuntu 22.04 or ROS 2 Humble installed (e.g., you are on Ubuntu 24.04 with Jazzy), you can run the entire simulation inside Docker! The container handles ROS 2 Humble internally, so no code changes are needed.

1. Make sure you have Docker and Docker Compose installed.
2. Allow X11 forwarding for GUI apps (Gazebo/RViz) so they can appear on your host machine:
   ```bash
   xhost +local:docker
   ```
3. Start the simulation stack:
   ```bash
   # First, move into the docker folder (from the root of the repo)
   cd docker
   # Then start the containers
   docker compose up -d --build
   ```
   > ⏱️ *Note: The first time you run this, it will take a few minutes to build the Docker image.*

4. This automatically launches 4 containers mirroring the 4-terminal setup below. The Gazebo and RViz windows will pop up on your screen.
5. To run the manual dashboard (optional), open a bash shell inside the simulation container:
   ```bash
   # Make sure you are still in the docker/ folder
   docker compose exec wafl-sim bash
   python3 /WAFL_2027/WAFL2026/Integration/wafl_manual_dashboard.py
   ```
6. To shut everything down:
   ```bash
   # Make sure you are still in the docker/ folder
   docker compose down
   ```

**If you are using Docker, you can skip all the native installation and build steps below!**

---

> 📌 **If you are running NATIVELY (without Docker), all the commands below run inside the cloned repo.**

---

### What is `$REPO` and do I need it?

`$REPO` is just a shortcut variable so you don't have to type the full path every time.

**Set it once at the start of every terminal session**, right after you open a new terminal:

```bash
# Paste this at the start of every new terminal — change the path if you cloned somewhere else
REPO=~/WAFL_2027
```

Then instead of typing `/home/yourname/WAFL_2027/WAFL2026/Localization/Simulation` every time,
you just type `$REPO/WAFL2026/Localization/Simulation`.

> ✅ **Does it always work?**  
> Yes — as long as you set it once at the top of your terminal. If you open a new terminal and forget to set it, any `$REPO/...` command will fail with `cd: /WAFL2026/...: No such file or directory`.  
> The fix is always the same: run `REPO=~/WAFL_2027` again.

---

## Step 4 — Build both workspaces (first time only)

The repo has **two separate ROS 2 workspaces**:

```
WAFL2026/Localization/Simulation/   ← wafl2026 package  (Gazebo + AMCL + EKF)
WAFL2026/Navigation/Simulation/     ← wafl_navigation package (Nav2 + caster controller)
```

### 4a — Build the localization/simulation workspace

```bash
source /opt/ros/humble/setup.bash

cd $REPO/WAFL2026/Localization/Simulation
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 4b — Build the navigation workspace

```bash
source /opt/ros/humble/setup.bash

cd $REPO/WAFL2026/Navigation/Simulation
rosdep install --from-paths wafl_navigation --ignore-src -r -y
colcon build --symlink-install
```

> You only need to build **once**. After that, skip straight to Step 5 every time.

---

## Step 5 — Run it (open 4 terminals)

> ⚠️ **Kill any stale Gazebo instances first** — do this before every launch session:
> ```bash
> killall -9 ign; pkill -f gazebo
> ```

---

### Terminal 1 — Gazebo simulation (Ignition + AMCL + EKF)

```bash
REPO=~/WAFL_2027                  # set this if you haven't already
source /opt/ros/humble/setup.bash
source $REPO/WAFL2026/Localization/Simulation/install/setup.bash

cd $REPO/WAFL2026/Localization/Simulation
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch wafl2026 humble4.launch.py
```

> ⏱️ Wait **~25 seconds** for the full startup sequence:  
> Gazebo → bridge (t=3s) → robot spawns (t=6s) → sensor fix (t=7s) → map server (t=9s) → AMCL activates (t=18s)  
> You will see the forklift appear in the Gazebo window.

---

### Terminal 2 — Nav2 navigation stack

```bash
REPO=~/WAFL_2027
source /opt/ros/humble/setup.bash
source $REPO/WAFL2026/Navigation/Simulation/install/setup.bash

ros2 launch wafl_navigation navigation.launch.py
```

---

### Terminal 3 — Caster steering controller

```bash
REPO=~/WAFL_2027
source /opt/ros/humble/setup.bash
source $REPO/WAFL2026/Navigation/Simulation/install/setup.bash

ros2 run wafl_navigation caster_controller
```

> ℹ️ **This node has no GUI.** It prints one line (`Caster Controller Started`) and then runs silently.  
> That is correct — it listens on `/cmd_vel` and publishes steering angles to Gazebo.  
> You will see log lines like `v=0.30 w=0.15 caster=14.0 deg` only when the robot is moving.

---

### Terminal 4 — RViz

> ⚠️ You must source **both** the ROS install AND the wafl2026 workspace.  
> If you only source `/opt/ros/humble`, RViz throws `Package [wafl2026] does not exist` errors and the robot model won't load.

```bash
REPO=~/WAFL_2027
source /opt/ros/humble/setup.bash
source $REPO/WAFL2026/Localization/Simulation/install/setup.bash

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> ℹ️ The `Warning: Ignoring XDG_SESSION_TYPE=wayland` and `GLSL link result` messages are harmless — ignore them.

---

## Step 6 — View the robot in RViz

### Flat top-down map view (default — recommended)

In RViz:
1. Top menu → **Panels → Views** (a Views panel opens on the right)
2. Change **Type** from `Orbit` to **`TopDownOrtho`**
3. Click **Zero** → view snaps flat looking straight down

In TopDownOrtho mode: **left-drag pans**, scroll wheel zooms. No 3D rotation.

---

### 3D robot model view

1. In the left **Displays** panel, find **RobotModel** → make sure it is **checked**
2. Set **Fixed Frame** (top of Displays panel) to `odom` (or `map` once AMCL is active)
3. The full 3D forklift appears in the viewport

| Action | Mouse |
|--------|-------|
| Orbit (rotate) | Left click + drag |
| Pan | Middle click + drag |
| Zoom | Scroll wheel |
| Focus on robot | `F` |

> ⚠️ If RobotModel shows **"Errors loading geometries"** → you launched RViz without sourcing the workspace. Relaunch Terminal 4 with both `source` lines.

---

## Step 7 — Send a navigation goal

1. In RViz toolbar → click **2D Pose Estimate** → click the robot's position on the map → drag arrow in the direction it faces → a green particle cloud confirms AMCL is active
2. Click **Nav2 Goal** → click anywhere on the map → drag to set heading → the robot plans a path and drives automatically

---

## Step 8 — Manual Control GUI (optional)

A PyQt5 dashboard for manually controlling drive distance, steering angle, and fork lift. Primarily for real-hardware testing.

### Install PyQt5 (once)

```bash
pip3 install PyQt5
```

### Launch

```bash
REPO=$(git -C ~/WAFL_2027 rev-parse --show-toplevel)
source /opt/ros/humble/setup.bash
source $REPO/WAFL2026/Localization/Simulation/install/setup.bash

python3 $REPO/WAFL2026/Integration/wafl_manual_dashboard.py
```

| Panel | What it does |
|-------|-------------|
| **Move Control** | Type a distance (float) → `Publish Once` / `Start Continuous` / `Stop Continuous` |
| **Steering** | Type an angle (±float degrees) → `Publish Steering` → `/target_angle` (real ESP32) |
| **Lifting Mechanism** | `UP` / `DOWN` / `STOP` → sends 1 / 2 / 0 to `/lift_cmd` (real ESP32) |
| **Start micro-ROS Agent** | Starts UDP bridge for ESP32s — **real hardware only**, not needed in simulation |

> ⚠️ In **simulation**, the Move and Steering panels are not wired to the Gazebo robot (they target the real ESP32 topics). Use the RViz **Nav2 Goal** button to drive the simulated robot.

---

## If you cloned to a different location

All commands above use `git -C ~/WAFL_2027 rev-parse --show-toplevel` to find the repo.  
If you cloned somewhere else (e.g. `~/projects/WAFL_2027`), either:

**Option A** — replace `~/WAFL_2027` in every command with your actual path, or  
**Option B** — set `REPO` manually at the start of each terminal:

```bash
REPO=/path/to/your/clone/of/WAFL_2027
```

---

## Troubleshooting

### ❌ Gazebo opens but robot doesn't appear
Wait up to 30 seconds — the timed startup sequence takes ~22s. If still missing, check Terminal 1 for errors.

### ❌ Map doesn't appear in RViz
The map server is a lifecycle node. If it didn't auto-activate:
```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### ❌ Gazebo hangs / freezes
```bash
killall -9 ign
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -r
```
Then re-run Terminal 1.

### ❌ `Package [wafl2026] does not exist` — meshes missing in RViz
You only sourced `/opt/ros/humble`. Close RViz and relaunch Terminal 4 with **both** `source` lines.

### ❌ `Package 'wafl2026' not found` (other terminals)
```bash
source $REPO/WAFL2026/Localization/Simulation/install/setup.bash
```

### ❌ `Package 'wafl_navigation' not found`
```bash
source $REPO/WAFL2026/Navigation/Simulation/install/setup.bash
```

### ❌ Caster controller terminal looks frozen / no output
That is normal — it is waiting for `/cmd_vel` from Nav2. It will print angles once the robot moves.

### ❌ EKF NaN / covariance errors on startup
Expected — the launch file has a timed startup to prevent this. Wait the full ~25s before diagnosing.

### ❌ Nav2 panel shows "unknown" for Navigation / Localization
The `wafl_navigation` workspace was not built or not sourced. Build it (Step 4b) then relaunch Terminal 2.

### ❌ VM / slow machine — Gazebo is very laggy

| Setting | Value |
|---------|-------|
| RAM | 16 GB |
| Processors | 4 |
| Graphics Memory | 128 MB |
| Accelerate 3D Graphics | ON |

---

## What's inside the simulation

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 LTS |
| ROS | Humble |
| Simulator | Ignition Gazebo (Fortress) |
| Localization | AMCL + EKF (robot_localization) |
| Navigation | Nav2 (DWB + NavFn) |

---

*For full architecture documentation see [WAFL2026_Architecture.md](WAFL2026_Architecture.md)*  
*For full project README see [WAFL2026/README.md](WAFL2026/README.md)*
