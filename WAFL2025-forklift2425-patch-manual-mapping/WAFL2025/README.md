# WAFL 2025

> **Warehouse Autonomous Forklift — 2024/2025 Graduation Project**  
> Mechatronics Department · Ain Shams University

---

## About

The **Autonomous Forklift Project (WAFL 2025)** is a graduation project from the Mechatronics Department at Ain Shams University. The goal is to design and implement an intelligent, fully autonomous forklift capable of navigating warehouse environments, localizing itself within pre-built maps, and executing autonomous pick-and-place tasks — all without human intervention.

The robot uses a **4-Wheel Swerve Drive (4WSD)** chassis enabling three distinct locomotion modes: front-steer, counter-steer, and crab (lateral) motion. The navigation stack is built on **ROS Noetic** and integrates LiDAR-based SLAM and localization (Hector SLAM + AMCL), path planning (move_base / NavFn), and a custom Pure Pursuit controller with multi-mode swerve kinematics.

<details>
<summary><strong>Team Members</strong></summary>

- Ahmed Yasser  
- Ahmed Gamal  
- Marina Alber  
- Malaak Mikhael  
- Mohamed Montasser  
- Omar Emad  
- Youssef Ahmed  

</details>

---

## System Overview

```
┌───────────────────────────────────────────────────────────┐
│                      WAFL 2025 Stack                       │
├──────────────┬──────────────────┬─────────────────────────┤
│  Perception  │   Localization   │       Navigation         │
│  (RPLiDAR)   │  AMCL / Hector   │  move_base + NavFnROS    │
├──────────────┴──────────────────┴─────────────────────────┤
│               Path Curvature Classifier                     │
│    (curvature_calc.py  →  /path_curvature topic)           │
├─────────────────────────────────────────────────────────────┤
│              Pure Pursuit + Multi-Mode Controller           │
│         (controller_hardware.py / robot_controller.py)      │
├─────────────────────────────────────────────────────────────┤
│        Custom Swerve Odometry  (swerve_odometry_node.py)    │
├─────────────────────────────────────────────────────────────┤
│      Low-Level Hardware  (Arduino + CAN Bus interface)      │
└─────────────────────────────────────────────────────────────┘
```

---

## Project Structure

```
WAFL2025/
├── CMakeLists.txt                  # Catkin build system
├── package.xml                     # ROS package manifest (v1.0.0, BSD)
├── README.md
│
├── arduino files/                  # Embedded firmware (Arduino sketches)
│   ├── low_level/                  # Main motor driver firmware
│   │   └── low_level.ino
│   ├── arduino_manual/             # Manual control firmware
│   ├── new_arduino_manual/         # Updated manual firmware
│   ├── manual_control/             # Manual drive logic
│   ├── IMU_test/                   # IMU sensor validation
│   ├── test_encoder/               # Encoder reading tests
│   ├── test_uart/                  # UART communication tests
│   ├── mega_test/                  # Arduino Mega integration tests
│   ├── rear_wheel_P_control/       # Rear wheel P-controller sketch
│   ├── home_test/                  # Home position test
│   └── hello_world/                # Basic connectivity test
│
├── config/                         # ROS configuration files
│   ├── ekf.yaml                    # Extended Kalman Filter (odom + IMU fusion)
│   ├── joint_names_WAFL2025.yaml   # Joint name definitions
│   ├── swerve_control.yaml         # ROS controller manager configuration
│   ├── my_robot_control.yaml       # Alternative controller config
│   ├── localization.rviz           # RViz preset for localization mode
│   ├── mapping.rviz                # RViz preset for SLAM mapping mode
│   ├── path_planning.rviz          # RViz preset for navigation/planning mode
│   └── trial.rviz                  # RViz preset for hardware trials
│
├── launch/                         # ROS launch files
│   ├── display.launch              # URDF viewer (RViz + joint_state_publisher_gui)
│   ├── gazebo.launch               # Spawn robot in Gazebo simulation
│   ├── hector_slam.launch          # SLAM mapping on real hardware
│   ├── amcl_localization.launch    # Localization on pre-built map (hardware)
│   ├── move_base.launch            # Full navigation stack (AMCL + move_base)
│   ├── move_robot.launch           # Simplified move command launcher
│   ├── base_test_noAMCL.launch     # Navigation test without localization
│   ├── trial_urdf.launch           # URDF + hardware controller trial
│   ├── real.launch                 # Full real-robot bring-up
│   └── setup_can.sh                # CAN interface setup script
│
├── maps/                           # Pre-built 2D occupancy grid maps (PGM + YAML)
│   ├── office_map.pgm              # Office environment (1024x1024 px, 5 cm/px)
│   ├── office_map.yaml
│   ├── first_map.pgm / first_map.yaml
│   ├── second_map.yaml.pgm / second_map.yaml.yaml
│   └── third_map.pgm / third_map.yaml
│
├── meshes/                         # 3D mesh files (STL/DAE) for URDF/Gazebo
│   ├── base_link.STL               # Main chassis body
│   ├── base_link(without).STL      # Chassis without fork assembly
│   ├── base_link(old).STL          # Legacy chassis version
│   ├── base_footprint.STL
│   ├── left_wheel.STL / right_wheel.STL
│   ├── left_web.STL / right_web.STL   # Swerve steering knuckles
│   ├── castor.STL / castor_wheel.STL  # Rear castor assembly
│   ├── hokuyo.dae                  # Hokuyo LiDAR visual mesh
│   └── kinect.dae                  # Microsoft Kinect visual mesh
│
├── msg/                            # Custom ROS message definitions
│   ├── WheelSpeeds.msg             # float32 left_speed, float32 right_speed
│   └── SteeringAngles.msg          # float32 front_angle, float32 rear_angle
│
├── param/                          # Navigation stack parameter files
│   ├── costmap_common_params.yaml  # Shared costmap (footprint, inflation, sources)
│   ├── global_costmap_params.yaml  # Global planner costmap settings
│   ├── local_costmap_params.yaml   # Local planner costmap settings
│   ├── global_planner_params.yaml  # NavFn global planner settings
│   ├── dwa_local_planner_params.yaml
│   ├── base_local_planner_params.yaml
│   └── move_base_params.yaml
│
├── scripts/                        # Python ROS nodes
│   ├── controller_hardware.py      # ★ Main autonomous path-following controller
│   ├── robot_controller.py         # Alternative controller variant
│   ├── swerve_odometry_node.py     # Custom swerve-drive odometry publisher
│   ├── curvature_calc.py           # Live path curvature classifier (10 Hz)
│   ├── supervisor.py               # High-level mission supervisor node
│   ├── middle_man.py               # CAN-ROS bridge / PID motor interface
│   ├── manual_drive_control.py     # cmd_vel → WheelSpeeds/SteeringAngles
│   ├── teleop_keyboard.py          # Interactive keyboard teleoperation
│   ├── can_trnaslate.py            # Raw CAN frame receiver (SocketCAN)
│   ├── can_sensory_data.py         # CAN sensor data parser
│   ├── curved.py                   # Curved path utility
│   ├── move_backward.py            # Backward motion helper
│   ├── swerve_test.py              # Swerve drive unit tests
│   ├── controller_test.py          # Controller unit tests
│   └── old/                        # Archived / legacy scripts
│
├── urdf/                           # Robot description files
│   ├── WAFL2025.urdf               # Simulation URDF (full robot + sensor plugins)
│   ├── WAFL2025.xacro              # Xacro macro source for URDF generation
│   ├── WAFL2025_hw.urdf            # Hardware URDF (real robot, no Gazebo plugins)
│   └── WAFL2025.csv                # Joint parameter reference table
│
└── worlds/
    └── home1.world                 # Custom Gazebo simulation environment
```

---

## Software Architecture

### 1. Locomotion — 4-Wheel Swerve Drive

WAFL 2025 uses a **4-Wheel Swerve Drive (4WSD)** chassis where both front and rear axle pairs steer independently.  
The drive mode is selected automatically based on the curvature `k` of the current path segment:

| Mode | Curvature Condition | Behavior |
|------|---------------------|----------|
| **Front-Steer** | `k > 2.1` | Sharp curve — only front wheels turn, rear fixed at 0° |
| **Counter-Steer** | `0.2 ≤ k ≤ 2.1` | Moderate curve — front and rear steer opposite directions for a tighter radius |
| **Crab** | `k < 0.2` | Straight / gentle path — all wheels align to the same lateral angle |

### 2. Navigation Pipeline

```
RPLiDAR A-series
      |
      v
  /scan topic
      |
   .-----------.        .-------------.
   | Hector    |  -OR-  |    AMCL     |
   |  SLAM     |        | Localize    |
   '-----------'        '------.------'
        |                      |
        v                      v
   /map topic           /amcl_pose topic
                               |
                       .-------+-------.
                       |   move_base   |
                       | NavFn global  |
                       | + DWA local   |
                       '-------+-------'
                               |
                  /move_base/NavfnROS/plan
                               |
                  .------------+------------.
                  |    curvature_calc.py    |
                  |  => /path_curvature     |
                  '------------+------------'
                               |
                  .------------+------------.
                  |  controller_hardware.py |
                  |  Pure Pursuit + PID     |
                  |  + Swerve mode select   |
                  '-----.----------.--------'
                        |          |
               /wheel_speeds  /steering_angles
                        |          |
                  .-----+----------+-----.
                  |      middle_man.py   |
                  |   PWM + direction    |
                  '----------+----------'
                             |
                      Arduino / CAN Bus
```

### 3. ROS Nodes Reference

| Node Name | Script | Frequency | Purpose |
|-----------|--------|-----------|---------|
| `path_controller_pid` | `controller_hardware.py` | 10 Hz | Pure Pursuit path follower with PID speed control and 3-mode swerve steering |
| `swerve_odometry` | `swerve_odometry_node.py` | 30 Hz | Computes `/odom` and TF from raw encoder + steering data |
| `curvature_calculator_live` | `curvature_calc.py` | 10 Hz | Computes signed path curvature → drives mode selection |
| `supervisor_node` | `supervisor.py` | 10 Hz | High-level mission logic: fork, lights, goal remapping |
| `swerve_motor_controller` | `middle_man.py` | event | Bridges ROS commands to low-level PWM motor signals |
| `swerve_drive_control` | `manual_drive_control.py` | event | Translates `cmd_vel` Twists to `WheelSpeeds` / `SteeringAngles` |
| `wafl_teleop_keyboard` | `teleop_keyboard.py` | interactive | Keyboard-driven teleoperation |
| `can_receiver_node` | `can_trnaslate.py` | 100 Hz | Receives raw SocketCAN frames → `/can_rx` |

### 4. Topics

#### Published

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | `swerve_odometry_node` | Robot pose + velocity estimate |
| `/wheel_speeds` | `WAFL2025/WheelSpeeds` | `controller_hardware` | Left/right wheel speed setpoints |
| `/steering_angles` | `WAFL2025/SteeringAngles` | `controller_hardware` | Front/rear steering angle setpoints |
| `/path_curvature` | `std_msgs/Float32MultiArray` | `curvature_calc` | Current signed path curvature |
| `/cmd_vel` | `geometry_msgs/Twist` | `teleop_keyboard` | Velocity command (manual mode) |
| `/fork_command` | `std_msgs/Bool` | `supervisor_node` | Fork lift up/down |
| `/light_command` | `std_msgs/Int8` | `supervisor_node` | Indicator light state (0–5) |
| `/motor_speed` | `std_msgs/Float32MultiArray` | `middle_man` | PWM + direction signal to Arduino |
| `/steering/front` | `std_msgs/Float32` | `middle_man` | Front steering angle to hardware |
| `/steering/rear` | `std_msgs/Float32` | `middle_man` | Rear steering angle to hardware |

#### Subscribed

| Topic | Type | Subscriber |
|-------|------|------------|
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `controller_hardware` |
| `/move_base/NavfnROS/plan` | `nav_msgs/Path` | `controller_hardware`, `curvature_calc` |
| `/path_curvature` | `std_msgs/Float32MultiArray` | `controller_hardware` |
| `/swerve/raw_data` | `std_msgs/Float32MultiArray` | `swerve_odometry`, `middle_man`, `supervisor` |
| `/scan` | `sensor_msgs/LaserScan` | `amcl`, `hector_mapping`, `rplidarNode` |
| `/pallet_detection` | `geometry_msgs/PoseStamped` | `supervisor_node` |
| `/destenation` | `move_base_msgs/MoveBaseActionGoal` | `supervisor_node` |
| `/fork_state` | `std_msgs/Bool` | `supervisor_node` |

### 5. Custom Message Definitions

**`WAFL2025/WheelSpeeds`** — `msg/WheelSpeeds.msg`
```
float32 left_speed
float32 right_speed
```

**`WAFL2025/SteeringAngles`** — `msg/SteeringAngles.msg`
```
float32 front_angle
float32 rear_angle
```

### 6. Controller Details

#### Pure Pursuit Controller (`controller_hardware.py`)

Implements the **Pure Pursuit** geometric steering law extended with:

- **`PoseHandler`** — subscribes to `/amcl_pose`, maintains `[x, y, yaw]`
- **`PathManager`** — consumes the NavFn global path, provides lookahead target via a sliding-window closest-point search; sets a `braking_flag` when within `braking_margin` of the endpoint
- **`ControlModeSelector`** — reads `/path_curvature` and maps it to one of 3 swerve modes (see table above)
- **`SpeedPID`** — standard PID (`kp=2.0, ki=0.0, kd=0.5`, output clipped to ±2.0) providing closed-loop speed around `v_setpoint = 1.0 m/s`; speed estimated by finite-differencing the AMCL pose
- **`SteeringController`** — core compute method:
  1. **In-place orientation fix** — if within `0.8 m` of the goal but heading error > 5°, executes an approximate in-place spin using ±45° swerve angles
  2. **Pure Pursuit steering** — computes steering angle from `arctan2(L·sin(α), lookahead)` where `α` is the angle to the lookahead point
  3. **Hysteresis direction switching** — crab mode uses 70°/110° thresholds to avoid rapid forward/backward oscillation
  4. **Braking** — returns zero velocity with the current steering angles when approaching goal
- **`CommandPublisher`** — splits the 4-element command `[v_left, v_right, steer_rear, steer_front]` into `WheelSpeeds` and `SteeringAngles` messages

#### Swerve Odometry (`swerve_odometry_node.py`)

Integrates swerve drive kinematics at **30 Hz**:

| Parameter | Value |
|-----------|-------|
| Wheel radius | 0.10 m |
| Wheelbase | 0.40 m |
| Track width | 0.30 m |
| Max steering angle | ±45° |
| Speed deadzone | < 0.7 RPM → 0 |

- **Crab mode** detected when `|delta_front - delta_rear| < 0.05 rad` → `omega = 0`, heading = average steer angle
- **Active mode** → instantaneous radius `R = wheelbase / (tan(delta_f) - tan(delta_r))`, `omega = v_avg / R`
- Publishes `/odom` and broadcasts `odom → base_link` TF

#### Curvature Calculator (`curvature_calc.py`)

Runs at **10 Hz**, locates the closest path point to the robot, then computes the **signed curvature** of the triplet `(p_{i-1}, p_i, p_{i+1})` using:

```
curvature = 2 · cross(d1, d2) / (|d1| · |d2| · (|d1| + |d2|))
```

Published to `/path_curvature` as a `Float32MultiArray`.

---

## Hardware Specification

| Component | Details |
|-----------|---------|
| **Chassis** | Custom 4-Wheel Swerve Drive frame |
| **Propulsion** | Left + Right independently-steered motorized swerve modules |
| **Passive support** | Rear castor assembly |
| **LiDAR** | RPLiDAR A-series — `/dev/ttyUSB0`, 115200 baud, `frame_id: laser` |
| **Depth camera** | Microsoft Kinect — `frame_id: kinect_link` |
| **Fork actuator** | Electrically controlled, command via `/fork_command` Bool topic |
| **Microcontroller** | Arduino Mega (low-level motor + steering control) |
| **Communication** | CAN Bus via SocketCAN (`vcan0` for sim / `can0` for hardware, 500 kbps) |
| **Compute** | Linux PC running ROS Noetic |

### Robot Footprint (Costmap)

```
[[-0.4, -0.28], [-0.4, 0.28], [0.9, 0.28], [0.9, -0.28]]
```
Approximately **1.3 m × 0.56 m** — `inflation_radius: 0.5 m`

---

## Prerequisites

| Dependency | Version |
|-----------|---------|
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| Python | 3.x |
| numpy | `pip3 install numpy` |

### Required ROS Packages

```bash
sudo apt install \
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
  ros-noetic-can-msgs
```

---

## Installation & Build

```bash
# 1. Place the package in your catkin workspace
cd ~/catkin_ws/src
cp -r /path/to/WAFL2025 .

# 2. Build
cd ~/catkin_ws
catkin_make

# 3. Source the workspace
source devel/setup.bash
```

> **Tip:** Add `source ~/catkin_ws/devel/setup.bash` to `~/.bashrc` so it is automatically sourced in every terminal.

---

## How to Run

### 1. Visualize Robot Model (URDF Viewer)

Opens RViz with the full URDF and an interactive joint-state slider GUI.

```bash
roslaunch WAFL2025 display.launch
```

---

### 2. Gazebo Simulation

Spawns the robot in an empty Gazebo world.

```bash
roslaunch WAFL2025 gazebo.launch
```

To use the custom warehouse environment:

```bash
rosrun gazebo_ros gazebo $(rospack find WAFL2025)/worlds/home1.world
```

---

### 3. Mapping — Build a New Map (Real Hardware)

Drive the robot around using keyboard teleoperation while Hector SLAM builds the map in real time.

```bash
# Terminal 1 — SLAM + LiDAR + Swerve Odometry + RViz (mapping.rviz)
roslaunch WAFL2025 hector_slam.launch

# Terminal 2 — Keyboard teleoperation
rosrun WAFL2025 teleop_keyboard.py
```

When satisfied with the map, save it:

```bash
rosrun map_server map_saver -f ~/catkin_ws/src/WAFL2025/maps/my_map
```

---

### 4. Localization Only (on a Pre-Built Map)

Load an existing map and run AMCL particle-filter localization with the full sensor and controller stack.

```bash
roslaunch WAFL2025 amcl_localization.launch \
  map_file:=$(rospack find WAFL2025)/maps/office_map.yaml
```

**What this starts:**
- `map_server` — serves the occupancy grid
- `amcl` — particle-filter localization (100–5000 particles, likelihood field laser model)
- `rplidarNode` — LiDAR driver on `/dev/ttyUSB0`
- `swerve_odometry_node` — custom odometry
- `curvature_calc` — live curvature computation
- `controller_hardware` — Pure Pursuit path follower
- `rviz` with `localization.rviz` preset

---

### 5. Full Autonomous Navigation *(Recommended)*

Brings up the entire navigation stack: AMCL + move_base + all custom nodes.

```bash
roslaunch WAFL2025 move_base.launch
```

**Optional arguments:**

```bash
# Override initial pose in map frame
roslaunch WAFL2025 move_base.launch arg_x:=1.0 arg_y:=2.0 arg_Y:=0.0

# Use a different pre-built map
roslaunch WAFL2025 move_base.launch \
  map_file:=/absolute/path/to/your_map.yaml
```

**Workflow once running:**
1. RViz opens automatically with `path_planning.rviz`
2. Use **"2D Pose Estimate"** to set the robot's starting position if AMCL has not converged
3. Use **"2D Nav Goal"** to publish a navigation goal anywhere on the map
4. move_base plans a path with NavFn; the Pure Pursuit controller follows it, switching swerve modes automatically based on path curvature

---

### 6. Full Real-Robot Bring-Up

```bash
# Step 1: Initialize the CAN interface (requires root, run once per boot)
cd $(rospack find WAFL2025)/launch
sudo bash setup_can.sh can0 500000

# Step 2: Launch the hardware stack
roslaunch WAFL2025 real.launch
```

**What this starts:**
- `robot_state_publisher` with `WAFL2025_hw.urdf`
- Static `map → odom` TF
- All ROS controllers via `controller_manager`:
  - `joint_state_controller`
  - `leftweb_steer_controller` / `rightweb_steer_controller`
  - `castorjoint_steer_controller`
  - `leftwheel_drive_controller` / `rightwheel_drive_controller` / `castorwheel_drive_controller`
- RViz with `trial.rviz`

---

### 7. Manual Teleoperation

```bash
# Terminal 1 — Start the swerve drive bridge
rosrun WAFL2025 manual_drive_control.py

# Terminal 2 — Start keyboard control
rosrun WAFL2025 teleop_keyboard.py
```

**Keyboard bindings:**

| Key | Action |
|-----|--------|
| `W` | Move forward |
| `S` | Move backward |
| `A` | Turn left |
| `D` | Turn right |
| `Q` | Strafe left |
| `E` | Strafe right |
| `Space` | Emergency stop |
| `Z` | Zero steering angles |
| `+` | Increase speed (+0.1 m/s, max 2.0 m/s) |
| `-` | Decrease speed (−0.1 m/s, min 0.1 m/s) |
| `Ctrl-C` | Exit |

---

### 8. Supervisor Node (Mission Management)

Manages high-level mission logic independently of the navigation stack.

```bash
rosrun WAFL2025 supervisor.py
```

**Capabilities:**
- Subscribes to `/pallet_detection` (PoseStamped) and raises the fork when the robot is within `0.5 m` of a pallet
- Lowers the fork automatically when moving away
- Controls indicator lights (`/light_command` Int8):
  - `0` — stopped
  - `1` — moving forward
  - `3` / `4` — turning left / right
  - `5` — reversing
- Remaps `/destenation` (MoveBaseActionGoal) → `/move_base_simple/goal` with zero yaw

---

## Configuration Guide

### Changing the Navigation Map

Pass `map_file` at launch time or edit the default in `launch/move_base.launch`:

```bash
roslaunch WAFL2025 move_base.launch \
  map_file:=/absolute/path/to/your_map.yaml
```

Expected map YAML format:
```yaml
image: your_map.pgm
resolution: 0.050000         # meters per pixel
origin: [-25.6, -25.6, 0.0] # [x, y, yaw] of the bottom-left corner
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

---

### Tuning the Pure Pursuit Controller (`controller_hardware.py`)

| Parameter | Location | Default | Effect |
|-----------|----------|---------|--------|
| `braking_margin` | `PathManager.__init__` | `0.3 m` | Distance at which the robot starts braking |
| `lookahead` | `ControllerNode.__init__` | `1.0 m` | Pure Pursuit lookahead distance |
| `base_speed` | `SteeringController.__init__` | `1.0 m/s` | Target cruise speed |
| `SpeedPID.kp` | `SpeedPID.__init__` | `2.0` | Proportional gain |
| `SpeedPID.kd` | `SpeedPID.__init__` | `0.5` | Derivative gain |
| `position_tolerance` | `SteeringController.__init__` | `0.8 m` | Radius at which in-place spin is triggered |
| `orientation_tolerance` | `SteeringController.__init__` | `5°` | Angle at which pose is declared reached |
| `spin_steer_angle` | `SteeringController.__init__` | `±45°` | Wheel angles during in-place rotation |

---

### Tuning the EKF (`config/ekf.yaml`)

The EKF fuses `/odom` (velocity only) and `/imu` (orientation + angular velocity + linear acceleration):

```yaml
ekf_filter_node:
  frequency: 50.0
  odom0: /odom
  imu0: /imu
  odom0_config: [false, false, false,   # X/Y/Z position disabled
                 false, false, false,   # orientation disabled
                 true,  true,  false,   # X/Y linear velocity used
                 false, false, true]    # Z angular velocity (yaw rate) used
```

---

### Adjusting the Robot Footprint (`param/costmap_common_params.yaml`)

```yaml
footprint: [[-0.4, -0.28], [-0.4, 0.28], [0.9, 0.28], [0.9, -0.28]]
inflation_radius: 0.5
cost_scaling_factor: 0.5
```

---

## ROS Graph Summary

```
[rplidarNode] ---/scan---> [amcl] ---/amcl_pose---> [controller_hardware]
                              |                              ^
                         /map |                             |
                              v                     /path_curvature
                        [move_base] --NavFnROS plan--> [curvature_calc]
                                                           |
                                                           v
[teleop_keyboard] --/cmd_vel--> [manual_drive_control]   [controller_hardware]
                                          |                      |
                                   /wheel_speeds           /wheel_speeds
                                   /steering_angles        /steering_angles
                                          '----------+----------'
                                                     |
                                              [middle_man] --/motor_speed--> Arduino
                                                     |
                                     [swerve_odometry] <-- /swerve/raw_data
                                                     |
                                                  /odom --> [amcl], [move_base]
```

---

## Troubleshooting

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Robot does not localize | AMCL particles have not converged | Use **"2D Pose Estimate"** in RViz to set the initial pose manually |
| `No goal pose set!` warning | No global plan has been received yet | Publish a navigation goal via RViz or `rostopic pub` |
| LiDAR not found | Wrong serial port | Run `ls /dev/ttyUSB*` and update `serial_port` in the relevant launch file |
| CAN interface error on startup | `can0` not up | Run `sudo bash launch/setup_can.sh can0 500000` before launching |
| Robot overshoots goal | `braking_margin` too small | Increase `braking_margin` in `controller_hardware.py` |
| Odometry drifts quickly | Velocity scaling constants incorrect | Tune the `0.3`, `0.4`, `0.05` scale factors in `swerve_odometry_node.py` |
| `No module named WAFL2025` | Workspace not sourced | Run `source ~/catkin_ws/devel/setup.bash` |
| Controllers fail to spawn | 5-second delay too short | Increase the `sleep 5` delay in `real.launch` controller spawner |

---

## License

BSD License — see [`package.xml`](package.xml).
