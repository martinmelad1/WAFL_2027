# WAFL 2026 — Complete Architecture & Codebase Walkthrough

> **Project:** Autonomous Forklift — Ain Shams University, Mechatronics Dept. (2025/2026)
> **Stack:** ROS 2 Humble · Ignition Gazebo · micro-ROS · Python · C++ (Arduino/ESP32) · YOLO / TensorRT

---

## 1. Big Picture

The robot is a **custom warehouse forklift** with three-wheel steering (two rear powered wheels + one front steerable caster). The software stack is split across **two laptops** and **three ESP32 microcontrollers**, all communicating over a shared Wi-Fi network via ROS 2 topics.

```
┌─────────────────────────────────────────────────────────────────┐
│                    WAFL2026 SYSTEM                              │
│                                                                 │
│  ┌──────────────┐    ┌────────────────┐    ┌────────────────┐  │
│  │  ESP32 #1    │    │  ESP32 #2      │    │  ESP32 #3      │  │
│  │  DRIVING     │    │  STEERING      │    │  LIFT + LIGHTS │  │
│  │  (micro-ROS) │    │  (micro-ROS)   │    │  (micro-ROS)   │  │
│  └──────┬───────┘    └───────┬────────┘    └────────┬───────┘  │
│         │                    │                      │           │
│  ───────┴────────────────────┴──────────────────────┴───────── │
│                     Wi-Fi / micro-ROS Agent                     │
│  ───────────────────────────────────────────────────────────── │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            Laptop A  (Robot Laptop)                     │   │
│  │  phone_imu_node → odom_fusion_node → /odom              │   │
│  │  encoder_velocity_node → /wheel_velocity                │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            Laptop B  (Navigation Laptop)                │   │
│  │  EKF → AMCL → Nav2 → /cmd_vel                          │   │
│  │  RPLiDAR → /scan                                        │   │
│  └─────────────────────────────────────────────────────────┘   │
│                                                                 │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │            Jetson (Perception / YOLO)                   │   │
│  │  Kinect v1 → pallet_pose_node (ArUco)                   │   │
│  │  Kinect v1 → pallet_front_angle_node (YOLO TensorRT)    │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. Repository Structure

```
WAFL2026/
├── README.md
│
├── Low level/                     ← Arduino/ESP32 firmware
│   └── Arduino Codes/
│       ├── Driving/               ← ESP32 #1: drive motors + encoder
│       │   ├── Driving.ino            (manual PWM, no ROS)
│       │   └── Driving_uROS.ino       (micro-ROS, ACTIVE version)
│       ├── Steering/              ← ESP32 #2: front caster stepper + DC
│       │   ├── Caster_only_uros.ino   (caster only)
│       │   └── steering_all_uROS.ino  (stepper + DC, ACTIVE version)
│       ├── Lifting&Aux/           ← ESP32 #3: lift motor + lights
│       │   ├── code.ino               (standalone test)
│       │   └── Lifting_aux_uROS.ino   (micro-ROS, ACTIVE version)
│       ├── Lights/                ← (unused, lights are in Lifting_aux)
│       └── Swivel _Stepper_/      ← earlier prototype
│
├── Localization/
│   ├── Hardware/                  ← Real robot localization
│   │   ├── encoder_odometry/      ← ROS2 pkg: /encoder_count → /wheel_velocity
│   │   ├── phone_imu/             ← ROS2 pkg: UDP phone IMU → /imu_yaw
│   │   ├── robot_odometry/        ← ROS2 pkg: fuses wheel_velocity + imu_yaw → /odom
│   │   └── hardware_wafl2026/     ← ROS2 pkg: EKF + AMCL + map + bringup
│   │       ├── config/
│   │       │   ├── ekf.yaml           (robot_localization EKF config)
│   │       │   └── amcl_config.yaml   (AMCL particle filter config)
│   │       ├── launch/
│   │       │   ├── bringup_all.launch.py       (ONE-COMMAND full bringup)
│   │       │   └── laptop_b_localization.launch.py
│   │       ├── hardware_wafl2026/
│   │       │   ├── odom_covariance_fix.py   (sim covariance patch)
│   │       │   └── pose_persistence_node.py (saves/restores AMCL pose)
│   │       ├── urdf/              ← robot URDF model
│   │       ├── map/               ← pre-built 2D map (.yaml + .pgm)
│   │       └── meshes/            ← 3D meshes for visualization
│   └── Simulation/
│       └── src/
│           ├── rplidar_ros/       ← RPLiDAR ROS2 driver (submodule/clone)
│           └── wafl2026/          ← Gazebo simulation world + robot model
│
├── Navigation/
│   ├── Hardware/                  ← (empty — nav2 runs on laptop B same as sim)
│   └── Simulation/
│       └── wafl_navigation/       ← ROS2 pkg: Nav2 bringup + caster controller
│           ├── config/
│           │   └── nav2_params.yaml   (full Nav2 stack config)
│           ├── launch/
│           │   └── navigation.launch.py
│           └── wafl_navigation/
│               └── caster_controller.py   (cmd_vel → /castor_cmd_pos)
│
├── Perception/                    ← ROS2 pkg: pallet_vision
│   ├── launch/
│   │   ├── aruco.launch.py
│   │   ├── yolo.launch.py
│   │   └── pallet_vision.launch.py    (combined: Kinect + ArUco + YOLO + Nav)
│   ├── models/                    ← TensorRT .engine files (not in repo, generated)
│   └── pallet_vision/
│       ├── pallet_pose_node.py        (ArUco 6-DOF pose)
│       ├── pallet_front_angle_node.py (YOLO 3-model pipeline)
│       └── pallet_navigator.py        (state machine → Nav2)
│
├── Integration/
│   └── wafl_manual_dashboard.py   ← PyQt5 GUI for manual testing
│
├── Documentation/                 ← (placeholder, empty)
└── Project files/
    └── WAFL_Logo.png
```

---

## 3. Layer 1 — Low-Level Firmware (ESP32 + micro-ROS)

All three ESP32s connect to the laptop Wi-Fi and register as ROS 2 nodes through a single **micro-ROS agent** running on the laptop (port 8888, UDP).

### 3.1 ESP32 #1 — Driving (`Driving_uROS.ino`)

| | |
|---|---|
| **File** | `Low level/Arduino Codes/Driving/Driving_uROS.ino` |
| **Authors** | Omar Emad & Mohamed Montasser |
| **Hardware** | 2× DC motors via H-bridge · 1× encoder (600 PPR) |

**What it does:**
- Subscribes to `/pwm_left` (`Float32`) and `/pwm_right` (`Float32`)
- Applies signed PWM `[-255..255]` to each drive motor
- Publishes raw encoder ticks to `/encoder_count` (`Float32`) continuously
- Negative PWM = reverse; encoder odom is signed by direction of left wheel

**Key parameters (hardcoded):**
```cpp
WHEEL_RADIUS   = 0.1 m
PULSES_PER_REV = 600
GEAR_RATIO     = 1885.0 / 600.0  ≈ 3.14
WiFi IP        = "192.168.0.126"  (micro-ROS agent laptop)
```

**Topics:**
- Sub: `/pwm_left`, `/pwm_right`
- Pub: `/encoder_count`

---

### 3.2 ESP32 #2 — Steering (`steering_all_uROS.ino`)

| | |
|---|---|
| **File** | `Low level/Arduino Codes/Steering/steering_all_uROS.ino` |
| **Hardware** | Stepper motor (TB6600) for front caster · DC motor for axle · 2× encoders |

**What it does:**
- Subscribes to `/target_angle` (`Float32`) — desired steering angle in **degrees**
- Converts angle → encoder target ticks using `(angle/360°) × 600 PPR`
- Runs a **closed-loop correction loop** every loop iteration using the steering encoder
  - If `|error| > DEADBAND (4 ticks)` → steps the stepper one pulse to correct
- Also controls a DC motor (axle/swivel) with encoder feedback for absolute positioning
- **Key insight:** The stepper only does micro-corrections. The main ROS command sets a target; the encoder holds the angle against vibration drift

**Topics:**
- Sub: `/target_angle`

---

### 3.3 ESP32 #3 — Lifting + Lights (`Lifting_aux_uROS.ino`)

| | |
|---|---|
| **File** | `Low level/Arduino Codes/Lifting&Aux/Lifting_aux_uROS.ino` |
| **Hardware** | 1× lifting DC motor · 2× limit switches (up/down) · 4× relay lights |

**What it does:**
- Subscribes to `/lift_cmd` (`Int32`): `0=STOP`, `1=UP`, `2=DOWN`
- Subscribes to `/lights_cmd` (`Int32MultiArray` of 4): front/right/left/back lights
- Safety logic: if the emergency switch `sw0` is pressed → immediate stop
- If a limit switch is hit, only allows motion away from that limit

**Topics:**
- Sub: `/lift_cmd`, `/lights_cmd`

---

## 4. Layer 2 — Localization

### 4.1 Phone IMU (`phone_imu/imu_udp_node.py`)

The team used a **smartphone as an IMU** (an app streams sensor data over UDP).

```
Phone App → UDP packet (CSV: "..., ..., ..., ..., yaw_degrees")
             ↓
    imu_udp_node.py (port 5005)
             ↓
    /imu_yaw (std_msgs/Float32, degrees)
```

**Key design choice:** Only the **yaw** (heading) is extracted from the 5-field CSV. Roll and pitch are ignored (2D robot).

---

### 4.2 Encoder Odometry (`encoder_odometry/encoder_velocity_node.py`)

```
/encoder_count (raw ticks from ESP32 #1)
    ↓
encoder_velocity_node.py
    ↓
/wheel_velocity (Float32, metres/second)
```

**Math:**
```python
wheel_velocity = (2π × r × Δticks) / (PPR × gear_ratio × Δt)
```
Where `r=0.1m`, `PPR=600`, `gear_ratio≈3.14`.

---

### 4.3 Odometry Fusion (`robot_odometry/odom_fusion_node.py`)

This is the **dead-reckoning odometry node** that integrates encoder speed + IMU heading into a 2D pose.

```
/wheel_velocity (m/s)   ─┐
/imu_yaw (degrees)      ─┴→ odom_fusion_node → /odom (nav_msgs/Odometry)
```

**Integration at 50 Hz:**
```python
self.x += v * cos(yaw) * dt
self.y += v * sin(yaw) * dt
```

**Critical design notes:**
- **No TF broadcast** from this node. The EKF (robot_localization) is the sole publisher of `odom → base_footprint` TF edge
- Has a `velocity_timeout` of 0.5s: if no encoder message arrives, assumes robot is stopped
- Has `max_dt` clamping (0.2s) to prevent large jumps on startup

---

### 4.4 EKF + AMCL (`hardware_wafl2026/config/ekf.yaml`)

**Extended Kalman Filter (`robot_localization` package):**
- Input: `/odom` — uses only `vx` and `vyaw` (velocities), NOT positions
  - This avoids double-integrating position (already done in odom_fusion_node)
- Output: smoothed pose + publishes `odom → base_footprint` TF
- Mode: `two_d_mode: true` (ignores Z/roll/pitch)
- Rate: 50 Hz

**AMCL (Adaptive Monte Carlo Localization):**
- Input: `/scan` (RPLiDAR), `/odom`
- Uses pre-built map + particle filter to localize in the map frame
- 500–5000 particles, likelihood field model, max range 8m
- Config: `DifferentialMotionModel` (robot moves like a diff-drive for AMCL purposes)

---

### 4.5 Pose Persistence (`hardware_wafl2026/pose_persistence_node.py`)

Solves a practical problem: **AMCL forgets position when you restart the robot**.

- Saves `/amcl_pose` to `~/.ros/amcl_last_pose.yaml` every 2 seconds
- On startup, reads the file and republishes to `/initialpose` **3 times** (1s apart) to overcome timing races with AMCL's volatile QoS
- Started **16 seconds** after launch (after AMCL activates)

---

### 4.6 Sensor Covariance Fix (`hardware_wafl2026/odom_covariance_fix.py`)

For the **simulation only**: Gazebo's DiffDrive plugin publishes all-zero covariance matrices. The EKF cannot invert a zero matrix. This node relays `/odom → /odom_fixed` and `/imu → /imu_fixed` with injected diagonal covariances.

---

### 4.7 Bringup — Single Command (`bringup_all.launch.py`)

The **master launch file** that starts everything on the real robot:

| Time | What starts |
|------|------------|
| t=0s | Robot odometry stack (encoder + IMU + odom_fusion) |
| t=0s | RPLiDAR driver |
| t=0s | Static TF: `rplidar_link → laser` |
| t=0s | Localization stack (RSP + EKF + map_server + AMCL) |
| t=10s | RViz2 |
| t=16s | Pose persistence node |

---

## 5. Layer 3 — Navigation

### 5.1 Nav2 Stack (`wafl_navigation/config/nav2_params.yaml`)

Full Nav2 stack configured for a **differential-drive forklift**:

| Component | Plugin | Notes |
|-----------|--------|-------|
| Global Planner | `NavfnPlanner` (Dijkstra) | `use_astar: false`, 0.5m tolerance |
| Local Planner | `DWBLocalPlanner` | 20 vx samples, 40 vθ samples |
| AMCL | `DifferentialMotionModel` | Same AMCL as localization |
| Global Costmap | Static + Obstacle + Inflation | 5cm resolution, 0.35m inflation |
| Local Costmap | Obstacle + Inflation | 3×3m rolling window |
| Behaviors | Spin, BackUp, Wait | Recovery behaviors |

**Speed limits:**
```yaml
max_vel_x: 1.00 m/s   min_vel_x: 0.15 m/s
max_vel_theta: 2.0 rad/s
```

**DWB critics (weights):**
- PathDist ×32, GoalDist ×24 → strong path following
- PathAlign ×12, GoalAlign ×12 → alignment preference
- RotateToGoal ×4 → smooth approach

---

### 5.2 Caster Controller (`wafl_navigation/caster_controller.py`)

Translates Nav2's `cmd_vel` (`Twist`) into a steering angle for the front caster.

```
/cmd_vel (Twist) → caster_controller → /castor_cmd_pos (Float64, radians)
```

**Steering kinematics:**
```python
# Normal motion: Ackermann-like angle
angle = atan2(-w * L, v)   # L=0.5m wheelbase, negate because caster is rear

# Pure rotation in place:
angle = ±π/2               # perpendicular to turn axis

# Clamped to ±80°
```

> ⚠️ **Note:** This node routes to `/castor_cmd_pos` (Gazebo joint controller topic), not `/target_angle` (the ESP32 micro-ROS topic). The connection between simulation caster commands and real hardware still needs bridging.

---

## 6. Layer 4 — Perception

The perception system has two parallel pipelines sharing the same **Kinect v1 camera** (RGB + depth).

### 6.1 Pipeline A — ArUco Markers (`pallet_pose_node.py`)

**Purpose:** Identify *which* pallet the robot is looking at, and *where* it is.

```
Kinect RGB → pallet_pose_node
    ↓ OpenCV ArUco detection (DICT_4X4_50)
    ↓ solvePnP (IPPE_SQUARE solver)
    ↓
/pallet_pose        (PoseStamped — full 6DOF in camera frame)
/fork_target        (PointStamped — 0.5m in front of marker)
/pallet_location    (String — "Location_A1", etc.)
/pallet_location_id (String — raw marker ID "0","1","2","3")
/pallet_marker      (Marker — RViz cube + text for visualization)
```

**Location database (hardcoded):**
```python
0 → "Location_A1"  (x=5.0,  y=2.0, yaw=0)
1 → "Location_A2"  (x=5.0,  y=5.5, yaw=0)
2 → "Location_B1"  (x=12.0, y=2.0, yaw=π/2)
3 → "Location_B2"  (x=12.0, y=5.5, yaw=π/2)
```

**Kinect v1 intrinsics (calibrated):**
```
fx=fy=526.607,  cx=318.525,  cy=241.181
k1=0.103, k2=-0.186 (barrel distortion)
```

---

### 6.2 Pipeline B — YOLO Fine Alignment (`pallet_front_angle_node.py`)

**Purpose:** Once near the pallet, compute precise angles for fork insertion using YOLO + depth.

**Hardware:** Runs on **NVIDIA Jetson** with **TensorRT** engines.

**3 YOLO models:**
| Model | Task | File |
|-------|------|------|
| `model_pallet` | Detect the full pallet bounding box | `bestt.engine` |
| `model_pocket` | Detect fork pocket inside the pallet | `besttt.engine` |
| `model_keypoint` | Detect 4 corner keypoints of pocket | `best_pose12.engine` |

**Processing pipeline (per RGB frame):**
1. **Detect pallet** bounding box
2. **Inside pallet ROI**: detect fork pockets
3. **Inside pocket ROI** (+ 22% padding): detect 4 keypoints
4. **Back-project** 4 keypoints using depth image → 4 × 3D points
5. **Fit coordinate frame:** x-axis = horizontal edge, z-axis = normal via cross product
6. **Compute angles:**
   - `pallet_yaw_offset_rad` — how much the pallet face is rotated in camera frame
   - `rotate_to_front_rad = -pallet_yaw` — how much to rotate forklift to face pallet squarely
   - `center_yaw_rad` — bearing of pallet center from camera optical axis
   - `final_z_m` — horizontal distance (depth corrected for camera height of 0.57m)
7. **Maintain 3-second circular running average** (simple + time-weighted)

**Fallback hierarchy:**
- Primary: keypoints from pocket ROI
- Fallback 1: keypoints from full pallet ROI
- Fallback 2: keypoints from entire image

**Published topics:**
```
/pallet/rotate_to_front_rad           (instantaneous)
/pallet/pallet_yaw_offset_rad
/pallet/center_yaw_rad
/pallet/final_z_m
/pallet/rotate_to_front_avg_rad       (3s simple circular mean)
/pallet/rotate_to_front_weighted_avg_rad (3s time-weighted)
... (same for pallet_yaw and center_yaw)
/pallet/debug_image                   (annotated frame for rqt_image_view)
```

---

### 6.3 High-Level Mission Planner (`pallet_navigator.py`)

State machine that bridges perception output and Nav2 navigation.

```
States: IDLE → MARKER_READ → NAVIGATING → IDLE
```

**Transitions:**
1. **IDLE → MARKER_READ:** ArUco node publishes `/pallet_location_id` → look up dropoff destination in DB
2. **MARKER_READ → NAVIGATING:** YOLO node publishes `final_z_m ≈ 0` (< 5cm) → pallet is lifted → send `NavigateToPose` action to Nav2
3. **NAVIGATING → IDLE:** Nav2 arrives at destination (success or failure) → reset

**Key assumption:** `final_z_m = 0` means the forklift has physically reached and lifted the pallet. The YOLO node should eventually publish 0 when the pallet is directly under the camera (i.e., lifted).

> ⚠️ **Gap:** The actual fork-insertion sequence (approach, align, insert forks, lift) is not automated. The system assumes a human or separate controller handles insertion; it only waits for the "lifted" signal.

---

## 7. Layer 5 — Integration

### 7.1 Manual Dashboard (`Integration/wafl_manual_dashboard.py`)

A PyQt5 GUI for manual testing and debugging of the real robot.

**Publishes:**
- `/move_distance` (`Float32`) → distance command (what listens? likely a motion controller not in this repo)
- `/target_angle` (`Float32`) → goes to ESP32 #2 steering
- `/lift_cmd` (`Int32`) → goes to ESP32 #3 lifting
- `/lights_cmd` is NOT in the GUI (lights controlled separately)

**Features:**
- One-shot or continuous publishing for drive commands
- Starts the micro-ROS agent subprocess (`ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888`)
- Spins ROS in a Qt timer (50ms = 20Hz)

---

## 8. Complete ROS Topic Graph

```
                ┌─────────────────────────────────────────────────────┐
                │                   MICRO-ROS (Wi-Fi)                │
                │                                                     │
  ESP32#1 ─────│──→ /encoder_count ──→ encoder_velocity_node         │
                │                            │                        │
                │              /pwm_left ←───┤ (from motion ctrl?)    │
                │              /pwm_right ←──┘                        │
                │                                                     │
  ESP32#2 ─────│── /target_angle ←── caster_controller (via bridge?) │
                │                                                     │
  ESP32#3 ─────│── /lift_cmd ←───── dashboard / pallet_navigator?    │
                │   /lights_cmd ←──                                   │
                └─────────────────────────────────────────────────────┘

  Phone ────────→ UDP:5005 → imu_udp_node → /imu_yaw ──────────────┐
                                                                     │
  encoder_velocity_node → /wheel_velocity ──────────────────────────┤
                                                                     ↓
                                                           odom_fusion_node
                                                                     │
                                                                /odom │
                                                                     ↓
  RPLiDAR ──────────────────────────────────→ /scan ──→ EKF + AMCL
                                                            │
                                                   /odom_filtered
                                                   TF: odom→base_footprint
                                                   TF: map→odom (AMCL)
                                                            │
                                                        Nav2 Stack
                                                            │
                                                        /cmd_vel
                                                            │
                                                    caster_controller
                                                            │
                                                   /castor_cmd_pos

  Kinect RGB ──→ pallet_pose_node ──→ /pallet_location_id ──→ pallet_navigator
  Kinect RGB+D → pallet_front_angle_node → /pallet/final_z_m ──→ pallet_navigator
                                                                       │
                                                              NavigateToPose → Nav2
```

---

## 9. Simulation vs Hardware

| Aspect | Simulation | Hardware |
|--------|-----------|---------|
| Simulator | Ignition Gazebo | — |
| Localization | AMCL + EKF + Gazebo DiffDrive | AMCL + EKF + encoder/IMU fusion |
| Odometry source | Gazebo DiffDrive → odom_covariance_fix → /odom_fixed | odom_fusion_node from real encoder + phone IMU |
| Laser | Simulated RPLIDAR in Gazebo | Real RPLiDAR A2M8 |
| Caster control | /castor_cmd_pos (Gazebo joint controller) | /target_angle → ESP32 #2 |
| IMU | Simulated Gazebo IMU | Phone app over UDP |

---

## 10. Known Gaps & Issues

| # | Issue | Location | Impact |
|---|-------|----------|--------|
| 1 | `/castor_cmd_pos` (sim) ≠ `/target_angle` (hardware) | caster_controller.py | Caster control not wired to real ESP32 |
| 2 | `/move_distance` topic consumer missing | dashboard.py | Drive commands from GUI go nowhere |
| 3 | Location DB hardcoded in two places | pallet_pose_node.py & pallet_navigator.py | Must be updated in both files if locations change |
| 4 | No fork-insertion sequence | pallet_navigator.py | Full autonomy not implemented; relies on manual fork operation |
| 5 | WiFi credentials hardcoded in firmware | All *.ino files | Must reflash on network change |
| 6 | No pose covariance propagation from phone IMU | odom_fusion_node.py | Fixed covariance values, not adaptive |
| 7 | AMCL uses DifferentialMotionModel despite non-diff-drive | amcl_config.yaml | May cause AMCL drift during turning |
| 8 | Simulation nav2_params.yaml uses `use_sim_time: true` | nav2_params.yaml | Would fail on real hardware without change |
| 9 | Documentation/ folder is empty | Documentation/ | No design docs |
| 10 | Localization/Simulation/src contains full ROS packages | git submodules? | Large untracked code, unclear if maintained |

---

## 11. Suggested Improvements

### Short-Term (to get it working better)
1. **Wire the caster properly:** Bridge `/cmd_vel → /target_angle` on real hardware, not just `/castor_cmd_pos` for simulation. Unify into a single node.
2. **Load location DB from YAML:** Replace the hardcoded `ID_TO_LOCATION` dict with a ROS parameter loaded from a shared YAML file.
3. **Replace phone IMU:** Use a proper IMU (e.g., MPU-6050 or BNO055 on the ESP32) for real `sensor_msgs/Imu` messages; remove the fragile UDP protocol.
4. **WiFi credentials:** Move to environment variables or a config file not committed to git.

### Medium-Term (to add autonomy)
5. **Fork-insertion sequence:** Implement an approach-and-insert state machine using `rotate_to_front_rad` and `center_yaw_rad` from the YOLO node to auto-align and drive into the pallet pockets.
6. **Better odometry:** Switch from single-encoder to two-wheel differential encoders for proper angular velocity measurement.
7. **AMCL motion model:** Use `OmniMotionModel` or properly model the caster steering in AMCL.

### Long-Term (architectural improvements)
8. **Mission management:** Replace the simple state machine with a proper BT (Behavior Tree) using Nav2's BT infrastructure.
9. **Multi-pallet support:** Currently only one marker per frame. Extend to detect all visible markers simultaneously.
10. **Safety zone:** Add an obstacle detection zone in front of the forks that halts the robot if something is between the forks unexpectedly.

---

## 12. How to Run (Summary)

### Simulation
```bash
# Terminal 1 — Gazebo
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch wafl2026 humble4.launch.py

# Terminal 2 — Nav2 Navigation
ros2 launch wafl_navigation navigation.launch.py

# Terminal 3 — Caster Controller
ros2 run wafl_navigation caster_controller

# Terminal 4 — RViz
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Real Robot
```bash
# Laptop A — Robot laptop (runs odometry)
ros2 launch hardware_wafl2026 bringup_all.launch.py

# Laptop B — Navigation laptop (automatic from bringup_all)
# Nothing extra needed — bringup_all includes localization + Nav2

# Jetson — Perception
ros2 launch pallet_vision pallet_vision.launch.py

# Manual control (optional)
python3 Integration/wafl_manual_dashboard.py
```

### micro-ROS Agent (must run before ESP32s boot)
```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

---

*Scanned and documented by Antigravity — September 2026*
