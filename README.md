# WAFL 2027 — Team README

> **Project:** Warehouse Autonomous Fork-Lift — Ain Shams University, Mechatronics & Robotics Engineering 2026/2027
> **Stack:** ROS 2 Jazzy · Gazebo Harmonic · Jetson Xavier AGX · micro-ROS · Python 3.12 · YOLO / TensorRT
> **Status:** Phase 1 in progress — Simulation + GUI + Manual Control

---

## Read This First

This file is the **single source of truth** for every team member.

Before writing a single line of code:
1. Read this whole file once — it takes 15 minutes
2. Find your assigned node(s) in §4
3. Look up every topic your node uses in the Master Topic Index (§7)
4. Follow the Team Rules (§12) — broken builds block everyone

---

## Table of Contents

1. [Big Picture](#1-big-picture)
2. [Build Order — Phase 1 then Phase 2](#2-build-order)
3. [Repository Structure](#3-repository-structure)
4. [Node Reference — Every Node Defined](#4-node-reference--every-node-defined)
5. [GUI Dashboard](#5-gui-dashboard--dashboard)
6. [Simulation — wafl_simulation](#6-simulation--wafl_simulation)
7. [Master Topic Index](#7-master-topic-index)
8. [Custom Messages & Actions](#8-custom-messages--actions--wafl_interfaces)
9. [Design Decisions Tracker](#9-design-decisions-tracker)
10. [Simulation vs Hardware Reference](#10-simulation-vs-hardware-reference)
11. [How to Run](#11-how-to-run)
12. [Team Rules](#12-team-rules)

---

## 1. Big Picture

WAFL 2027 is a **fully autonomous warehouse forklift** built on a single Jetson Xavier AGX. It:

- Receives transport missions from a **WMS (Warehouse Management System)**
- **Localizes** itself in a live-built SLAM map (no pre-built map required)
- **Plans and executes** collision-free routes through the warehouse using Nav2
- **Identifies and aligns** with pallets using a **single front-facing camera** (Kinect v1: ArUco markers + YOLO)
- **Autonomously inserts forks, lifts the pallet, transports and places it**
- **Reports mission status** back to the WMS in real time
- Monitors its **battery** and charges autonomously when needed
- Is operated and monitored via a **PySide6 GUI dashboard** with 3 tabs

This is a major upgrade from WAFL 2026. The primary new systems: WMS integration, fork insertion automation, SLAM (live mapping), GUI dashboard, autonomous charging.

---

## 2. Build Order

We build in two phases. **Phase 2 cannot start until Phase 1 is solid.**

```
╔══════════════════════════════════════════════════════════════╗
║  PHASE 1 — Build This First (Simulation + GUI + Manual)      ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  wafl_interfaces   →  define all custom msgs first           ║
║  wafl_description  →  URDF in Gazebo Harmonic                ║
║  wafl_simulation   →  Gazebo world + ros_gz_bridge           ║
║  wafl_localization →  EKF + SLAM Toolbox (sim mode)          ║
║  wafl_navigation   →  Nav2 + unified base_controller         ║
║  wafl_perception   →  ArUco + YOLO (CPU mode in sim)         ║
║  dashboard/        →  PySide6 GUI (all 3 tabs)               ║
║  wafl_mission      →  camera_stream_node + STUBS for rest    ║
║                                                              ║
║  GOAL: single command launches sim, robot navigates,         ║
║        GUI shows map + camera + status, manual override ok   ║
╠══════════════════════════════════════════════════════════════╣
║  PHASE 2 — Layer Autonomy on Top                             ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  fork_insertion_node  →  full state machine                  ║
║  mission_manager_node →  top-level orchestrator              ║
║  wms_bridge_node      →  real MQTT to WMS server             ║
║  battery_monitor_node →  real BMS hardware                   ║
║  charging_dock_node   →  autonomous docking                  ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
```

**Phase 1 Checklist:**
- [ ] `wafl_interfaces` — all custom msgs/srvs/actions compiled
- [ ] URDF spawning in Gazebo Harmonic without errors
- [ ] Robot drives in Gazebo when `base_controller` publishes `/cmd_vel_out` (Gazebo never reads raw `/cmd_vel`)
- [ ] SLAM Toolbox builds a map; map visible in RViz
- [ ] Nav2 sends robot to a goal pose autonomously
- [ ] ArUco node detects simulated pallet markers
- [ ] GUI Tab 1 shows live map, robot pose, camera feed
- [ ] GUI Tab 2 shows node health + `/rosout` log stream
- [ ] GUI Tab 3 shows stub mission list from FastAPI WMS
- [ ] E-STOP button halts robot immediately
- [ ] Manual joystick mode works: GUI → `/manual_cmd_vel` → robot moves

---

## 3. Repository Structure

```
WAFL_2027/
│
├── README.md                        ← THIS FILE — single source of truth
│
├── docker/
│   ├── Dockerfile                   ← Ubuntu 24.04 + ROS 2 Jazzy + all deps
│   ├── docker-compose.yml           ← One command: docker compose up
│   └── entrypoint.sh
│
├── src/                             ← Single colcon workspace
│   │
│   ├── wafl_interfaces/             ← BUILD FIRST — all custom msg/srv/action
│   │   ├── msg/
│   │   │   ├── WmsMission.msg
│   │   │   ├── WmsMissionStatus.msg
│   │   │   ├── RobotStatus.msg
│   │   │   └── Alarm.msg
│   │   ├── srv/
│   │   │   └── SetMission.srv
│   │   └── action/
│   │       └── InsertForks.action
│   │
│   ├── wafl_description/            ← URDF, meshes, joint definitions
│   │   ├── urdf/
│   │   │   └── wafl.urdf.xacro
│   │   ├── meshes/
│   │   └── launch/
│   │       ├── view_urdf.launch.py
│   │       └── rsp.launch.py        ← robot_state_publisher (TF tree)
│   │
│   ├── wafl_simulation/             ← Gazebo Harmonic world + spawn + bridge
│   │   ├── wafl_simulation/
│   │   │   └── sim_fork_adapter.py  ← /lift_cmd <-> fork joint, /fork_height (sim only)
│   │   ├── worlds/
│   │   │   └── warehouse.world      ← shelves, pallets, ArUco markers, dock
│   │   ├── models/                  ← pallet models with ArUco textures
│   │   ├── launch/
│   │   │   └── gazebo.launch.py
│   │   └── config/
│   │       └── ros_gz_bridge.yaml   ← topic bridging Gazebo <-> ROS 2
│   │
│   ├── wafl_localization/           ← EKF + SLAM + odom pipeline
│   │   ├── wafl_localization/
│   │   │   ├── encoder_velocity_node.py
│   │   │   ├── odom_fusion_node.py
│   │   │   └── odom_covariance_fix.py   ← sim only
│   │   ├── config/
│   │   │   ├── ekf.yaml
│   │   │   └── slam_toolbox.yaml
│   │   └── launch/
│   │       ├── localization.launch.py
│   │       └── slam_mapping.launch.py
│   │
│   ├── wafl_navigation/             ← Nav2 + base_controller
│   │   ├── wafl_navigation/
│   │   │   └── base_controller.py       ← unified steer + drive: sim + hardware via param
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   ├── wafl_perception/             ← ArUco + YOLO + debug image
│   │   ├── wafl_perception/
│   │   │   ├── pallet_pose_node.py
│   │   │   └── pallet_front_angle_node.py
│   │   ├── models/                  ← TensorRT .engine files (NOT in git)
│   │   ├── config/
│   │   │   └── locations.yaml       ← pallet location DB (shared YAML)
│   │   └── launch/
│   │       ├── aruco.launch.py
│   │       ├── yolo.launch.py
│   │       └── perception.launch.py
│   │
│   ├── wafl_fork_insertion/         ← Fork automation state machine
│   │   ├── wafl_fork_insertion/
│   │   │   └── fork_insertion_node.py   ← STUB in Phase 1
│   │   └── launch/
│   │       └── fork_insertion.launch.py
│   │
│   ├── wafl_mission/                ← Mission manager + WMS bridge + camera stream
│   │   ├── wafl_mission/
│   │   │   ├── mission_manager_node.py  ← STUB in Phase 1
│   │   │   ├── wms_bridge_node.py       ← STUB in Phase 1
│   │   │   └── camera_stream_node.py    ← Phase 1 priority
│   │   └── config/
│   │       └── wms_config.yaml
│   │
│   ├── wafl_energy/                 ← Battery + charging dock (Phase 2)
│   │   └── wafl_energy/
│   │       ├── battery_monitor_node.py  ← STUB in Phase 1
│   │       └── charging_dock_node.py    ← Phase 2 only
│   │
│   └── wafl_bringup/                ← ament package (ros2 launch needs a package)
│       └── launch/
│           ├── simulation.launch.py ← Full sim: Gazebo + SLAM + Nav2 + Perc
│           ├── hardware.launch.py   ← Full hardware bringup
│           └── mapping.launch.py    ← SLAM mapping session only
│
├── dashboard/                       ← PySide6 GUI (Phase 1 priority)
│   ├── main.py                      ← entry point
│   ├── ros_bridge.py                ← rclpy spin thread → Qt signals
│   ├── tabs/
│   │   ├── robot_tab.py             ← Tab 1: Map + controls + camera
│   │   ├── logs_tab.py              ← Tab 2: Node health + event log
│   │   └── wms_tab.py              ← Tab 3: WMS package grid + missions
│   ├── widgets/
│   │   ├── map_widget.py            ← OccupancyGrid → QImage renderer
│   │   ├── camera_widget.py         ← TCP JPEG stream receiver
│   │   ├── battery_widget.py
│   │   └── joystick_widget.py
│   ├── requirements.txt
│   └── assets/
│       └── styles.qss
│
├── wms_server/                      ← FastAPI WMS stub (Phase 1)
│   ├── main.py
│   ├── requirements.txt
│   └── data/
│       ├── packages.json
│       └── locations.json
│
├── firmware/                        ← ESP32 Arduino sketches
│   ├── esp32_driving/
│   │   └── Driving_uROS.ino
│   ├── esp32_steering_imu/
│   │   └── Steering_IMU_uROS.ino   ← UPGRADED: real IMU, no phone
│   └── esp32_lift_lights/
│       └── Lifting_aux_uROS.ino
│
└── docs/
    ├── WAFL2027_Architecture.md     ← Long-form rationale (README wins on any conflict)
    └── diagrams/                    ← drawio source + png/svg exports
```

---

## 4. Node Reference — Every Node Defined

> **Legend:** `[P1]` = Phase 1 | `[P2]` = Phase 2 | `[STUB]` = Phase 1 stub only

---

### 4.1 Hardware / Firmware (ESP32 + micro-ROS)

The three ESP32s communicate with the Jetson via a **micro-ROS agent** running on the Jetson (UDP port 8888). Start the agent **before** powering the ESP32s.

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### ESP32 #1 — Driving `[FIRMWARE]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/target_speed_left` | `Float32` | Target speed (m/s) for left drive; ESP32 runs the PID against encoder feedback |
| Sub | `/target_speed_right` | `Float32` | Target speed (m/s) for right drive; ESP32 runs the PID against encoder feedback |
| Pub | `/encoder_count` | `Float32` | Raw encoder ticks (left wheel) |

Hardware: 2× DC motors + H-bridge, 1× encoder (600 PPR, gear ratio ≈ 3.14)
**Control:** the ESP32 closes the speed loop (PID) itself and owns the tick→speed conversion; its wheel radius / gear ratio must match the `encoder_velocity_node` params.
> ⚠️ With a single encoder (left wheel) only the left loop has feedback; the right wheel runs feed-forward (open loop) until D8 (right-wheel encoder) is resolved.

#### ESP32 #2 — Steering + IMU `[FIRMWARE — UPGRADED]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/target_angle` | `Float32` | Desired caster angle in degrees |
| Pub | `/imu/data` | `sensor_msgs/Imu` | Orientation + angular velocity + acceleration, with covariances (BNO055: fused 9-DOF · MPU-6050: 6-DOF, yaw drifts) |

Hardware: Stepper motor (TB6600) + MPU-6050 or BNO055 IMU
> ⚠️ A stepper has no position feedback: it needs a home switch (or potentiometer) at boot so `/target_angle` is absolute (D16). MPU-6050 has no magnetometer, so yaw drifts — BNO055 recommended (D19).
**2027 change:** Replaces fragile phone-IMU UDP protocol entirely. Uses proper `sensor_msgs/Imu` with real covariance matrices.

#### ESP32 #3 — Lift + Lights `[FIRMWARE]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/lift_cmd` | `Int32` | 0=STOP, 1=UP, 2=DOWN |
| Sub | `/lights_cmd` | `Int32MultiArray` | 4-element array, 0/1 per relay |
| Pub | `/fork_height` | `Float32` | Fork height in mm |

Hardware: 1× lift DC motor, 2× limit switches (top/bottom), 4× relay-controlled lights, potentiometer or encoder for fork height.
**Single-writer rule (lift / lights):** several nodes publish these topics, so ownership is by mode. GUI Tab 1 publishes `/lift_cmd` / `/lights_cmd` **only in MANUAL**; `fork_insertion_node`, `charging_dock_node` (lift) and `mission_manager_node` (lights) publish **only in AUTO**. On `/e_stop` the fork and dock nodes publish `/lift_cmd = 0` (STOP).

#### Sensor drivers `[hardware only]`

| Driver | Publishes | Notes |
|--------|-----------|-------|
| RPLiDAR driver (`rplidar_ros`) | `/scan` | frame `laser_link` |
| Kinect v1 driver (libfreenect-based ROS 2 node) | `/camera/image_raw`, `/camera/depth/image_raw` | frame `camera_link`; confirm Jazzy / Jetson support early |

Both are started by `hardware.launch.py`. In simulation the same topics come from `ros_gz_bridge` (§6).

---

### 4.2 Localization Layer — `wafl_localization`

#### `encoder_velocity_node` `[P1]`

**Purpose:** Convert raw encoder ticks → linear velocity (m/s).

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/encoder_count` | `Float32` |
| Pub | `/wheel_velocity` | `Float32` |

**Params:** `wheel_radius` (0.1 m), `pulses_per_rev` (600), `gear_ratio` (3.14)
**Rate:** Event-driven on incoming encoder messages (~50 Hz)
**Math:** `v = (2π × r × Δticks) / (PPR × gear_ratio × Δt)`

---

#### `odom_fusion_node` `[P1]`

**Purpose:** Fuse wheel velocity + IMU heading → dead-reckoning `/odom`.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/wheel_velocity` | `Float32` | From encoder_velocity_node |
| Sub | `/imu/data` | `sensor_msgs/Imu` | Extracts yaw from quaternion |
| Pub | `/odom` | `nav_msgs/Odometry` | 2D dead-reckoning pose |

**Params:** `velocity_timeout` (0.5 s), `max_dt` (0.2 s)
**Rate:** 50 Hz timer
**Important:** Does NOT broadcast TF. The EKF is the sole `odom → base_footprint` TF publisher.

---

#### `odom_covariance_fix` `[P1 — SIMULATION ONLY]`

**Purpose:** Gazebo DiffDrive outputs zero covariance matrices — EKF cannot invert them. Relay node that injects valid diagonal covariances.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/odom` | `Odometry` |
| Sub | `/imu` | `Imu` |
| Pub | `/odom_fixed` | `Odometry` |
| Pub | `/imu_fixed` | `Imu` |

Only launched in simulation (`sim:=true`). In simulation `odom_fusion_node` is **not** launched — Gazebo DiffDrive is the `/odom` source.

---

#### EKF — `robot_localization` `[P1]`

**Purpose:** Fuse odometry + IMU → smooth filtered odometry. The sole publisher of `odom → base_footprint` TF.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/odom` or `/odom_fixed` | `Odometry` | Uses velocity only (vx, vyaw) |
| Sub | `/imu/data` or `/imu_fixed` | `Imu` | Full IMU with covariances |
| Pub | `/odometry/filtered` | `Odometry` | Smooth fused output |
| Pub | TF `odom → base_footprint` | — | Sole publisher of this TF edge |

**Config:** `config/ekf.yaml` | **Rate:** 50 Hz | **Mode:** `two_d_mode: true`

---

#### SLAM Toolbox `[P1]`

**Purpose:** Build live occupancy map from LiDAR; publish `map → odom` TF.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/scan` | `sensor_msgs/LaserScan` |
| Sub | TF `odom → base_footprint` | — (SLAM Toolbox reads odometry through TF, not a topic) |
| Pub | `/map` | `nav_msgs/OccupancyGrid` |
| Pub | TF `map → odom` | — |

**Config:** `config/slam_toolbox.yaml`
**Mode:** Async mapping. Map can be serialized and reloaded for localization-only mode.

---

#### `robot_state_publisher` `[P1]` (launched from `wafl_description/rsp.launch.py`)

**Purpose:** Publish the static robot TF tree from the URDF (`wafl_description`). Without it, LiDAR scans and camera frames have no transform to the robot and SLAM / Nav2 / pallet nodes all fail.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/joint_states` | `JointState` | Caster / fork joints (sim: from the bridge · hardware: open, see D21) |
| Pub | `/tf_static`, `/tf` | `TFMessage` | `base_footprint → base_link → laser_link / camera_link / imu_link / fork_link` |

**Full TF tree — exactly one publisher per edge:**
```
map ──(SLAM Toolbox)──► odom ──(EKF)──► base_footprint ──(robot_state_publisher)──► base_link ──► sensors
```
Gazebo's DiffDrive TF publishing **must be disabled**, otherwise two nodes publish `odom → base_footprint`.
**Clock:** every node runs with `use_sim_time` and, in sim, `/clock` must be bridged from Gazebo (§6).

---

### 4.3 Navigation Layer — `wafl_navigation`

#### `base_controller` `[P1]`

**Purpose:** Convert a velocity command into **left/right wheel speed setpoints** and a **front caster angle**. Single owner of ESP32 #1 (driving) and ESP32 #2 (steering), and the single **arbiter** of who may command motion. Unified node for sim and hardware.

**Robot geometry:** differential rear drive (2 DC motors) + one front caster steered by a stepper.

| Direction | Topic | Type | Condition |
|-----------|-------|------|-----------|
| Sub | `/manual_cmd_vel` | `Twist` | MANUAL mode — highest priority |
| Sub | `/fork_insertion/cmd_vel` | `Twist` | Fine approach (APPROACH → INSERT_FORKS) |
| Sub | `/charging_dock/cmd_vel` | `Twist` | Final dock approach |
| Sub | `/cmd_vel` | `Twist` | AUTO mode (Nav2) — lowest priority |
| Sub | `/gui/mode` | `String` | MANUAL / AUTO selector |
| Sub | `/e_stop` | `Bool` | Latched stop |
| Pub | `/cmd_vel_out` | `Twist` | Arbitrated command, both modes (sim: Gazebo DiffDrive input) |
| Pub | `/target_angle` | `Float32` (degrees) | `sim:=false` (hardware) |
| Pub | `/target_speed_left` | `Float32` (m/s) | `sim:=false` (hardware) |
| Pub | `/target_speed_right` | `Float32` (m/s) | `sim:=false` (hardware) |
| Pub | `/castor_cmd_pos` | `Float64` (radians) | `sim:=true` (simulation; the topic keeps the 2026 spelling "castor") |

**Arbitration (evaluated every cycle):** `/e_stop` → output zero · MANUAL → only `/manual_cmd_vel` · AUTO → `/fork_insertion/cmd_vel` or `/charging_dock/cmd_vel` if received in the last 0.2 s, otherwise `/cmd_vel`.
**Command timeout:** if the selected source is silent for `cmd_timeout` (0.5 s) output zero. The ESP32s apply the same timeout to `/target_speed_*` / `/target_angle`.
**Wheel speeds (differential):** `v_left = v − ω·W/2`, `v_right = v + ω·W/2`, `W` = `track_width` (m, **measure on the robot**).
**Caster angle:** `angle = sgn(v)·atan2(−ω·L, |v|)` (sgn(0)=+1), clamped ±80°, `L` = 0.5 m axle→caster. The `|v|`/`sgn` form keeps the angle correct when reversing.
**No PID on the Jetson** — ESP32 #1 closes the speed loop on its encoder and outputs PWM itself. `/wheel_velocity` feeds odometry only, never `base_controller`.
**Simulation:** hardware topics are not published. Gazebo DiffDrive consumes `/cmd_vel_out` (**never** raw `/cmd_vel`, or manual / fork / dock commands would bypass arbitration) and the caster follows `/castor_cmd_pos`.
**Params:** `track_width`, `wheelbase_L`, `max_angle_deg`, `cmd_timeout`, `sim` (selects sim vs hardware outputs; independent of the `use_sim_time` clock flag).

---

#### Nav2 Stack `[P1]`

**Config:** `config/nav2_params.yaml`

| Direction | Topic/Interface | Type |
|-----------|----------------|------|
| Sub | `/odometry/filtered` | `Odometry` |
| Sub | `/map` | `OccupancyGrid` |
| Sub | `/scan` | `LaserScan` |
| Pub | `/cmd_vel` | `Twist` |
| Pub | `/plan` | `Path` |
| Action Server | `navigate_to_pose` | `NavigateToPose` |

**Global planner:** NavFn (default) or SmacPlanner2D (`planner_server`).
**Local controller (path tracker):** **Regulated Pure Pursuit** (`nav2_regulated_pure_pursuit_controller`) recommended for aisle driving; DWB is the 2026 baseline; MPPI to be benchmarked (D6). Speed: `max_vel_x: 1.0 m/s`, `max_vel_theta: 1.5 rad/s`.
**Boundary with `base_controller`:** the Nav2 controller outputs only body velocity `(v, ω)` on `/cmd_vel` — it knows nothing about wheels or the caster. `base_controller` arbitrates and converts that to wheel speeds + caster angle. If Nav2's velocity smoother / collision monitor are enabled, their final output must be remapped to `/cmd_vel`.

**Pose source:** TF (`map → odom → base_footprint`); `/odometry/filtered` supplies velocity only. Nav2 treats the robot as differential-drive; the caster is `base_controller`'s job.

---

### 4.4 Perception Layer — `wafl_perception`

> **Camera note:** The robot has **one front-facing camera only** (Kinect v1 RGB + Depth).
> There is no rear camera.

#### `pallet_pose_node` — ArUco `[P1]`

**Purpose:** Identify which pallet is visible + coarse 6-DOF pose in camera frame.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/camera/image_raw` | `sensor_msgs/Image` | Kinect v1 RGB |
| Pub | `/pallet_pose` | `PoseStamped` | 6-DOF pose in camera frame |
| Pub | `/fork_target` | `PointStamped` | 0.5m in front of marker |
| Pub | `/pallet_location` | `String` | e.g. "Location_A1" |
| Pub | `/pallet_location_id` | `String` | Marker ID "0","1","2","3" |
| Pub | `/pallet_marker` | `visualization_msgs/Marker` | RViz visualization cube |

**Param:** `locations_yaml` — path to `config/locations.yaml`
**Detector:** OpenCV ArUco `DICT_4X4_100` (pallet IDs 0–3, charging dock ID 99), `solvePnP` IPPE_SQUARE
**Intrinsics (Kinect v1):** `fx=fy=526.607`, `cx=318.525`, `cy=241.181`

---

#### `pallet_front_angle_node` — YOLO `[P1 — CPU sim, TensorRT hardware]`

**Purpose:** Fine-grained pallet alignment angles for fork insertion. 3-stage YOLO pipeline.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/camera/image_raw` | `Image` | Kinect v1 RGB |
| Sub | `/camera/depth/image_raw` | `Image` | Kinect v1 Depth |
| Pub | `/pallet/rotate_to_front_rad` | `Float32` | Rotation to face pallet squarely |
| Pub | `/pallet/center_yaw_rad` | `Float32` | Lateral bearing to pallet center |
| Pub | `/pallet/final_z_m` | `Float32` | Distance to drive for fork insertion |
| Pub | `/pallet/pallet_yaw_offset_rad` | `Float32` | Pallet face angle |
| Pub | `/pallet/debug_image` | `CompressedImage` | Annotated YOLO overlay → GUI |

**3-model pipeline:**
1. `model_pallet` → detect full pallet bounding box
2. `model_pocket` → detect fork pocket inside pallet ROI
3. `model_keypoint` → detect 4 corner keypoints of pocket → back-project with depth → compute angles

**Averaging:** a 3-second circular running mean + time-weighted mean is applied internally; the topics above carry the already-averaged values (no `_avg` / `_weighted_avg` variants are published).
**Sim mode:** Use CPU YOLO or publish synthetic mock data.
**Hardware:** TensorRT `.engine` files in `models/` (gitignored — generate on Jetson).

---

### 4.5 Mission, WMS & Fork Layer — `wafl_mission`, `wafl_fork_insertion`

#### `camera_stream_node` `[P1]`

**Purpose:** Relay front camera frames from ROS to the GUI over TCP (~15 fps JPEG).

| Direction | Interface | Type |
|-----------|-----------|------|
| Sub | `/camera/image_raw` | `Image` |
| Pub | TCP socket (configurable port) | JPEG-compressed frames |

**Implementation:** `cv2.imencode('.jpg', frame)` → raw bytes over TCP → GUI decodes with `cv2.imdecode`.

---

#### `wms_bridge_node` `[P1 STUB → P2 REAL]`

**Purpose:** Bridge WMS server (external MQTT) ↔ ROS 2 topic graph.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/robot/status` | `RobotStatus` | Forward to WMS |
| Sub | `/robot/state` | `String` | Forward to WMS |
| Sub | `/battery/state` | `BatteryState` | Forward to WMS |
| Sub | `/wms/manual_mission` | `String` JSON | From GUI Tab 3 |
| Pub | `/wms/mission` | `WmsMission` | Incoming mission from WMS |
| Pub | `/wms/connection_status` | `Bool` | WMS server reachable |
| Pub | `/wms/mission_status` | `WmsMissionStatus` | Active + queued missions |
| Pub | `/wms/package_list` | `String` JSON | Package DB snapshot |
| Pub | `/wms/location_grid` | `String` JSON | Warehouse grid state |

**External:** Phase 1 — HTTP REST to the FastAPI WMS stub (publishes stub data); Phase 2 — MQTT (Mosquitto).

---

#### `mission_manager_node` `[P1 STUB → P2 FULL]`

**Purpose:** Top-level state machine orchestrating all mission phases.

**States:** `IDLE → NAVIGATE_TO_PICKUP → FORK_INSERTION → NAVIGATE_TO_DROP → DROP → REPORT → CHARGE → RESUME`, plus `MANUAL` and `ESTOPPED` (entered from any state via `/gui/mode` / `/e_stop`).

| Direction | Topic / Interface | Type |
|-----------|------------------|------|
| Sub | `/wms/mission` | `WmsMission` |
| Sub | `/battery/low_alert` | `Bool` |
| Sub | `/battery/state` | `BatteryState` (fills `RobotStatus.battery_percentage`) |
| Sub | `/fork_height` | `Float32` (fills `RobotStatus.fork_height_mm`) |
| Sub | `/fork_insertion/state` | `String` |
| Sub | `/fork_insertion/done` | `Bool` |
| Sub | `/pallet_location_id` | `String` |
| Sub | `/charging_dock/state` | `String` |
| Sub | `/gui/mode` | `String` |
| Sub | `/e_stop` | `Bool` |
| Sub | TF `map → base_footprint` | (fills `RobotStatus.robot_pose`) |
| Pub | `/robot/state` | `String` |
| Pub | `/robot/status` | `RobotStatus` |
| Pub | `/lights_cmd` | `Int32MultiArray` |
| Pub | `/mission_manager/fork_goal` | `WmsMission` |
| Pub | `/charging_dock/start` | `Bool` |
| Action Client | `navigate_to_pose` | `NavigateToPose` |
| Action Client | `insert_forks` | `InsertForks.action` |

**E-stop:** `/e_stop` = true → cancel the active Nav2 goal and `insert_forks` goal, switch to MANUAL, state = `ESTOPPED`. Cleared only by GUI [RESUME AUTO], which publishes `/e_stop = false` and then `/gui/mode = "AUTO"`. `base_controller`, `fork_insertion_node` and `charging_dock_node` also subscribe to `/e_stop` directly, so motion stops even if this node is busy or dead.

**Manual override flow:**
```
GUI → /gui/mode = "MANUAL"
  mission_manager cancels the active Nav2 goal (see D20)
  base_controller switches to /manual_cmd_vel input

GUI → /gui/mode = "AUTO"
  mission_manager resumes saved mission state
```

---

#### `fork_insertion_node` `[P1 STUB → P2 FULL STATE MACHINE]`

**Purpose:** Autonomous fork insertion.

**Full state machine (Phase 2):**
```
IDLE → APPROACH → ROTATE_ALIGN → LATERAL_CENTER → INSERT_FORKS → LIFT → CONFIRM → NAVIGATE_TO_DROP → DROP → IDLE
```

**Phase 1 stub:** Publishes fake state transitions on a timer — tests GUI Logs tab without real logic.

| Direction | Topic / Interface | Type | State Used |
|-----------|------------------|------|------------|
| Sub | `/pallet/rotate_to_front_rad` | `Float32` | ROTATE_ALIGN |
| Sub | `/pallet/center_yaw_rad` | `Float32` | LATERAL_CENTER |
| Sub | `/pallet/final_z_m` | `Float32` | INSERT_FORKS |
| Sub | `/fork_height` | `Float32` | LIFT, CONFIRM |
| Sub | `/pallet_pose` | `PoseStamped` | APPROACH |
| Sub | `/fork_target` | `PointStamped` | APPROACH |
| Sub | `/e_stop` | `Bool` | All states — stop, `/lift_cmd = 0` |
| Sub | `/mission_manager/fork_goal` | `WmsMission` | Start trigger |
| Pub | `/fork_insertion/cmd_vel` | `Twist` | APPROACH → INSERT_FORKS fine drive (consumed by `base_controller`) |
| Pub | `/lift_cmd` | `Int32` | LIFT / DROP |
| Pub | `/fork_insertion/state` | `String` | All states → GUI + mission_manager |
| Pub | `/fork_insertion/done` | `Bool` | Signals completion |
| Action Server | `insert_forks` | `InsertForks.action` | Called by mission_manager |

---

### 4.6 Energy Layer — `wafl_energy`

#### `battery_monitor_node` `[P1 STUB → P2 REAL]`

Phase 1 stub: publishes slowly decaying simulated battery (starts 100%, drains at configurable rate).
Phase 2: reads real BMS via I2C.

| Direction | Topic | Type |
|-----------|-------|------|
| Pub | `/battery/state` | `sensor_msgs/BatteryState` |
| Pub | `/battery/low_alert` | `Bool` |

---

#### `charging_dock_node` `[P2]`

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/charging_dock/start` | `Bool` — from `mission_manager` (CHARGE state) |
| Sub | `/pallet_location_id` | `String` — watch for "99" |
| Sub | `/pallet_pose` | `PoseStamped` |
| Sub | `/battery/state` | `BatteryState` — confirm charging |
| Pub | `/charging_dock/state` | `String` — IDLE \| NAVIGATING \| ALIGNING \| DOCKED \| FAILED |
| Pub | `/charging_dock/cmd_vel` | `Twist` — final approach, via `base_controller` |
| Pub | `/lift_cmd` | `Int32` — forks down before docking |
| Sub | `/e_stop` | `Bool` — stop, `/lift_cmd = 0` |
| Action Client | `navigate_to_pose` | `NavigateToPose` |

---

## 5. GUI Dashboard — `dashboard/`

**Technology:** PySide6 (Python) with optional `QWebEngineView` for rich panels.
**ROS integration:** `rclpy` spins in a background QThread → emits Qt signals → UI updates safely (no GIL deadlocks).

### Tab 1 — Robot Control & Status

```
┌──────────────────────────────────────────────────────────────────┐
│  [ROBOT]  [LOGS]  [WMS]                                          │
├──────────────────┬──────────────────┬────────────────────────────┤
│  WAREHOUSE MAP   │  ROBOT STATUS    │  MISSION                   │
│                  │                  │                            │
│  Live /map       │  State: NAVIGATE │  Active: M-001  A1→B2     │
│  Robot icon      │  Speed: 0.35 m/s │  ─────────────────────    │
│  /plan overlay   │  Fork: 0 cm      │  Queue:                   │
│                  │  Battery: 87%    │   M-002: C3→A1            │
│                  │  [████████  ]    │                            │
│                  │                  │  [Inject Mission]         │
│                  │  [E-STOP]        │                            │
│                  │  [RESUME AUTO]   │                            │
│                  │  [Manual Mode]   │                            │
├──────────────────┴──────────────────┴────────────────────────────┤
│  FRONT CAMERA — TCP stream from camera_stream_node               │
│  [640×480 JPEG @ ~15 fps]                                        │
├──────────────────────────────────────────────────────────────────┤
│  MANUAL JOYSTICK — visible only in Manual Mode                   │
│  [↑][↓][←][→]   Lift: [UP][DOWN]   Lights: [ON][OFF]            │
└──────────────────────────────────────────────────────────────────┘
```

**Subscribes:** `/map`, TF `map → base_footprint` (robot icon), `/odometry/filtered` (actual speed), `/plan`, `/robot/state`, `/fork_height`, `/battery/state`, `/wms/mission_status`, `/pallet/debug_image`
**Publishes:** `/e_stop`, `/manual_cmd_vel`, `/lift_cmd`, `/lights_cmd`, `/gui/mode`, `/wms/manual_mission`
**Receives:** TCP JPEG stream (front camera only)

---

### Tab 2 — Logs & System Health

```
┌──────────────────────────────────────────────────────────────────┐
│  SYSTEM HEALTH              │  EVENT LOG                         │
│  ─────────────────────────  │  ──────────────────────────────   │
│  ✅ slam_node               │  [10:34:01] INFO  Mission started  │
│  ✅ nav2_stack              │  [10:35:12] WARN  Battery < 20%   │
│  ✅ micro_ros_agent         │  [10:35:45] INFO  Pallet detected  │
│  ⚠️ camera_stream_node     │  [10:36:00] ERROR Fork timeout    │
│  ❌ battery_monitor_node   │                                    │
│                             │  [Filter: ALL ▼]  [Clear]         │
│  ESP32 #1 DRIVING:  ✅     │                                    │
│  ESP32 #2 STEERING: ✅     │                                    │
│  ESP32 #3 LIFT:     ✅     │                                    │
│  WMS Server: ✅ CONNECTED   │                                    │
└──────────────────────────────────────────────────────────────────┘
```

**Node health:** GUI queries ROS node graph every 2 seconds.
**ESP32 liveness:** inferred from topic publication rate (silence = dead).
**Event log:** subscribes to `/rosout` — all nodes' `self.get_logger()` calls appear here automatically.

**Subscribes:** `/rosout`, `/wms/connection_status`, `/fork_insertion/state`, `/charging_dock/state`, `/encoder_count` (ESP32 #1 heartbeat), `/imu/data` (ESP32 #2 heartbeat), `/fork_height` (ESP32 #3 heartbeat)

---

### Tab 3 — WMS

```
┌──────────────────────────────────────────────────────────────────┐
│  WAREHOUSE GRID             │  PACKAGE DATABASE                  │
│  ─────────────────────────  │  ID    | Location | Status         │
│  A1: [PKG-042] ████         │  P-042 | A1       | AT_LOCATION    │
│  A2: [empty]                │  P-043 | B2       | IN_TRANSIT     │
│  B1: [PKG-043] ████         │  P-044 | —        | DELIVERED      │
│  B2: [empty]                │                                    │
│                             │  [+ Add Package]  [Edit]           │
│  MISSION QUEUE              │  MISSION HISTORY                   │
│  [+ New Mission]            │  M-001 | A1→B2 | DONE  10:34      │
│  Priority | From | To       │  M-000 | C1→D3 | DONE  09:11      │
│  1        | A1   | B2       │  M-099 | B1→A2 | FAILED  08:55    │
└──────────────────────────────────────────────────────────────────┘
```

**Data ownership rule:**

| Data | Owner | GUI Tab 3 |
|------|-------|-----------|
| Package IDs, locations, status | **WMS Server DB** | Reads via `/wms/package_list` |
| Mission queue | **WMS Server** | Reads via `/wms/mission_status` |
| Dispatch a mission | GUI sends → WMS validates | Publishes `/wms/manual_mission` |
| Robot state / battery / camera | **ROS topics direct** | Not via WMS |

**Subscribes:** `/wms/package_list`, `/wms/mission_status`, `/wms/location_grid`
**Publishes:** `/wms/manual_mission`

---

## 6. Simulation — `wafl_simulation`

### Sensors Provided by Gazebo

| Sensor | Gazebo Plugin | ROS Topic |
|--------|--------------|-----------|
| LiDAR | `GpuLidarSensor` | `/scan` |
| Front RGB | `CameraSensor` | `/camera/image_raw` |
| Front Depth | `DepthCameraSensor` | `/camera/depth/image_raw` |
| IMU | `ImuSensor` | `/imu` (raw → `odom_covariance_fix` → `/imu_fixed`) |
| Wheel odometry | `DiffDrivePlugin` (TF publishing **disabled** — the EKF owns `odom → base_footprint`) | `/odom` (output), `/cmd_vel_out` (input — **not** `/cmd_vel`) |
| Caster joint | Revolute + `ros_gz_bridge` | `/castor_cmd_pos` (input) |
| Fork joint | Prismatic + JointController | `/fork_cmd_vel` (input), `/joint_states` (output) — wrapped by `sim_fork_adapter` |
| Simulation clock | Gazebo | `/clock` (**required** for `use_sim_time:=true`) |

### Warehouse World Contents
- Warehouse floor with shelves and aisles
- Pallet models with ArUco marker textures (IDs 0–3)
- Charging dock model with ArUco ID=99
- RPLiDAR-like geometry for realistic scan behavior

### `ros_gz_bridge.yaml` Key Mappings

```yaml
- ros_topic_name: "/scan"
  gz_topic_name:  "/lidar/scan"
  ros_type_name:  "sensor_msgs/msg/LaserScan"
  gz_type_name:   "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name:  "/camera/image"
  ros_type_name:  "sensor_msgs/msg/Image"
  gz_type_name:   "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/depth/image_raw"
  gz_topic_name:  "/camera/depth"
  ros_type_name:  "sensor_msgs/msg/Image"
  gz_type_name:   "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu"                      # raw sim IMU; hardware publishes /imu/data instead
  gz_topic_name:  "/imu"
  ros_type_name:  "sensor_msgs/msg/Imu"
  gz_type_name:   "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"                    # without this, every use_sim_time node stalls
  gz_topic_name:  "/clock"
  ros_type_name:  "rosgraph_msgs/msg/Clock"
  gz_type_name:   "gz.msgs.Clock"
  direction: GZ_TO_ROS

- ros_topic_name: "/odom"
  gz_topic_name:  "/model/wafl/odometry"
  ros_type_name:  "nav_msgs/msg/Odometry"
  gz_type_name:   "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/cmd_vel_out"              # arbitrated output of base_controller
  gz_topic_name:  "/model/wafl/cmd_vel"
  ros_type_name:  "geometry_msgs/msg/Twist"
  gz_type_name:   "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/castor_cmd_pos"
  gz_topic_name:  "/model/wafl/joint/caster_joint/cmd_pos"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/fork_cmd_vel"
  gz_topic_name:  "/model/wafl/joint/fork_joint/cmd_vel"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/joint_states"
  gz_topic_name:  "/world/warehouse/model/wafl/joint_state"
  ros_type_name:  "sensor_msgs/msg/JointState"
  gz_type_name:   "gz.msgs.Model"
  direction: GZ_TO_ROS
```

### `sim_fork_adapter` `[P1 — SIMULATION ONLY]`

**Purpose:** Hardware has ESP32 #3 (`/lift_cmd` in, `/fork_height` out). Gazebo has a prismatic joint. This node makes the sim look like the ESP32 so `fork_insertion_node` is identical in both modes.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/lift_cmd` | `Int32` | 0=STOP, 1=UP, 2=DOWN → joint velocity |
| Sub | `/joint_states` | `JointState` | Read fork joint position |
| Pub | `/fork_cmd_vel` | `Float64` | → Gazebo fork joint |
| Pub | `/fork_height` | `Float32` (mm) | Same contract as ESP32 #3 |

Top/bottom limit switches are emulated by the joint limits.

### The One Rule for Sim vs Hardware

Every launch file accepts `use_sim_time` (clock only). The two bringup files also set `sim` (`true` in simulation): it selects which nodes and outputs are active (`odom_covariance_fix`, `sim_fork_adapter`, `base_controller` outputs). No code changes needed to switch:

```bash
# Simulation
ros2 launch wafl_bringup simulation.launch.py use_sim_time:=true

# Hardware
ros2 launch wafl_bringup hardware.launch.py use_sim_time:=false
```

---

## 7. Master Topic Index

> **QoS:** `/e_stop`, `/gui/mode`, `/robot/state` use `transient_local` (latched) so a late-joining node (GUI reconnect, restarted `base_controller`) immediately learns the current value. Sensor streams use `sensor_data` (best-effort).

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/encoder_count` | `Float32` | ESP32 #1 | `encoder_velocity_node`, GUI Tab2 (heartbeat) |
| `/wheel_velocity` | `Float32` | `encoder_velocity_node` | `odom_fusion_node` |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 #2 (hardware only; sim uses `/imu`) | `odom_fusion_node`, EKF, GUI Tab2 (heartbeat) |
| `/imu` (sim raw) | `Imu` | `ros_gz_bridge` | `odom_covariance_fix` |
| `/imu_fixed` | `Imu` | `odom_covariance_fix` | EKF (sim only) |
| `/odom` | `Odometry` | `odom_fusion_node` / Gazebo | `odom_covariance_fix`, EKF |
| `/odom_fixed` | `Odometry` | `odom_covariance_fix` | EKF (sim only) |
| `/odometry/filtered` | `Odometry` | EKF | Nav2 (velocity only), GUI Tab1 (actual speed) |
| `/scan` | `LaserScan` | RPLiDAR / Gazebo | SLAM Toolbox, Nav2 |
| `/map` | `OccupancyGrid` | SLAM Toolbox | Nav2, GUI Tab1 |
| `/cmd_vel` | `Twist` | Nav2 | `base_controller` |
| `/manual_cmd_vel` | `Twist` | GUI Tab1 | `base_controller` (manual mode) |
| `/fork_insertion/cmd_vel` | `Twist` | `fork_insertion_node` | `base_controller` |
| `/charging_dock/cmd_vel` | `Twist` | `charging_dock_node` | `base_controller` |
| `/cmd_vel_out` | `Twist` | `base_controller` | Gazebo DiffDrive (sim only) |
| `/target_angle` | `Float32` | `base_controller` | ESP32 #2 |
| `/castor_cmd_pos` | `Float64` | `base_controller` | Gazebo (sim only) |
| `/plan` | `Path` | Nav2 | GUI Tab1 |
| `/target_speed_left` | `Float32` | `base_controller` | ESP32 #1 |
| `/target_speed_right` | `Float32` | `base_controller` | ESP32 #1 |
| `/lift_cmd` | `Int32` | `fork_insertion_node`, `charging_dock_node`, GUI Tab1 | ESP32 #3 / `sim_fork_adapter` |
| `/lights_cmd` | `Int32MultiArray` | `mission_manager_node`, GUI Tab1 | ESP32 #3 |
| `/fork_height` | `Float32` | ESP32 #3 / `sim_fork_adapter` | `fork_insertion_node`, `mission_manager_node`, GUI Tab1, GUI Tab2 (heartbeat) |
| `/fork_cmd_vel` | `Float64` | `sim_fork_adapter` | Gazebo fork joint (sim only) |
| `/joint_states` | `JointState` | Gazebo via bridge (sim) · hardware: open (D21) | `robot_state_publisher`, `sim_fork_adapter` |
| `/camera/image_raw` | `Image` | Kinect / Gazebo | `pallet_pose_node`, `pallet_front_angle_node`, `camera_stream_node` |
| `/camera/depth/image_raw` | `Image` | Kinect / Gazebo | `pallet_front_angle_node` |
| `/pallet_pose` | `PoseStamped` | `pallet_pose_node` | `fork_insertion_node`, `charging_dock_node` |
| `/pallet_location` | `String` | `pallet_pose_node` | (informational logging) |
| `/pallet_location_id` | `String` | `pallet_pose_node` | `mission_manager_node`, `charging_dock_node` |
| `/fork_target` | `PointStamped` | `pallet_pose_node` | `fork_insertion_node` |
| `/pallet_marker` | `visualization_msgs/Marker` | `pallet_pose_node` | RViz |
| `/pallet/rotate_to_front_rad` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/center_yaw_rad` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/final_z_m` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/pallet_yaw_offset_rad` | `Float32` | `pallet_front_angle_node` | (logging) |
| `/pallet/debug_image` | `CompressedImage` | `pallet_front_angle_node` | GUI Tab1 |
| `/wms/mission` | `WmsMission` | `wms_bridge_node` | `mission_manager_node` |
| `/wms/manual_mission` | `String` JSON | GUI Tab3 | `wms_bridge_node` |
| `/wms/mission_status` | `WmsMissionStatus` | `wms_bridge_node` | GUI Tab1, GUI Tab3 |
| `/wms/package_list` | `String` JSON | `wms_bridge_node` | GUI Tab3 |
| `/wms/location_grid` | `String` JSON | `wms_bridge_node` | GUI Tab3 |
| `/wms/connection_status` | `Bool` | `wms_bridge_node` | GUI Tab2 |
| `/robot/state` | `String` | `mission_manager_node` | GUI Tab1, `wms_bridge_node` |
| `/robot/status` | `RobotStatus` | `mission_manager_node` | `wms_bridge_node` |
| `/mission_manager/fork_goal` | `WmsMission` | `mission_manager_node` | `fork_insertion_node` |
| `/fork_insertion/state` | `String` | `fork_insertion_node` | GUI Tab2, `mission_manager_node` |
| `/fork_insertion/done` | `Bool` | `fork_insertion_node` | `mission_manager_node` |
| `/battery/state` | `BatteryState` | `battery_monitor_node` | GUI Tab1, `wms_bridge_node`, `mission_manager_node`, `charging_dock_node` |
| `/battery/low_alert` | `Bool` | `battery_monitor_node` | `mission_manager_node` |
| `/charging_dock/start` | `Bool` | `mission_manager_node` | `charging_dock_node` |
| `/charging_dock/state` | `String` | `charging_dock_node` | `mission_manager_node`, GUI Tab2 |
| `/e_stop` | `Bool` | GUI Tab1 | `mission_manager_node`, `base_controller`, `fork_insertion_node`, `charging_dock_node` |
| `/gui/mode` | `String` | GUI Tab1 | `mission_manager_node`, `base_controller` |
| `/rosout` | `rcl_interfaces/Log` | All nodes | GUI Tab2 |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo via `ros_gz_bridge` (sim only) | All nodes (`use_sim_time:=true`) |
| `/tf` | `tf2_msgs/TFMessage` | EKF (`odom→base_footprint`), SLAM Toolbox (`map→odom`), `robot_state_publisher` (movable-joint frames) | Nav2, SLAM Toolbox, `mission_manager_node` (pose), GUI Tab1 (robot icon) |
| `/tf_static` | `tf2_msgs/TFMessage` | `robot_state_publisher` | Nav2, `pallet_*` nodes, GUI |

---

## 8. Custom Messages & Actions — `wafl_interfaces`

> **Build this package first.** Every other package depends on it.

### `WmsMission.msg`
```
string mission_id
string type              # PICKUP_TRANSPORT | CHARGE | MANUAL
string pickup_location
string dropoff_location
int32  priority
string timestamp
```

### `WmsMissionStatus.msg`
```
WmsMission active_mission
WmsMission[] queued_missions
WmsMission[] completed_missions
```

### `RobotStatus.msg`
```
string mission_id
string state
float32 battery_percentage
float32 fork_height_mm
geometry_msgs/Pose robot_pose
string timestamp
```

### `Alarm.msg`
```
string level       # INFO | WARN | ERROR | FATAL
string source      # node name
string message
string timestamp
```

### `SetMission.srv`
```
WmsMission mission
---
bool accepted
string reason
```

### `InsertForks.action`
```
# Goal
WmsMission mission
---
# Result
bool success
string final_state
---
# Feedback
string current_state
float32 progress_percent
```

---

## 9. Design Decisions Tracker

> Resolve open decisions early. Update this table when a decision is made — notify the whole team.

| # | Decision | Status | Choice | Change if... |
|---|----------|--------|--------|--------------|
| D1 | SLAM algorithm | ✅ DECIDED | SLAM Toolbox async | Poor localization in Gazebo → try Cartographer |
| D2 | Localization mode | ✅ DECIDED | SLAM Toolbox localization mode | SLAM drift → SLAM + AMCL |
| D3 | WMS protocol | ✅ DECIDED | MQTT (Mosquitto) | Broker hard to deploy → FastAPI WebSocket |
| D4 | GUI technology | ✅ DECIDED | PySide6 + QtWebEngine | Moving to browser → React + web_video_server |
| D5 | WMS Phase 1 server | ✅ DECIDED | FastAPI stub | — |
| D6 | Nav2 local planner | ⬜ OPEN | Regulated Pure Pursuit (RPP) vs DWB | RPP tracks straight warehouse aisles and curves smoothly without DWB jitter; benchmark MPPI if dynamic obstacle dodging needed |
| D7 | Charging dock detection | ✅ DECIDED | ArUco ID=99 (reuses pipeline) | Bad lighting → IR beacon |
| D8 | Dual encoders | ⬜ OPEN | Single encoder + IMU yaw | Poor turns → add right-wheel encoder |
| D9 | Fork height sensing | ⬜ OPEN | Potentiometer | — |
| D10 | Load cell | ⬜ OPEN | Skipped | Needed for CONFIRM state reliability |
| D11 | Camera streaming | ✅ DECIDED | MJPEG over TCP | Browser GUI → WebRTC |
| D12 | Fork handoff mechanism | ⬜ OPEN | Three overlapping paths exist today: `/mission_manager/fork_goal` topic, `insert_forks` action, `/fork_insertion/done` topic | **Recommend action only** (goal/result/feedback/cancel); delete the topic pair |
| D13 | Who owns `DROP` | ⬜ OPEN | Listed in both `mission_manager` and `fork_insertion_node` state machines | Recommend `InsertForks.action` goal gets an `operation` field (PICKUP \| DROP); `mission_manager` owns NAVIGATE_TO_DROP only |
| D14 | Unused interfaces | ⬜ OPEN | `Alarm.msg` and `SetMission.srv` have no publisher / server | Wire `/alarms` + a service, or delete them |
| D15 | Safety chain | ⬜ OPEN | Software: `/e_stop` reaches `base_controller`, `fork_insertion_node`, `charging_dock_node`, 0.5 s command timeout. **Hardware e-stop that cuts motor power is required** — the GUI button goes over Wi-Fi | Not optional for a forklift |
| D16 | Caster homing / limits | ⬜ OPEN | Stepper (TB6600) has no position feedback → add home switch or potentiometer. ±80° clamp vs 90° needed for in-place rotation | Constrain Nav2 rotation behaviour or raise clamp |
| D17 | GUI robot pose source | ✅ DECIDED | TF `map → base_footprint` (`/odometry/filtered` is odom-frame and drifts against the map) | — |
| D18 | Averaged pallet topics | ✅ DECIDED | Averaging is internal; only the averaged values are published on the topics in §7 (no `_avg` variants) | — |
| D19 | IMU part | ⬜ OPEN | MPU-6050 is 6-DOF (no magnetometer → yaw drifts); BNO055 gives fused orientation | Prefer BNO055 |
| D20 | Manual mode vs Nav2 lifecycle | ⬜ OPEN | Currently "deactivate Nav2 lifecycle nodes" (slow, costmaps restart) | Recommend: cancel the goal and let `base_controller` arbitrate |
| D21 | Hardware `/joint_states` | ⬜ OPEN | Nothing publishes it on hardware, so `robot_state_publisher` has no caster / fork frames | `base_controller` publishes the caster joint (last `/target_angle`) + a fork-height → joint adapter, or make those joints fixed in the hardware URDF |

### Research Links

| Topic | Links |
|-------|-------|
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox |
| Cartographer ROS | https://google-cartographer-ros.readthedocs.io |
| Nav2 SLAM guide | https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html |
| MPPI planner | https://navigation.ros.org/configuration/packages/configuring-mppic.html |
| DWB planner | https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html |
| MQTT (Mosquitto) | https://mosquitto.org/ |
| Paho-MQTT Python | https://pypi.org/project/paho-mqtt/ |
| FastAPI WebSocket | https://fastapi.tiangolo.com/advanced/websockets/ |

---

## 10. Simulation vs Hardware Reference

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Compute | Developer laptop / Docker | Jetson Xavier AGX |
| OS | Ubuntu 24.04 (host or Docker) | Ubuntu 24.04 (Jetson) |
| LiDAR | `GpuLidarSensor` | Real RPLiDAR A2M8 |
| Camera | `CameraSensor` + `DepthCameraSensor` | Kinect v1 (front only) |
| IMU | `ImuSensor` plugin | MPU-6050 / BNO055 on ESP32 #2 |
| Odometry | Gazebo DiffDrive → `odom_covariance_fix` → EKF | `encoder_velocity_node` + `odom_fusion_node` → EKF |
| Caster | `/castor_cmd_pos` (Gazebo joint) | `/target_angle` → ESP32 #2 |
| Driving | Gazebo DiffDrive plugin ← `/cmd_vel_out` | `/target_speed_left`, `/target_speed_right` → ESP32 #1 (PID on the ESP32) |
| Lift | Gazebo prismatic joint via `sim_fork_adapter` | `/lift_cmd` → ESP32 #3 |
| Fork height | `sim_fork_adapter` from joint state | Potentiometer → ESP32 #3 → `/fork_height` |
| Clock | `/clock` from Gazebo bridge | System clock |
| WMS | FastAPI stub (localhost) | Real WMS server (networked) |
| Charging dock | ArUco ID=99 in world model | Real dock with ArUco marker |
| `use_sim_time` (clock) | `true` | `false` |
| `sim` (mode) | `true` | `false` |
| micro-ROS agent | Not needed | Required on Jetson, start first |

---

## 11. How to Run

### Docker (Recommended)
```bash
# First time
docker compose build

# Start simulation
docker compose up -d

# GUI (separate terminal)
cd dashboard && python3 main.py

# WMS stub
cd wms_server && uvicorn main:app --port 8080
```

### Native — Simulation
```bash
# Terminal 1
ros2 launch wafl_bringup simulation.launch.py use_sim_time:=true

# Terminal 2
cd dashboard && python3 main.py

# Terminal 3
cd wms_server && uvicorn main:app --port 8080
```

### Native — Real Hardware
```bash
# Jetson: Terminal 1 (start BEFORE powering ESP32s)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Jetson: Terminal 2
ros2 launch wafl_bringup hardware.launch.py use_sim_time:=false

# Operator laptop: GUI
cd dashboard && python3 main.py
```

### Debug Commands
```bash
ros2 node list                          # all running nodes
ros2 topic list -t                      # all topics + types
ros2 topic echo /robot/state            # monitor state
ros2 topic hz /odometry/filtered        # should be ~50 Hz
ros2 run tf2_tools view_frames          # check TF tree
```

---

## 12. Team Rules

1. **Never commit to `main`** — use feature branches: `feat/slam`, `feat/gui-tab1`, `feat/aruco-node`
2. **Every package needs a `README.md`** — what it does, what topics it uses, how to run it
3. **Update the decisions table (§9)** when you resolve an open decision — notify the team
4. **No hardcoded IPs, passwords, or paths** — use `.env` files or ROS parameters
5. **All launch files must support `use_sim_time` parameter** — no exceptions
6. **Test in simulation before touching real hardware**
7. **Run `colcon build` before pushing** — broken builds block the whole team
8. **`wafl_interfaces` is built first, always** — if you change a `.msg` file, tell everyone immediately
9. **TensorRT `.engine` files go in `wafl_perception/models/`** — gitignored, never commit
10. **Log with `self.get_logger()`, not `print()`** — logs appear in GUI Tab 2 automatically

---

*WAFL 2027 — Ain Shams University, Mechatronics & Robotics Engineering*
*Maintained by team lead. Last updated: see git log.*
