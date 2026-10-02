# WAFL 2027 — Team README

> **Project:** Warehouse Autonomous Fork-Lift — Ain Shams University, Mechatronics & Robotics Engineering 2026/2027
> **Stack:** ROS 2 Jazzy · Gazebo Harmonic · Jetson Xavier AGX · micro-ROS · Python 3.12 · YOLO / TensorRT
> **Status:** Phase 1 in progress — Simulation + GUI + Manual Control

---

## Read This First

This file is the **single source of truth** for every team member.

Before writing a single line of code:
1. Read this whole file once — it takes 15 minutes
2. Find your assigned node(s) in 4
3. Look up every topic your node uses in the Master Topic Index (7)
4. Follow the Team Rules (9) — broken builds block everyone

---

## Table of Contents

1. [Big Picture](#1-big-picture)
2. [Build Order — Phase 1 then Phase 2](#2-build-order)
3. [Repository Structure](#3-repository-structure)
4. [Node Reference — Every Node Defined](#4-node-reference)
5. [GUI Dashboard](#5-gui-dashboard)
6. [Simulation — Gazebo Harmonic](#6-simulation)
7. [Master Topic Index](#7-master-topic-index)
8. [Custom Messages & Actions](#8-custom-messages--actions)
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
║  wafl_navigation   →  Nav2 + unified caster_controller       ║
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
- [ ] Robot drives in Gazebo and responds to `/cmd_vel`
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
├── LAUNCH.md                        ← Start here: how to run everything
├── WAFL2027_Architecture.md         ← Master architecture document
├── WAFL2027_Team_README.md          ← THIS FILE
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
│   │       └── view_urdf.launch.py
│   │
│   ├── wafl_simulation/             ← Gazebo Harmonic world + spawn + bridge
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
│   ├── wafl_navigation/             ← Nav2 + caster controller
│   │   ├── wafl_navigation/
│   │   │   └── caster_controller.py     ← unified: sim + hardware via param
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
│   └── wafl_energy/                 ← Battery + charging dock (Phase 2)
│       └── wafl_energy/
│           ├── battery_monitor_node.py  ← STUB in Phase 1
│           └── charging_dock_node.py    ← Phase 2 only
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
├── bringup/                         ← Master launch files
│   ├── simulation.launch.py         ← Full sim: Gazebo + SLAM + Nav2 + Perc
│   ├── hardware.launch.py           ← Full hardware bringup
│   └── mapping.launch.py            ← SLAM mapping session only
│
└── docs/
    └── diagrams/
        └── WAFL2027_Architecture_Graph.md  ← Mermaid diagram (editable)
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
| Sub | `/pwm_left` | `Float32` | ±255 PWM to left drive motor |
| Sub | `/pwm_right` | `Float32` | ±255 PWM to right drive motor |
| Pub | `/encoder_count` | `Float32` | Raw encoder ticks (left wheel) |

Hardware: 2× DC motors + H-bridge, 1× encoder (600 PPR, gear ratio ≈ 3.14)

#### ESP32 #2 — Steering + IMU `[FIRMWARE — UPGRADED]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/target_angle` | `Float32` | Desired caster angle in degrees |
| Pub | `/imu/data` | `sensor_msgs/Imu` | Full 9-DOF + covariances |

Hardware: Stepper motor (TB6600) + MPU-6050 or BNO055 IMU
**2027 change:** Replaces fragile phone-IMU UDP protocol entirely. Uses proper `sensor_msgs/Imu` with real covariance matrices.

#### ESP32 #3 — Lift + Lights `[FIRMWARE]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/lift_cmd` | `Int32` | 0=STOP, 1=UP, 2=DOWN |
| Sub | `/lights_cmd` | `Int32MultiArray` | 4-element array, 0/1 per relay |
| Pub | `/fork_height` | `Float32` | Fork height in mm |

Hardware: 1× lift DC motor, 2× limit switches (top/bottom), 4× relay-controlled lights, potentiometer or encoder for fork height.

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

Only launched when `use_sim_time:=true`.

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
| Sub | `/odometry/filtered` | `Odometry` |
| Pub | `/map` | `nav_msgs/OccupancyGrid` |
| Pub | TF `map → odom` | — |

**Config:** `config/slam_toolbox.yaml`
**Mode:** Async mapping. Map can be serialized and reloaded for localization-only mode.

---

### 4.3 Navigation Layer — `wafl_navigation`

#### `caster_controller` `[P1 — KEY FIX FROM 2026]`

**Purpose:** Convert `cmd_vel` → front caster steering angle. Unified node for sim and hardware.

| Direction | Topic | Type | Condition |
|-----------|-------|------|-----------|
| Sub | `/cmd_vel` | `Twist` | Auto mode |
| Sub | `/manual_cmd_vel` | `Twist` | Manual override from GUI |
| Pub | `/target_angle` | `Float32` (degrees) | `use_sim_time=false` (hardware) |
| Pub | `/castor_cmd_pos` | `Float64` (radians) | `use_sim_time=true` (simulation) |

**Param:** `use_sim_time` — set at launch, no code change needed.
**Kinematics:** `angle = atan2(-ω × L, v)`, clamped ±80°, L = 0.5 m wheelbase.

> ⚠️ **2026 bug this fixes:** In 2026, the caster controller always published to `/castor_cmd_pos` only. On real hardware this had no effect. The 2027 unified node uses `use_sim_time` to select the correct output topic automatically.

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

Local planner: **DWB** (default, proven from 2026). MPPI to be benchmarked.
Speed: `max_vel_x: 1.0 m/s`, `max_vel_theta: 1.5 rad/s`

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
**Detector:** OpenCV ArUco `DICT_4X4_50`, `solvePnP` IPPE_SQUARE
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

**Averaging:** 3-second circular running mean + time-weighted mean (both published with `_avg` and `_weighted_avg` suffixes)
**Sim mode:** Use CPU YOLO or publish synthetic mock data.
**Hardware:** TensorRT `.engine` files in `models/` (gitignored — generate on Jetson).

---

### 4.5 Mission & WMS Layer — `wafl_mission`

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

**External:** MQTT (Mosquitto). Phase 1 stub reads FastAPI WMS and publishes fake data.

---

#### `mission_manager_node` `[P1 STUB → P2 FULL]`

**Purpose:** Top-level state machine orchestrating all mission phases.

**States:** `IDLE → NAVIGATE_TO_PICKUP → FORK_INSERTION → NAVIGATE_TO_DROP → DROP → REPORT → CHARGE → RESUME`

| Direction | Topic / Interface | Type |
|-----------|------------------|------|
| Sub | `/wms/mission` | `WmsMission` |
| Sub | `/battery/low_alert` | `Bool` |
| Sub | `/fork_insertion/done` | `Bool` |
| Sub | `/gui/mode` | `String` |
| Sub | `/e_stop` | `Bool` |
| Pub | `/robot/state` | `String` |
| Pub | `/robot/status` | `RobotStatus` |
| Pub | `/lights_cmd` | `Int32MultiArray` |
| Pub | `/mission_manager/fork_goal` | `WmsMission` |
| Action Client | `navigate_to_pose` | `NavigateToPose` |
| Action Client | `insert_forks` | `InsertForks.action` |

**Manual override flow:**
```
GUI → /gui/mode = "MANUAL"
  mission_manager deactivates Nav2 lifecycle nodes
  caster_controller switches to /manual_cmd_vel input

GUI → /gui/mode = "AUTO"
  mission_manager re-activates Nav2 lifecycle nodes
  resumes saved mission state
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
| Sub | `/mission_manager/fork_goal` | `WmsMission` | Start trigger |
| Pub | `/pwm_left` | `Float32` | INSERT_FORKS fine drive |
| Pub | `/pwm_right` | `Float32` | INSERT_FORKS fine drive |
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
| Sub | `/pallet_location_id` | `String` — watch for "99" |
| Sub | `/pallet_pose` | `PoseStamped` |
| Pub | `/charging_dock/state` | `String` |
| Pub | `/lift_cmd` | `Int32` — forks down before docking |
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

**Subscribes:** `/map`, `/odometry/filtered`, `/plan`, `/robot/state`, `/cmd_vel`, `/fork_height`, `/battery/state`, `/wms/mission_status`
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

**Subscribes:** `/rosout`, `/wms/connection_status`, `/encoder_count` (ESP32 #1 heartbeat), `/imu/data` (ESP32 #2 heartbeat), `/fork_height` (ESP32 #3 heartbeat)

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
| IMU | `ImuSensor` | `/imu/data` |
| Wheel odometry | `DiffDrivePlugin` | `/odom` |
| Caster joint | Prismatic + `ros_gz_bridge` | `/castor_cmd_pos` (input) |

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

- ros_topic_name: "/imu/data"
  gz_topic_name:  "/imu"
  ros_type_name:  "sensor_msgs/msg/Imu"
  gz_type_name:   "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/castor_cmd_pos"
  gz_topic_name:  "/model/wafl/joint/caster_joint/cmd_pos"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ
```

### The One Rule for Sim vs Hardware

Every launch file accepts `use_sim_time` parameter — no code changes needed to switch:

```bash
# Simulation
ros2 launch bringup simulation.launch.py use_sim_time:=true

# Hardware
ros2 launch bringup hardware.launch.py use_sim_time:=false
```

---

## 7. Master Topic Index

| Topic | Type | Publisher | Subscriber(s) |
|-------|------|-----------|---------------|
| `/encoder_count` | `Float32` | ESP32 #1 | `encoder_velocity_node`, GUI Tab2 (heartbeat) |
| `/wheel_velocity` | `Float32` | `encoder_velocity_node` | `odom_fusion_node` |
| `/imu/data` | `sensor_msgs/Imu` | ESP32 #2 / Gazebo | `odom_fusion_node`, EKF, GUI Tab2 (heartbeat) |
| `/imu` (sim raw) | `Imu` | `ros_gz_bridge` | `odom_covariance_fix` |
| `/imu_fixed` | `Imu` | `odom_covariance_fix` | EKF (sim only) |
| `/odom` | `Odometry` | `odom_fusion_node` / Gazebo | `odom_covariance_fix`, EKF |
| `/odom_fixed` | `Odometry` | `odom_covariance_fix` | EKF (sim only) |
| `/odometry/filtered` | `Odometry` | EKF | Nav2, SLAM Toolbox, GUI Tab1 |
| `/scan` | `LaserScan` | RPLiDAR / Gazebo | SLAM Toolbox, Nav2 |
| `/map` | `OccupancyGrid` | SLAM Toolbox | Nav2, GUI Tab1 |
| `/cmd_vel` | `Twist` | Nav2 | `caster_controller`, GUI Tab1 (speed readout) |
| `/manual_cmd_vel` | `Twist` | GUI Tab1 | `caster_controller` (manual mode) |
| `/target_angle` | `Float32` | `caster_controller` | ESP32 #2 |
| `/castor_cmd_pos` | `Float64` | `caster_controller` | Gazebo (sim only) |
| `/plan` | `Path` | Nav2 | GUI Tab1 |
| `/pwm_left` | `Float32` | `fork_insertion_node` | ESP32 #1 |
| `/pwm_right` | `Float32` | `fork_insertion_node` | ESP32 #1 |
| `/lift_cmd` | `Int32` | `fork_insertion_node`, GUI Tab1 | ESP32 #3 |
| `/lights_cmd` | `Int32MultiArray` | `mission_manager_node`, GUI Tab1 | ESP32 #3 |
| `/fork_height` | `Float32` | ESP32 #3 | `fork_insertion_node`, GUI Tab1, GUI Tab2 (heartbeat) |
| `/camera/image_raw` | `Image` | Kinect / Gazebo | `pallet_pose_node`, `pallet_front_angle_node`, `camera_stream_node` |
| `/camera/depth/image_raw` | `Image` | Kinect / Gazebo | `pallet_front_angle_node` |
| `/pallet_pose` | `PoseStamped` | `pallet_pose_node` | `fork_insertion_node` |
| `/pallet_location` | `String` | `pallet_pose_node` | (informational logging) |
| `/pallet_location_id` | `String` | `pallet_pose_node` | `mission_manager_node` |
| `/fork_target` | `PointStamped` | `pallet_pose_node` | `fork_insertion_node` |
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
| `/battery/state` | `BatteryState` | `battery_monitor_node` | GUI Tab1, `wms_bridge_node` |
| `/battery/low_alert` | `Bool` | `battery_monitor_node` | `mission_manager_node` |
| `/charging_dock/state` | `String` | `charging_dock_node` | GUI Tab2 |
| `/e_stop` | `Bool` | GUI Tab1 | `mission_manager_node` |
| `/gui/mode` | `String` | GUI Tab1 | `mission_manager_node` |
| `/rosout` | `rcl_interfaces/Log` | All nodes | GUI Tab2 |

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
| D6 | Nav2 local planner | ⬜ OPEN | DWB (default) | Jerky trajectory → benchmark MPPI |
| D7 | Charging dock detection | ✅ DECIDED | ArUco ID=99 (reuses pipeline) | Bad lighting → IR beacon |
| D8 | Dual encoders | ⬜ OPEN | Single encoder + IMU yaw | Poor turns → add right-wheel encoder |
| D9 | Fork height sensing | ⬜ OPEN | Potentiometer | — |
| D10 | Load cell | ⬜ OPEN | Skipped | Needed for CONFIRM state reliability |
| D11 | Camera streaming | ✅ DECIDED | MJPEG over TCP | Browser GUI → WebRTC |

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
| Driving | Gazebo DiffDrive plugin | `/pwm_left`, `/pwm_right` → ESP32 #1 |
| Lift | Gazebo prismatic joint | `/lift_cmd` → ESP32 #3 |
| Fork height | Simulated joint state | Potentiometer → ESP32 #3 → `/fork_height` |
| WMS | FastAPI stub (localhost) | Real WMS server (networked) |
| Charging dock | ArUco ID=99 in world model | Real dock with ArUco marker |
| `use_sim_time` | `true` | `false` |
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
ros2 launch bringup simulation.launch.py use_sim_time:=true

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
ros2 launch bringup hardware.launch.py use_sim_time:=false

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
