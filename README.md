# WAFL 2027 — Team README

> **Project:** Warehouse Autonomous Fork-Lift — Ain Shams University, Mechatronics & Robotics Engineering 2026/2027
> **Stack:** ROS 2 Jazzy · Gazebo Harmonic · Jetson Xavier AGX · micro-ROS · Python 3.12 · YOLO / TensorRT · FastAPI + HTML/JS (web GUI)
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
5. [GUI Dashboard — Web (FastAPI + HTML/JS)](#5-gui-dashboard--wafl_dashboard)
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
- Is operated and monitored from **any web browser** — a 3-tab dashboard (Robot · Logs · WMS) written in HTML/JS and served by a **FastAPI** back-end running on the robot

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
║  wafl_perception   →  ArUco + YOLO + camera_stream_node      ║
║  wafl_mission      →  STUBS (mission manager, WMS bridge)    ║
║  wafl_fork_insertion / wafl_energy  →  STUB nodes (fake data)║
║  wafl_dashboard    →  FastAPI + HTML/JS web GUI (3 tabs)     ║
║  wafl_bringup      →  simulation / hardware launch files     ║
║  wms_server/       →  FastAPI WMS stub (outside src/)        ║
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
- [ ] Robot drives and steers in Gazebo when `base_controller` publishes the per-joint sim commands (§4.3, §6) — Gazebo never reads raw `/cmd_vel`
- [ ] SLAM Toolbox builds a map; map visible in RViz
- [ ] Nav2 sends robot to a goal pose autonomously
- [ ] ArUco node detects simulated pallet markers
- [ ] Dashboard opens at `http://localhost:8000`; a page reload restores the full state (§5.3)
- [ ] GUI Tab 1 shows live map, robot pose, MJPEG camera feed
- [ ] GUI Tab 2 shows node health + live `/rosout` log stream
- [ ] GUI Tab 3 shows stub packages + missions from the FastAPI WMS stub (via the `wms_bridge_node` stub)
- [ ] E-STOP button halts robot immediately; RESET E-STOP → RESUME AUTO recovers it
- [ ] Manual joystick mode works: browser → `dashboard_server` → `/manual_cmd_vel` → robot moves; releasing the key stops it
- [ ] Dashboard drive modes work in sim: NORMAL, CRAB (sideways) and JOINT jog (front steering, rear steering, each front wheel, lift)

---

## 3. Repository Structure

```
WAFL_2027/
│
├── README.md                        ← THIS FILE — single source of truth
│
├── docker/
│   ├── Dockerfile                   ← Ubuntu 24.04 + ROS 2 Jazzy + all deps
│   ├── docker-compose.yml           ← One command: docker compose -f docker/docker-compose.yml up
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
│   ├── wafl_perception/             ← ArUco + YOLO + camera stream + debug image
│   │   ├── wafl_perception/
│   │   │   ├── pallet_pose_node.py
│   │   │   ├── pallet_front_angle_node.py
│   │   │   └── camera_stream_node.py    ← Phase 1 priority
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
│   ├── wafl_mission/                ← Mission manager + WMS bridge
│   │   ├── wafl_mission/
│   │   │   ├── mission_manager_node.py  ← STUB in Phase 1
│   │   │   └── wms_bridge_node.py       ← STUB in Phase 1
│   │   └── config/
│   │       └── wms_config.yaml
│   │
│   ├── wafl_energy/                 ← Battery + charging dock (Phase 2)
│   │   └── wafl_energy/
│   │       ├── battery_monitor_node.py  ← STUB in Phase 1
│   │       └── charging_dock_node.py    ← Phase 2 only
│   │
│   ├── wafl_dashboard/              ← Web GUI: FastAPI back-end + HTML/JS front-end (Phase 1 priority)
│   │   ├── wafl_dashboard/
│   │   │   ├── server.py                ← FastAPI app (REST + WebSocket + MJPEG); `dashboard_server` entry point
│   │   │   ├── ros_node.py              ← rclpy node, spun in a background thread
│   │   │   └── map_render.py            ← OccupancyGrid → PNG
│   │   ├── static/                      ← installed to share/ via setup.py data_files
│   │   │   ├── index.html               ← single page, 3 tabs: ROBOT · LOGS · WMS
│   │   │   ├── css/style.css
│   │   │   └── js/
│   │   │       ├── app.js               ← tab switching, WebSocket client + auto-reconnect
│   │   │       ├── robot_tab.js         ← Tab 1: map canvas, camera, controls, joystick
│   │   │       ├── logs_tab.js          ← Tab 2: node health + log stream
│   │   │       └── wms_tab.js           ← Tab 3: package grid + missions
│   │   ├── launch/
│   │   │   └── dashboard.launch.py
│   │   └── requirements.txt             ← fastapi, uvicorn[standard]
│   │
│   └── wafl_bringup/                ← ament package (ros2 launch needs a package)
│       └── launch/
│           ├── simulation.launch.py ← Full sim: Gazebo + SLAM + Nav2 + Perception + Dashboard
│           ├── hardware.launch.py   ← Full hardware bringup (includes Dashboard)
│           └── mapping.launch.py    ← SLAM mapping session only
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

**Fixed-rate publishing (heartbeat contract):** each ESP32 publishes its feedback topic at a fixed rate **even when the value has not changed** — `/encoder_count` and `/imu/data` at ~50 Hz, `/fork_height` at ≥ 10 Hz. The dashboard uses these topics as heartbeats (silent > 1 s = dead, §5 Tab 2), so publish-on-change firmware would show a stationary robot as dead.

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

#### ESP32 #1 — Driving `[FIRMWARE]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/target_speed_left` | `Float32` | Target speed (m/s) for the **front-left** drive wheel; ESP32 runs the PID against encoder feedback |
| Sub | `/target_speed_right` | `Float32` | Target speed (m/s) for the **front-right** drive wheel; ESP32 runs the PID against encoder feedback |
| Pub | `/encoder_count` | `Float32` | Raw encoder ticks (front-left wheel) |

Hardware: 2× DC motors + H-bridge (one per **front** drive wheel, beside the forks; the rear wheel is not driven), 1× encoder (600 PPR, gear ratio ≈ 3.14)
**Control:** the ESP32 closes the speed loop (PID) itself and owns the tick→speed conversion; its wheel radius / gear ratio must match the `encoder_velocity_node` params.
> ⚠️ With a single encoder (front-left wheel) only the left loop has feedback; the right wheel runs feed-forward (open loop) until D8 (right-wheel encoder) is resolved.

#### ESP32 #2 — Steering + IMU `[FIRMWARE — UPGRADED]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/target_front_angle` | `Float32` | Desired angle `α` in degrees of **both front wheels** (one steering motor, chain/sprocket) — **new in 2027, board allocation assumed, see D23** |
| Sub | `/target_angle` | `Float32` | Desired angle `β` in degrees of the **rear** (steered, undriven) wheel |
| Pub | `/imu/data` | `sensor_msgs/Imu` | Orientation + angular velocity + acceleration, with covariances (BNO055: fused 9-DOF · MPU-6050: 6-DOF, yaw drifts) |

Hardware: 2× stepper motor (TB6600) — one steers the rear wheel, one steers both front wheels through a chain/sprocket — + MPU-6050 or BNO055 IMU
**2027 change:** Replaces fragile phone-IMU UDP protocol entirely. Uses proper `sensor_msgs/Imu` with real covariance matrices.
> ⚠️ A stepper has no position feedback: it needs a home switch (or potentiometer) at boot so `/target_angle` and `/target_front_angle` are absolute (D16). MPU-6050 has no magnetometer, so yaw drifts — BNO055 recommended (D19).

#### ESP32 #3 — Lift + Lights `[FIRMWARE]`

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/lift_cmd` | `Int32` | 0=STOP, 1=UP, 2=DOWN |
| Sub | `/lights_cmd` | `Int32MultiArray` | 4-element array, 0/1 per relay |
| Pub | `/fork_height` | `Float32` | Fork height in mm |

Hardware: 1× lift DC motor, 2× limit switches (top/bottom), 4× relay-controlled lights, potentiometer or encoder for fork height.
**Single-writer rule (lift / lights):** several nodes publish these topics, so ownership is by mode. GUI Tab 1 publishes `/lift_cmd` / `/lights_cmd` **only in MANUAL** (enforced by `dashboard_server`, §5.3); `fork_insertion_node`, `charging_dock_node` (lift) and `mission_manager_node` (lights) publish **only in AUTO**. On `/e_stop` the fork and dock nodes publish `/lift_cmd = 0` (STOP).

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

**Purpose:** Gazebo's `OdometryPublisher` outputs zero covariance matrices — EKF cannot invert them. Relay node that injects valid diagonal covariances.

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/odom` | `Odometry` |
| Sub | `/imu` | `Imu` |
| Pub | `/odom_fixed` | `Odometry` |
| Pub | `/imu_fixed` | `Imu` |

Only launched in simulation (`sim:=true`). In simulation `odom_fusion_node` is **not** launched — Gazebo's `OdometryPublisher` (ground-truth pose, no wheel slip) is the `/odom` source.

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
| Sub | `/joint_states` | `JointState` | Steering / drive-wheel / fork joints (sim: from the bridge · hardware: open, see D21) |
| Pub | `/tf_static`, `/tf` | `TFMessage` | `base_footprint → base_link → laser_link / camera_link / imu_link / fork_link` |

**Full TF tree — exactly one publisher per edge:**
```
map ──(SLAM Toolbox)──► odom ──(EKF)──► base_footprint ──(robot_state_publisher)──► base_link ──► sensors
```
Gazebo's odometry-plugin TF publishing **must be disabled**, otherwise two nodes publish `odom → base_footprint`.
**Clock:** every node runs with `use_sim_time` and, in sim, `/clock` must be bridged from Gazebo (§6).

---

### 4.3 Navigation Layer — `wafl_navigation`

#### `base_controller` `[P1]`

**Purpose:** Convert a body velocity command into **front-left / front-right drive-wheel speeds**, a **front steering angle `α`** (both front wheels together) and a **rear steering angle `β`**. Single owner of ESP32 #1 (driving) and ESP32 #2 (steering), and the single **arbiter** of who may command motion. Unified node for sim and hardware.

**Robot geometry (confirmed by the team lead):** three-wheeled forklift, **forks at the front**.
- **2 front drive wheels**, one on each side of the forks, each with its own DC motor (forward / reverse). They are the only driven wheels.
- **1 front steering motor** turns **both** front wheels together through a chain/sprocket, so they are always **parallel at the same angle `α`** (`α = 0` = straight ahead, positive = wheels point left).
- **1 rear wheel** on the centreline at the back, **steered by its own motor** (angle `β`) and **not driven** — it rolls freely.
- **1 lift motor** (ESP32 #3) raises the forks.

Commands `(v, ω)` are expressed at the **midpoint of the front drive axle**, which must be the origin of `base_footprint` (the URDF must place it there). `W` = `track_width` between the front wheels; `L` = `wheelbase_L` from the front-axle midpoint to the rear wheel (positive; the rear wheel is **behind** the axle).

| Direction | Topic | Type | Condition |
|-----------|-------|------|-----------|
| Sub | `/manual_cmd_vel` | `Twist` | MANUAL mode — highest priority (`linear.y` is used only in CRAB mode) |
| Sub | `/fork_insertion/cmd_vel` | `Twist` | Fine approach (APPROACH → INSERT_FORKS) |
| Sub | `/charging_dock/cmd_vel` | `Twist` | Final dock approach |
| Sub | `/cmd_vel` | `Twist` | AUTO mode (Nav2) — lowest priority |
| Sub | `/gui/mode` | `String` | MANUAL / AUTO selector |
| Sub | `/e_stop` | `Bool` | Latched stop |
| Sub | `/manual_joint_cmd` | `Float32MultiArray` | JOINT mode only (MANUAL): `[alpha_deg, beta_deg, v_left_mps, v_right_mps]` |
| Sub | `/drive_mode` | `String` | `NORMAL` (default), `CRAB` or `JOINT`; CRAB / JOINT are honoured only in MANUAL (see below) |
| Pub | `/target_front_angle` | `Float32` (degrees) | `sim:=false` (hardware) — `α`, both front wheels |
| Pub | `/target_angle` | `Float32` (degrees) | `sim:=false` (hardware) — `β`, **rear** wheel |
| Pub | `/target_speed_left` | `Float32` (m/s) | `sim:=false` (hardware) — front-left wheel |
| Pub | `/target_speed_right` | `Float32` (m/s) | `sim:=false` (hardware) — front-right wheel |
| Pub | `/front_steer_cmd_pos` | `Float64` (radians) | `sim:=true` — `α`, both front steering joints |
| Pub | `/castor_cmd_pos` | `Float64` (radians) | `sim:=true` — `β`, **rear** steering joint (the topic keeps the 2026 spelling "castor") |
| Pub | `/front_left_wheel_cmd_vel` | `Float64` (rad/s) | `sim:=true` — front-left wheel angular velocity |
| Pub | `/front_right_wheel_cmd_vel` | `Float64` (rad/s) | `sim:=true` — front-right wheel angular velocity |

**Arbitration (evaluated every cycle):** `/e_stop` → output zero · MANUAL → only `/manual_cmd_vel` (or `/manual_joint_cmd` in JOINT mode) · AUTO → `/fork_insertion/cmd_vel` or `/charging_dock/cmd_vel` if received in the last 0.2 s, otherwise `/cmd_vel`.
**Startup default:** until the first `/gui/mode` sample arrives, mode = MANUAL — the robot never starts moving autonomously on its own.
**Command timeout:** if the selected source is silent for `cmd_timeout` (0.5 s) output zero. The ESP32s apply the same timeout to `/target_speed_*` / `/target_angle` / `/target_front_angle`.
**Why several modes:** two parallel front wheels steered together cannot rotate the body when `α ≠ 0` (their axles never intersect), so rotation exists only with `α = 0`, and translation in an arbitrary direction exists only with all three wheels parallel. `JOINT` is a commissioning / test mode that moves each actuator directly.

| Mode | `α` (front) | `β` (rear) | Uses | Who may use it |
|------|-------------|------------|------|----------------|
| `NORMAL` | held at 0 | follows the turn (formula below) | `v`, `ω` (`linear.y` ignored) — differential drive of the front pair | everyone; **the only mode Nav2 and the autonomy nodes use** |
| `CRAB` | `atan(vy/vx)`, range ±90° | `β = α` | `vx`, `vy`, **`ω` forced to 0** — sideways / diagonal translation | MANUAL only (dashboard) |
| `JOINT` | commanded directly | commanded directly | `/manual_joint_cmd` — `α`, `β`, `v_left`, `v_right` set independently | MANUAL only (dashboard jog panel) |

**JOINT:** `base_controller` clamps `α` to ±`max_front_angle_deg`, `β` to ±`max_rear_angle_deg` and the speeds to `max_wheel_speed`, then outputs them unchanged (no kinematic coupling — the operator is responsible for a sensible combination). On `cmd_timeout` the **speeds go to zero and the angles hold** (a stepper cannot "go to zero"). Entering or leaving JOINT: stop first, then recentre `α = β = 0` before NORMAL.
**NORMAL wheel speeds:** `v_left = v − ω·W/2`, `v_right = v + ω·W/2`, `W` = `track_width` (m, from the CAD, verify on the robot).
**CRAB:** `s = hypot(vx, vy)` with the sign of `vx` (of `vy` if `vx = 0`); `α = β = atan2(vy, vx)` folded into ±90° (add / subtract 180° and negate `s`); both front wheels run at `s`. Output zero speed for `steer_settle_time` after `α` or `β` changes by more than `steer_deadband_deg` (steppers have no feedback, D16). To leave CRAB: stop, steer `α = β = 0`, wait the settle time, then switch to NORMAL. An `/e_stop` or a switch to AUTO forces NORMAL (stop, then recentre as above). `/drive_mode = CRAB` received outside MANUAL is ignored.
**NORMAL rear angle:** `β = rear_direction · atan2(−ω·L·sgn(v), |v|)` (sgn(0)=+1), clamped ±`max_rear_angle_deg`, `L` = `wheelbase_L` (sim assumption 0.5 m — **measure on the robot**). The rear wheel sits behind the axle, so for a left turn (`ω > 0`, forward) it swings **right** (`β < 0`). The `|v|`/`sgn` form keeps `β` correct when reversing. In-place rotation (`v = 0`) asks for `|β| = 90°` — see D16 for the clamp.
**Sign convention (REP-103):** `ω > 0` = counter-clockwise (left turn); for both `α` and `β`, a positive angle = the wheel points left (rotation about +z). The Gazebo steering joints (axis +z) use this convention, so `front_direction = rear_direction = +1` in sim. On hardware, verify that a positive `/target_front_angle` and a positive `/target_angle` turn their wheels left; if a stepper is wired the other way set that direction param to −1 — fix it in the param, never in firmware.
**No PID on the Jetson** — ESP32 #1 closes the speed loop on its encoder and outputs PWM itself. `/wheel_velocity` feeds odometry only, never `base_controller`.
**Simulation:** hardware topics are not published. There is **no DiffDrive plugin** (it cannot model steered wheels). Instead Gazebo consumes the per-joint commands `/front_steer_cmd_pos`, `/castor_cmd_pos`, `/front_left_wheel_cmd_vel`, `/front_right_wheel_cmd_vel` (wheel rad/s = speed / `wheel_radius`), published only by `base_controller` after arbitration — raw `/cmd_vel` is **never** read, or manual / fork / dock commands would bypass arbitration. Odometry comes from Gazebo's `OdometryPublisher` (§6).
**Params:** `track_width`, `wheelbase_L`, `wheel_radius` (sim), `max_front_angle_deg` (90), `max_rear_angle_deg`, `max_wheel_speed`, `front_direction`, `rear_direction` (+1 / −1), `steer_settle_time`, `steer_deadband_deg`, `cmd_timeout`, `sim` (selects sim vs hardware outputs; independent of the `use_sim_time` clock flag).

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
**Boundary with `base_controller`:** the Nav2 controller outputs only body velocity `(v, ω)` on `/cmd_vel` — it knows nothing about wheels or steering. `base_controller` arbitrates and converts that to wheel speeds + the front and rear steering angles. If Nav2's velocity smoother / collision monitor are enabled, their final output must be remapped to `/cmd_vel`.

**Pose source:** TF (`map → odom → base_footprint`); `/odometry/filtered` supplies velocity only. Nav2 treats the robot as differential-drive (valid in NORMAL mode); both steering angles are `base_controller`'s job. Nav2 never uses CRAB.

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

#### `camera_stream_node` `[P1]`

**Purpose:** Produce a lightweight, rate-limited JPEG stream of the front camera for the web dashboard. Raw 640×480 RGB at 30 fps is ~27 MB/s — far too heavy to push to a browser over Wi-Fi.

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/camera/image_raw` | `Image` | Kinect v1 RGB |
| Pub | `/camera/stream/compressed` | `CompressedImage` (JPEG) | Throttled + compressed, consumed by `dashboard_server` |

**Params:** `fps` (15), `jpeg_quality` (70), `width` (640)
**Implementation:** throttle to `fps` → optional resize → `cv2.imencode('.jpg', frame)` → publish. `dashboard_server` re-serves these bytes to browsers as MJPEG (`/stream/camera`, §5.2) without re-encoding.

---

### 4.5 Mission, WMS & Fork Layer — `wafl_mission`, `wafl_fork_insertion`

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

**E-stop:** GUI [E-STOP] publishes `/e_stop = true` **and** `/gui/mode = "MANUAL"`. `mission_manager_node` cancels the active Nav2 and `insert_forks` goals and enters `ESTOPPED`. `base_controller`, `fork_insertion_node` and `charging_dock_node` also subscribe to `/e_stop` directly, so motion stops even if this node is busy or dead.

**E-stop recovery — two explicit operator steps:**
1. GUI [RESET E-STOP] → `/e_stop = false`. Mode stays MANUAL (state `MANUAL`), so the operator can jog the robot clear first.
2. GUI [RESUME AUTO] → `/gui/mode = "AUTO"`. The button is disabled while the e-stop is latched.

**Manual override flow:**
```
GUI → /gui/mode = "MANUAL"
  mission_manager cancels the active Nav2 / insert_forks goals, sends /charging_dock/start = false,
    and saves the mission state (see D20)
  fork_insertion_node / charging_dock_node stop publishing (they also subscribe to /gui/mode)
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
| Sub | `/gui/mode` | `String` | All states — MANUAL → abort, stop publishing (single-writer rule) |
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

| Direction | Topic | Type | Notes |
|-----------|-------|------|-------|
| Sub | `/charging_dock/start` | `Bool` | From `mission_manager` (CHARGE state); `false` = abort |
| Sub | `/pallet_location_id` | `String` | Watch for "99" |
| Sub | `/pallet_pose` | `PoseStamped` | Dock marker pose |
| Sub | `/battery/state` | `BatteryState` | Confirm charging |
| Sub | `/e_stop` | `Bool` | Stop, `/lift_cmd = 0` |
| Sub | `/gui/mode` | `String` | MANUAL → abort, stop publishing (single-writer rule) |
| Pub | `/charging_dock/state` | `String` | IDLE \| NAVIGATING \| ALIGNING \| DOCKED \| FAILED |
| Pub | `/charging_dock/cmd_vel` | `Twist` | Final approach, via `base_controller` |
| Pub | `/lift_cmd` | `Int32` | Forks down before docking |
| Action Client | `navigate_to_pose` | `NavigateToPose` | Coarse approach to the dock |

---

## 5. GUI Dashboard — `wafl_dashboard`

**Technology:** browser front-end in plain **HTML5 + CSS + vanilla JavaScript** (no framework, no build step), served by a **FastAPI** back-end, `dashboard_server`, which is also a ROS 2 node (`rclpy`).
**Where it runs:** `dashboard_server` runs on the Jetson (hardware) or the sim machine / container. The operator only needs a browser at `http://<jetson-ip>:8000` — no ROS install on the operator laptop and no DDS traffic over Wi-Fi.
**Golden rule:** the browser never talks to ROS directly (no rosbridge). Every topic the GUI uses is subscribed / published by `dashboard_server`; the browser only talks to FastAPI.

### 5.1 Architecture

```
 Operator laptop (any browser)          Jetson / sim machine
┌─────────────────────────────┐         ┌──────────────────────────────────────────┐
│ index.html + js/ + css/     │         │ dashboard_server (FastAPI + rclpy)       │
│ [ROBOT]  [LOGS]  [WMS]      │         │                                          │
│                             │◄── WS ──┤ /ws        telemetry, health, logs       │
│ <canvas> map + pose + plan  ├── WS ──►│            joystick (10 Hz)              │
│ <img> MJPEG camera          │◄─ HTTP ─┤ /stream/*  MJPEG camera streams          │
│ buttons / forms             ├─ HTTP ─►│ /api/*     REST commands + snapshots     │
└─────────────────────────────┘  Wi-Fi  │            ▲ subscribe      │ publish    │
                                        │            │  (rclpy thread) ▼           │
                                        │       ROS 2 topic graph (DDS, local)     │
                                        └──────────────────────────────────────────┘
```

### 5.2 Browser ↔ Back-end Interfaces

| Channel | Endpoint | Direction | Content |
|---------|----------|-----------|---------|
| Static | `GET /` | server → browser | `index.html`, `css/`, `js/` |
| WebSocket | `/ws` | server → browser | JSON `{"type": ..., "data": ...}`: `snapshot`, `status`, `pose`, `plan`, `battery`, `mission_status`, `map_updated`, `health`, `log`, `wms` |
| WebSocket | `/ws` | browser → server | `{"type": "joy", "v": m/s, "w": rad/s, "vy": m/s}` → `/manual_cmd_vel` (`vy` honoured only in CRAB) |
| WebSocket | `/ws` | browser → server | `{"type": "joint", "alpha": deg, "beta": deg, "vl": m/s, "vr": m/s}` → `/manual_joint_cmd` (JOINT mode only, 10 Hz while a jog button is held) |
| MJPEG | `GET /stream/camera` | server → browser | `multipart/x-mixed-replace` from `/camera/stream/compressed` |
| MJPEG | `GET /stream/debug` | server → browser | `multipart/x-mixed-replace` from `/pallet/debug_image` (YOLO overlay) |
| REST | `GET /api/map.png`, `GET /api/map/meta` | server → browser | Latest `/map` as PNG + resolution / origin / size |
| REST | `GET /api/logs?limit=500` | server → browser | Log ring-buffer history |
| REST | `GET /api/health` | server → browser | Node + ESP32 + WMS health (JSON) |
| REST | `POST /api/estop` `{"active": bool}` | browser → ROS | `/e_stop` (`true` also sends `/gui/mode = "MANUAL"`) |
| REST | `POST /api/mode` `{"mode": "MANUAL"\|"AUTO"}` | browser → ROS | `/gui/mode` (AUTO rejected while e-stop is latched) |
| REST | `POST /api/lift` `{"cmd": 0\|1\|2}` | browser → ROS | `/lift_cmd` — MANUAL only |
| REST | `POST /api/lights` `{"relays": [0,1,0,0]}` | browser → ROS | `/lights_cmd` — MANUAL only |
| REST | `POST /api/drive_mode` `{"mode": "NORMAL"\|"CRAB"\|"JOINT"}` | browser → ROS | `/drive_mode` — MANUAL only (§4.3) |
| REST | `POST /api/mission` `{"pickup", "dropoff", "priority"}` | browser → ROS | `/wms/manual_mission` |

**Server-side throttling:** `pose` / speed 10 Hz · `status` / `battery` 2 Hz · map ≤ 1 Hz (server renders the `OccupancyGrid` to PNG, pushes `map_updated`, browser re-fetches `/api/map.png?v=<stamp>`) · logs batched every 200 ms.

### 5.3 Implementation Rules

- **Threading:** the `rclpy` executor spins in a background thread started in FastAPI's `lifespan`; ROS callbacks hand data to asyncio with `loop.call_soon_threadsafe(...)`. Never spin ROS on the event-loop thread.
- **One worker only:** run uvicorn with a single worker — every extra worker would create a duplicate ROS node with duplicate publishers.
- **Joystick deadman:** the browser sends `joy` at 10 Hz while a key / button is held and `v = w = 0` on release. `dashboard_server` publishes zero if the last `joy` is older than 0.3 s or the WebSocket closes; `base_controller`'s 0.5 s `cmd_timeout` is the second layer.
- **Single-writer rule enforced server-side:** `joy`, `joint`, `/api/lift`, `/api/lights` and `/api/drive_mode` are rejected (HTTP 409) unless mode is MANUAL.
- **Reload-safe:** on every WebSocket (re)connect the server first sends a `snapshot` (mode, e-stop, last status, mission lists, last 500 log lines), so a page reload or Wi-Fi drop loses nothing. The browser auto-reconnects and shows a `WS: LIVE / LOST` badge.
- **Not a safety device:** the browser E-STOP travels over Wi-Fi — a hardware e-stop that cuts motor power is still required (D15).

### Tab 1 — Robot Control & Status

```
┌──────────────────────────────────────────────────────────────────┐
│  WAFL 2027   [ROBOT]  [LOGS]  [WMS]     WS: ● LIVE   MODE: AUTO  │
├──────────────────┬──────────────────┬────────────────────────────┤
│  WAREHOUSE MAP   │  ROBOT STATUS    │  MISSION                   │
│  <canvas>        │                  │                            │
│  Live /map       │  State: NAVIGATE │  Active: M-001  A1→B2      │
│  Robot icon      │  Speed: 0.35 m/s │  ─────────────────────     │
│  /plan overlay   │  Fork: 0 mm      │  Queue:                    │
│                  │  Battery: 87%    │   M-002: C3→A1             │
│                  │  [████████  ]    │                            │
│                  │                  │  [Inject Mission]          │
│                  │  [E-STOP]        │                            │
│                  │  [RESET E-STOP]  │                            │
│                  │  [RESUME AUTO]   │                            │
│                  │  [MANUAL MODE]   │                            │
├──────────────────┴──────────────────┴────────────────────────────┤
│  FRONT CAMERA — MJPEG /stream/camera       [RAW | YOLO overlay]  │
│  640×480 @ ~15 fps — plain <img> tag, no plugin/decoder needed   │
├──────────────────────────────────────────────────────────────────┤
│  MANUAL JOYSTICK — visible only in MANUAL mode (keys: W A S D)   │
│  [↑][↓][←][→]   Lift: [UP][STOP][DOWN]   Lights: [ON][OFF]       │
│  Drive mode: [NORMAL] [CRAB] [JOINT]   (CRAB/JOINT: MANUAL only) │
│  JOINT: Front α [◄][CENTRE][►]    Rear β [◄][CENTRE][►]          │
│  Left wheel [FWD][BWD]   Right wheel [FWD][BWD]   [STOP ALL]     │
└──────────────────────────────────────────────────────────────────┘
```

**Buttons:** [E-STOP] → `/e_stop = true` + `/gui/mode = MANUAL` · [RESET E-STOP] → `/e_stop = false` · [RESUME AUTO] → `/gui/mode = AUTO` (disabled while e-stopped) · [MANUAL MODE] → `/gui/mode = MANUAL`. See §4.5 for the full recovery flow.
**Subscribes (via `dashboard_server`):** `/map`, TF `map → base_footprint` (robot icon), `/odometry/filtered` (actual speed), `/plan`, `/robot/state`, `/fork_height`, `/battery/state`, `/wms/mission_status`, `/camera/stream/compressed`, `/pallet/debug_image`, `/joint_states` (jog panel readout)
**Publishes (via `dashboard_server`):** `/e_stop`, `/gui/mode`, `/manual_cmd_vel`, `/manual_joint_cmd`, `/drive_mode`, `/lift_cmd`, `/lights_cmd`, `/wms/manual_mission`
**Manual drive:** in NORMAL the joystick sends `v` and `w`. The `[NORMAL | CRAB | JOINT]` toggle (MANUAL only, `POST /api/drive_mode`) switches mode. In CRAB the joystick sends `v` (forward) and `vy` (sideways) and `w` is ignored. In JOINT the jog panel is shown: the front `α` and rear `β` [◄] / [►] buttons step the angle by 5° (hold = repeat), [CENTRE] sets 0°, and each front wheel has [FWD] / [BWD] (fixed jog speed, e.g. 0.2 m/s, only while held); [STOP ALL] zeroes both wheels. Lift [UP] / [STOP] / [DOWN] and the lights work in every mode. The server refuses CRAB / JOINT unless the mode is MANUAL and the e-stop is clear, and the panel shows the live `α`, `β` and wheel speeds from `/joint_states`.
**Video:** MJPEG `/stream/camera` (raw) or `/stream/debug` (YOLO overlay), toggled in the browser — front camera only

---

### Tab 2 — Logs & System Health

```
┌──────────────────────────────────────────────────────────────────┐
│  SYSTEM HEALTH              │  EVENT LOG                         │
│  ─────────────────────────  │  ──────────────────────────────   │
│  ✅ slam_toolbox            │  [10:34:01] INFO  Mission started  │
│  ✅ nav2 (lifecycle)        │  [10:35:12] WARN  Battery < 20%   │
│  ✅ base_controller         │  [10:35:45] INFO  Pallet detected  │
│  ⚠️ camera_stream_node     │  [10:36:00] ERROR Fork timeout    │
│  ❌ battery_monitor_node   │                                    │
│                             │  [Level: ALL ▼] [Node: ALL ▼]     │
│  ESP32 #1 DRIVING:  ✅     │  [Pause] [Clear]                   │
│  ESP32 #2 STEERING: ✅     │                                    │
│  ESP32 #3 LIFT:     ✅     │                                    │
│  WMS Server: ✅ CONNECTED   │                                    │
└──────────────────────────────────────────────────────────────────┘
```

**Node health:** `dashboard_server` reads the ROS node graph every 2 s and pushes a `health` message; Nav2 is shown as healthy only when its lifecycle nodes (`bt_navigator`, `controller_server`, `planner_server`) are `active`.
**ESP32 liveness:** inferred from topic publication rate (silent > 1 s = dead). This relies on the fixed-rate publishing contract in §4.1 — the ESP32s publish even when values are unchanged. The micro-ROS agent is not a ROS node — it is healthy when the three ESP32 heartbeats arrive.
**Event log:** `dashboard_server` subscribes to `/rosout`, keeps the last 500 entries in a ring buffer, and streams new ones over `/ws` — every node's `self.get_logger()` call appears here automatically. Filtering happens in the browser.

**Subscribes (via `dashboard_server`):** `/rosout`, `/wms/connection_status`, `/fork_insertion/state`, `/charging_dock/state`, `/encoder_count` (ESP32 #1 heartbeat), `/imu/data` (ESP32 #2 heartbeat), `/fork_height` (ESP32 #3 heartbeat)

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

**Subscribes (via `dashboard_server`):** `/wms/package_list`, `/wms/mission_status`, `/wms/location_grid`
**Publishes (via `dashboard_server`):** `/wms/manual_mission` (from `POST /api/mission`)
**Why not call the WMS REST API directly from the browser?** The WMS protocol changes from REST (Phase 1) to MQTT (Phase 2); routing through `wms_bridge_node` keeps the dashboard unchanged.

---

## 6. Simulation — `wafl_simulation`

### Sensors Provided by Gazebo

| Sensor | Gazebo Plugin | ROS Topic |
|--------|--------------|-----------|
| LiDAR | `GpuLidarSensor` | `/scan` |
| Front RGB | `CameraSensor` | `/camera/image_raw` |
| Front Depth | `DepthCameraSensor` | `/camera/depth/image_raw` |
| IMU | `ImuSensor` | `/imu` (raw → `odom_covariance_fix` → `/imu_fixed`) |
| Odometry | `gz::sim::systems::OdometryPublisher` (ground-truth pose; TF publishing **disabled** — the EKF owns `odom → base_footprint`). **No DiffDrive** — it cannot model steered wheels | `/odom` (output) |
| Front steering joints (`front_left_steer_joint`, `front_right_steer_joint`) | Revolute (axis +z) + one `JointPositionController` each, both on the **same** gz topic, so they stay parallel like the real chain | `/front_steer_cmd_pos` (input) |
| Rear steering joint (`caster_joint`) | Revolute (axis +z) + `JointPositionController` | `/castor_cmd_pos` (input) |
| Front drive wheels (`front_left_wheel_joint`, `front_right_wheel_joint`) | Continuous + velocity `JointController` | `/front_left_wheel_cmd_vel`, `/front_right_wheel_cmd_vel` (input, rad/s) |
| Rear wheel (`caster_wheel_joint`) | Continuous, **passive** (no plugin, low friction) — it must roll freely | — |
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

- ros_topic_name: "/front_steer_cmd_pos"      # both front steering joints subscribe to this one gz topic
  gz_topic_name:  "/model/wafl/front_steer/cmd_pos"          # set <topic> explicitly in both plugins
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/front_left_wheel_cmd_vel"
  gz_topic_name:  "/model/wafl/joint/front_left_wheel_joint/cmd_vel"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/front_right_wheel_cmd_vel"
  gz_topic_name:  "/model/wafl/joint/front_right_wheel_joint/cmd_vel"
  ros_type_name:  "std_msgs/msg/Float64"
  gz_type_name:   "gz.msgs.Double"
  direction: ROS_TO_GZ

- ros_topic_name: "/castor_cmd_pos"
  gz_topic_name:  "/model/wafl/joint/caster_joint/cmd_pos"   # set <topic> explicitly in the plugin — the default includes the axis index (/0/cmd_pos)
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

> **QoS:** `/e_stop`, `/gui/mode`, `/robot/state` use `transient_local` (latched) so a late-joining node (restarted `base_controller`, `mission_manager_node`, `dashboard_server`) immediately learns the current value. The latched sample lives in the publisher: if `dashboard_server` dies, subscribers keep their last value and manual motion stops through the command timeouts (§4.3, §5.3). Sensor and camera streams use `sensor_data` (best-effort).
>
> **"GUI TabN"** below means the `dashboard_server` node, which subscribes / publishes on behalf of that browser tab (§5).

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
| `/manual_joint_cmd` | `Float32MultiArray` | GUI Tab1 | `base_controller` (JOINT mode only) |
| `/fork_insertion/cmd_vel` | `Twist` | `fork_insertion_node` | `base_controller` |
| `/charging_dock/cmd_vel` | `Twist` | `charging_dock_node` | `base_controller` |
| `/drive_mode` | `String` (latched) | GUI Tab1 | `base_controller` |
| `/target_front_angle` | `Float32` | `base_controller` | ESP32 #2 (front wheels, `α`) |
| `/target_angle` | `Float32` | `base_controller` | ESP32 #2 (rear wheel, `β`) |
| `/front_steer_cmd_pos` | `Float64` | `base_controller` | Gazebo (sim only, both front steering joints) |
| `/castor_cmd_pos` | `Float64` | `base_controller` | Gazebo (sim only, rear steering joint) |
| `/front_left_wheel_cmd_vel` | `Float64` | `base_controller` | Gazebo (sim only) |
| `/front_right_wheel_cmd_vel` | `Float64` | `base_controller` | Gazebo (sim only) |
| `/plan` | `Path` | Nav2 | GUI Tab1 |
| `/target_speed_left` | `Float32` | `base_controller` | ESP32 #1 |
| `/target_speed_right` | `Float32` | `base_controller` | ESP32 #1 |
| `/lift_cmd` | `Int32` | `fork_insertion_node`, `charging_dock_node`, GUI Tab1 | ESP32 #3 / `sim_fork_adapter` |
| `/lights_cmd` | `Int32MultiArray` | `mission_manager_node`, GUI Tab1 | ESP32 #3 |
| `/fork_height` | `Float32` | ESP32 #3 / `sim_fork_adapter` | `fork_insertion_node`, `mission_manager_node`, GUI Tab1, GUI Tab2 (heartbeat) |
| `/fork_cmd_vel` | `Float64` | `sim_fork_adapter` | Gazebo fork joint (sim only) |
| `/joint_states` | `JointState` | Gazebo via bridge (sim) · hardware: open (D21) | `robot_state_publisher`, `sim_fork_adapter`, GUI Tab1 (jog readout) |
| `/camera/image_raw` | `Image` | Kinect / Gazebo | `pallet_pose_node`, `pallet_front_angle_node`, `camera_stream_node` |
| `/camera/depth/image_raw` | `Image` | Kinect / Gazebo | `pallet_front_angle_node` |
| `/camera/stream/compressed` | `CompressedImage` | `camera_stream_node` | GUI Tab1 (MJPEG `/stream/camera`) |
| `/pallet_pose` | `PoseStamped` | `pallet_pose_node` | `fork_insertion_node`, `charging_dock_node` |
| `/pallet_location` | `String` | `pallet_pose_node` | (informational logging) |
| `/pallet_location_id` | `String` | `pallet_pose_node` | `mission_manager_node`, `charging_dock_node` |
| `/fork_target` | `PointStamped` | `pallet_pose_node` | `fork_insertion_node` |
| `/pallet_marker` | `visualization_msgs/Marker` | `pallet_pose_node` | RViz |
| `/pallet/rotate_to_front_rad` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/center_yaw_rad` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/final_z_m` | `Float32` | `pallet_front_angle_node` | `fork_insertion_node` |
| `/pallet/pallet_yaw_offset_rad` | `Float32` | `pallet_front_angle_node` | (logging) |
| `/pallet/debug_image` | `CompressedImage` | `pallet_front_angle_node` | GUI Tab1 (MJPEG `/stream/debug`) |
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
| `/gui/mode` | `String` | GUI Tab1 | `mission_manager_node`, `base_controller`, `fork_insertion_node`, `charging_dock_node` |
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
| D4 | GUI technology | ✅ DECIDED | Web dashboard: HTML + CSS + vanilla JS front-end, FastAPI + `rclpy` back-end (`wafl_dashboard`, §5) | Front-end outgrows vanilla JS → add a framework (e.g. Vite + React); back-end API stays |
| D5 | WMS Phase 1 server | ✅ DECIDED | FastAPI stub | — |
| D6 | Nav2 local planner | ⬜ OPEN | Leaning **Regulated Pure Pursuit** (tracks straight aisles and curves smoothly without DWB jitter, §4.3); DWB = 2026 baseline | RPP can't handle dynamic obstacles well enough in sim → benchmark MPPI |
| D7 | Charging dock detection | ✅ DECIDED | ArUco ID=99 (reuses pipeline) | Bad lighting → IR beacon |
| D8 | Dual encoders | ⬜ OPEN | Single encoder + IMU yaw. Note: the encoder is on the front-LEFT wheel, so `/wheel_velocity` is `v_left`, not the body speed — `odom_fusion_node` must use `v ≈ v_left + ω·W/2` (ω from the IMU) | Poor turns → add right-wheel encoder |
| D9 | Fork height sensing | ⬜ OPEN | Potentiometer | — |
| D10 | Load cell | ⬜ OPEN | Skipped | Needed for CONFIRM state reliability |
| D11 | Camera streaming | ✅ DECIDED | `camera_stream_node` (JPEG, 15 fps) → `dashboard_server` → MJPEG over HTTP (`<img>` tag) | Latency / Wi-Fi bandwidth too high → WebRTC |
| D12 | Fork handoff mechanism | ⬜ OPEN | Three overlapping paths exist today: `/mission_manager/fork_goal` topic, `insert_forks` action, `/fork_insertion/done` topic | **Recommend action only** (goal/result/feedback/cancel); delete the topic pair |
| D13 | Who owns `DROP` | ⬜ OPEN | Listed in both `mission_manager` and `fork_insertion_node` state machines | Recommend `InsertForks.action` goal gets an `operation` field (PICKUP \| DROP); `mission_manager` owns NAVIGATE_TO_DROP only |
| D14 | Unused interfaces | ⬜ OPEN | `Alarm.msg` and `SetMission.srv` have no publisher / server | Wire `/alarms` + a service, or delete them |
| D15 | Safety chain | ⬜ OPEN | Software: `/e_stop` reaches `base_controller`, `fork_insertion_node`, `charging_dock_node`, 0.5 s command timeout. **Hardware e-stop that cuts motor power is required** — the GUI button goes over Wi-Fi | Not optional for a forklift |
| D16 | Steering homing / limits | ⬜ OPEN | Both steppers (front pair and rear wheel) have no position feedback → add a home switch or potentiometer to each. Rear: ±80° clamp vs 90° needed for in-place rotation. Front: CRAB needs ±90° — the mechanism can turn 90° (confirmed), clamp to the real mechanical stops | Constrain Nav2 rotation behaviour, raise the clamp, or limit CRAB to the real range |
| D17 | GUI robot pose source | ✅ DECIDED | TF `map → base_footprint` (`/odometry/filtered` is odom-frame and drifts against the map) | — |
| D18 | Averaged pallet topics | ✅ DECIDED | Averaging is internal; only the averaged values are published on the topics in §7 (no `_avg` variants) | — |
| D19 | IMU part | ⬜ OPEN | MPU-6050 is 6-DOF (no magnetometer → yaw drifts); BNO055 gives fused orientation | Prefer BNO055 |
| D20 | Manual mode vs Nav2 lifecycle | ⬜ OPEN | Proposed (used in §4.5): cancel the active goal and let `base_controller` arbitrate. Deactivating Nav2 lifecycle nodes is rejected (slow, costmaps restart) | Confirm in sim; revisit only if goal cancel proves unreliable |
| D21 | Hardware `/joint_states` | ⬜ OPEN | Nothing publishes it on hardware, so `robot_state_publisher` has no steering / fork frames | `base_controller` publishes the steering joints (last `/target_front_angle`, `/target_angle`) + a fork-height → joint adapter, or make those joints fixed in the hardware URDF |
| D22 | Dashboard access & control | ⬜ OPEN | Phase 1: LAN only, no login; every open browser can send commands | Before hardware demos: one "controller" browser at a time (others view-only) + a shared access token |
| D23 | Front-pair steering kinematics | ⬜ OPEN (proposed in §4.3) | Modes NORMAL (`α = 0`, differential + rear follows), MANUAL-only CRAB (`α = β`) and JOINT (direct jog). **Confirmed:** two independent front drive motors; the front pair can turn ±90°; `track_width`, `wheelbase_L`, wheel radius come from the CAD. **Still open:** (1) which ESP32 / board drives the front steering motor — *ask the low-level team*; until answered, `/target_front_angle` on ESP32 #2 is a placeholder; (2) hardware odometry does **not** model CRAB (it assumes forward motion only, so the map position would drift when crabbing; sim is unaffected) — use CRAB for short manual moves only; (3) the sim files (`ros_gz_bridge.yaml`, `generate_assets.py`, `wafl_gz_plugins.xacro`) and the new URDF must follow the joint names in §6 | Mechanism differs → revise §4.3 |

### Research Links

| Topic | Links |
|-------|-------|
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox |
| Cartographer ROS | https://google-cartographer-ros.readthedocs.io |
| Nav2 SLAM guide | https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html |
| Regulated Pure Pursuit | https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html |
| MPPI planner | https://docs.nav2.org/configuration/packages/configuring-mppic.html |
| DWB planner | https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html |
| MQTT (Mosquitto) | https://mosquitto.org/ |
| Paho-MQTT Python | https://pypi.org/project/paho-mqtt/ |
| FastAPI WebSocket | https://fastapi.tiangolo.com/advanced/websockets/ |
| FastAPI StreamingResponse (MJPEG) | https://fastapi.tiangolo.com/advanced/custom-response/#streamingresponse |
| FastAPI lifespan (start rclpy thread) | https://fastapi.tiangolo.com/advanced/events/ |

---

## 10. Simulation vs Hardware Reference

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Compute | Developer laptop / Docker | Jetson Xavier AGX |
| OS | Ubuntu 24.04 (host or Docker) | Ubuntu 24.04 (Jetson) |
| LiDAR | `GpuLidarSensor` | Real RPLiDAR A2M8 |
| Camera | `CameraSensor` + `DepthCameraSensor` | Kinect v1 (front only) |
| IMU | `ImuSensor` plugin | MPU-6050 / BNO055 on ESP32 #2 |
| Odometry | Gazebo `OdometryPublisher` (ground truth) → `odom_covariance_fix` → EKF | `encoder_velocity_node` + `odom_fusion_node` → EKF |
| Front steering (both wheels, `α`) | `/front_steer_cmd_pos` (two Gazebo joints, same command) | `/target_front_angle` → ESP32 #2 |
| Rear steering (`β`) | `/castor_cmd_pos` (Gazebo joint) | `/target_angle` → ESP32 #2 |
| Driving (front wheels only) | Gazebo joint velocity controllers ← `/front_left_wheel_cmd_vel`, `/front_right_wheel_cmd_vel` | `/target_speed_left`, `/target_speed_right` → ESP32 #1 (PID on the ESP32) |
| Lift | Gazebo prismatic joint via `sim_fork_adapter` | `/lift_cmd` → ESP32 #3 |
| Fork height | `sim_fork_adapter` from joint state | Potentiometer or encoder (D9 open) → ESP32 #3 → `/fork_height` |
| Clock | `/clock` from Gazebo bridge | System clock |
| WMS | FastAPI stub (localhost) | Real WMS server (networked) |
| Dashboard | `dashboard_server` on the laptop / container → browser at `http://localhost:8000` | `dashboard_server` on the Jetson → operator browser at `http://<jetson-ip>:8000` |
| Charging dock | ArUco ID=99 in world model | Real dock with ArUco marker |
| `use_sim_time` (clock) | `true` | `false` |
| `sim` (mode) | `true` | `false` |
| micro-ROS agent | Not needed | Required on Jetson, start first |

---

## 11. How to Run

> The dashboard is started by both bringup launch files — there is no separate GUI process. Open **http://localhost:8000** (sim) or **http://\<jetson-ip\>:8000** (hardware) in any browser.

### Docker (Recommended)
```bash
# All commands from the repo root — the compose file lives in docker/

# First time
docker compose -f docker/docker-compose.yml build

# Terminal 1 — WMS stub (start first so wms_bridge_node connects immediately; it retries otherwise)
cd wms_server && uvicorn main:app --host 0.0.0.0 --port 8080

# Terminal 2 — simulation (Gazebo + SLAM + Nav2 + perception + dashboard_server)
docker compose -f docker/docker-compose.yml up -d

# GUI: open in a browser
xdg-open http://localhost:8000
```
> `docker-compose.yml` uses `network_mode: host`, so DDS discovery works and ports 8000 / 8080 are reachable without extra port mapping.

### Native — Simulation
```bash
# Terminal 1 — WMS stub
cd wms_server && uvicorn main:app --port 8080

# Terminal 2 — full simulation (includes dashboard_server)
ros2 launch wafl_bringup simulation.launch.py use_sim_time:=true

# Browser
xdg-open http://localhost:8000
```

### Native — Real Hardware
```bash
# Jetson: Terminal 1 (start BEFORE powering ESP32s)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# Jetson: Terminal 2 (includes dashboard_server)
ros2 launch wafl_bringup hardware.launch.py use_sim_time:=false

# Operator laptop: browser only — no ROS install needed
#   http://<jetson-ip>:8000
```

### Dashboard Only (GUI development)
```bash
# Restart only the dashboard against an already-running sim / robot
ros2 launch wafl_dashboard dashboard.launch.py port:=8000
```
Front-end files (`static/`) are plain HTML/JS/CSS — no npm, no build step. After editing, run `colcon build --packages-select wafl_dashboard`, restart the dashboard and refresh the browser.

### Debug Commands
```bash
ros2 node list                          # all running nodes
ros2 topic list -t                      # all topics + types
ros2 topic echo /robot/state            # monitor state
ros2 topic hz /odometry/filtered        # should be ~50 Hz
ros2 topic hz /camera/stream/compressed # should be ~15 Hz
ros2 run tf2_tools view_frames          # check TF tree
curl http://localhost:8000/api/health   # dashboard view of node / ESP32 / WMS health
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
11. **The browser never talks to ROS directly** — every new GUI feature is a `dashboard_server` endpoint or WebSocket message type, documented in §5.2

---

*WAFL 2027 — Ain Shams University, Mechatronics & Robotics Engineering*
*Maintained by team lead. Last updated: see git log.*
