# WAFL 2027 — Master Architecture & Design Document

> **Project:** Warehouse Automation with an Autonomous Fork-Lift Mobile Robot
> **Institution:** Ain Shams University — Mechatronics & Robotics Engineering (2026/2027)
> **Status:** 🔴 Brainstorm / Design Phase — This is a living document. Sections marked `[OPEN]` are pending decisions.
> **Stack (target):** ROS 2 Jazzy · Gazebo Harmonic · Jetson Xavier · micro-ROS · Python · C++ · YOLO / TensorRT

---

## Preface — What This Document Is

This document is the **master architecture reference** for WAFL 2027. It is written before a single line of code is committed. Its job is to:

1. Capture everything learned from WAFL 2025 and WAFL 2026
2. Define the target architecture clearly enough that any team member can understand the whole system
3. Identify every open design decision so the team can resolve them early — not mid-implementation
4. Serve as the source of truth that `LAUNCH.md`, individual package READMEs, and all documentation point back to

**Read this before touching any code. Update it when a design decision is resolved.**

---

## 1. The Big Picture

WAFL 2027 is a **fully autonomous, WMS-connected warehouse forklift** that:

- Receives transport missions from a **Warehouse Management System (WMS)**
- **Localizes itself** in a live-built map (SLAM — no pre-built map required)
- **Plans and executes** collision-free routes through the warehouse
- **Identifies and aligns** with pallets using computer vision (ArUco + YOLO)
- **Autonomously inserts forks, lifts the pallet, transports it, and places it** at the destination
- **Reports mission status** back to the WMS in real time
- Monitors its own **battery** and navigates to a charging dock when needed

This is a **continuation and major upgrade** of WAFL 2026. The foundation (robot chassis, ESP32 firmware, ROS 2 navigation stack, perception pipeline) carries over. The new systems added in 2027: WMS integration, full fork-insertion automation, SLAM (live mapping), GUI dashboard, and autonomous charging.

---

## 2. System Architecture Overview

```
╔══════════════════════════════════════════════════════════════════════════╗
║                          WAFL 2027 SYSTEM                               ║
╠══════════════════════════════════════════════════════════════════════════╣
║                                                                          ║
║  ┌─────────────────────────────────────────────────────────────────┐    ║
║  │              EXTERNAL LAYER (Off-Robot)                          │    ║
║  │                                                                  │    ║
║  │   ┌─────────────────┐         ┌──────────────────────────────┐  │    ║
║  │   │   WMS Server    │◄───────►│     GUI Dashboard            │  │    ║
║  │   │ (Mission Queue) │  MQTT   │  (Operator UI)               │  │    ║
║  │   │                 │  or     │  • Map + robot position       │  │    ║
║  │   │  REST / WS API  │  WS     │  • Mission queue panel        │  │    ║
║  │   └────────┬────────┘         │  • Live camera feed (TCP)     │  │    ║
║  │            │                  │  • Battery + charging status  │  │    ║
║  │            │  Task dispatch   │  • E-Stop + manual override   │  │    ║
║  │            │  Status report   │  • Alarm / safety log         │  │    ║
║  │            │                  │  • WMS connection status      │  │    ║
║  │            │                  └──────────────────────────────┘  │    ║
║  └────────────┼─────────────────────────────────────────────────────┘    ║
║               │  Wi-Fi                                                    ║
║  ┌────────────▼─────────────────────────────────────────────────────┐    ║
║  │              ON-ROBOT LAYER (Jetson Xavier AGX)                   │    ║
║  │                                                                   │    ║
║  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    ║
║  │  │  SLAM / Mapping  │  │   Navigation    │  │  WMS Bridge     │  │    ║
║  │  │  SLAM Toolbox   │  │   Nav2 Stack    │  │  mission_node   │  │    ║
║  │  │  robot_loc. EKF │  │   DWB / BT Nav  │  │  status_pub     │  │    ║
║  │  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  │    ║
║  │           │                    │                     │           │    ║
║  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    ║
║  │  │  Perception     │  │  Fork Insertion │  │  Camera Stream  │  │    ║
║  │  │  ArUco + YOLO   │  │  State Machine  │  │  TCP sender     │  │    ║
║  │  │  TensorRT       │  │  align->insert  │  │  (GUI feed)     │  │    ║
║  │  └────────┬────────┘  └────────┬────────┘  └─────────────────┘  │    ║
║  │           │                    │                                  │    ║
║  │  ┌─────────────────────────────▼──────────────────────────────┐  │    ║
║  │  │                 micro-ROS Agent (UDP :8888)                 │  │    ║
║  │  └──────┬────────────────────────┬────────────────┬───────────┘  │    ║
║  └──────────┼────────────────────────┼────────────────┼──────────────┘    ║
║             │  Wi-Fi (micro-ROS)     │                │                   ║
║  ┌──────────▼──────┐  ┌─────────────▼──────┐  ┌──────▼──────────────┐   ║
║  │  ESP32 #1       │  │  ESP32 #2           │  │  ESP32 #3           │   ║
║  │  DRIVING        │  │  STEERING + IMU     │  │  LIFT + LIGHTS      │   ║
║  │  2x DC motors   │  │  Stepper (TB6600)   │  │  Lift DC motor      │   ║
║  │  Encoders       │  │  MPU-6050/BNO055    │  │  2x limit switches  │   ║
║  │  /pwm_left      │  │  /target_angle      │  │  /lift_cmd          │   ║
║  │  /pwm_right     │  │  /imu/data  (NEW)   │  │  /lights_cmd        │   ║
║  │  /encoder_count │  │  (replaces phone)   │  │  /fork_height (NEW) │   ║
║  └─────────────────┘  └────────────────────┘  └─────────────────────┘   ║
╚══════════════════════════════════════════════════════════════════════════╝
```

---

## 3. What Changed from 2026 to 2027

| Area | WAFL 2026 | WAFL 2027 |
|------|-----------|-----------|
| **OS / ROS** | Ubuntu 22.04 + ROS 2 Humble | Ubuntu 24.04 + ROS 2 Jazzy |
| **Simulator** | Ignition Gazebo (Fortress) | Gazebo Harmonic |
| **Compute** | 2 laptops + Jetson Nano | 1 Jetson Xavier (all on-robot) |
| **IMU** | Phone app over UDP (fragile) | Proper IMU on ESP32 #2 (MPU-6050 / BNO055) |
| **Mapping** | Pre-built static map (.yaml + .pgm) | Live SLAM (SLAM Toolbox or equivalent) |
| **Fork insertion** | Manual — human operates lift | Automated state machine (approach → align → insert → lift) |
| **WMS** | None | WMS integration: mission dispatch + status reporting |
| **GUI** | PyQt5 manual dashboard (basic) | Full operator dashboard (map, missions, cameras, battery, alarms) |
| **Charging** | Manual | Autonomous charging dock detection + alignment + docking |
| **Repo structure** | Split workspaces (Localization/Simulation + Navigation/Simulation) | Unified single `src/` workspace |
| **Docker** | Optional Compose wrapper | First-class: `docker compose up` starts everything |
| **Battery** | No monitoring | BMS: SoC estimation, low-battery alert, charging trigger |

---

## 4. Compute Layout — Jetson Xavier

In 2027, a **single Jetson Xavier AGX** runs the entire software stack. There are no separate laptops.

```
Jetson Xavier AGX
├── [Process] slam_node             <- SLAM Toolbox, live mapping
├── [Process] ekf_node              <- robot_localization EKF
├── [Process] nav2_stack            <- full Nav2 (planner, controller, costmaps)
├── [Process] micro_ros_agent       <- UDP bridge to all 3 ESP32s
├── [Process] perception_node       <- Kinect -> ArUco + YOLO TensorRT
├── [Process] fork_insertion_node   <- state machine for pallet approach/lift
├── [Process] wms_bridge_node       <- communicates with WMS server
├── [Process] camera_stream_node    <- TCP stream -> GUI dashboard
├── [Process] battery_monitor_node  <- SoC estimation, charge triggering
└── [Process] mission_manager_node  <- top-level state machine (IDLE->MISSION->CHARGE)
```

> **Why one Xavier?** Eliminates the inter-laptop Wi-Fi dependency for ROS topics that caused latency and sync issues in 2026. All nodes on one machine = zero-latency IPC via DDS.

> **Risk:** If the Xavier overheats or crashes, the entire system is down.
> **Mitigation:** Thermal monitoring node + watchdog on all critical processes.

---

## 5. Software Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| OS | Ubuntu | 24.04 LTS (Noble) |
| ROS | ROS 2 | Jazzy Jalisco |
| Simulator | Gazebo | Harmonic (ros-jazzy-ros-gz) |
| Navigation | Nav2 | Jazzy-compatible |
| Localization | robot_localization (EKF) + SLAM Toolbox | latest |
| Perception | OpenCV + YOLO + TensorRT | latest |
| Firmware bridge | micro-ROS | Jazzy agent |
| GUI | `[OPEN]` — PySide6 / QML / Web | `[OPEN]` |
| WMS protocol | `[OPEN]` — MQTT / WebSocket / REST | `[OPEN]` |
| Language | Python 3.12 (nodes) + C++ (Arduino/ESP32) | — |
| Build system | colcon + CMake | — |
| Container | Docker + Docker Compose | latest |

---

## 6. Layer 1 — Low-Level Firmware (ESP32 + micro-ROS)

> **Status:** Carry over from 2026 with targeted upgrades.

### 6.1 ESP32 #1 — Driving

No major changes expected. Candidate improvements:
- Add **second encoder** for the right wheel → enables proper differential odometry (angular velocity from encoders, not just IMU)
- Expose `/odom_raw` (`nav_msgs/Odometry`) instead of raw `/encoder_count` so the EKF can consume it directly

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/pwm_left` | Sub | `Float32` | ±255 |
| `/pwm_right` | Sub | `Float32` | ±255 |
| `/encoder_count` | Pub | `Float32` | raw ticks |

### 6.2 ESP32 #2 — Steering + IMU (UPGRADED)

**Key 2027 change:** Replace the phone IMU with a proper IMU chip (MPU-6050 or BNO055) on ESP32 #2. This eliminates:
- The fragile UDP phone-app protocol
- Dependency on a personal phone for the robot to function
- `imu_udp_node.py` (deleted in 2027)

The ESP32 now publishes `sensor_msgs/Imu` messages with real covariances. The EKF consumes this directly.

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/target_angle` | Sub | `Float32` | degrees, from caster_controller |
| `/imu/data` | Pub | `sensor_msgs/Imu` | **NEW** — replaces `/imu_yaw` Float32 |

### 6.3 ESP32 #3 — Lift + Lights

No major changes. Candidate additions:
- `/fork_height` publisher (potentiometer or encoder on the lift) — **required** for fork-insertion automation
- `/fork_load` topic for weight sensing (if mechanical team adds load cell)

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/lift_cmd` | Sub | `Int32` | 0=STOP, 1=UP, 2=DOWN |
| `/lights_cmd` | Sub | `Int32MultiArray` | 4 relay lights |
| `/fork_height` | Pub | `Float32` | **NEW** — height in mm |

---

## 7. Layer 2 — Localization & Mapping

### 7.1 IMU Pipeline (Revised)

```
ESP32 #2 (MPU-6050 / BNO055)
    |
    |  micro-ROS (Wi-Fi UDP)
    v
/imu/data  (sensor_msgs/Imu, full 9-DOF, proper covariance)
    |
    |-> EKF (robot_localization)
    |-> SLAM Toolbox (optional IMU fusion)
```

The old chain `phone -> UDP -> imu_udp_node -> /imu_yaw` is **removed**.

### 7.2 Odometry Pipeline (Revised)

```
/encoder_count (ESP32 #1)
    |
    v
encoder_velocity_node
    |
    v
/wheel_velocity  (Float32, m/s)
    |
    v
odom_fusion_node -> /odom (nav_msgs/Odometry)
    |
    |-> EKF
    |-> SLAM Toolbox
```

> **Improvement target:** Replace single-encoder `/encoder_count` with dual-encoder differential odometry to get angular velocity from wheels (not just IMU). Makes odometry more robust in turns.

### 7.3 EKF (robot_localization)

Configuration largely unchanged from 2026. Key updates:
- IMU input changes from `/imu_yaw` (Float32, yaw-only) to `/imu/data` (full `sensor_msgs/Imu`)
- Proper covariance matrices from IMU chip (no more hardcoded values)
- Rate remains 50 Hz

### 7.4 SLAM `[OPEN DECISION D1]`

**The 2026 system used a pre-built static map**. WAFL 2027 targets live SLAM so:
1. No manual mapping session required before deployment
2. The map can update as the warehouse changes

**Recommended option: SLAM Toolbox (async mapping mode)**
- Native ROS 2 support, actively maintained
- Can serialize/deserialize a map and switch to localization-only mode
- Works with 2D LiDAR (RPLiDAR) out of the box
- Supports optional IMU fusion

**Open questions for the team to resolve:**
- Run SLAM continuously or map-once then localize?
- Save the map across sessions and reload on next startup?
- Is 2D SLAM sufficient or do we need 3D (shelves blocking laser)?

**SLAM Toolbox topics:**
```
/scan (RPLiDAR) -> slam_toolbox -> /map (OccupancyGrid)
                                -> TF: map -> odom
```

---

## 8. Layer 3 — Navigation

### 8.1 Nav2 Stack

Nav2 configuration upgrades for Jazzy:
- **BT Navigator** (Behavior Tree) — more robust recovery than the simple action calls used in 2026
- **MPPI Controller** as an alternative to DWB for smoother trajectories `[OPEN D6: benchmark both]`
- **Regulated Pure Pursuit** for straight-aisle driving segments
- Costmap plugins: static + obstacle + inflation (same as 2026)

**Speed targets (TBD by mechanical team):**
```yaml
max_vel_x: 1.0 m/s
max_vel_theta: 1.5 rad/s
```

### 8.2 Caster Controller (Unified — 2026 Bug Fixed)

**2026 bug:** The caster controller published to `/castor_cmd_pos` (simulation) which was NOT connected to the real ESP32 (`/target_angle`). In 2027, a single `caster_controller` node handles both:

```python
# Simulation (use_sim_time=true):  publishes to /castor_cmd_pos
# Hardware (use_sim_time=false):   publishes to /target_angle
```

The mode is controlled by a ROS parameter at launch — **no code changes needed to switch between sim and hardware**.

### 8.3 Localization Mode During Navigation `[OPEN DECISION D2]`

| Option | Pros | Cons |
|--------|------|------|
| **SLAM Toolbox localization mode** | One system does mapping + localization | Less tested in Nav2 |
| **SLAM for mapping + AMCL for nav** | AMCL is proven with Nav2 | Two systems to configure |

**Team action:** Test SLAM Toolbox localization mode first. Fall back to SLAM + AMCL if localization quality is insufficient.

---

## 9. Layer 4 — Perception

No sensor hardware changes (same RPLiDAR + Kinect v1). Software pipeline upgraded.

### 9.1 Pipeline A — ArUco Pose Estimation

Carried over from 2026. Changes:
- Location database moved from hardcoded Python dict to a **YAML config file** (loaded as ROS parameter) — fixes the 2026 "hardcoded in two places" issue
- Support for more than 4 pallet locations (scalable)

```
Kinect RGB -> pallet_pose_node (ArUco DICT_4X4_50)
    |
    |-> /pallet_pose         (PoseStamped — 6DOF in camera frame)
    |-> /fork_target         (PointStamped — 0.5m in front of marker)
    |-> /pallet_location     (String — "Location_A1")
    |-> /pallet_location_id  (String — marker ID)
```

### 9.2 Pipeline B — YOLO Fine Alignment (TensorRT on Xavier)

Carried over from 2026. The Jetson Xavier replaces the Jetson Nano — significantly faster inference.

3-model pipeline (same as 2026):
1. `model_pallet` → full pallet bounding box
2. `model_pocket` → fork pocket inside pallet
3. `model_keypoint` → 4 corner keypoints of pocket

New additions in 2027:
- **Rear camera node** (`rear_safety_node.py`): detects obstacles behind the robot during reverse motion
- Publishes `/rear_obstacle_detected` (Bool) → Nav2 costmap or emergency stop

### 9.3 Fork Insertion Automation `[NEW IN 2027]`

**This is the most significant new software component.**

In 2026, the state machine waited for `final_z_m ≈ 0` as a proxy for "pallet lifted by a human." In 2027, the robot does this autonomously.

**State machine:**

```
IDLE -> APPROACH -> ROTATE_ALIGN -> LATERAL_CENTER -> INSERT_FORKS -> LIFT -> CONFIRM -> NAVIGATE_TO_DROP -> DROP -> IDLE
```

Detailed:
```
WMS task -> IDLE
               |
          APPROACH        (Nav2 coarse goal: get to ~1m in front of pallet)
               |
          ROTATE_ALIGN    (rotate until rotate_to_front_rad ~= 0 from YOLO)
               |
          LATERAL_CENTER  (strafe/steer until center_yaw_rad ~= 0 from YOLO)
               |
          INSERT_FORKS    (drive forward final_z_m distance from YOLO)
               |
          LIFT            (/lift_cmd = UP until /fork_height = target OR limit switch)
               |
          CONFIRM         (verify pallet lifted: /fork_height + /fork_load)
               |
          NAVIGATE_TO_DROP (Nav2 goal = dropoff location from WMS mission)
               |
          DROP            (/lift_cmd = DOWN, then reverse clear of pallet)
               |
          IDLE            (report to WMS: COMPLETED)
```

**Key inputs to the state machine:**

| Input | Source | Used in state |
|-------|--------|---------------|
| `rotate_to_front_rad` | YOLO node | ROTATE_ALIGN |
| `center_yaw_rad` | YOLO node | LATERAL_CENTER |
| `final_z_m` | YOLO node | INSERT_FORKS |
| `/fork_height` | ESP32 #3 | LIFT, CONFIRM |
| `/fork_load` | ESP32 #3 (optional) | CONFIRM |
| Nav2 NavigateToPose result | Action server | APPROACH, NAVIGATE_TO_DROP |

**Open questions for the team:**
- What is the "ready to insert" distance threshold for `final_z_m`?
- How do we detect pallet-dropped (for DROP state)?
- What happens if YOLO loses the pallet mid-approach (recovery behavior)?

---

## 10. Layer 5 — WMS & IoT Integration `[NEW IN 2027]`

### 10.1 Architecture

```
WMS Server                     Jetson Xavier
+------------------+           +-------------------------------------------+
| Task queue       |           |  wms_bridge_node.py                        |
| Mission dispatch |<--------->|  - Subscribes to WMS task channel          |
| Status receive   |  MQTT     |  - Converts to ROS mission msg             |
| Completion log   |  or WS    |  - Publishes /wms/mission                  |
+------------------+           |  - Subscribes to /robot/status             |
                                |  - Reports to WMS on completion            |
                                +-------------------------------------------+
```

### 10.2 Protocol `[OPEN DECISION D3]`

| Option | Pros | Cons |
|--------|------|------|
| **MQTT** | Lightweight, async, pub-sub, standard IoT | Needs broker (Mosquitto) |
| **WebSocket** | Bidirectional, easy GUI integration | More complex server |
| **REST API** | Simple, stateless | Not real-time, polling required |

**Recommendation:** Start with MQTT (simpler, well-tested, easy to debug with `mosquitto_pub`/`sub`).

### 10.3 Mission Message Format (Draft)

WMS sends to robot:
```json
{
  "mission_id": "M-2027-001",
  "type": "PICKUP_TRANSPORT",
  "pickup_location": "A1",
  "dropoff_location": "B2",
  "priority": 1,
  "timestamp": "2027-01-15T10:30:00Z"
}
```

Robot reports to WMS:
```json
{
  "mission_id": "M-2027-001",
  "status": "COMPLETED",
  "robot_id": "WAFL-01",
  "timestamp": "2027-01-15T10:34:22Z",
  "notes": ""
}
```

### 10.4 TCP Camera Streaming

```
Kinect RGB frame -> camera_stream_node.py
    |
    |  TCP socket (JPEG compressed, ~15 fps)
    v
GUI Dashboard (live video panel)
```

Simple implementation: OpenCV `imencode` -> socket send -> GUI `cv2.imdecode`.

---

## 11. Layer 6 — GUI Dashboard `[NEW IN 2027]`

### 11.1 Technology `[OPEN DECISION D4]`

| Option | Pros | Cons |
|--------|------|------|
| **PySide6 / PyQt6** | Python, easy ROS integration, fast to build | Less polished 3D map |
| **QML (Qt/C++)** | Native performance, professional, project spec mentions it | Steeper learning curve |
| **Web app (React + rosbridge)** | Any browser, easy to share | Needs rosbridge server |

**Recommendation:** PySide6 first (fastest path). Upgrade to QML if time and skill permit.

### 11.2 Dashboard Layout

```
+------------------------------------------------------------------+
|  WAFL 2027 — Operator Dashboard              [WMS: CONNECTED]    |
+------------------------+---------------------+-------------------+
|   WAREHOUSE MAP        |   MISSION QUEUE     |   ROBOT STATUS   |
|                        |                     |                   |
|  [Live 2D/3D map      |  ACTIVE:            |  Battery: 87%    |
|   with robot icon     |    M-001: A1 -> B2  |  [=========. ]   |
|   and route path]     |                     |                   |
|                        |  PENDING:           |  Speed: 0.35 m/s |
|  [Shelves and pallets |    M-002: C3 -> A1  |  State: NAVIGATE  |
|   in the world]       |                     |                   |
|                        |  DONE:              |  Fork: 0 cm      |
|                        |    M-000: B1 -> D4  |                   |
|                        |                     |  [E-STOP] (RED)  |
+------------------------+---------------------+                   |
|  FRONT CAMERA (TCP)    |  REAR CAMERA (TCP)  |  [Override]      |
|  [live feed]           |  [safety cam]       +-------------------+
+------------------------+---------------------+
|  ALARMS & EVENTS                                                  |
|  [10:34] INFO  Mission M-001 started                             |
|  [10:35] WARN  Low battery — approaching threshold               |
|  [10:36] INFO  Pallet detected at A1 (confidence: 94%)          |
+------------------------------------------------------------------+
```

### 11.3 ROS Interface for GUI

```
Subscriptions:
  /map                        (OccupancyGrid — SLAM map)
  /robot/pose                 (PoseWithCovarianceStamped)
  /battery/state              (sensor_msgs/BatteryState)
  /robot/state                (String — current state machine state)
  /alarms                     (custom Alarm.msg)
  /pallet/debug_image         (CompressedImage — YOLO overlay)

Publications:
  /e_stop                     (Bool — emergency stop)
  /manual_cmd_vel             (Twist — joystick override)
  /wms/manual_mission         (String JSON — inject mission manually)
```

---

## 12. Layer 7 — Energy & Autonomous Charging `[NEW IN 2027]`

### 12.1 Battery Monitoring

New node: `battery_monitor_node.py`
- Reads voltage/current from BMS hardware (I2C to Xavier, or via ESP32 analog read)
- Publishes `sensor_msgs/BatteryState` to GUI + mission manager

```
BMS (hardware) -> battery_monitor_node -> /battery/state
                                       -> /battery/low_alert (Bool)
```

### 12.2 Charging Dock — Autonomous Docking `[OPEN DECISION D7]`

**Approach options:**

| Option | How | Pros | Cons |
|--------|-----|------|------|
| **ArUco on dock** | Detect ArUco ID=99 -> use fork insertion pipeline to align | Reuses existing perception | Dock must be lit |
| **IR beacon** | Sensor on robot detects IR on dock | Simple, reliable | Extra hardware |
| **Known pose in map** | Nav2 navigates to fixed coordinates of dock | No extra hardware | Dock must not move |

**Recommended:** ArUco on dock (reuses existing pipeline). A dedicated ArUco ID (e.g., ID=99) triggers docking sequence instead of fork insertion.

### 12.3 Mission Manager Integration

```
LOW BATTERY detected (/battery/low_alert = true)
    |
    v
mission_manager_node:
  1. Pause current mission (if safe — robot not mid-insertion)
  2. Save mission state to file
  3. Navigate to charging dock (Nav2 goal)
  4. Dock and charge (until SoC > 80%)
  5. Undock
  6. Resume saved mission from saved state
```

---

## 13. Complete ROS 2 Topic Graph (Target)

```
ESP32 #1:
  /encoder_count -> encoder_velocity_node -> /wheel_velocity
  /pwm_left  <- [motion controller, TBD]
  /pwm_right <-

ESP32 #2:
  /imu/data -> EKF (robot_localization) -> /odometry/filtered
  /target_angle <- caster_controller

ESP32 #3:
  /lift_cmd    <- fork_insertion_node
  /lights_cmd  <- mission_manager_node
  /fork_height -> fork_insertion_node

Kinect RGB:
  -> pallet_pose_node -> /pallet_pose
                      -> /pallet_location_id

Kinect RGB+D:
  -> pallet_front_angle_node -> /pallet/rotate_to_front_rad
                             -> /pallet/center_yaw_rad
                             -> /pallet/final_z_m

RPLiDAR:
  /scan -> slam_toolbox -> /map
        -> EKF          -> /odometry/filtered
        -> AMCL [optional]

/odometry/filtered -> Nav2 -> /cmd_vel
/map               -> Nav2

/cmd_vel -> caster_controller -> /target_angle      (hardware mode)
                              -> /castor_cmd_pos    (simulation mode)

WMS Server -> wms_bridge_node -> /wms/mission
/robot/status -> wms_bridge_node -> WMS Server

/wms/mission        -> mission_manager_node
/battery/low_alert  -> mission_manager_node
/pallet_location_id -> fork_insertion_node (trigger)
mission_manager_node -> fork_insertion_node (start signal)
mission_manager_node -> Nav2 NavigateToPose action

battery_monitor_node -> /battery/state -> GUI
camera_stream_node   -> TCP:PORT       -> GUI (live feed)
```

---

## 14. Repository Structure (Target)

```
WAFL_2027/
|-- LAUNCH.md                        <- Start here: how to run the sim
|-- WAFL2027_Architecture.md         <- This file
|
|-- docker/
|   |-- Dockerfile                   <- Ubuntu 24.04 + ROS 2 Jazzy + all deps
|   |-- docker-compose.yml           <- One command brings up everything
|   `-- entrypoint.sh
|
|-- src/                             <- Single colcon workspace
|   |
|   |-- wafl_description/            <- URDF, meshes, launch for robot model
|   |   |-- urdf/
|   |   |-- meshes/
|   |   `-- launch/view_urdf.launch.py
|   |
|   |-- wafl_simulation/             <- Gazebo Harmonic world + spawn + bridge
|   |   |-- worlds/
|   |   |-- launch/gazebo.launch.py
|   |   `-- config/ros_gz_bridge.yaml
|   |
|   |-- wafl_localization/           <- EKF + SLAM + AMCL
|   |   |-- config/
|   |   |   |-- ekf.yaml
|   |   |   `-- slam_toolbox.yaml
|   |   `-- launch/
|   |       |-- localization.launch.py
|   |       `-- slam_mapping.launch.py
|   |
|   |-- wafl_navigation/             <- Nav2 + caster controller
|   |   |-- config/nav2_params.yaml
|   |   |-- launch/navigation.launch.py
|   |   `-- wafl_navigation/
|   |       `-- caster_controller.py (UNIFIED: sim + hardware)
|   |
|   |-- wafl_perception/             <- ArUco + YOLO + rear safety
|   |   |-- launch/
|   |   |   |-- aruco.launch.py
|   |   |   |-- yolo.launch.py
|   |   |   `-- perception.launch.py
|   |   |-- models/                  <- TensorRT .engine files (NOT in git)
|   |   `-- wafl_perception/
|   |       |-- pallet_pose_node.py
|   |       |-- pallet_front_angle_node.py
|   |       `-- rear_safety_node.py   (NEW)
|   |
|   |-- wafl_fork_insertion/         <- NEW: fork automation state machine
|   |   |-- launch/
|   |   `-- wafl_fork_insertion/
|   |       `-- fork_insertion_node.py
|   |
|   |-- wafl_mission/                <- NEW: top-level mission manager + WMS
|   |   |-- config/
|   |   |   `-- locations.yaml       <- pallet location DB (replaces hardcoded dict)
|   |   `-- wafl_mission/
|   |       |-- mission_manager_node.py
|   |       `-- wms_bridge_node.py
|   |
|   |-- wafl_energy/                 <- NEW: battery monitor + charging dock
|   |   `-- wafl_energy/
|   |       |-- battery_monitor_node.py
|   |       `-- charging_dock_node.py
|   |
|   `-- wafl_interfaces/             <- Custom ROS 2 msg/srv/action definitions
|       |-- msg/
|       |   |-- WmsMission.msg
|       |   |-- RobotStatus.msg
|       |   `-- Alarm.msg
|       |-- srv/
|       |   `-- SetMission.srv
|       `-- action/
|           `-- InsertForks.action
|
|-- firmware/                        <- ESP32 Arduino sketches
|   |-- esp32_driving/
|   |   `-- Driving_uROS.ino
|   |-- esp32_steering_imu/          <- ESP32 #2 with IMU (UPGRADED)
|   |   `-- Steering_IMU_uROS.ino
|   `-- esp32_lift_lights/
|       `-- Lifting_aux_uROS.ino
|
|-- dashboard/                       <- GUI operator dashboard
|   |-- main.py                      <- PySide6 / QML entry point
|   |-- requirements.txt
|   `-- assets/
|
|-- bringup/                         <- Master launch files
|   |-- simulation.launch.py         <- Full sim: Gazebo + Nav2 + SLAM + Perception
|   |-- hardware.launch.py           <- Full hardware bringup
|   `-- mapping.launch.py            <- SLAM mapping session
|
`-- docs/
    |-- images/
    `-- videos/
```

---

## 15. Simulation — Gazebo Harmonic

### 15.1 Why Gazebo Harmonic

| | Ignition Fortress (2026) | Gazebo Harmonic (2027) |
|--|--|--|
| ROS 2 support | Humble | Jazzy (official) |
| Plugin API | gz-sim6 | gz-sim8 |
| `ros_gz` bridge | ros_humble_ros_gz | ros_jazzy_ros_gz |
| Maintenance | LTS until 2026 | Active development |

### 15.2 What the Simulation Must Include

- [ ] Forklift URDF ported to Gazebo Harmonic (updated plugin namespaces: `gz::sim::systems::*`)
- [ ] Warehouse world with shelves, aisles, pallets with ArUco markers on them
- [ ] Simulated RPLiDAR plugin (`GpuLidarSensor`)
- [ ] Simulated Kinect v1 RGB+D plugin (`DepthCameraSensor` + `CameraSensor`)
- [ ] Simulated IMU plugin (replacing phone IMU — `ImuSensor`)
- [ ] Simulated charging dock (ArUco ID=99 on the dock model)
- [ ] DiffDrive plugin for odometry (odom_covariance_fix still needed in sim)

### 15.3 One-Command Launch Goal

```bash
# Docker (recommended):
docker compose up -d

# Or native:
ros2 launch bringup simulation.launch.py
```

This single command starts: Gazebo + SLAM + Nav2 + Perception + Mission Manager + WMS bridge.

---

## 16. Known Gaps from 2026 — Status in 2027

| 2026 Gap | 2026 Impact | 2027 Resolution |
|----------|------------|-----------------|
| `/castor_cmd_pos` != `/target_angle` | Caster not wired to real hardware | FIXED: unified caster_controller with `use_sim_time` param |
| Phone IMU — fragile UDP | Robot needs a phone | FIXED: proper IMU on ESP32 #2 |
| Location DB hardcoded in two Python files | Must edit two files per change | FIXED: `locations.yaml` as ROS parameter |
| No fork-insertion sequence | Full autonomy impossible | FIXED: `fork_insertion_node.py` state machine |
| Pre-built static map required | Manual mapping needed | ADDRESSED: SLAM Toolbox live mapping |
| No WMS integration | No mission dispatch | FIXED: `wms_bridge_node.py` |
| No battery monitoring | Robot can die mid-mission | ADDRESSED: `battery_monitor_node.py` |
| PyQt5 basic manual dashboard | Limited operator visibility | FIXED: full operator dashboard |
| WiFi credentials hardcoded in firmware | Must reflash on network change | ADDRESSED: move to `config.h` header |
| Documentation folder empty | No design docs | FIXED: this document |
| Two separate workspaces (painful builds) | Build complexity, two source steps | FIXED: single `src/` colcon workspace |
| AMCL DifferentialMotionModel for non-diff-drive | Localization drift in turns | ADDRESSED: SLAM Toolbox localization mode |

---

## 17. Open Design Decisions Tracker

> These must be resolved before implementation begins. Assign an owner and deadline.

| # | Decision | Options | Owner | Status |
|---|----------|---------|-------|--------|
| D1 | SLAM algorithm | SLAM Toolbox vs Cartographer vs LIO-SAM | — | OPEN |
| D2 | Localization mode during nav | SLAM-only vs SLAM + AMCL | — | OPEN |
| D3 | WMS protocol | MQTT vs WebSocket vs REST | — | OPEN |
| D4 | GUI technology | PySide6 vs QML vs Web app | — | OPEN |
| D5 | WMS server | Custom (FastAPI) vs existing (Odoo) vs stub only | — | OPEN |
| D6 | Nav2 local planner | DWB vs MPPI vs Regulated Pure Pursuit | — | OPEN |
| D7 | Charging dock detection | ArUco vs IR beacon vs known fixed pose | — | OPEN |
| D8 | Dual encoders for odometry | Add right-wheel encoder vs rely on IMU for yaw | — | OPEN |
| D9 | Fork height sensing | Potentiometer vs encoder vs limit-switches only | — | OPEN |
| D10 | Load cell | Add for pallet confirmation vs skip | — | OPEN |
| D11 | Camera streaming format | MJPEG over TCP vs WebRTC vs ROS CompressedImage | — | OPEN |

---

## 18. Implementation Roadmap (Suggested)

### Phase 1 — Foundation (Weeks 1–3)
- [ ] Set up repo structure (single `src/` workspace)
- [ ] Update Docker to Ubuntu 24.04 + Jazzy + Harmonic
- [ ] Port URDF to Gazebo Harmonic (gz-sim8 plugins)
- [ ] Get robot spawning and controllable in Gazebo Harmonic
- [ ] Port Nav2 stack to Jazzy
- [ ] Verify `docker compose up` works end-to-end

### Phase 2 — SLAM & Localization (Weeks 3–5)
- [ ] Integrate SLAM Toolbox in simulation
- [ ] IMU sim -> EKF -> SLAM pipeline working
- [ ] Resolve D1 + D2 (SLAM algorithm + localization mode)
- [ ] Test Nav2 with live SLAM map
- [ ] Robot navigates autonomously in simulation

### Phase 3 — Perception & Fork Insertion (Weeks 5–8)
- [ ] Port ArUco node to Jazzy + Harmonic camera topics
- [ ] Port YOLO node to Jazzy
- [ ] Implement `fork_insertion_node.py` state machine
- [ ] Simulate full pickup: detect -> approach -> align -> insert -> lift

### Phase 4 — WMS & GUI (Weeks 8–11)
- [ ] Resolve D3 (WMS protocol) and D4 (GUI tech)
- [ ] Build WMS stub server (simple task queue)
- [ ] Implement `wms_bridge_node.py`
- [ ] Implement `mission_manager_node.py`
- [ ] Build GUI dashboard (map, missions, camera, battery, alarms)
- [ ] Test full end-to-end: WMS dispatch -> robot picks pallet -> reports done

### Phase 5 — Hardware Integration (Weeks 10–14)
- [ ] Flash updated firmware to all 3 ESP32s
- [ ] Replace phone IMU with hardware IMU on ESP32 #2
- [ ] Test real robot with new unified caster_controller
- [ ] Tune EKF with real IMU data
- [ ] Test fork insertion on real hardware

### Phase 6 — Energy & Hardening (Weeks 13–16)
- [ ] Implement BMS monitoring node
- [ ] Implement charging dock detection + docking sequence
- [ ] Test low-battery -> dock -> resume mission flow
- [ ] Full warehouse mission tests end-to-end on real robot
- [ ] Fix all remaining bugs, optimize performance

---

## 19. Simulation vs Hardware Reference

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Compute | Developer laptop (Docker) | Jetson Xavier AGX |
| OS | Ubuntu 24.04 (host or Docker) | Ubuntu 24.04 (Jetson) |
| ROS | Jazzy | Jazzy |
| Simulator | Gazebo Harmonic | — |
| LiDAR | Simulated GpuLidarSensor | Real RPLiDAR A2M8 |
| Camera | Simulated DepthCameraSensor + CameraSensor | Real Kinect v1 |
| IMU | Simulated ImuSensor plugin | MPU-6050 / BNO055 on ESP32 #2 |
| Odometry | Gazebo DiffDrive -> odom_covariance_fix | encoder_velocity_node + odom_fusion_node |
| Caster control | `/castor_cmd_pos` (Gazebo joint) | `/target_angle` (ESP32 #2) |
| Driving | Gazebo DiffDrive plugin | `/pwm_left`, `/pwm_right` (ESP32 #1) |
| Lift | Simulated prismatic joint | `/lift_cmd` (ESP32 #3) |
| WMS | Local stub server | Real WMS server (networked) |
| Charging dock | Simulated ArUco marker in world | Real physical dock with ArUco |
| `use_sim_time` | `true` | `false` |

---

## 20. Team Rules

1. **Never commit to `main`** — use feature branches (`feat/fork-insertion`, `feat/wms-bridge`)
2. **Every new package needs a `README.md`** — what it does, what topics it uses, how to run it
3. **Update this document** when a design decision is resolved (update the tracker in §17)
4. **No hardcoded IPs, passwords, or credentials** — use `.env` files or ROS parameters
5. **All launch files must support `use_sim_time` parameter** so they work in both sim and hardware
6. **Test in simulation first** before touching real hardware
7. **Run `colcon build` before pushing** — broken builds block the whole team

---

## Appendix A — ROS 2 Jazzy vs Humble Key Differences

| Topic | Humble | Jazzy |
|-------|--------|-------|
| Gazebo | Ignition Fortress (gz-sim6) | Gazebo Harmonic (gz-sim8) |
| Bridge package | `ros_gz` (Humble) | `ros_gz` (Jazzy) |
| Nav2 BT plugins | Some deprecated APIs | Updated, new plugins available |
| Python version | 3.10 | 3.12 |
| `ament_cmake_python` | stable | stable |
| Lifecycle nodes | Same API | Same API |
| DDS | FastDDS | FastDDS (Zenoh available) |

---

## Appendix B — Glossary

| Term | Definition |
|------|-----------|
| SLAM | Simultaneous Localization And Mapping — build a map while knowing where you are in it |
| EKF | Extended Kalman Filter — fuses sensor data (IMU + encoders) into smooth odometry |
| AMCL | Adaptive Monte Carlo Localization — particle filter localizing in a known map |
| Nav2 | ROS 2 navigation framework (path planning + obstacle avoidance + behavior trees) |
| DWB | Dynamic Window Approach B — local path planner in Nav2 |
| MPPI | Model Predictive Path Integral — smoother local planner alternative to DWB |
| micro-ROS | ROS 2 for microcontrollers (ESP32) communicating via UDP bridge |
| TensorRT | NVIDIA inference engine — accelerates YOLO on Jetson GPU |
| ArUco | Square fiducial markers for 6-DOF pose estimation via camera |
| BMS | Battery Management System |
| SoC | State of Charge — battery percentage |
| WMS | Warehouse Management System — dispatches transport missions to the robot |
| QML | Qt Modeling Language — declarative UI for Qt |
| DDS | Data Distribution Service — the middleware ROS 2 uses for communication |

---

*Document created: September 2026 — Antigravity x WAFL 2027 Team*
*Last updated: See git log*
