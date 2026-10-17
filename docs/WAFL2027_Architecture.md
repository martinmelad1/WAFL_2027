# WAFL 2027 — Master Architecture & Design Document

> **Project:** Warehouse Autonomous Fork-Lift — Ain Shams University, Mechatronics & Robotics Engineering 2026/2027
> **Status:** 🟡 Phase 1 in progress — Simulation + GUI + Manual Control. Living document; sections marked `[OPEN]` are pending decisions.
> **Stack:** ROS 2 Jazzy · Gazebo Harmonic · Jetson Xavier AGX · micro-ROS · Python 3.12 · YOLO / TensorRT · FastAPI + HTML/JS (web GUI)
> **Authority:** [`README.md`](../README.md) is the single source of truth for nodes, topics, interfaces and decisions. This document holds the long-form rationale and history. **README wins on any conflict.**

---

## Preface — What This Document Is

This document is the **long-form architecture rationale** for WAFL 2027. Its job is to:

1. Capture everything learned from WAFL 2025 and WAFL 2026
2. Explain *why* the architecture in `README.md` looks the way it does, so any team member can understand the whole system
3. Track design decisions and the reasoning behind them (the authoritative table is README §9)
4. Give the bigger picture behind the per-node specs in `README.md` and the per-package READMEs

**Read the README first, then this document for the "why". When a design decision is resolved, update README §9 first, then this document.**

---

## 1. The Big Picture

WAFL 2027 is a **fully autonomous, WMS-connected warehouse forklift** built on a single Jetson Xavier AGX. It:

- Receives transport missions from a **Warehouse Management System (WMS)**
- **Localizes itself** in a live-built map (SLAM — no pre-built map required)
- **Plans and executes** collision-free routes through the warehouse using Nav2
- **Identifies and aligns** with pallets using a **single front-facing camera** (Kinect v1: ArUco markers + YOLO)
- **Autonomously inserts forks, lifts the pallet, transports it, and places it** at the destination
- **Reports mission status** back to the WMS in real time
- Monitors its own **battery** and navigates to a charging dock when needed
- Is operated and monitored from **any web browser** — a 3-tab dashboard (Robot · Logs · WMS) in HTML/JS served by a **FastAPI** back-end running on the robot

This is a **continuation and major upgrade** of WAFL 2026. The foundation (robot chassis, ESP32 firmware, ROS 2 navigation stack, perception pipeline) carries over. The new systems added in 2027: WMS integration, full fork-insertion automation, SLAM (live mapping), web GUI dashboard, and autonomous charging.

---

## 2. System Architecture Overview

```
╔══════════════════════════════════════════════════════════════════════════╗
║                             WAFL 2027 SYSTEM                             ║
╠══════════════════════════════════════════════════════════════════════════╣
║  EXTERNAL LAYER (off-robot)                                              ║
║  ┌───────────────────────┐                 ┌───────────────────────────┐ ║
║  │ WMS Server            │                 │ Operator browser HTML/JS  │ ║
║  │ - mission queue       │                 │ tabs: ROBOT | LOGS | WMS  │ ║
║  │ - package DB          │                 │ - map + pose + route      │ ║
║  │ REST (P1) / MQTT (P2) │                 │ - live MJPEG camera       │ ║
║  └───────────┬───────────┘                 │ - E-STOP + joystick       │ ║
║              │                             │ - logs + node health      │ ║
║              │ REST / MQTT                 │ - packages + missions     │ ║
║              │                             └─────────────┬─────────────┘ ║
║              │                                           │ HTTP/WS/MJPEG ║
║              │ Wi-Fi                                     │ Wi-Fi         ║
║  ┌───────────▼───────────────────────────────────────────▼────────────┐  ║
║  │ ON-ROBOT LAYER (Jetson Xavier AGX)                                 │  ║
║  │ ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐  │  ║
║  │ │ WMS Bridge        │ │ Mission Manager   │ │ Dashboard Server  │  │  ║
║  │ │ wms_bridge_node   │ │ top-level FSM     │ │ FastAPI + rclpy   │  │  ║
║  │ │ WMS <-> ROS       │ │ IDLE->...->CHARGE │ │ REST/WS/MJPEG     │  │  ║
║  │ └───────────────────┘ └───────────────────┘ └───────────────────┘  │  ║
║  │ ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐  │  ║
║  │ │ SLAM + Localiz.   │ │ Navigation        │ │ Perception        │  │  ║
║  │ │ SLAM Toolbox      │ │ Nav2 (RPP / DWB)  │ │ ArUco + YOLO      │  │  ║
║  │ │ robot_loc. EKF    │ │ base_controller   │ │ TensorRT          │  │  ║
║  │ └───────────────────┘ └───────────────────┘ └───────────────────┘  │  ║
║  │ ┌───────────────────┐ ┌───────────────────┐ ┌───────────────────┐  │  ║
║  │ │ Fork Insertion    │ │ Camera Stream     │ │ Energy            │  │  ║
║  │ │ state machine     │ │ JPEG ~15 fps      │ │ battery monitor   │  │  ║
║  │ │ align -> insert   │ │ -> dashboard      │ │ charging dock     │  │  ║
║  │ └───────────────────┘ └───────────────────┘ └───────────────────┘  │  ║
║  │ ┌────────────────────────────────────────────────────────────────┐ │  ║
║  │ │               micro-ROS Agent (UDP :8888)                      │ │  ║
║  │ └────────┬───────────────────────┬───────────────────────┬───────┘ │  ║
║  └──────────┼───────────────────────┼───────────────────────┼─────────┘  ║
║             │ Wi-Fi (micro-ROS)     │                       │            ║
║  ┌──────────▼──────────┐ ┌──────────▼──────────┐ ┌──────────▼──────────┐ ║
║  │ ESP32 #1 DRIVING    │ │ ESP32 #2 STEER+IMU  │ │ ESP32 #3 LIFT       │ ║
║  │ 2x DC motors + PID  │ │ Stepper (TB6600)    │ │ Lift DC motor       │ ║
║  │ encoder (left)      │ │ MPU-6050 / BNO055   │ │ 2x limit switches   │ ║
║  │ /target_speed_left  │ │ /target_angle       │ │ /lift_cmd           │ ║
║  │ /target_speed_right │ │ /imu/data           │ │ /lights_cmd         │ ║
║  │ /encoder_count      │ │                     │ │ /fork_height        │ ║
║  └─────────────────────┘ └─────────────────────┘ └─────────────────────┘ ║
╚══════════════════════════════════════════════════════════════════════════╝
```

---

## 3. What Changed from 2026 to 2027

| Area | WAFL 2026 | WAFL 2027 |
|------|-----------|-----------|
| **OS / ROS** | Ubuntu 22.04 + ROS 2 Humble | Ubuntu 24.04 + ROS 2 Jazzy |
| **Simulator** | Ignition Gazebo (Fortress) | Gazebo Harmonic |
| **Compute** | 2 laptops + Jetson Nano | 1 Jetson Xavier AGX (all on-robot) |
| **IMU** | Phone app over UDP (fragile) | Proper IMU on ESP32 #2 (MPU-6050 / BNO055) |
| **Mapping** | Pre-built static map (.yaml + .pgm) | Live SLAM (SLAM Toolbox, async) |
| **Fork insertion** | Manual — human operates lift | Automated state machine (approach → align → insert → lift) |
| **WMS** | None | WMS integration: mission dispatch + status reporting (REST stub in P1, MQTT in P2) |
| **GUI** | PyQt5 manual dashboard (basic) | Web dashboard: FastAPI + HTML/JS, 3 tabs (Robot · Logs · WMS), MJPEG camera |
| **Charging** | Manual | Autonomous charging dock detection (ArUco ID 99) + alignment + docking |
| **Repo structure** | Split workspaces (Localization/Simulation + Navigation/Simulation) | Unified single `src/` workspace |
| **Docker** | Optional Compose wrapper | First-class: one `docker compose up` starts the sim stack |
| **Battery** | No monitoring | BMS: SoC estimation, low-battery alert, charging trigger |

---

## 4. Compute Layout — Jetson Xavier

In 2027, a **single Jetson Xavier AGX** runs the entire on-robot software stack. There are no separate laptops; the operator only needs a browser.

```
Jetson Xavier AGX
├── [Process] micro_ros_agent        <- UDP :8888 bridge to all 3 ESP32s (start first)
├── [Process] rplidar_ros / kinect   <- sensor drivers (/scan, /camera/*)
├── [Process] robot_state_publisher  <- URDF TF tree
├── [Process] encoder_velocity_node  <- ticks -> /wheel_velocity
├── [Process] odom_fusion_node       <- wheel velocity + IMU yaw -> /odom
├── [Process] ekf_node               <- robot_localization EKF (odom -> base_footprint)
├── [Process] slam_toolbox           <- live mapping / localization (map -> odom)
├── [Process] nav2_stack             <- planner, controller (RPP), costmaps, BT navigator
├── [Process] base_controller        <- cmd arbitration -> wheel speeds + caster angle
├── [Process] pallet_pose_node       <- Kinect RGB -> ArUco
├── [Process] pallet_front_angle_node<- Kinect RGB+D -> YOLO TensorRT
├── [Process] fork_insertion_node    <- state machine for pallet approach/lift
├── [Process] mission_manager_node   <- top-level state machine (IDLE->MISSION->CHARGE)
├── [Process] wms_bridge_node        <- communicates with WMS server
├── [Process] camera_stream_node     <- JPEG CompressedImage ~15 fps -> dashboard_server
├── [Process] battery_monitor_node   <- SoC estimation, low-battery alert
├── [Process] charging_dock_node     <- ArUco ID 99 docking (Phase 2)
└── [Process] dashboard_server       <- FastAPI + rclpy: REST / WebSocket / MJPEG on :8000
```

> **Why one Xavier?** Eliminates the inter-laptop Wi-Fi dependency for ROS topics that caused latency and sync issues in 2026. All nodes on one machine = local DDS, and the only Wi-Fi traffic is HTTP/WebSocket/MJPEG to the browser plus micro-ROS to the ESP32s.

> **Risk:** If the Xavier overheats or crashes, the entire system is down.
> **Mitigation:** Thermal monitoring + watchdog on all critical processes; command timeouts on `base_controller` and the ESP32s stop the robot if commands stop arriving; a hardware e-stop is required (D15).

---

## 5. Software Stack

| Layer | Technology | Version |
|-------|-----------|---------|
| OS | Ubuntu | 24.04 LTS (Noble) |
| ROS | ROS 2 | Jazzy Jalisco |
| Simulator | Gazebo | Harmonic (ros-jazzy-ros-gz) |
| Navigation | Nav2 | Jazzy-compatible |
| Localization | robot_localization (EKF) + SLAM Toolbox | latest |
| Perception | OpenCV ArUco + YOLO + TensorRT | latest |
| Firmware bridge | micro-ROS | Jazzy agent |
| GUI | Web: HTML5 + CSS + vanilla JS front-end, FastAPI + `rclpy` back-end (D4) | — |
| WMS protocol | Phase 1: HTTP REST to FastAPI stub · Phase 2: MQTT (Mosquitto) (D3, D5) | — |
| Language | Python 3.12 (nodes, dashboard) + C++ (Arduino/ESP32) | — |
| Build system | colcon (ament_python + ament_cmake) | — |
| Container | Docker + Docker Compose (`network_mode: host`) | latest |

---

## 6. Layer 1 — Low-Level Firmware (ESP32 + micro-ROS)

> **Status:** Carry over from 2026 with targeted upgrades. Full topic contract: README §4.1.

All three ESP32s talk to the Jetson through the micro-ROS agent (UDP 8888). Each ESP32 publishes its feedback topic at a **fixed rate even when the value is unchanged** — the dashboard uses these topics as heartbeats. Each ESP32 also applies a command timeout: if no setpoint arrives for 0.5 s it stops.

### 6.1 ESP32 #1 — Driving

The ESP32 **closes the speed loop (PID) itself** and owns the tick→speed conversion. The Jetson sends wheel speed setpoints in m/s; there is no PID on the Jetson.

Candidate improvements:
- Add a **second encoder** for the right wheel → proper differential odometry and closed-loop control of both wheels (D8). Until then the right wheel runs feed-forward (open loop).
- Expose `/odom_raw` (`nav_msgs/Odometry`) instead of raw `/encoder_count` so the EKF can consume it directly

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/target_speed_left` | Sub | `Float32` | Target speed in m/s; ESP32 PID against encoder |
| `/target_speed_right` | Sub | `Float32` | Target speed in m/s (open loop until D8) |
| `/encoder_count` | Pub | `Float32` | Raw ticks, left wheel (600 PPR, gear ratio ≈ 3.14) |

### 6.2 ESP32 #2 — Steering + IMU (UPGRADED)

**Key 2027 change:** Replace the phone IMU with a proper IMU chip (MPU-6050 or BNO055) on ESP32 #2. This eliminates:
- The fragile UDP phone-app protocol
- Dependency on a personal phone for the robot to function
- `imu_udp_node.py` (deleted in 2027)

The ESP32 now publishes `sensor_msgs/Imu` messages with real covariances. The EKF consumes this directly.

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/target_angle` | Sub | `Float32` | Caster angle in degrees, from `base_controller` |
| `/imu/data` | Pub | `sensor_msgs/Imu` | **NEW** — replaces `/imu_yaw` Float32 |

> ⚠️ The stepper (TB6600) has no position feedback — it needs a home switch or potentiometer at boot so `/target_angle` is absolute (D16). The MPU-6050 has no magnetometer, so yaw drifts — BNO055 recommended (D19).

### 6.3 ESP32 #3 — Lift + Lights

Additions for 2027:
- `/fork_height` publisher (potentiometer or encoder on the lift, D9) — **required** for fork-insertion automation
- `/fork_load` topic for weight sensing (only if the mechanical team adds a load cell, D10)

| Topic | Direction | Type | Notes |
|-------|-----------|------|-------|
| `/lift_cmd` | Sub | `Int32` | 0=STOP, 1=UP, 2=DOWN |
| `/lights_cmd` | Sub | `Int32MultiArray` | 4-element array, 0/1 per relay |
| `/fork_height` | Pub | `Float32` | **NEW** — height in mm |

**Single-writer rule:** the GUI publishes `/lift_cmd` / `/lights_cmd` only in MANUAL; `fork_insertion_node`, `charging_dock_node` (lift) and `mission_manager_node` (lights) only in AUTO.

---

## 7. Layer 2 — Localization & Mapping

### 7.1 IMU Pipeline (Revised)

```
Hardware:
ESP32 #2 (MPU-6050 / BNO055)
    |  micro-ROS (Wi-Fi UDP)
    v
/imu/data  (sensor_msgs/Imu, real covariances;
            BNO055: fused 9-DOF orientation · MPU-6050: 6-DOF, yaw drifts)
    |-> odom_fusion_node (yaw)
    |-> EKF (robot_localization)

Simulation:
Gazebo ImuSensor -> /imu -> odom_covariance_fix -> /imu_fixed -> EKF
```

The old chain `phone -> UDP -> imu_udp_node -> /imu_yaw` is **removed**.

### 7.2 Odometry Pipeline (Revised)

```
Hardware:
/encoder_count (ESP32 #1)
    v
encoder_velocity_node -> /wheel_velocity (Float32, m/s)
    v
odom_fusion_node (+ /imu/data yaw) -> /odom (nav_msgs/Odometry, no TF)
    v
EKF

Simulation:
Gazebo DiffDrive -> /odom -> odom_covariance_fix -> /odom_fixed -> EKF
(odom_fusion_node is not launched in sim)
```

> **Improvement target:** Replace single-encoder `/encoder_count` with dual-encoder differential odometry to get angular velocity from wheels (not just IMU). Makes odometry more robust in turns (D8).

### 7.3 EKF (robot_localization)

Configuration largely unchanged from 2026. Key updates:
- IMU input changes from `/imu_yaw` (Float32, yaw-only) to `/imu/data` (full `sensor_msgs/Imu`; `/imu_fixed` in sim)
- Odometry input uses velocity only (vx, vyaw)
- Proper covariance matrices from IMU chip (no more hardcoded values)
- `two_d_mode: true`, rate 50 Hz
- The EKF is the **sole** publisher of the `odom → base_footprint` TF (Gazebo DiffDrive TF publishing is disabled)

### 7.4 SLAM `[DECIDED D1: SLAM Toolbox async]`

**The 2026 system used a pre-built static map**. WAFL 2027 uses live SLAM so:
1. No manual mapping session required before deployment
2. The map can update as the warehouse changes

**Chosen: SLAM Toolbox (async mapping mode)**
- Native ROS 2 support, actively maintained
- Can serialize/deserialize a map and switch to localization-only mode
- Works with 2D LiDAR (RPLiDAR) out of the box

**Fallback:** if localization is poor in Gazebo → try Cartographer.

**Still to confirm during Phase 1:**
- Run SLAM continuously or map-once then localize?
- Save the map across sessions and reload on next startup?
- Is 2D SLAM sufficient or do shelves block the laser too much?

**SLAM Toolbox interfaces:**
```
/scan (RPLiDAR / Gazebo)          -> slam_toolbox -> /map (OccupancyGrid)
TF odom -> base_footprint (EKF)   ->              -> TF: map -> odom
```
SLAM Toolbox reads odometry through TF, not a topic.

### 7.5 TF Tree — One Publisher per Edge

```
map ──(SLAM Toolbox)──► odom ──(EKF)──► base_footprint ──(robot_state_publisher)──► base_link ──► sensors
```

In sim, `/clock` must be bridged from Gazebo and every node runs with `use_sim_time:=true`. On hardware, nothing publishes `/joint_states` yet, so caster / fork frames are an open issue (D21).

---

## 8. Layer 3 — Navigation

### 8.1 Nav2 Stack

Nav2 configuration upgrades for Jazzy:
- **BT Navigator** (Behavior Tree) — more robust recovery than the simple action calls used in 2026
- **Global planner:** NavFn (default) or SmacPlanner2D
- **Local controller:** **Regulated Pure Pursuit** recommended for aisle driving; DWB is the 2026 baseline; MPPI to be benchmarked `[OPEN D6]`
- Costmap plugins: static + obstacle + inflation (same as 2026)

Nav2 outputs only body velocity `(v, ω)` on `/cmd_vel` and treats the robot as differential drive. The caster is `base_controller`'s job. If the velocity smoother / collision monitor are enabled, their final output must be remapped to `/cmd_vel`.

**Speed targets (TBD by mechanical team):**
```yaml
max_vel_x: 1.0 m/s
max_vel_theta: 1.5 rad/s
```

### 8.2 Base Controller (Unified — 2026 Bug Fixed)

**2026 bug:** The caster controller published to `/castor_cmd_pos` (simulation), which was NOT connected to the real ESP32 (`/target_angle`). In 2027 a single `base_controller` node handles both. It is the only node that commands ESP32 #1 and #2, and it decides which source may command motion:

```
Inputs:  /manual_cmd_vel  /fork_insertion/cmd_vel  /charging_dock/cmd_vel  /cmd_vel
         /gui/mode (MANUAL | AUTO)   /e_stop (latched)

Arbitration: /e_stop -> zero · MANUAL -> /manual_cmd_vel only
             AUTO -> fork / dock cmd_vel if fresh (< 0.2 s), else /cmd_vel (Nav2)

Outputs: /cmd_vel_out (always; Gazebo DiffDrive input in sim)
         sim:=true  -> /castor_cmd_pos (Float64, rad)
         sim:=false -> /target_angle (deg), /target_speed_left, /target_speed_right (m/s)
```

The mode is selected by the **`sim` launch parameter** (independent of the `use_sim_time` clock flag), so **no code changes are needed to switch between sim and hardware**. Startup default is MANUAL; a 0.5 s `cmd_timeout` outputs zero if the selected source goes silent. Wheel-speed and caster-angle formulas: README §4.3.

### 8.3 Localization Mode During Navigation `[DECIDED D2: SLAM Toolbox localization mode]`

| Option | Pros | Cons |
|--------|------|------|
| **SLAM Toolbox localization mode** ✅ | One system does mapping + localization | Less tested in Nav2 |
| **SLAM for mapping + AMCL for nav** (fallback) | AMCL is proven with Nav2 | Two systems to configure |

**Fallback:** if SLAM drifts, switch to SLAM + AMCL.

---

## 9. Layer 4 — Perception

No sensor hardware changes: same RPLiDAR + **one front-facing Kinect v1** (RGB + Depth). **There is no rear camera.** The software pipeline is upgraded.

### 9.1 Pipeline A — ArUco Pose Estimation (`pallet_pose_node`)

Carried over from 2026. Changes:
- Location database moved from a hardcoded Python dict to a **YAML config file** (`wafl_perception/config/locations.yaml`, loaded via the `locations_yaml` param) — fixes the 2026 "hardcoded in two places" issue
- Dictionary **`DICT_4X4_100`** (pallet IDs 0–3, charging dock ID 99), `solvePnP` IPPE_SQUARE
- Support for more than 4 pallet locations (scalable)

```
Kinect RGB (/camera/image_raw) -> pallet_pose_node (ArUco DICT_4X4_100)
    |-> /pallet_pose         (PoseStamped — 6DOF in camera frame)
    |-> /fork_target         (PointStamped — 0.5m in front of marker)
    |-> /pallet_location     (String — "Location_A1")
    |-> /pallet_location_id  (String — marker ID)
    |-> /pallet_marker       (visualization_msgs/Marker — RViz)
```

### 9.2 Pipeline B — YOLO Fine Alignment (`pallet_front_angle_node`, TensorRT on Xavier)

Carried over from 2026. The Jetson Xavier replaces the Jetson Nano — significantly faster inference. CPU YOLO or synthetic mock data in sim.

3-model pipeline (same as 2026):
1. `model_pallet` → full pallet bounding box
2. `model_pocket` → fork pocket inside pallet
3. `model_keypoint` → 4 corner keypoints of pocket → back-project with depth → angles

```
Kinect RGB + Depth -> pallet_front_angle_node
    |-> /pallet/rotate_to_front_rad
    |-> /pallet/center_yaw_rad
    |-> /pallet/final_z_m
    |-> /pallet/pallet_yaw_offset_rad
    |-> /pallet/debug_image   (CompressedImage, YOLO overlay -> dashboard)
```

Averaging (3-second running mean + time-weighted mean) is internal. Only the averaged values are published (D18).

### 9.3 Fork Insertion Automation `[NEW IN 2027]`

**This is the most significant new software component.**

In 2026, the state machine waited for `final_z_m ≈ 0` as a proxy for "pallet lifted by a human." In 2027, the robot does this autonomously.

**State machine:**

```
IDLE -> APPROACH -> ROTATE_ALIGN -> LATERAL_CENTER -> INSERT_FORKS -> LIFT -> CONFIRM -> NAVIGATE_TO_DROP -> DROP -> IDLE
```

Detailed:
```
mission_manager goal -> IDLE
               |
          APPROACH        (coarse Nav2 goal ~1m in front of pallet, then fine drive on /pallet_pose, /fork_target)
               |
          ROTATE_ALIGN    (rotate until /pallet/rotate_to_front_rad ~= 0)
               |
          LATERAL_CENTER  (steer with the caster until /pallet/center_yaw_rad ~= 0 — the robot cannot strafe)
               |
          INSERT_FORKS    (drive forward /pallet/final_z_m via /fork_insertion/cmd_vel)
               |
          LIFT            (/lift_cmd = UP until /fork_height = target OR limit switch)
               |
          CONFIRM         (verify pallet lifted: /fork_height [+ /fork_load if fitted])
               |
          NAVIGATE_TO_DROP (Nav2 goal = dropoff location from WMS mission)
               |
          DROP            (/lift_cmd = DOWN, then reverse clear of pallet)
               |
          IDLE            (result -> mission_manager -> WMS: COMPLETED)
```

> **Open:** who owns NAVIGATE_TO_DROP / DROP — `mission_manager` or `fork_insertion_node` (D13) — and whether the handoff should be the `insert_forks` action only (D12).

All fine motion goes out on `/fork_insertion/cmd_vel` and is arbitrated by `base_controller`. The node stops and publishes `/lift_cmd = 0` on `/e_stop`, and aborts on `/gui/mode = MANUAL`.

**Key inputs to the state machine:**

| Input | Source | Used in state |
|-------|--------|---------------|
| `/pallet_pose`, `/fork_target` | `pallet_pose_node` | APPROACH |
| `/pallet/rotate_to_front_rad` | `pallet_front_angle_node` | ROTATE_ALIGN |
| `/pallet/center_yaw_rad` | `pallet_front_angle_node` | LATERAL_CENTER |
| `/pallet/final_z_m` | `pallet_front_angle_node` | INSERT_FORKS |
| `/fork_height` | ESP32 #3 / `sim_fork_adapter` | LIFT, CONFIRM |
| `/fork_load` | ESP32 #3 (optional, D10) | CONFIRM |
| Nav2 NavigateToPose result | Action server | APPROACH, NAVIGATE_TO_DROP |

**Open questions for the team:**
- What is the "ready to insert" distance threshold for `final_z_m`?
- How do we detect pallet-dropped (for DROP state)?
- What happens if YOLO loses the pallet mid-approach (recovery behavior)?

---

## 10. Layer 5 — WMS & IoT Integration `[NEW IN 2027]`

### 10.1 Architecture

```
WMS Server                         Jetson Xavier
+------------------+               +--------------------------------------------+
| Task queue       |               | wms_bridge_node.py                         |
| Package DB       |<------------->| - Publishes /wms/mission (WmsMission)      |
| Mission dispatch | P1: HTTP REST | - Publishes /wms/mission_status,           |
| Status receive   | P2: MQTT      |   /wms/package_list, /wms/location_grid,   |
| Completion log   |               |   /wms/connection_status                   |
+------------------+               | - Subscribes /robot/status, /robot/state,  |
                                   |   /battery/state, /wms/manual_mission      |
                                   +--------------------------------------------+
```

The WMS server owns package and mission data. The dashboard only reads it through `wms_bridge_node`'s topics, so the dashboard doesn't change when the protocol moves from REST to MQTT.

### 10.2 Protocol `[DECIDED D3 / D5]`

| Option | Pros | Cons |
|--------|------|------|
| **MQTT** ✅ (Phase 2) | Lightweight, async, pub-sub, standard IoT | Needs broker (Mosquitto) |
| **WebSocket** (fallback) | Bidirectional | More complex server |
| **REST API** ✅ (Phase 1 stub) | Simple, stateless, easy FastAPI stub | Not real-time, polling required |

**Decision:** Phase 1 uses HTTP REST against a FastAPI WMS stub (`wms_server/`, port 8080). Phase 2 switches to MQTT (Mosquitto). If the broker is hard to deploy → FastAPI WebSocket.

### 10.3 Mission Message Format (Draft)

WMS sends to robot (mapped to `WmsMission.msg`):
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

Robot reports to WMS (from `RobotStatus.msg`):
```json
{
  "mission_id": "M-2027-001",
  "status": "COMPLETED",
  "robot_id": "WAFL-01",
  "timestamp": "2027-01-15T10:34:22Z",
  "notes": ""
}
```

### 10.4 Camera Streaming `[DECIDED D11]`

```
Kinect RGB (/camera/image_raw) -> camera_stream_node
    |  throttle to 15 fps, resize, cv2.imencode('.jpg', q=70)
    v
/camera/stream/compressed (sensor_msgs/CompressedImage)
    v
dashboard_server -> MJPEG over HTTP (GET /stream/camera) -> browser <img> tag
```

Raw 640×480 RGB at 30 fps is ~27 MB/s, far too heavy for Wi-Fi. `dashboard_server` re-serves the JPEG bytes without re-encoding. The YOLO overlay (`/pallet/debug_image`) is served the same way on `/stream/debug`. If latency or bandwidth becomes a problem → WebRTC.

---

## 11. Layer 6 — GUI Dashboard `[NEW IN 2027]`

### 11.1 Technology `[DECIDED D4]`

| Option | Pros | Cons |
|--------|------|------|
| PySide6 / PyQt6 | Python, easy ROS integration | Needs a ROS install + DDS over Wi-Fi on the operator laptop |
| QML (Qt/C++) | Native performance, professional | Steeper learning curve, same deployment issue |
| **Web: FastAPI + HTML/JS** ✅ | Any browser, no ROS on the operator laptop, no DDS over Wi-Fi, no build step | Need to write a small back-end (REST/WS/MJPEG) |

**Decision:** browser front-end in plain HTML5 + CSS + vanilla JavaScript (no framework, no build step), served by `dashboard_server` — a FastAPI app that is also an `rclpy` node (`wafl_dashboard` package, port 8000). If the front-end outgrows vanilla JS, add a framework; the back-end API stays the same.

**Golden rule:** the browser never talks to ROS directly (no rosbridge). Every topic the GUI uses is subscribed/published by `dashboard_server`.

### 11.2 Dashboard Layout — 3 Tabs

```
┌──────────────────────────────────────────────────────────────────┐
│  WAFL 2027   [ROBOT]  [LOGS]  [WMS]     WS: ● LIVE   MODE: AUTO  │
├──────────────────┬──────────────────┬────────────────────────────┤
│  WAREHOUSE MAP   │  ROBOT STATUS    │  MISSION                   │
│  <canvas>        │  State: NAVIGATE │  Active: M-001  A1→B2      │
│  Live /map       │  Speed: 0.35 m/s │  Queue:                    │
│  Robot icon      │  Fork: 0 mm      │   M-002: C3→A1             │
│  /plan overlay   │  Battery: 87%    │  [Inject Mission]          │
│                  │  [E-STOP] [RESET E-STOP]                      │
│                  │  [RESUME AUTO] [MANUAL MODE]                  │
├──────────────────┴──────────────────┴────────────────────────────┤
│  FRONT CAMERA — MJPEG /stream/camera       [RAW | YOLO overlay]  │
├──────────────────────────────────────────────────────────────────┤
│  MANUAL JOYSTICK (MANUAL mode only, W A S D) · Lift · Lights     │
└──────────────────────────────────────────────────────────────────┘
```

- **Tab 1 — Robot:** map, pose, plan, status, mission, front camera (raw or YOLO overlay), E-STOP / mode buttons, manual joystick, lift and lights.
- **Tab 2 — Logs:** node health (including Nav2 lifecycle), ESP32 heartbeats, WMS connection, live `/rosout` stream with filters.
- **Tab 3 — WMS:** warehouse grid, package database, mission queue and history, new mission form.

Full layouts, endpoints, throttling and implementation rules are in README §5.

### 11.3 ROS Interface for the Dashboard (via `dashboard_server`)

```
Subscriptions:
  /map                         (OccupancyGrid — rendered to PNG, ≤ 1 Hz)
  TF map -> base_footprint     (robot icon, D17)
  /odometry/filtered           (actual speed)
  /plan                        (Path)
  /robot/state                 (String)
  /fork_height                 (Float32)
  /battery/state               (sensor_msgs/BatteryState)
  /wms/mission_status, /wms/package_list, /wms/location_grid, /wms/connection_status
  /camera/stream/compressed    (MJPEG /stream/camera)
  /pallet/debug_image          (MJPEG /stream/debug)
  /rosout                      (log stream, 500-entry ring buffer)
  /fork_insertion/state, /charging_dock/state
  /encoder_count, /imu/data, /fork_height   (ESP32 heartbeats)

Publications:
  /e_stop                      (Bool — latched)
  /gui/mode                    (String MANUAL | AUTO — latched)
  /manual_cmd_vel              (Twist — joystick, MANUAL only, 0.3 s deadman)
  /lift_cmd, /lights_cmd       (MANUAL only)
  /wms/manual_mission          (String JSON — inject mission)
```

> **Not a safety device:** the browser E-STOP goes over Wi-Fi. A hardware e-stop that cuts motor power is still required (D15).

---

## 12. Layer 7 — Energy & Autonomous Charging `[NEW IN 2027]`

### 12.1 Battery Monitoring

Node: `battery_monitor_node.py`
- **Phase 1 stub:** publishes a slowly decaying simulated battery (starts at 100%, configurable drain rate)
- **Phase 2:** reads voltage/current from the BMS hardware (I2C)
- Publishes `sensor_msgs/BatteryState` to the dashboard, `mission_manager_node`, `wms_bridge_node` and `charging_dock_node`

```
BMS (hardware) -> battery_monitor_node -> /battery/state
                                       -> /battery/low_alert (Bool)
```

### 12.2 Charging Dock — Autonomous Docking `[DECIDED D7: ArUco ID 99]`

| Option | How | Pros | Cons |
|--------|-----|------|------|
| **ArUco on dock** ✅ | Detect ArUco ID=99 → align with the perception pipeline | Reuses existing perception | Dock must be lit |
| **IR beacon** (fallback) | Sensor on robot detects IR on dock | Simple, reliable | Extra hardware |
| **Known pose in map** | Nav2 navigates to fixed coordinates of dock | No extra hardware | Dock must not move |

`charging_dock_node` (Phase 2) starts on `/charging_dock/start`, does a coarse Nav2 approach, lowers the forks, aligns on the ID 99 marker (`/pallet_location_id`, `/pallet_pose`), and drives the final approach on `/charging_dock/cmd_vel` through `base_controller`. It reports on `/charging_dock/state` (IDLE | NAVIGATING | ALIGNING | DOCKED | FAILED). If lighting is bad → IR beacon.

### 12.3 Mission Manager Integration

```
LOW BATTERY detected (/battery/low_alert = true)
    |
    v
mission_manager_node:
  1. Pause current mission (if safe — robot not mid-insertion)
  2. Save mission state
  3. /charging_dock/start = true  (state CHARGE)
  4. Dock and charge (until SoC > 80%)
  5. Undock
  6. Resume saved mission (state RESUME)
```

---

## 13. Complete ROS 2 Topic Graph (Target)

> The authoritative per-topic table is the README §7 Master Topic Index.

```
ESP32 #1:
  /encoder_count -> encoder_velocity_node -> /wheel_velocity -> odom_fusion_node -> /odom -> EKF
  /target_speed_left, /target_speed_right <- base_controller (sim:=false)

ESP32 #2:
  /imu/data -> odom_fusion_node, EKF
  /target_angle <- base_controller (sim:=false)

ESP32 #3 (sim: sim_fork_adapter):
  /lift_cmd    <- fork_insertion_node, charging_dock_node (AUTO) · dashboard (MANUAL)
  /lights_cmd  <- mission_manager_node (AUTO) · dashboard (MANUAL)
  /fork_height -> fork_insertion_node, mission_manager_node, dashboard

Simulation:
  Gazebo /odom -> odom_covariance_fix -> /odom_fixed -> EKF
  Gazebo /imu  -> odom_covariance_fix -> /imu_fixed  -> EKF
  Gazebo /clock -> all nodes (use_sim_time)

EKF -> /odometry/filtered (Nav2 velocity, dashboard speed) + TF odom -> base_footprint

RPLiDAR / Gazebo:
  /scan -> slam_toolbox -> /map + TF map -> odom
        -> Nav2 (obstacle layer)

Kinect RGB:
  -> pallet_pose_node -> /pallet_pose, /fork_target            -> fork_insertion_node
                      -> /pallet_pose, /pallet_location_id     -> charging_dock_node
                      -> /pallet_location_id                   -> mission_manager_node
  -> camera_stream_node -> /camera/stream/compressed           -> dashboard (MJPEG)

Kinect RGB+D:
  -> pallet_front_angle_node -> /pallet/rotate_to_front_rad, /pallet/center_yaw_rad,
                                /pallet/final_z_m              -> fork_insertion_node
                             -> /pallet/debug_image            -> dashboard (MJPEG)

/map + TF -> Nav2 -> /cmd_vel, /plan

/manual_cmd_vel, /fork_insertion/cmd_vel, /charging_dock/cmd_vel, /cmd_vel, /gui/mode, /e_stop
  -> base_controller -> /cmd_vel_out                                    (always; Gazebo DiffDrive in sim)
                     -> /target_angle, /target_speed_left, /target_speed_right  (sim:=false)
                     -> /castor_cmd_pos                                 (sim:=true)

WMS Server <-> wms_bridge_node -> /wms/mission, /wms/mission_status, /wms/package_list,
                                  /wms/location_grid, /wms/connection_status
/robot/status, /robot/state, /battery/state, /wms/manual_mission -> wms_bridge_node

/wms/mission, /battery/low_alert, /fork_insertion/state|done, /charging_dock/state
  -> mission_manager_node -> /robot/state, /robot/status, /mission_manager/fork_goal,
                             /charging_dock/start, navigate_to_pose, insert_forks

battery_monitor_node -> /battery/state, /battery/low_alert
/e_stop, /gui/mode (dashboard, latched) -> base_controller, mission_manager_node,
                                           fork_insertion_node, charging_dock_node
/rosout (all nodes) -> dashboard (Tab 2)
```

---

## 14. Repository Structure (Target)

> Mirrors README §3 — update both together.

```
WAFL_2027/
|-- README.md                        <- Single source of truth
|
|-- docker/
|   |-- Dockerfile                   <- Ubuntu 24.04 + ROS 2 Jazzy + all deps
|   |-- docker-compose.yml           <- docker compose -f docker/docker-compose.yml up
|   `-- entrypoint.sh
|
|-- src/                             <- Single colcon workspace
|   |-- wafl_interfaces/             <- BUILD FIRST — custom msg/srv/action
|   |   |-- msg/  (WmsMission, WmsMissionStatus, RobotStatus, Alarm)
|   |   |-- srv/  (SetMission)
|   |   `-- action/ (InsertForks)
|   |
|   |-- wafl_description/            <- URDF, meshes
|   |   |-- urdf/wafl.urdf.xacro
|   |   |-- meshes/
|   |   `-- launch/ (view_urdf.launch.py, rsp.launch.py)
|   |
|   |-- wafl_simulation/             <- Gazebo Harmonic world + spawn + bridge
|   |   |-- wafl_simulation/sim_fork_adapter.py   (sim only)
|   |   |-- worlds/warehouse.world
|   |   |-- models/                  <- pallets with ArUco textures, dock
|   |   |-- launch/gazebo.launch.py
|   |   `-- config/ros_gz_bridge.yaml
|   |
|   |-- wafl_localization/           <- EKF + SLAM + odom pipeline
|   |   |-- wafl_localization/
|   |   |   |-- encoder_velocity_node.py
|   |   |   |-- odom_fusion_node.py
|   |   |   `-- odom_covariance_fix.py   (sim only)
|   |   |-- config/ (ekf.yaml, slam_toolbox.yaml)
|   |   `-- launch/ (localization.launch.py, slam_mapping.launch.py)
|   |
|   |-- wafl_navigation/             <- Nav2 + base_controller
|   |   |-- wafl_navigation/base_controller.py (UNIFIED: sim + hardware via `sim` param)
|   |   |-- config/nav2_params.yaml
|   |   `-- launch/navigation.launch.py
|   |
|   |-- wafl_perception/             <- ArUco + YOLO + camera stream + debug image
|   |   |-- wafl_perception/
|   |   |   |-- pallet_pose_node.py
|   |   |   |-- pallet_front_angle_node.py
|   |   |   `-- camera_stream_node.py     (P1 priority)
|   |   |-- models/                  <- TensorRT .engine files (NOT in git)
|   |   |-- config/locations.yaml    <- pallet location DB (replaces hardcoded dict)
|   |   `-- launch/ (aruco.launch.py, yolo.launch.py, perception.launch.py)
|   |
|   |-- wafl_fork_insertion/         <- fork automation state machine (STUB in P1)
|   |   |-- wafl_fork_insertion/fork_insertion_node.py
|   |   `-- launch/fork_insertion.launch.py
|   |
|   |-- wafl_mission/                <- mission manager + WMS bridge
|   |   |-- wafl_mission/
|   |   |   |-- mission_manager_node.py   (STUB in P1)
|   |   |   `-- wms_bridge_node.py        (STUB in P1)
|   |   `-- config/wms_config.yaml
|   |
|   |-- wafl_energy/                 <- battery + charging dock
|   |   `-- wafl_energy/
|   |       |-- battery_monitor_node.py   (STUB in P1)
|   |       `-- charging_dock_node.py     (P2 only)
|   |
|   |-- wafl_dashboard/              <- Web GUI: FastAPI back-end + HTML/JS front-end
|   |   |-- wafl_dashboard/ (server.py, ros_node.py, map_render.py)
|   |   |-- static/ (index.html, css/style.css, js/app.js, robot_tab.js, logs_tab.js, wms_tab.js)
|   |   |-- launch/dashboard.launch.py
|   |   `-- requirements.txt
|   |
|   `-- wafl_bringup/                <- master launch files
|       `-- launch/
|           |-- simulation.launch.py <- Gazebo + SLAM + Nav2 + Perception + Dashboard
|           |-- hardware.launch.py   <- Full hardware bringup (includes Dashboard)
|           `-- mapping.launch.py    <- SLAM mapping session only
|
|-- wms_server/                      <- FastAPI WMS stub (Phase 1)
|   |-- main.py
|   |-- requirements.txt
|   `-- data/ (packages.json, locations.json)
|
|-- firmware/                        <- ESP32 Arduino sketches
|   |-- esp32_driving/Driving_uROS.ino
|   |-- esp32_steering_imu/Steering_IMU_uROS.ino   (UPGRADED: real IMU)
|   `-- esp32_lift_lights/Lifting_aux_uROS.ino
|
`-- docs/
    |-- WAFL2027_Architecture.md     <- This file
    `-- diagrams/                    <- drawio source + png/svg exports
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
- [ ] Warehouse world with shelves, aisles, pallets with ArUco markers (IDs 0–3)
- [ ] Simulated RPLiDAR (`GpuLidarSensor`) → `/scan`
- [ ] Simulated Kinect v1 RGB+D (`CameraSensor` + `DepthCameraSensor`) → `/camera/image_raw`, `/camera/depth/image_raw`
- [ ] Simulated IMU (`ImuSensor`) → `/imu` → `odom_covariance_fix` → `/imu_fixed`
- [ ] Simulated charging dock (ArUco ID=99 on the dock model)
- [ ] `gz::sim::systems::DiffDrive` driven by `/cmd_vel_out` (never raw `/cmd_vel`), **TF publishing disabled**; `odom_covariance_fix` still needed for its zero covariances
- [ ] Caster joint position controller on `/castor_cmd_pos`; fork prismatic joint wrapped by `sim_fork_adapter`
- [ ] `/clock` bridged (required for `use_sim_time:=true`)

Bridge mappings: README §6.

### 15.3 One-Command Launch Goal

```bash
# Docker (recommended):
docker compose -f docker/docker-compose.yml up -d

# Or native:
ros2 launch wafl_bringup simulation.launch.py use_sim_time:=true
```

This single command starts Gazebo + SLAM + Nav2 + Perception + stubs + `dashboard_server` (browser at `http://localhost:8000`). The WMS stub (`wms_server/`, port 8080) is started separately (README §11).

---

## 16. Known Gaps from 2026 — Status in 2027

| 2026 Gap | 2026 Impact | 2027 Resolution |
|----------|------------|-----------------|
| `/castor_cmd_pos` != `/target_angle` | Caster not wired to real hardware | FIXED: unified `base_controller` with `sim` param |
| Phone IMU — fragile UDP | Robot needs a phone | FIXED: proper IMU on ESP32 #2 |
| Location DB hardcoded in two Python files | Must edit two files per change | FIXED: `wafl_perception/config/locations.yaml` as ROS parameter |
| No fork-insertion sequence | Full autonomy impossible | ADDRESSED: `fork_insertion_node.py` state machine (stub P1, full P2) |
| Pre-built static map required | Manual mapping needed | ADDRESSED: SLAM Toolbox live mapping |
| No WMS integration | No mission dispatch | ADDRESSED: `wms_bridge_node.py` (REST stub P1, MQTT P2) |
| No battery monitoring | Robot can die mid-mission | ADDRESSED: `battery_monitor_node.py` (stub P1, BMS P2) |
| PyQt5 basic manual dashboard | Limited operator visibility | FIXED: 3-tab web dashboard (`wafl_dashboard`) |
| WiFi credentials hardcoded in firmware | Must reflash on network change | ADDRESSED: move to `config.h` header |
| Documentation folder empty | No design docs | FIXED: `README.md` (spec) + this document (rationale) |
| Two separate workspaces (painful builds) | Build complexity, two source steps | FIXED: single `src/` colcon workspace |
| AMCL DifferentialMotionModel for non-diff-drive | Localization drift in turns | ADDRESSED: SLAM Toolbox localization mode |

---

## 17. Design Decisions Tracker

> **Authoritative table: README §9.** This is a summary — when a decision changes, update README first, then this table.

| # | Decision | Status | Choice / Current position |
|---|----------|--------|---------------------------|
| D1 | SLAM algorithm | ✅ DECIDED | SLAM Toolbox async (fallback Cartographer) |
| D2 | Localization mode during nav | ✅ DECIDED | SLAM Toolbox localization mode (fallback SLAM + AMCL) |
| D3 | WMS protocol | ✅ DECIDED | MQTT (Mosquitto) in Phase 2 (fallback FastAPI WebSocket) |
| D4 | GUI technology | ✅ DECIDED | Web: HTML/CSS/vanilla JS + FastAPI + `rclpy` |
| D5 | WMS Phase 1 server | ✅ DECIDED | FastAPI stub |
| D6 | Nav2 local controller | ⬜ OPEN | Leaning RPP; DWB baseline; benchmark MPPI |
| D7 | Charging dock detection | ✅ DECIDED | ArUco ID=99 (fallback IR beacon) |
| D8 | Dual encoders for odometry | ⬜ OPEN | Single encoder + IMU yaw for now |
| D9 | Fork height sensing | ⬜ OPEN | Potentiometer (candidate) vs encoder |
| D10 | Load cell | ⬜ OPEN | Skipped for now |
| D11 | Camera streaming | ✅ DECIDED | JPEG `CompressedImage` → `dashboard_server` → MJPEG over HTTP |
| D12 | Fork handoff mechanism | ⬜ OPEN | Recommend `insert_forks` action only |
| D13 | Who owns `DROP` | ⬜ OPEN | Recommend `operation` field in `InsertForks` goal |
| D14 | Unused interfaces | ⬜ OPEN | `Alarm.msg`, `SetMission.srv` — wire or delete |
| D15 | Safety chain | ⬜ OPEN | Hardware e-stop that cuts motor power is required |
| D16 | Caster homing / limits | ⬜ OPEN | Home switch or potentiometer; ±80° clamp |
| D17 | GUI robot pose source | ✅ DECIDED | TF `map → base_footprint` |
| D18 | Averaged pallet topics | ✅ DECIDED | Averaging internal; no `_avg` topics |
| D19 | IMU part | ⬜ OPEN | Prefer BNO055 |
| D20 | Manual mode vs Nav2 lifecycle | ⬜ OPEN | Cancel goal + `base_controller` arbitration |
| D21 | Hardware `/joint_states` | ⬜ OPEN | Publish caster/fork joints or make them fixed |
| D22 | Dashboard access & control | ⬜ OPEN | LAN only in P1; single controller + token later |

---

## 18. Implementation Roadmap (Suggested)

Matches the two-phase build order in README §2: **Phase 1** = simulation + GUI + manual control, **Phase 2** = autonomy, hardware and energy on top.

### Phase 1a — Foundation (Weeks 1–3)
- [ ] Set up repo structure (single `src/` workspace), build `wafl_interfaces` first
- [ ] Update Docker to Ubuntu 24.04 + Jazzy + Harmonic
- [ ] Port URDF to Gazebo Harmonic (gz-sim8 plugins), `robot_state_publisher`
- [ ] Get robot spawning and driving in Gazebo via `base_controller` → `/cmd_vel_out`
- [ ] Verify `docker compose up` works end-to-end

### Phase 1b — SLAM, Localization & Navigation (Weeks 3–5)
- [ ] EKF + SLAM Toolbox in simulation (`odom_covariance_fix`, `/clock`)
- [ ] Port Nav2 stack to Jazzy (RPP controller)
- [ ] Test Nav2 with live SLAM map — robot navigates autonomously in simulation

### Phase 1c — Perception, Dashboard & Stubs (Weeks 5–8)
- [ ] Port ArUco node (`DICT_4X4_100`) and YOLO node (CPU in sim) to Jazzy + Harmonic topics
- [ ] `camera_stream_node` + `dashboard_server` (3 tabs, MJPEG, E-STOP, manual joystick)
- [ ] FastAPI WMS stub + `wms_bridge_node` stub (REST)
- [ ] Stubs for `mission_manager_node`, `fork_insertion_node`, `battery_monitor_node`
- [ ] Phase 1 checklist in README §2 all green

### Phase 2a — Autonomy (Weeks 8–11)
- [ ] Implement full `fork_insertion_node` state machine (resolve D12, D13)
- [ ] Implement full `mission_manager_node`
- [ ] Switch `wms_bridge_node` to MQTT
- [ ] Simulate full pickup: detect → approach → align → insert → lift → drop
- [ ] Test full end-to-end: WMS dispatch → robot picks pallet → reports done

### Phase 2b — Hardware Integration (Weeks 10–14)
- [ ] Flash updated firmware to all 3 ESP32s (fixed-rate heartbeats, command timeouts)
- [ ] Replace phone IMU with hardware IMU on ESP32 #2
- [ ] Hardware e-stop (D15), caster homing (D16), `/joint_states` on hardware (D21)
- [ ] Test real robot with the unified `base_controller`; tune EKF with real IMU data
- [ ] Test fork insertion on real hardware

### Phase 2c — Energy & Hardening (Weeks 13–16)
- [ ] Implement real BMS monitoring
- [ ] Implement charging dock detection + docking sequence
- [ ] Test low-battery → dock → resume mission flow
- [ ] Full warehouse mission tests end-to-end on real robot
- [ ] Fix all remaining bugs, optimize performance

---

## 19. Simulation vs Hardware Reference

> Mirrors README §10.

| Aspect | Simulation | Hardware |
|--------|-----------|----------|
| Compute | Developer laptop / Docker | Jetson Xavier AGX |
| OS | Ubuntu 24.04 (host or Docker) | Ubuntu 24.04 (Jetson) |
| ROS | Jazzy | Jazzy |
| Simulator | Gazebo Harmonic | — |
| LiDAR | `GpuLidarSensor` | Real RPLiDAR A2M8 |
| Camera | `CameraSensor` + `DepthCameraSensor` | Kinect v1 (front only) |
| IMU | `ImuSensor` plugin | MPU-6050 / BNO055 on ESP32 #2 |
| Odometry | Gazebo DiffDrive → `odom_covariance_fix` → EKF | `encoder_velocity_node` + `odom_fusion_node` → EKF |
| Caster | `/castor_cmd_pos` (Gazebo joint) | `/target_angle` → ESP32 #2 |
| Driving | Gazebo DiffDrive ← `/cmd_vel_out` | `/target_speed_left`, `/target_speed_right` → ESP32 #1 (PID on the ESP32) |
| Lift | Gazebo prismatic joint via `sim_fork_adapter` | `/lift_cmd` → ESP32 #3 |
| Fork height | `sim_fork_adapter` from joint state | Potentiometer or encoder (D9) → ESP32 #3 → `/fork_height` |
| Clock | `/clock` from Gazebo bridge | System clock |
| WMS | FastAPI stub (localhost) | Real WMS server (networked) |
| Dashboard | `dashboard_server` on laptop / container → `http://localhost:8000` | `dashboard_server` on Jetson → `http://<jetson-ip>:8000` |
| Charging dock | ArUco ID=99 in world model | Real dock with ArUco marker |
| `use_sim_time` (clock) | `true` | `false` |
| `sim` (mode) | `true` | `false` |
| micro-ROS agent | Not needed | Required on Jetson, start first |

---

## 20. Team Rules

> Mirrors README §12.

1. **Never commit to `main`** — use feature branches (`feat/slam`, `feat/gui-tab1`, `feat/aruco-node`)
2. **Every package needs a `README.md`** — what it does, what topics it uses, how to run it
3. **Update the decisions table (README §9)** when you resolve an open decision, then this document's §17 — notify the team
4. **No hardcoded IPs, passwords, or paths** — use `.env` files or ROS parameters
5. **All launch files must support the `use_sim_time` parameter** — no exceptions
6. **Test in simulation before touching real hardware**
7. **Run `colcon build` before pushing** — broken builds block the whole team
8. **`wafl_interfaces` is built first, always** — if you change a `.msg` file, tell everyone immediately
9. **TensorRT `.engine` files go in `wafl_perception/models/`** — gitignored, never commit
10. **Log with `self.get_logger()`, not `print()`** — logs appear in GUI Tab 2 automatically
11. **The browser never talks to ROS directly** — every new GUI feature is a `dashboard_server` endpoint or WebSocket message type, documented in README §5.2

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
| RPP | Regulated Pure Pursuit — Nav2 local controller (path tracker), good for straight aisles |
| DWB | Nav2's Dynamic Window Approach local controller (successor of DWA) |
| MPPI | Model Predictive Path Integral — sampling-based Nav2 local controller, alternative to DWB / RPP |
| micro-ROS | ROS 2 for microcontrollers (ESP32) communicating via UDP bridge |
| TensorRT | NVIDIA inference engine — accelerates YOLO on Jetson GPU |
| ArUco | Square fiducial markers for 6-DOF pose estimation via camera |
| BMS | Battery Management System |
| SoC | State of Charge — battery percentage |
| WMS | Warehouse Management System — dispatches transport missions to the robot |
| FastAPI | Python web framework used for the dashboard back-end and the WMS stub |
| MJPEG | Motion JPEG — a stream of JPEG frames over HTTP, displayable in a plain `<img>` tag |
| DDS | Data Distribution Service — the middleware ROS 2 uses for communication |

---

*Document created: September 2026 — Antigravity x WAFL 2027 Team*
*Last updated: See git log*
