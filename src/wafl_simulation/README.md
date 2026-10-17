# wafl_simulation

Gazebo Harmonic warehouse world, models, robot plugin block and the `ros_gz_bridge` config for WAFL 2027.
Follows README §6 / §7 of the project (topic names, QoS notes, "one writer per TF edge").

```
wafl_simulation/
├── worlds/warehouse.world            world "warehouse": hall, 2 racks, 4 pallets, dock   (GENERATED)
├── models/                           wafl_pallet_{0..3}, wafl_rack_3bay, wafl_charging_dock (GENERATED)
├── launch/gazebo.launch.py           gz sim + robot_state_publisher + spawn + bridge
├── config/ros_gz_bridge.yaml         Gazebo <-> ROS 2 topic mapping (hand-edited)
├── urdf/wafl_gz_plugins.xacro        gz plugins + sensors, to be included by the REAL URDF (GENERATED)
├── urdf/wafl_sim_bootstrap.urdf.xacro  placeholder robot until wafl_description exists    (GENERATED)
├── scripts/generate_assets.py        single source of truth for everything marked GENERATED
├── scripts/validate_assets.py        offline consistency checks (no ROS / Gazebo needed)
├── docs/layout.png                   top-down plan of the world
└── ../wafl_perception/config/locations.yaml   location DB generated from the same layout
```

## Run

```bash
# Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher
colcon build --symlink-install --packages-select wafl_simulation && source install/setup.bash

ros2 launch wafl_simulation gazebo.launch.py                    # GUI
ros2 launch wafl_simulation gazebo.launch.py headless:=true     # no display
ros2 launch wafl_simulation gazebo.launch.py robot_xacro:=$(ros2 pkg prefix --share wafl_description)/urdf/wafl.urdf.xacro
```

Standalone (world only, no ROS): `export GZ_SIM_RESOURCE_PATH=$PWD/models && gz sim -r worlds/warehouse.world`

Launch arguments: `use_sim_time` `world` `robot_xacro` `start_rsp` `bridge_config` `headless`
`spawn_x/y/z/yaw` `spawn_delay` (see the docstring in `gazebo.launch.py`).

## Layout (world frame, metres) — see `docs/layout.png`

| | |
|---|---|
| Hall | 14.0 × 8.5 m, walls 2.5 m |
| Aisle | along x, 3.5 m wide (`|y| < 1.75`) |
| Racks | north (`y = +2.25`) and south (`y = −2.25`), 3 bays each (1.8 m), 2 beam levels, floor level left free |
| Pallets | EUR 1200×800, 3 kg carton on top, ArUco on the fork-entry face. A1=ID0, A2=ID1 (north), B1=ID2, B2=ID3 (south) |
| Free bays | A3, B3 = drop-off spots |
| Dock | `x = 6.1`, back panel at `x = 6.5`, ArUco ID 99, faces −x |
| Spawn | `(−5.5, 0, yaw 0)` — **this is the SLAM map origin** |

## Contracts other packages rely on

* **World `warehouse`, model `wafl`** — hard-wired in the bridge and the plugin block.
* **TF:** the DiffDrive TF goes to an unbridged topic. Only the EKF publishes `odom → base_footprint`.
* **Actuation:** Gazebo reads `/cmd_vel_out` (never `/cmd_vel`), `/castor_cmd_pos` (rad), `/fork_cmd_vel` (m/s, + = up).
* **Sim IMU/odom** arrive as `/imu` and `/odom` with **zero covariance** → `odom_covariance_fix` → EKF.
* **Map frame ≠ world frame.** SLAM Toolbox starts the map at the spawn pose, so Nav2 goals must use the
  `map:` poses in `locations.yaml` (`world:` poses are only for Gazebo / debugging).
* **Marker geometry:** DICT_4X4_100, black square 0.15 m, white quiet zone included in the texture (quad 0.2 m).
  Marker centre is 0.444 m above the floor, on the −x (entry) face of the carton.

## ASSUMPTIONS — nothing below is measured; change in `generate_assets.py` and re-run

| Item | Value used | Where |
|---|---|---|
| Robot geometry (track, wheel r, caster offset, mass, fork size, lift travel) | 0.60 m, 0.10 m, 0.50 m, 55 kg, 0.8×0.12×0.035 m, 0.8 m | `R` dict |
| RPLiDAR A2M8 | 360°, 720 rays, 0.15–12 m, 10 Hz, σ = 1 cm | plugin block |
| Kinect RGB | 640×480, HFOV **62.5°** (derived from the README intrinsics fx = 526.607, *not* the 57° IR figure), 30 Hz | `SENSORS` |
| Kinect depth | registered to RGB, 32FC1 metres, 0.5–4.5 m | `SENSORS` |
| IMU | 100 Hz, σ_gyro = 3e-4 rad/s, σ_acc = 0.017 m/s² | plugin block |
| Approach stand-off | base_footprint 2.1 m from the pallet entry face (fork tip 1.6 m + 0.5 m) | `APPROACH_OFFSET` |
| `locations.yaml` schema | proposed here; **perception owner must confirm** | |
| Camera/LiDAR mount | camera x = 0.5, z = 0.5 m; LiDAR z = 1.3 m | `R` dict |

## Known limitations / heads-up for teammates

1. **LiDAR is mounted at 1.3 m** in the bootstrap robot so the 2D scan does not hit the mast. It therefore does
   **not see pallets or cargo** (< 0.75 m), only racks, rack cartons and walls. Revisit with the real mount height.
2. **Depth min range 0.5 m** (like the real Kinect): during the last ~0.5 m of fork insertion there is no depth.
3. **Depth encoding:** sim publishes `32FC1` in metres; a libfreenect driver may give `16UC1` in mm.
   `pallet_front_angle_node` should accept both.
4. **Camera frame:** the sensor frame is `camera_link` (x forward) as in the README, but OpenCV `solvePnP` poses are in the
   optical convention (z forward). `pallet_pose_node` needs the optical→`camera_link` rotation (or a `camera_optical_link`
   in the real URDF).
5. Pallets are dynamic bodies (9 kg). Fork/pallet friction and DART contact tuning are untested.
6. `sim_fork_adapter.py` (teammate) must publish `/fork_cmd_vel` and read the fork joint from `/joint_states` (`fork_joint`).

## First run checklist (not verifiable offline — I had no Gazebo while generating this)

- [ ] `ros2 launch wafl_simulation gazebo.launch.py` → world loads, robot spawns at the aisle end, no red errors.
- [ ] `ros2 topic hz /clock /scan /camera/image_raw /camera/depth/image_raw /imu /odom /joint_states` all publish.
- [ ] `ros2 topic pub -r 10 /cmd_vel_out geometry_msgs/Twist "{linear: {x: 0.3}}"` → robot drives forward.
- [ ] `ros2 topic pub --once /castor_cmd_pos std_msgs/Float64 "{data: 0.5}"` → caster turns; `/fork_cmd_vel 0.1` → forks rise.
- [ ] **Marker orientation:** drive to A1 and run `pallet_pose_node` (or `cv2.aruco` on `/camera/image_raw`): ID 0 must be
      detected. If the marker is missing, regenerate with `--flip-v` and/or `--mirror-u` (a mirrored ArUco is never detected).
- [ ] Robot carrying a pallet does not tip (validator predicts CoM at 0.29 m with the caster at 0.50 m).

## Regenerate / validate

```bash
python3 scripts/generate_assets.py [--marker-size 0.12] [--flip-v] [--mirror-u] [--skip-urdf]
python3 scripts/validate_assets.py        # 43 offline checks
```
`generate_assets.py` needs `opencv-python` (with `aruco`), `numpy`, `matplotlib`. It never touches
`config/ros_gz_bridge.yaml` or `launch/gazebo.launch.py`.
