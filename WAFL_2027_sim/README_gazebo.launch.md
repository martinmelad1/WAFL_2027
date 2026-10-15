# `gazebo.launch.py` — Gazebo Harmonic + robot + ros_gz_bridge

Launch file of the `wafl_simulation` package. It brings up the simulation layer of WAFL 2027 only.

## What it starts

1. **Resource paths.** Appends `wafl_simulation/models`, `wafl_simulation`'s install `share` and `wafl_description`'s install `share` to `GZ_SIM_RESOURCE_PATH`. This lets Gazebo resolve `model://wafl_*` (pallets, racks, dock) and `package://wafl_description/meshes/...` URIs.
2. **Gazebo Harmonic** through `ros_gz_sim/gz_sim.launch.py`, loading the chosen world. It runs with `-r` (start unpaused). With `headless:=true` it adds `-s --headless-rendering` (no GUI, sensors still render).
3. **`robot_state_publisher`** (only if `start_rsp:=true`). The URDF is built in memory by `build_robot_description()` from `wafl_simulation.urdf_patch`, which patches `wafl_sim_robot.urdf.xacro` for Gazebo.
4. **Spawn.** After `spawn_delay` seconds, `ros_gz_sim create` spawns the robot from the `robot_description` topic into world `warehouse` as model `wafl`.
5. **`ros_gz_bridge`** (`parameter_bridge`) configured from `config/ros_gz_bridge.yaml`.

It does **not** start SLAM, EKF, Nav2, perception or the GUI. Those come from `wafl_bringup/simulation.launch.py`.

## Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `true` | Passed to `robot_state_publisher`. |
| `world` | `<share>/worlds/warehouse.world` | World SDF to load. |
| `robot_xacro` | `<share>/urdf/wafl_sim_robot.urdf.xacro` | Robot description source. |
| `start_rsp` | `true` | Start `robot_state_publisher` here. Set `false` if another launch file already does. |
| `bridge_config` | `<share>/config/ros_gz_bridge.yaml` | Bridge topic list. |
| `headless` | `false` | Run the Gazebo server without the GUI. |
| `spawn_x` / `spawn_y` / `spawn_z` | `-5.5` / `0.0` / `0.02` | Spawn position (m). |
| `spawn_yaw` | `0.0` | Spawn yaw (rad). |
| `spawn_delay` | `4.0` | Seconds to wait before spawning, so Gazebo and `robot_state_publisher` are ready. |

## Usage

```bash
# Default: GUI, robot at the west end of the aisle
ros2 launch wafl_simulation gazebo.launch.py

# Headless (CI, remote machine)
ros2 launch wafl_simulation gazebo.launch.py headless:=true

# Custom spawn pose
ros2 launch wafl_simulation gazebo.launch.py spawn_x:=2.0 spawn_y:=-1.0 spawn_yaw:=1.57

# Used from wafl_bringup, which already runs robot_state_publisher
ros2 launch wafl_simulation gazebo.launch.py start_rsp:=false
```

## Hard-coded names (do not change independently)

| Constant | Value | Must match |
|----------|-------|------------|
| `WORLD_NAME` | `warehouse` | `<world name>` in `warehouse.world`, `/world/warehouse/...` in the bridge YAML. |
| `MODEL_NAME` | `wafl` | `/model/wafl/...` topics in the bridge YAML and the plugins in the robot xacro. |

## Dependencies
`ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `wafl_description` (meshes), and the Python module `wafl_simulation.urdf_patch`. The package `setup.py` / `CMakeLists.txt` must install `worlds`, `models`, `config`, `urdf` and `launch`.

## Troubleshooting
- **Robot does not appear.** The spawn reads the `robot_description` topic. Make sure `robot_state_publisher` is running (`start_rsp:=true` or another launch file) and try a larger `spawn_delay`.
- **Models / meshes missing (grey or absent pallets).** Check that `GZ_SIM_RESOURCE_PATH` includes the model folders and that `wafl_description` is built and sourced.
- **Nodes stall.** `use_sim_time:=true` needs `/clock` from the bridge. Check that the bridge node is alive.
