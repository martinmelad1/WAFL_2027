# `warehouse.world` — WAFL 2027 Gazebo Harmonic world

SDF 1.9 world file for the WAFL 2027 simulation. It lives in `wafl_simulation/worlds/`.

> **Generated file.** The header says it is produced by `scripts/generate_assets.py`.
> Edit the generator, not this file, or your changes will be overwritten.

## Hard contracts

| Item | Value | Why it must not change |
|------|-------|------------------------|
| World name | `warehouse` | `ros_gz_bridge.yaml` uses `/world/warehouse/...` (joint states). |
| Robot | **not in this file** | `gazebo.launch.py` spawns it as model `wafl` at runtime. |
| Frame | x = along the aisle, y = across | Robot spawn default is `(-5.5, 0, 0.02)`. |

## What is in the world

### Simulation setup
- Physics step 1 ms, real-time factor 1.0.
- System plugins: `Physics`, `UserCommands`, `SceneBroadcaster`, `Sensors` (render engine `ogre2`) and `Imu`.
  The `Sensors` and `Imu` systems are required for the robot's LiDAR, depth camera and IMU to publish.
- Ambient light, a directional "sun" with shadows, and a light-grey background.

### Static geometry
| Model | Description |
|-------|-------------|
| `floor` | 14.4 m x 8.9 m slab, friction mu = 1.0. |
| `walls` | Four walls, 2.5 m high. North and south are 14.2 m long, east and west are 8.5 m long. |
| `bay_markings` | Coloured floor rectangles (1.6 m x 0.9 m): bays A1-A3 along y = +2.25 and B1-B3 along y = -2.25, plus the dock zone. Visual only, no collision. |

### Bays and objects
Bays sit at x = -1.8, 0 and 1.8.

| Bay | Position (x, y) | Floor colour | Contents |
|-----|-----------------|--------------|----------|
| A1 | (-1.8, 2.25) | blue | `pallet_A1` (`wafl_pallet_0`) |
| A2 | (0, 2.25) | blue | `pallet_A2` (`wafl_pallet_1`) |
| A3 | (1.8, 2.25) | green | empty (drop target) |
| B1 | (-1.8, -2.25) | blue | `pallet_B1` (`wafl_pallet_2`) |
| B2 | (0, -2.25) | blue | `pallet_B2` (`wafl_pallet_3`) |
| B3 | (1.8, -2.25) | green | empty (drop target) |
| Dock | (6.1, 0) | yellow | `charging_dock` (`wafl_charging_dock`) |

- Racks: `rack_north` at (0, 2.25) and `rack_south` at (0, -2.25) rotated 180 deg, both `model://wafl_rack_3bay`.
- Pallets are rotated +90 deg (north side) and -90 deg (south side) so their pockets face the aisle.
- By the project convention (main README sec. 6), the pallet models carry ArUco textures with IDs 0-3 and the dock carries ID 99. Check the model files if you change this.

## Model dependencies
The `<include>` tags use `model://` URIs, resolved through `GZ_SIM_RESOURCE_PATH`. `gazebo.launch.py` appends `wafl_simulation/models` automatically. These models must exist there:

- `wafl_rack_3bay`
- `wafl_pallet_0`, `wafl_pallet_1`, `wafl_pallet_2`, `wafl_pallet_3`
- `wafl_charging_dock`

## Running it
Normally through the launch file:

```bash
ros2 launch wafl_simulation gazebo.launch.py
```

To open only the world (no robot, no bridge), for example when checking models:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:<path-to>/wafl_simulation/models
gz sim -r worlds/warehouse.world
```

## Changing the layout
1. Edit `scripts/generate_assets.py`.
2. Regenerate the world.
3. If you move a bay or the dock, update the Nav2 goals / `locations.yaml` and the robot spawn arguments to match.
