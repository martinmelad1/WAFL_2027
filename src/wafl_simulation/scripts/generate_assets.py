#!/usr/bin/env python3
"""
generate_assets.py - single source of truth for the WAFL 2027 simulation assets.

Generates, from the layout constants below:

  models/wafl_pallet_{0..3}/   EUR pallet + cargo box + ArUco marker (ID = suffix)
  models/wafl_rack_3bay/       3-bay, 2-level rack (floor level left free for pallets)
  models/wafl_charging_dock/   dock plate + back panel with ArUco ID 99
  worlds/warehouse.world       world "warehouse" (SDF 1.9, Gazebo Harmonic)
  urdf/wafl_gz_plugins.xacro   gz plugin + sensor block (reusable by the REAL URDF)
  config/sim_controllers.yaml  wheel mode (kinematic|force) + controller gains - written ONLY if missing
  urdf/wafl_sim_bootstrap.urdf.xacro   placeholder robot so the sim runs today
  ../wafl_perception/config/locations.yaml   proposed location DB (matches the world)
  docs/layout.png              top-down plan of the generated world

It does NOT touch config/ros_gz_bridge.yaml or launch/gazebo.launch.py (hand-edited).

Usage:
  python3 scripts/generate_assets.py                    # defaults
  python3 scripts/generate_assets.py --marker-size 0.12 # change physical marker size
  python3 scripts/generate_assets.py --flip-v           # if markers render upside-down
  python3 scripts/generate_assets.py --mirror-u         # if markers render mirrored

Every number marked ASSUMPTION is a default chosen because the real value is not known yet.
"""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np

# --------------------------------------------------------------------------- #
#  Names that MUST stay in sync with ros_gz_bridge.yaml / gazebo.launch.py
# --------------------------------------------------------------------------- #
WORLD_NAME = "warehouse"
MODEL_NAME = "wafl"
ARUCO_DICT = "DICT_4X4_100"          # README wins over the architecture doc (50)
PALLET_IDS = [0, 1, 2, 3]
DOCK_ID = 99

# --------------------------------------------------------------------------- #
#  Layout (world frame, metres).  x = along the aisle, y = across it.
# --------------------------------------------------------------------------- #
FLOOR_X, FLOOR_Y = 14.0, 8.5            # ASSUMPTION: compact test hall
WALL_H, WALL_T = 2.5, 0.2
AISLE_HALF = 1.75                       # aisle width 3.5 m (right-angle stacking)
RACK_DEPTH = 1.0
RACK_CY = AISLE_HALF + RACK_DEPTH / 2   # 2.25
BAY_W = 1.8
BAY_X = [-1.8, 0.0, 1.8]
RACK_LEN = BAY_W * 3                    # 5.4
RACK_H = 2.2
BEAM_LEVELS = [1.0, 2.0]

# EUR pallet 1200 x 800 mm.  Pallet frame: +x = fork direction (into the pallet),
# y = 1.2 m wide, entry face at x = -0.4, fork pockets at y = +-0.264.
PAL_D, PAL_W, PAL_H = 0.8, 1.2, 0.144
BLOCK = 0.145
COL_Y = PAL_W / 2 - BLOCK / 2           # 0.5275
POCKET_Y = COL_Y / 2                    # 0.26375  (pocket centre)
CARGO = (0.7, 1.0, 0.6)                 # ASSUMPTION: light carton on each pallet
CARGO_MASS, PALLET_MASS = 3.0, 6.0      # ASSUMPTION

SPAWN = (-5.5, 0.0, 0.0)                # robot spawn (x, y, yaw) in the world frame
DOCK_X = 6.1                            # dock model origin; back panel at x = 6.5
APPROACH_OFFSET = 2.1                   # ASSUMPTION: base_footprint -> pallet entry face
                                        # (fork tip 1.6 m ahead + 0.5 m stand-off)
DOCK_APPROACH_OFFSET = 2.5              # base_footprint -> dock marker plane

# name, aruco id (None = empty bay), row, bay index
LOCATIONS = [
    ("A1", 0, "N", 0), ("A2", 1, "N", 1), ("A3", None, "N", 2),
    ("B1", 2, "S", 0), ("B2", 3, "S", 1), ("B3", None, "S", 2),
]

# --------------------------------------------------------------------------- #
#  Robot (bootstrap) - ALL ASSUMPTIONS until the real URDF exists
# --------------------------------------------------------------------------- #
R = dict(
    wheel_radius=0.10,      # README encoder_velocity_node default
    wheel_width=0.06,
    track_width=0.60,       # ASSUMPTION - measure on the robot
    caster_x=0.50,          # README: L = 0.5 m axle -> caster
    caster_r=0.05,
    chassis=(0.8, 0.5, 0.25),
    chassis_cx=0.25,
    mast_x=0.78,
    fork_len=0.80, fork_w=0.12, fork_t=0.035, fork_gap=0.01,
    fork_travel=0.80,       # ASSUMPTION
    lidar_z=1.30,           # above the mast: see README "known limitations"
    cam_x=0.50, cam_z=0.50,
    base_z=0.10,            # base_link height above base_footprint (= axle height)
)
SENSORS = dict(
    # Kinect v1 RGB intrinsics from the README: fx = 526.607, 640 px wide
    cam_hfov=2 * math.atan(320.0 / 526.607),     # = 1.0911 rad (62.5 deg)
    depth_near=0.5, depth_far=4.5,               # ASSUMPTION (Kinect v1 usable range)
    cam_rate=30, lidar_rate=10, lidar_samples=720, imu_rate=100,
)


def f(v):
    s = f"{v:.5f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def nums(*v):
    return " ".join(f(x) for x in v)


def ynums(*v):
    """comma-separated, for YAML flow sequences"""
    return ", ".join(f(x) for x in v)


# --------------------------------------------------------------------------- #
#  SDF helpers
# --------------------------------------------------------------------------- #
def mat(rgb, spec=0.1):
    r, g, b = rgb
    return (f"<material><ambient>{nums(r, g, b)} 1</ambient><diffuse>{nums(r, g, b)} 1</diffuse>"
            f"<specular>{nums(spec, spec, spec)} 1</specular></material>")


def vbox(name, size, pose, rgb):
    return (f'<visual name="{name}"><pose>{nums(*pose)} 0 0 0</pose><geometry><box><size>{nums(*size)}'
            f"</size></box></geometry>{mat(rgb)}</visual>")


def cbox(name, size, pose, mu=1.0):
    return (f'<collision name="{name}"><pose>{nums(*pose)} 0 0 0</pose><geometry><box><size>{nums(*size)}'
            f"</size></box></geometry><surface><friction><ode><mu>{f(mu)}</mu><mu2>{f(mu)}</mu2>"
            f"</ode></friction></surface></collision>")


def box_inertia(m, x, y, z):
    return (m / 12 * (y * y + z * z), m / 12 * (x * x + z * z), m / 12 * (x * x + y * y))


def sdf_inertial(m, pose, inertia):
    ixx, iyy, izz = inertia
    return (f"<inertial><pose>{nums(*pose)} 0 0 0</pose><mass>{f(m)}</mass><inertia>"
            f"<ixx>{f(ixx)}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{f(iyy)}</iyy><iyz>0</iyz><izz>{f(izz)}</izz>"
            f"</inertia></inertial>")


WOOD = (0.76, 0.60, 0.42)
CARD = (0.65, 0.50, 0.35)


# --------------------------------------------------------------------------- #
#  ArUco textures + marker quad meshes
# --------------------------------------------------------------------------- #
def make_marker_png(path, marker_id, px_per_cell=100):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, ARUCO_DICT))
    side = 6 * px_per_cell                                  # 4x4 bits + 1-bit border each side
    if hasattr(cv2.aruco, "generateImageMarker"):
        img = cv2.aruco.generateImageMarker(d, marker_id, side, borderBits=1)
    else:
        img = cv2.aruco.drawMarker(d, marker_id, side, borderBits=1)
    img = cv2.copyMakeBorder(img, px_per_cell, px_per_cell, px_per_cell, px_per_cell,
                             cv2.BORDER_CONSTANT, value=255)   # 1-cell white quiet zone
    cv2.imwrite(str(path), img)


def write_marker_mesh(mesh_dir, marker_id, marker_size, flip_v, mirror_u):
    """Quad in the y-z plane, outward normal -x, UVs chosen so that a viewer standing at
    -x looking toward +x (up = +z, left = +y) sees the image upright and un-mirrored."""
    mesh_dir.mkdir(parents=True, exist_ok=True)
    make_marker_png(mesh_dir / f"aruco_{marker_id}.png", marker_id)
    h = marker_size * 8 / 6 / 2                              # quad includes the white quiet zone
    # BL, BR, TR, TL as seen by the viewer (CCW => normal -x)
    verts = [(0, +h, -h), (0, -h, -h), (0, -h, +h), (0, +h, +h)]
    uvs = [(0, 0), (1, 0), (1, 1), (0, 1)]
    if mirror_u:
        uvs = [(1 - u, v) for u, v in uvs]
    if flip_v:
        uvs = [(u, 1 - v) for u, v in uvs]
    obj = [f"mtllib aruco_{marker_id}.mtl", f"o aruco_{marker_id}"]
    obj += [f"v {nums(*v)}" for v in verts]
    obj += [f"vt {nums(*uv)}" for uv in uvs]
    obj += ["vn -1 0 0", f"usemtl aruco_{marker_id}", "f 1/1/1 2/2/1 3/3/1 4/4/1", ""]
    (mesh_dir / f"aruco_{marker_id}.obj").write_text("\n".join(obj))
    (mesh_dir / f"aruco_{marker_id}.mtl").write_text(
        f"newmtl aruco_{marker_id}\nKa 1 1 1\nKd 1 1 1\nKs 0 0 0\nmap_Kd aruco_{marker_id}.png\n")


def marker_visual(model, marker_id, pose):
    return (f'<visual name="aruco_{marker_id}"><cast_shadows>false</cast_shadows><pose>{nums(*pose)} 0 0 0</pose>'
            f"<geometry><mesh><uri>model://{model}/meshes/aruco_{marker_id}.obj</uri></mesh></geometry></visual>")


def model_config(name, desc):
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.9">model.sdf</sdf>\n  <author><name>WAFL 2027 team</name><email>-</email></author>\n'
            f"  <description>{desc}</description>\n</model>\n")


def write_model(models_dir, name, sdf_body, desc):
    d = models_dir / name
    d.mkdir(parents=True, exist_ok=True)
    (d / "model.config").write_text(model_config(name, desc))
    (d / "model.sdf").write_text(f'<?xml version="1.0"?>\n<sdf version="1.9">\n{sdf_body}\n</sdf>\n')


# --------------------------------------------------------------------------- #
#  Models
# --------------------------------------------------------------------------- #
def pallet_model(models_dir, marker_id, marker_size, flip_v, mirror_u):
    name = f"wafl_pallet_{marker_id}"
    write_marker_mesh(models_dir / name / "meshes", marker_id, marker_size, flip_v, mirror_u)
    cz = PAL_H + CARGO[2] / 2                               # cargo centre height
    m = PALLET_MASS + CARGO_MASS
    com_z = (PALLET_MASS * 0.072 + CARGO_MASS * cz) / m
    ix, iy, iz = box_inertia(m, PAL_D, PAL_W, cz + CARGO[2] / 2)
    parts = [sdf_inertial(m, (0, 0, com_z), (ix, iy, iz))]
    # collisions: top deck + 3 block columns (leave 2 open fork pockets) + cargo
    parts.append(cbox("deck", (PAL_D, PAL_W, 0.022), (0, 0, PAL_H - 0.011)))
    for i, y in enumerate((-COL_Y, 0.0, COL_Y)):
        parts.append(cbox(f"column_{i}", (PAL_D, BLOCK, PAL_H - 0.022), (0, y, (PAL_H - 0.022) / 2)))
    parts.append(cbox("cargo", CARGO, (0, 0, cz)))
    # visuals: 5 deck boards, 9 blocks, 3 runners, cargo, marker
    pitch = BLOCK + (PAL_D - 5 * BLOCK) / 4
    for i in range(5):
        parts.append(vbox(f"board_{i}", (BLOCK, PAL_W, 0.022), (-0.3275 + i * pitch, 0, PAL_H - 0.011), WOOD))
    for ix_, x in enumerate((-0.3275, 0.0, 0.3275)):
        for iy_, y in enumerate((-COL_Y, 0.0, COL_Y)):
            parts.append(vbox(f"block_{ix_}{iy_}", (BLOCK, BLOCK, 0.1), (x, y, 0.072), WOOD))
    for i, y in enumerate((-COL_Y, 0.0, COL_Y)):
        parts.append(vbox(f"runner_{i}", (PAL_D, BLOCK, 0.022), (0, y, 0.011), WOOD))
    parts.append(vbox("cargo", CARGO, (0, 0, cz), CARD))
    parts.append(marker_visual(name, marker_id, (-CARGO[0] / 2 - 0.0005, 0, cz)))
    body = (f'<model name="{name}">\n  <static>false</static>\n  <link name="link">\n    '
            + "\n    ".join(parts) + "\n  </link>\n</model>")
    write_model(models_dir, name, body, f"EUR pallet (1200x800) carrying a carton; ArUco {ARUCO_DICT} ID {marker_id} "
                                        f"on the fork-entry face (-x). Forks enter along +x.")


def rack_model(models_dir):
    name = "wafl_rack_3bay"
    up_x = [-RACK_LEN / 2 + i * BAY_W for i in range(4)]
    fy = RACK_DEPTH / 2 - 0.04
    parts = ['<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>']
    BLUE, ORANGE, CARTON = (0.1, 0.25, 0.65), (0.9, 0.45, 0.05), (0.72, 0.58, 0.40)
    for i, x in enumerate(up_x):
        for j, y in enumerate((-fy, fy)):
            p, s = (x, y, RACK_H / 2), (0.08, 0.08, RACK_H)
            parts += [cbox(f"up_{i}_{j}", s, p), vbox(f"up_{i}_{j}", s, p, BLUE)]
    for lv, z in enumerate(BEAM_LEVELS):
        for j, y in enumerate((-fy, fy)):
            p, s = (0, y, z), (RACK_LEN + 0.08, 0.05, 0.10)
            parts += [cbox(f"beam_{lv}_{j}", s, p), vbox(f"beam_{lv}_{j}", s, p, ORANGE)]
        for i, x in enumerate(up_x):
            p, s = (x, 0, z), (0.05, 2 * fy, 0.10)
            parts += [cbox(f"cross_{lv}_{i}", s, p), vbox(f"cross_{lv}_{i}", s, p, ORANGE)]
    # decorative cartons on level 1 of every bay (visible to a lidar mounted at ~1.3 m)
    for b, bx in enumerate(BAY_X):
        p, s = (bx, 0, BEAM_LEVELS[0] + 0.05 + 0.25), (1.0, 0.7, 0.5)
        parts += [cbox(f"carton_{b}", s, p), vbox(f"carton_{b}", s, p, CARTON)]
    body = (f'<model name="{name}">\n  <static>true</static>\n  <link name="link">\n    '
            + "\n    ".join(parts) + "\n  </link>\n</model>")
    write_model(models_dir, name, body,
                "3-bay x 2-level pallet rack. Front faces -y. Floor level is left empty for floor pallets.")


def dock_model(models_dir, marker_size, flip_v, mirror_u):
    name = "wafl_charging_dock"
    write_marker_mesh(models_dir / name / "meshes", DOCK_ID, marker_size, flip_v, mirror_u)
    PLATE, PANEL, YEL, GREY, GRN = (0.8, 0.8, 0.01), (0.05, 1.0, 0.8), (0.95, 0.8, 0.1), (0.35, 0.35, 0.38), (0.1, 0.8, 0.2)
    pp, np_ = (0, 0, PLATE[2] / 2), (0.4, 0, PANEL[2] / 2)
    parts = ['<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial>',
             cbox("plate", PLATE, pp), vbox("plate", PLATE, pp, YEL),
             cbox("panel", PANEL, np_), vbox("panel", PANEL, np_, GREY),
             f'<visual name="status_led"><pose>0.372 0 0.74 0 0 0</pose><geometry><sphere><radius>0.025</radius></sphere></geometry>{mat(GRN)}</visual>',
             marker_visual(name, DOCK_ID, (0.4 - PANEL[0] / 2 - 0.0005, 0, 0.45))]
    body = (f'<model name="{name}">\n  <static>true</static>\n  <link name="link">\n    '
            + "\n    ".join(parts) + "\n  </link>\n</model>")
    write_model(models_dir, name, body,
                f"Charging dock: floor plate + back panel with ArUco {ARUCO_DICT} ID {DOCK_ID} (faces -x).")


# --------------------------------------------------------------------------- #
#  Layout maths (world <-> map).  SLAM's map origin = robot start pose.
# --------------------------------------------------------------------------- #
def world_to_map(x, y, yaw):
    sx, sy, syaw = SPAWN
    dx, dy = x - sx, y - sy
    c, s = math.cos(-syaw), math.sin(-syaw)
    return (c * dx - s * dy, s * dx + c * dy, yaw - syaw)


def bay_pose(row, bay):
    """Pallet-frame pose (x, y, yaw) of a bay: +x of the pallet frame points into the rack."""
    bx = BAY_X[bay]
    return (bx, +RACK_CY, math.pi / 2) if row == "N" else (bx, -RACK_CY, -math.pi / 2)


def approach_pose(px, py, yaw, offset):
    """base_footprint pose that stands `offset` m from the pallet entry face, facing the pallet."""
    d = (math.cos(yaw), math.sin(yaw))
    face = (px - PAL_D / 2 * d[0], py - PAL_D / 2 * d[1])
    return (face[0] - offset * d[0], face[1] - offset * d[1], yaw)


# --------------------------------------------------------------------------- #
#  World
# --------------------------------------------------------------------------- #
def world_file(path):
    inc = []
    for name, mid, row, bay in LOCATIONS:
        if mid is None:
            continue
        x, y, yaw = bay_pose(row, bay)
        inc.append(f'    <include><uri>model://wafl_pallet_{mid}</uri><name>pallet_{name}</name>'
                   f'<pose>{nums(x, y, 0, 0, 0, yaw)}</pose></include>')
    # painted bay outlines (visual only): blue = pre-loaded pallet, green = free drop-off bay
    bays = []
    for name, mid, row, bay in LOCATIONS:
        x, y, _ = bay_pose(row, bay)
        col = (0.2, 0.45, 0.9) if mid is not None else (0.2, 0.8, 0.35)
        bays.append(vbox(f"bay_{name}", (BAY_W - 0.2, RACK_DEPTH - 0.1, 0.004), (x, y, 0.002), col))
    bays.append(vbox("dock_zone", (1.0, 1.0, 0.004), (DOCK_X, 0, 0.002), (0.95, 0.8, 0.1)))
    walls = []
    hx, hy = FLOOR_X / 2, FLOOR_Y / 2
    for n, size, pose in [("wall_n", (FLOOR_X + WALL_T, WALL_T, WALL_H), (0, hy + WALL_T / 2, WALL_H / 2)),
                          ("wall_s", (FLOOR_X + WALL_T, WALL_T, WALL_H), (0, -hy - WALL_T / 2, WALL_H / 2)),
                          ("wall_e", (WALL_T, FLOOR_Y, WALL_H), (hx + WALL_T / 2, 0, WALL_H / 2)),
                          ("wall_w", (WALL_T, FLOOR_Y, WALL_H), (-hx - WALL_T / 2, 0, WALL_H / 2))]:
        walls += [cbox(n, size, pose), vbox(n, size, pose, (0.82, 0.82, 0.80))]
    sdf = f"""<?xml version="1.0"?>
<!-- WAFL 2027 warehouse. GENERATED by scripts/generate_assets.py - edit the generator, not this file.
     World name MUST stay "{WORLD_NAME}" (ros_gz_bridge.yaml uses /world/{WORLD_NAME}/...).
     Robot is NOT in this file: gazebo.launch.py spawns it as model "{MODEL_NAME}".
     Frame: x = along the aisle, y = across, robot spawn (see gazebo.launch.py) = ({nums(*SPAWN)}). -->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.45 0.45 0.45 1</ambient>
      <background>0.75 0.8 0.85 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry><box><size>{nums(FLOOR_X + 0.4, FLOOR_Y + 0.4, 0.1)}</size></box></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <pose>0 0 -0.05 0 0 0</pose>
          <geometry><box><size>{nums(FLOOR_X + 0.4, FLOOR_Y + 0.4, 0.1)}</size></box></geometry>
          {mat((0.55, 0.55, 0.57), 0.05)}
        </visual>
      </link>
    </model>

    <model name="walls">
      <static>true</static>
      <link name="link">
        {chr(10).join("        " + w for w in walls).strip()}
      </link>
    </model>

    <model name="bay_markings">
      <static>true</static>
      <link name="link">
        {chr(10).join("        " + b for b in bays).strip()}
      </link>
    </model>

    <include><uri>model://wafl_rack_3bay</uri><name>rack_north</name><pose>{nums(0, RACK_CY, 0, 0, 0, 0)}</pose></include>
    <include><uri>model://wafl_rack_3bay</uri><name>rack_south</name><pose>{nums(0, -RACK_CY, 0, 0, 0, math.pi)}</pose></include>

{chr(10).join(inc)}

    <include><uri>model://wafl_charging_dock</uri><name>charging_dock</name><pose>{nums(DOCK_X, 0, 0, 0, 0, 0)}</pose></include>
  </world>
</sdf>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(sdf)


# --------------------------------------------------------------------------- #
#  locations.yaml (proposed schema - perception owner to confirm)
# --------------------------------------------------------------------------- #
def locations_yaml(path, marker_size):
    L = []
    L.append(f"""# WAFL 2027 - pallet / dock location database   (PROPOSED SCHEMA - perception owner to confirm)
# GENERATED by wafl_simulation/scripts/generate_assets.py from the simulated warehouse layout.
#
# Two frames are given for every pose:
#   world : Gazebo world frame (what the .world file uses)
#   map   : SLAM map frame. SLAM Toolbox starts the map at the robot's start pose, so
#           map = world expressed relative to the spawn pose {nums(*SPAWN)} (x, y, yaw).
#           >>> Nav2 goals and mission_manager must use the `map` poses. <<<
# Poses are (x [m], y [m], yaw [rad]).
#   pallet_pose   : pallet CENTRE; yaw = direction the forks travel to enter it
#   approach_pose : base_footprint pose to reach in the APPROACH state, facing the pallet
#                   (ASSUMPTION: {APPROACH_OFFSET} m from the pallet entry face = fork tip 1.6 m + 0.5 m stand-off;
#                    recompute with the real robot geometry)
# Empty bays (marker_id: null) are drop-off spots.

aruco:
  dictionary: {ARUCO_DICT}
  marker_size_m: {f(marker_size)}     # side of the black square (quiet zone excluded)

spawn:                     # robot start pose in the simulated world
  world: [{ynums(*SPAWN)}]

locations:""")
    for name, mid, row, bay in LOCATIONS:
        px, py, pyaw = bay_pose(row, bay)
        ax, ay, ayaw = approach_pose(px, py, pyaw, APPROACH_OFFSET)
        mp, ma = world_to_map(px, py, pyaw), world_to_map(ax, ay, ayaw)
        L.append(f"""  - name: Location_{name}
    label: {name}
    marker_id: {"null" if mid is None else mid}
    pallet_present_at_start: {"false" if mid is None else "true"}
    pallet_pose:   {{world: [{ynums(px, py, pyaw)}], map: [{ynums(*mp)}]}}
    approach_pose: {{world: [{ynums(ax, ay, ayaw)}], map: [{ynums(*ma)}]}}""")
    dm = (DOCK_X + 0.4 - 0.025, 0.0, 0.0)
    da = (dm[0] - DOCK_APPROACH_OFFSET, 0.0, 0.0)
    L.append(f"""
charging_dock:
  name: Charging_Dock
  marker_id: {DOCK_ID}
  marker_pose:   {{world: [{ynums(*dm)}], map: [{ynums(*world_to_map(*dm))}]}}    # marker plane, faces -x
  approach_pose: {{world: [{ynums(*da)}], map: [{ynums(*world_to_map(*da))}]}}    # base_footprint, heading +x (ASSUMPTION: {DOCK_APPROACH_OFFSET} m from marker)
""")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(L))


# --------------------------------------------------------------------------- #
#  Robot: gz plugin block (reusable) + bootstrap URDF (placeholder)
# --------------------------------------------------------------------------- #
GZ_PLUGINS_XACRO = """<?xml version="1.0"?>
<!--
  wafl_gz_plugins.xacro - Gazebo Harmonic plugins + sensors for the WAFL robot.
  GENERATED by scripts/generate_assets.py (but meant to be copied/included by the REAL wafl_description URDF).

  Include from the robot xacro with:
      <xacro:include filename="$(find wafl_simulation)/urdf/wafl_gz_plugins.xacro"/>

  CONTRACT (README sec. 6 / D23). The robot URDF MUST provide these joints (names are what
  config/ros_gz_bridge.yaml relies on) - geometry, origins and axes are NOT used here:
      front_left_steer_joint, front_right_steer_joint  revolute, axis +z   (front pair, steered together)
      caster_joint                                     revolute, axis +z   (REAR wheel steering)
      front_left_wheel_joint, front_right_wheel_joint  continuous          (the only driven wheels)
      caster_wheel_joint                               continuous, PASSIVE (no plugin, must roll freely)
      fork_joint                                       prismatic, up = +
  links : base_footprint (origin = midpoint of the front drive axle), laser_link, camera_link (x forward),
          imu_link
  Gazebo model name MUST be "@MODEL@" (gazebo.launch.py spawns with -name @MODEL@).

  Design rules taken from the README:
    * NO DiffDrive (it cannot model steered wheels) and NO /cmd_vel_out: Gazebo only receives per-joint commands,
      published by base_controller AFTER arbitration. Raw /cmd_vel is never read.
    * Odometry = OdometryPublisher (ground truth). Its TF goes to an UNBRIDGED topic: the EKF is the only
      odom->base_footprint publisher.
    * Both front steering joints subscribe to the SAME gz topic, so they stay parallel like the real chain.
    * Wheel mode (kinematic | force) and every gain live in config/sim_controllers.yaml - no regeneration needed.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="sim_controllers_yaml" default="$(find wafl_simulation)/config/sim_controllers.yaml"/>
  <xacro:property name="sim_cfg_file" value="$(arg sim_controllers_yaml)"/>
  <xacro:property name="sim_cfg" value="${xacro.load_yaml(sim_cfg_file)}"/>
  <xacro:property name="wheel_force" value="${sim_cfg['wheel_drive']['mode'] == 'force'}"/>

  <!-- Front drive wheel: velocity command in rad/s. kinematic = joint velocity is set directly;
       force = PID applies torque (use_force_commands). -->
  <xacro:macro name="wheel_velocity_controller" params="joint">
    <gazebo>
      <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
        <joint_name>${joint}</joint_name>
        <topic>/model/@MODEL@/joint/${joint}/cmd_vel</topic>
        <initial_velocity>0.0</initial_velocity>
        <xacro:if value="${wheel_force}">
          <use_force_commands>true</use_force_commands>
          <p_gain>${sim_cfg['wheel_drive']['p_gain']}</p_gain>
          <i_gain>${sim_cfg['wheel_drive']['i_gain']}</i_gain>
          <d_gain>${sim_cfg['wheel_drive']['d_gain']}</d_gain>
          <i_max>${sim_cfg['wheel_drive']['i_max']}</i_max>
          <i_min>${sim_cfg['wheel_drive']['i_min']}</i_min>
          <cmd_max>${sim_cfg['wheel_drive']['cmd_max']}</cmd_max>
          <cmd_min>${sim_cfg['wheel_drive']['cmd_min']}</cmd_min>
        </xacro:if>
      </plugin>
    </gazebo>
  </xacro:macro>

  <!-- Steering joint: position command in radians (a stepper on the real robot). -->
  <xacro:macro name="steer_position_controller" params="joint topic">
    <gazebo>
      <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
        <joint_name>${joint}</joint_name>
        <topic>${topic}</topic>
        <p_gain>${sim_cfg['steering']['p_gain']}</p_gain>
        <i_gain>${sim_cfg['steering']['i_gain']}</i_gain>
        <d_gain>${sim_cfg['steering']['d_gain']}</d_gain>
        <i_max>${sim_cfg['steering']['i_max']}</i_max>
        <i_min>${sim_cfg['steering']['i_min']}</i_min>
        <cmd_max>${sim_cfg['steering']['cmd_max']}</cmd_max>
        <cmd_min>${sim_cfg['steering']['cmd_min']}</cmd_min>
      </plugin>
    </gazebo>
  </xacro:macro>

  <xacro:wheel_velocity_controller joint="front_left_wheel_joint"/>
  <xacro:wheel_velocity_controller joint="front_right_wheel_joint"/>

  <!-- both front steering joints share ONE gz topic -> always parallel (same as the real chain/sprocket) -->
  <xacro:steer_position_controller joint="front_left_steer_joint"  topic="/model/@MODEL@/front_steer/cmd_pos"/>
  <xacro:steer_position_controller joint="front_right_steer_joint" topic="/model/@MODEL@/front_steer/cmd_pos"/>
  <!-- rear wheel steering; topic set explicitly (the default would include the axis index, /0/cmd_pos) -->
  <xacro:steer_position_controller joint="caster_joint" topic="/model/@MODEL@/joint/caster_joint/cmd_pos"/>

  <gazebo>
    <!-- ground-truth odometry. Output topic is bridged to /odom; the TF it also emits goes to a dead-end topic. -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>odom</odom_frame>
      <robot_base_frame>base_footprint</robot_base_frame>
      <odom_topic>/model/@MODEL@/odometry</odom_topic>
      <tf_topic>/model/@MODEL@/tf_unused_ekf_owns_odom_tf</tf_topic>
      <odom_publish_frequency>50</odom_publish_frequency>
      <dimensions>2</dimensions>
    </plugin>

    <plugin filename="gz-sim-joint-controller-system" name="gz::sim::systems::JointController">
      <joint_name>fork_joint</joint_name>
      <topic>/model/@MODEL@/joint/fork_joint/cmd_vel</topic>
      <initial_velocity>0.0</initial_velocity>
    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>
  </gazebo>
"""

SIM_CONTROLLERS_YAML = """# sim_controllers.yaml - tuning for the Gazebo joint controllers (read by wafl_gz_plugins.xacro).
# Change a value or the mode here and relaunch - NO regeneration needed.
# Written once by scripts/generate_assets.py (never overwritten, so your edits are safe).

wheel_drive:                # front-left / front-right drive wheels, command = rad/s
  mode: kinematic           # kinematic = joint velocity set directly (exact, no tuning)
                            # force     = PID applies torque (acceleration lag, slip). PLACEHOLDER gains below.
  p_gain: 8.0               # force mode only (numbers copied from the old caster controller)
  i_gain: 0.1
  d_gain: 0.4
  i_max: 1.0
  i_min: -1.0
  cmd_max: 20.0
  cmd_min: -20.0

steering:                   # front_left/right_steer_joint + caster_joint (rear), command = radians
  p_gain: 8.0
  i_gain: 0.1
  d_gain: 0.4
  i_max: 1.0
  i_min: -1.0
  cmd_max: 20.0
  cmd_min: -20.0
"""


def sim_controllers_yaml(path):
    if path.exists():                       # keep hand-tuned values
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(SIM_CONTROLLERS_YAML)


def plugins_xacro(path):
    S = SENSORS
    lid_min = -math.pi
    lid_max = math.pi - 2 * math.pi / S["lidar_samples"]     # exactly 0.5 deg steps, no duplicate ray
    plugins = GZ_PLUGINS_XACRO.replace("@MODEL@", MODEL_NAME)
    xml = plugins + f"""

  <!-- RPLiDAR A2M8-like (ASSUMPTION: 360 deg, 0.5 deg steps, 0.15-12 m, 10 Hz) -->
  <gazebo reference="laser_link">
    <sensor name="rplidar" type="gpu_lidar">
      <pose>0 0 0 0 0 0</pose>
      <topic>/lidar/scan</topic>
      <gz_frame_id>laser_link</gz_frame_id>
      <update_rate>{S['lidar_rate']}</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <lidar>
        <scan>
          <horizontal>
            <samples>{S['lidar_samples']}</samples>
            <resolution>1</resolution>
            <min_angle>{f(lid_min)}</min_angle>
            <max_angle>{f(lid_max)}</max_angle>
          </horizontal>
        </scan>
        <range><min>0.15</min><max>12.0</max><resolution>0.01</resolution></range>
        <noise><type>gaussian</type><mean>0.0</mean><stddev>0.01</stddev></noise>
      </lidar>
    </sensor>
  </gazebo>

  <!-- Kinect v1 RGB + registered depth. HFOV {f(S['cam_hfov'])} rad is derived from the README intrinsics
       (fx = 526.607 px @ 640 px) so ArUco pose estimation in sim matches pallet_pose_node. -->
  <gazebo reference="camera_link">
    <sensor name="rgb_camera" type="camera">
      <pose>0 0 0 0 0 0</pose>
      <topic>/camera/image</topic>
      <gz_frame_id>camera_link</gz_frame_id>
      <update_rate>{S['cam_rate']}</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>{f(S['cam_hfov'])}</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.05</near><far>20.0</far></clip>
      </camera>
    </sensor>
    <sensor name="depth_camera" type="depth_camera">
      <pose>0 0 0 0 0 0</pose>
      <topic>/camera/depth</topic>
      <gz_frame_id>camera_link</gz_frame_id>
      <update_rate>{S['cam_rate']}</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <camera>
        <horizontal_fov>{f(S['cam_hfov'])}</horizontal_fov>
        <image><width>640</width><height>480</height><format>R_FLOAT32</format></image>
        <clip><near>{f(S['depth_near'])}</near><far>{f(S['depth_far'])}</far></clip>
      </camera>
    </sensor>
  </gazebo>

  <!-- IMU (ASSUMPTION: 100 Hz, small gaussian noise; replace with BNO055 numbers once chosen, D19) -->
  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <pose>0 0 0 0 0 0</pose>
      <topic>/imu</topic>
      <gz_frame_id>imu_link</gz_frame_id>
      <update_rate>{S['imu_rate']}</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <imu>
        <angular_velocity>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.0003</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.0003</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.0003</stddev></noise></z>
        </angular_velocity>
        <linear_acceleration>
          <x><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></x>
          <y><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></y>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.017</stddev></noise></z>
        </linear_acceleration>
      </imu>
    </sensor>
  </gazebo>
</robot>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(xml)


def urdf_inertial(m, pose, inertia):
    ixx, iyy, izz = inertia
    return (f'<inertial><origin xyz="{nums(*pose)}" rpy="0 0 0"/><mass value="{f(m)}"/>'
            f'<inertia ixx="{f(ixx)}" ixy="0" ixz="0" iyy="{f(iyy)}" iyz="0" izz="{f(izz)}"/></inertial>')


def ubox(size, pose, rgb=None, tag="visual", name=None):
    nm = f' name="{name}"' if name else ""
    col = ""
    if rgb is not None and tag == "visual":
        col = f'<material name="{name}_m"><color rgba="{nums(*rgb)} 1"/></material>'
    return (f'<{tag}{nm}><origin xyz="{nums(*pose)}" rpy="0 0 0"/><geometry><box size="{nums(*size)}"/></geometry>'
            f"{col}</{tag}>")


def bootstrap_urdf(path):
    r = R
    ch, cx = r["chassis"], r["chassis_cx"]
    ch_pose = (cx, 0, 0.175 - r["base_z"])               # chassis spans world z 0.05..0.30
    wr, ww = r["wheel_radius"], r["wheel_width"]
    wheel_i = (1.5 / 12 * (3 * wr * wr + ww * ww), 1.5 / 2 * wr * wr, 1.5 / 12 * (3 * wr * wr + ww * ww))
    sph_i = 0.4 * 0.5 * r["caster_r"] ** 2
    ty = r["track_width"] / 2
    fy = POCKET_Y
    mx = r["mast_x"]
    lz_rel = r["lidar_z"] - r["base_z"]
    cam_rel = (r["cam_x"], 0, r["cam_z"] - r["base_z"])

    def cyl(rad, length, pose, rpy, rgb, name):
        return (f'<visual name="{name}"><origin xyz="{nums(*pose)}" rpy="{nums(*rpy)}"/><geometry>'
                f'<cylinder radius="{f(rad)}" length="{f(length)}"/></geometry>'
                f'<material name="{name}_m"><color rgba="{nums(*rgb)} 1"/></material></visual>')

    mast = "\n    ".join([
        ubox((0.04, 0.04, 1.2), (mx, +0.38, 0.6 - r["base_z"]), (0.25, 0.25, 0.28), name="mast_l"),
        ubox((0.04, 0.04, 1.2), (mx, -0.38, 0.6 - r["base_z"]), (0.25, 0.25, 0.28), name="mast_r"),
        ubox((0.04, 0.80, 0.04), (mx, 0, 1.18 - r["base_z"]), (0.25, 0.25, 0.28), name="mast_top"),
        cyl(0.02, 0.95, (0.0, 0, 0.775 - r["base_z"] + 0.0), (0, 0, 0), (0.15, 0.15, 0.15), "lidar_pole"),
        cyl(0.012, r["cam_z"] - 0.3, (r["cam_x"], 0, (0.3 + r["cam_z"]) / 2 - r["base_z"] - 0.0), (0, 0, 0),
            (0.15, 0.15, 0.15), "cam_pole"),
    ])
    # carriage (frame origin on the floor at the mast; +z up). plates behind each fork, crossbar at the bottom.
    ft, fl, fw = r["fork_t"], r["fork_len"], r["fork_w"]
    fz = r["fork_gap"] + ft / 2
    car_parts, car_col = [], []
    for sgn, tag in ((+1, "l"), (-1, "r")):
        car_parts.append(ubox((0.03, fw, 0.30), (0, sgn * fy, 0.15), (0.9, 0.7, 0.1), name=f"plate_{tag}"))
        car_parts.append(ubox((fl, fw, ft), (fl / 2 + 0.015, sgn * fy, fz), (0.85, 0.65, 0.1), name=f"fork_{tag}"))
        car_col.append(ubox((fl, fw, ft), (fl / 2 + 0.015, sgn * fy, fz), tag="collision"))
        car_col.append(ubox((0.03, fw, 0.30), (0, sgn * fy, 0.15), tag="collision"))
    car_parts.append(ubox((0.03, 0.62, 0.05), (0, 0, 0.025), (0.9, 0.7, 0.1), name="crossbar"))
    car_col.append(ubox((0.03, 0.62, 0.05), (0, 0, 0.025), tag="collision"))
    car_m = 6.0
    car_i = box_inertia(car_m, fl, 0.62, 0.30)

    fk = lambda v: v  # noqa
    xml = f"""<?xml version="1.0"?>
<!--
  wafl_sim_bootstrap.urdf.xacro - PLACEHOLDER robot so that gazebo.launch.py runs TODAY.
  GENERATED by scripts/generate_assets.py.  Replace it with wafl_description/urdf/wafl.urdf.xacro once that exists
  (it only has to keep the joint/link names listed in wafl_gz_plugins.xacro and include that file).

  ASSUMPTIONS (nothing here is measured): track {f(r['track_width'])} m, wheel radius {f(r['wheel_radius'])} m,
  caster {f(r['caster_x'])} m ahead of the axle, forks {f(fl)} m long at +-{f(fy)} m (EUR pocket centres),
  lift travel {f(r['fork_travel'])} m, base mass 55 kg with the battery at the rear (counterweight - without it the
  robot tips forward when it carries a pallet; the validator checks this).
  Frames: base_footprint (floor, rear-axle centre) -> base_link (axle height) -> sensors.
  The LiDAR is at z = {f(r['lidar_z'])} m (above the mast) so the 2D scan does not hit the robot itself;
  consequence: it does not see pallets/cargo (< 0.75 m) - only racks, cartons, walls.
-->
<robot name="{MODEL_NAME}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find wafl_simulation)/urdf/wafl_gz_plugins.xacro"/>

  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 {f(r['base_z'])}" rpy="0 0 0"/>
  </joint>

  <link name="base_link">
    {urdf_inertial(55.0, (0.05, 0, ch_pose[2]), box_inertia(55.0, *ch))}
    {ubox(ch, ch_pose, (0.2, 0.35, 0.6), name="chassis")}
    {ubox(ch, ch_pose, tag="collision")}
    {mast}
  </link>

  <!-- Front-left steering knuckle + drive wheel -->
  <link name="front_left_steer_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.5"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><cylinder radius="0.03" length="0.06"/></geometry><material name="steer_l_m"><color rgba="0.2 0.2 0.2 1"/></material></visual>
  </link>
  <joint name="front_left_steer_joint" type="revolute">
    <parent link="base_link"/><child link="front_left_steer_link"/>
    <origin xyz="0 {f(ty)} 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="20" velocity="5"/>
    <dynamics damping="0.05" friction="0"/>
  </joint>

  <link name="front_left_wheel_link">
    {urdf_inertial(1.5, (0, 0, 0), wheel_i)}
    <visual><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(wr)}" length="{f(ww)}"/></geometry><material name="wheel_l_m"><color rgba="0.05 0.05 0.05 1"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(wr)}" length="{f(ww)}"/></geometry></collision>
  </link>
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="front_left_steer_link"/><child link="front_left_wheel_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>

  <!-- Front-right steering knuckle + drive wheel -->
  <link name="front_right_steer_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.5"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><cylinder radius="0.03" length="0.06"/></geometry><material name="steer_r_m"><color rgba="0.2 0.2 0.2 1"/></material></visual>
  </link>
  <joint name="front_right_steer_joint" type="revolute">
    <parent link="base_link"/><child link="front_right_steer_link"/>
    <origin xyz="0 {f(-ty)} 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.5708" upper="1.5708" effort="20" velocity="5"/>
    <dynamics damping="0.05" friction="0"/>
  </joint>

  <link name="front_right_wheel_link">
    {urdf_inertial(1.5, (0, 0, 0), wheel_i)}
    <visual><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(wr)}" length="{f(ww)}"/></geometry><material name="wheel_r_m"><color rgba="0.05 0.05 0.05 1"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(wr)}" length="{f(ww)}"/></geometry></collision>
  </link>
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="front_right_steer_link"/><child link="front_right_wheel_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>

  <!-- Rear caster: steering joint (stepper on hardware) + passive rolling wheel -->
  <link name="caster_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.5"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial>
    <visual><origin xyz="0 0 {-r['caster_r']/2}" rpy="0 0 0"/><geometry><cylinder radius="0.02" length="{f(r['caster_r'])}"/></geometry><material name="caster_m"><color rgba="0.2 0.2 0.2 1"/></material></visual>
  </link>
  <joint name="caster_joint" type="revolute">
    <parent link="base_link"/><child link="caster_link"/>
    <origin xyz="{f(r['caster_x'])} 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.3963" upper="1.3963" effort="20" velocity="5"/>
    <dynamics damping="0.05" friction="0"/>
  </joint>

  <link name="caster_wheel_link">
    {urdf_inertial(0.5, (0, 0, 0), (0.0005, 0.0005, 0.0005))}
    <visual><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(r['caster_r'])}" length="0.04"/></geometry><material name="caster_wheel_m"><color rgba="0.05 0.05 0.05 1"/></material></visual>
    <collision><origin xyz="0 0 0" rpy="1.5708 0 0"/><geometry><cylinder radius="{f(r['caster_r'])}" length="0.04"/></geometry></collision>
  </link>
  <joint name="caster_wheel_joint" type="continuous">
    <parent link="caster_link"/><child link="caster_wheel_link"/>
    <origin xyz="0.03 0 {f(-r['caster_r'])}" rpy="0 0 0"/><axis xyz="0 1 0"/>
  </joint>

  <link name="fork_carriage_link">
    {urdf_inertial(car_m, (fl / 2, 0, 0.1), car_i)}
    {chr(10).join("    " + p for p in car_parts).strip()}
    {chr(10).join("    " + p for p in car_col).strip()}
  </link>
  <joint name="fork_joint" type="prismatic">
    <parent link="base_link"/><child link="fork_carriage_link"/>
    <origin xyz="{f(mx)} 0 {f(-r['base_z'])}" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="0" upper="{f(r['fork_travel'])}" effort="1500" velocity="0.3"/>
    <dynamics damping="20" friction="0"/>
  </joint>

  <link name="laser_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.2"/><inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/></inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><cylinder radius="0.04" length="0.05"/></geometry><material name="laser_m"><color rgba="0.8 0.1 0.1 1"/></material></visual>
  </link>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/><child link="laser_link"/><origin xyz="0 0 {f(lz_rel)}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.2"/><inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0002"/></inertial>
    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><box size="0.04 0.25 0.05"/></geometry><material name="cam_m"><color rgba="0.1 0.1 0.1 1"/></material></visual>
  </link>
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/><child link="camera_link"/><origin xyz="{nums(*cam_rel)}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial><origin xyz="0 0 0" rpy="0 0 0"/><mass value="0.05"/><inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/></inertial>
  </link>
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/><child link="imu_link"/><origin xyz="0.1 0 0.05" rpy="0 0 0"/>
  </joint>

  <gazebo reference="front_left_wheel_link"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
  <gazebo reference="front_right_wheel_link"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>
  <gazebo reference="caster_wheel_link"><mu1>0.01</mu1><mu2>0.01</mu2></gazebo>
  <gazebo reference="fork_carriage_link"><mu1>0.8</mu1><mu2>0.8</mu2></gazebo>
</robot>
"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(xml)


# --------------------------------------------------------------------------- #
#  Layout plot
# --------------------------------------------------------------------------- #
def layout_png(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, FancyArrow

    fig, ax = plt.subplots(figsize=(13, 8.5))
    ax.add_patch(Rectangle((-FLOOR_X / 2, -FLOOR_Y / 2), FLOOR_X, FLOOR_Y, fc="#e9e9e9", ec="k", lw=2))
    for cy in (RACK_CY, -RACK_CY):
        ax.add_patch(Rectangle((-RACK_LEN / 2, cy - RACK_DEPTH / 2), RACK_LEN, RACK_DEPTH, fc="#c9d6f2", ec="#1a3fa6", lw=1.5))
    for name, mid, row, bay in LOCATIONS:
        x, y, yaw = bay_pose(row, bay)
        ax.add_patch(Rectangle((x - (BAY_W - 0.2) / 2, y - (RACK_DEPTH - 0.1) / 2), BAY_W - 0.2, RACK_DEPTH - 0.1,
                               fc="none", ec="#2a8c3a" if mid is None else "#2a5bd7", lw=1, ls="--"))
        ax.text(x, y + (0.62 if row == "N" else -0.62), f"{name}" + ("" if mid is None else f"  (ID {mid})"),
                ha="center", va="center", fontsize=9, weight="bold")
        if mid is not None:
            c, s = math.cos(yaw), math.sin(yaw)
            w, d = PAL_W, PAL_D
            corners = [(-d / 2, -w / 2), (d / 2, -w / 2), (d / 2, w / 2), (-d / 2, w / 2)]
            pts = [(x + c * a - s * b, y + s * a + c * b) for a, b in corners]
            ax.add_patch(plt.Polygon(pts, fc="#d9b27c", ec="#6b4a1f"))
            ax.add_patch(FancyArrow(x - 0.2 * c, y - 0.2 * s, 0.4 * c, 0.4 * s, width=0.04, color="#6b4a1f"))
        ax_, ay_, ayaw = approach_pose(x, y, yaw, APPROACH_OFFSET)
        ax.plot(ax_, ay_, "^" if row == "N" else "v", color="#d6336c", ms=7)
    ax.add_patch(Rectangle((DOCK_X - 0.4, -0.4), 0.8, 0.8, fc="#f2d63d", ec="k"))
    ax.add_patch(Rectangle((DOCK_X + 0.375, -0.5), 0.05, 1.0, fc="#555", ec="k"))
    ax.text(DOCK_X, 0.75, f"DOCK (ID {DOCK_ID})", ha="center", fontsize=9, weight="bold")
    ax.plot(DOCK_X + 0.376 - DOCK_APPROACH_OFFSET, 0, ">", color="#d6336c", ms=7)
    ax.add_patch(FancyArrow(SPAWN[0], SPAWN[1], 0.9, 0, width=0.12, color="#e8590c"))
    ax.text(SPAWN[0], SPAWN[1] - 0.45, "SPAWN = map origin", ha="center", fontsize=9, color="#e8590c")
    ax.plot([], [], "^", color="#d6336c", label="approach pose (base_footprint, arrow = heading)")
    ax.legend(loc="lower right")
    ax.set_xlim(-FLOOR_X / 2 - 0.5, FLOOR_X / 2 + 0.5)
    ax.set_ylim(-FLOOR_Y / 2 - 0.5, FLOOR_Y / 2 + 0.5)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("WAFL 2027 warehouse (world frame) - blue = pre-loaded pallet, green = free drop-off bay")
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110, bbox_inches="tight")


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pkg", default=str(Path(__file__).resolve().parents[1]), help="wafl_simulation package dir")
    ap.add_argument("--perception", default=None, help="wafl_perception package dir (default: sibling)")
    ap.add_argument("--marker-size", type=float, default=0.15, help="black-square side length in metres")
    ap.add_argument("--flip-v", action="store_true", help="flip marker texture vertically")
    ap.add_argument("--mirror-u", action="store_true", help="mirror marker texture horizontally")
    ap.add_argument("--skip-urdf", action="store_true", help="do not (re)write the bootstrap URDF / plugin block")
    a = ap.parse_args()

    pkg = Path(a.pkg)
    perc = Path(a.perception) if a.perception else pkg.parent / "wafl_perception"
    models = pkg / "models"
    for mid in PALLET_IDS:
        pallet_model(models, mid, a.marker_size, a.flip_v, a.mirror_u)
    rack_model(models)
    dock_model(models, a.marker_size, a.flip_v, a.mirror_u)
    world_file(pkg / "worlds" / "warehouse.world")
    locations_yaml(perc / "config" / "locations.yaml", a.marker_size)
    layout_png(pkg / "docs" / "layout.png")
    if not a.skip_urdf:
        plugins_xacro(pkg / "urdf" / "wafl_gz_plugins.xacro")
        sim_controllers_yaml(pkg / "config" / "sim_controllers.yaml")
        bootstrap_urdf(pkg / "urdf" / "wafl_sim_bootstrap.urdf.xacro")
    print("generated into", pkg)


if __name__ == "__main__":
    main()
