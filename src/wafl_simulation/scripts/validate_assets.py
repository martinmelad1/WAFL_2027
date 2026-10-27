#!/usr/bin/env python3
"""
validate_assets.py - offline consistency checks for the wafl_simulation package.

Needs only python3 + PyYAML + OpenCV (cv2.aruco); does NOT need ROS or Gazebo, so it runs on any laptop/CI.
It cannot replace a first run in Gazebo - see README "First run checklist".

    python3 scripts/validate_assets.py            # exit code 0 = everything passed
"""
import ast
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml

PKG = Path(__file__).resolve().parents[1]
PERC = PKG.parent / "wafl_perception"
results = []


def check(name, ok, detail=""):
    results.append((ok, name, detail))
    print(("PASS  " if ok else "FAIL  ") + name + (f"   [{detail}]" if detail else ""))


# ----------------------------------------------------------------------------- XML + references
xml_files = sorted(list(PKG.glob("models/*/model.sdf")) + list(PKG.glob("models/*/model.config"))
                   + list(PKG.glob("worlds/*.world")) + list(PKG.glob("urdf/*.xacro")) + [PKG / "package.xml"])
bad = []
for p in xml_files:
    try:
        ET.parse(p)
    except ET.ParseError as e:
        bad.append(f"{p.name}: {e}")
check(f"{len(xml_files)} XML files are well-formed", not bad, "; ".join(bad))

models = {p.name for p in (PKG / "models").iterdir() if p.is_dir()}
cfg_bad = [m for m in models if ET.parse(PKG / "models" / m / "model.config").getroot().findtext("name") != m]
check("model.config <name> matches directory name", not cfg_bad, ",".join(cfg_bad))

world_path = PKG / "worlds" / "warehouse.world"
world = ET.parse(world_path).getroot()
wname = world.find("world").get("name")
check("world is named 'warehouse'", wname == "warehouse", wname)

uris = re.findall(r"model://([^<\"]+)", world_path.read_text())
for m in (PKG / "models").glob("*/model.sdf"):
    uris += re.findall(r"model://([^<\"]+)", m.read_text())
missing = []
for u in set(uris):
    target = PKG / "models" / u
    if not target.exists():
        missing.append(u)
check(f"all {len(set(uris))} model:// URIs resolve to files", not missing, ",".join(missing))

# ----------------------------------------------------------------------------- ArUco textures + mesh UVs
d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
for mid in (0, 1, 2, 3, 99):
    mdl = "wafl_charging_dock" if mid == 99 else f"wafl_pallet_{mid}"
    img = cv2.imread(str(PKG / "models" / mdl / "meshes" / f"aruco_{mid}.png"), cv2.IMREAD_GRAYSCALE)
    if hasattr(cv2.aruco, "ArucoDetector"):
        det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
        _, ids, _ = det.detectMarkers(img)
    else:
        _, ids, _ = cv2.aruco.detectMarkers(img, d)
    check(f"aruco_{mid}.png decodes as ID {mid} (DICT_4X4_100)", ids is not None and ids.flatten().tolist() == [mid])
    obj = (PKG / "models" / mdl / "meshes" / f"aruco_{mid}.obj").read_text().splitlines()
    mtl = (PKG / "models" / mdl / "meshes" / f"aruco_{mid}.mtl").read_text()
    v = np.array([[float(t) for t in l.split()[1:]] for l in obj if l.startswith("v ")])
    vt = np.array([[float(t) for t in l.split()[1:]] for l in obj if l.startswith("vt ")])
    n = np.cross(v[1] - v[0], v[2] - v[1])
    ok_n = n[0] < 0 and abs(n[1]) < 1e-9 and abs(n[2]) < 1e-9
    # viewer stands at -x, looks toward +x: viewer-left = +y, up = +z. image u grows to the viewer's RIGHT (-y), v grows UP (+z)
    cu = np.corrcoef(-v[:, 1], vt[:, 0])[0, 1]
    cv = np.corrcoef(v[:, 2], vt[:, 1])[0, 1]
    check(f"aruco_{mid}.obj: outward normal -x, UVs upright & unmirrored (OBJ v-up)", ok_n and cu > 0.9 and cv > 0.9,
          f"normal={n.round(3).tolist()} corr_u={cu:.2f} corr_v={cv:.2f}")
    check(f"aruco_{mid}.mtl references its png", f"map_Kd aruco_{mid}.png" in mtl)

# ----------------------------------------------------------------------------- robot description
plug_txt = (PKG / "urdf" / "wafl_gz_plugins.xacro").read_text()
urdf_txt = (PKG / "urdf" / "wafl_sim_bootstrap.urdf.xacro").read_text()


def expand(main, plugins):
    defaults = dict(re.findall(r'<xacro:arg name="(\w+)"\s+default="([^"]*)"', plugins))
    inner = re.search(r"<robot[^>]*>(.*)</robot>", plugins, re.S).group(1)
    inner = re.sub(r"<xacro:arg[^>]*/>", "", inner)
    main = re.sub(r"<xacro:include[^>]*/>", lambda m: inner, main)
    return re.sub(r"\$\(arg (\w+)\)", lambda m: defaults.get(m.group(1), ""), main)


robot = ET.fromstring(expand(urdf_txt, plug_txt).split("?>", 1)[1])
links = {l.get("name"): l for l in robot.findall("link")}
joints = {j.get("name"): j for j in robot.findall("joint")}
children = {j.find("child").get("link") for j in joints.values()}
roots = [l for l in links if l not in children]
check("URDF: single root link = base_footprint", roots == ["base_footprint"], str(roots))
check("URDF: every joint's parent/child link exists",
      all(j.find("parent").get("link") in links and j.find("child").get("link") in links for j in joints.values()))
need_j = ["front_left_steer_joint", "front_right_steer_joint", "front_left_wheel_joint", "front_right_wheel_joint", "caster_joint", "caster_wheel_joint", "fork_joint"]
need_l = ["laser_link", "camera_link", "imu_link"]
check("URDF has the joints/links the plugin block needs", all(j in joints for j in need_j) and all(l in links for l in need_l))
plug_joints = [j for j in set(re.findall(r'joint="(\w+)"', plug_txt) + re.findall(r"<joint_name>(\w+)<", plug_txt)) if j != "joint"]
check("every joint named in a gz plugin exists in the URDF", all(j in joints for j in plug_joints), ",".join(sorted(plug_joints)))
refs = re.findall(r'<gazebo reference="(\w+)"', plug_txt)
check("every <gazebo reference> link exists", all(r in links for r in refs), ",".join(refs))
inertia_bad = []
for name, l in links.items():
    i = l.find("inertial")
    if i is None:
        continue
    m = float(i.find("mass").get("value"))
    a = i.find("inertia")
    ixx, iyy, izz = (float(a.get(k)) for k in ("ixx", "iyy", "izz"))
    if m <= 0 or min(ixx, iyy, izz) <= 0 or ixx + iyy < izz or ixx + izz < iyy or iyy + izz < ixx:
        inertia_bad.append(name)
check("URDF inertias positive and satisfy the triangle inequality", not inertia_bad, ",".join(inertia_bad))

# wheel geometry (front drive wheels)
urdf_r = float(links["front_left_wheel_link"].find("collision/geometry/cylinder").get("radius"))
ty = abs(float(joints["front_left_steer_joint"].find("origin").get("xyz").split()[1])) * 2
check("front drive wheels have radius 0.1 m and track width matches URDF", abs(urdf_r - 0.1) < 1e-6 and ty > 0.3,
      f"radius {urdf_r}; track {ty}")


def xyz(j):
    return np.array([float(t) for t in joints[j].find("origin").get("xyz").split()])


# static tip-over check (joints at zero, no rpy used between links)
pos = {"base_footprint": np.zeros(3)}
for _ in range(10):
    for j in joints.values():
        p, c = j.find("parent").get("link"), j.find("child").get("link")
        if p in pos:
            pos[c] = pos[p] + np.array([float(t) for t in j.find("origin").get("xyz").split()])
M, C = 0.0, np.zeros(3)
for name, l in links.items():
    i = l.find("inertial")
    if i is None:
        continue
    m = float(i.find("mass").get("value"))
    C += m * (pos[name] + np.array([float(t) for t in i.find("origin").get("xyz").split()]))
    M += m
C /= M
pal_m = 9.0
pal_x = 0.78 + 0.015 + 0.4
C2 = (C * M + np.array([pal_x, 0, 0.2]) * pal_m) / (M + pal_m)
caster_x = float(joints["caster_joint"].find("origin").get("xyz").split()[0])
check("static tip-over: CoM with a lifted pallet stays inside the wheel/caster triangle", 0.0 < C2[0] < caster_x - 0.05,
      f"CoM x = {C[0]:.2f} m empty, {C2[0]:.2f} m loaded, caster at {caster_x:.2f} m")

# forks vs EUR pockets
pocket_c, pocket_half = 0.26375, (0.455 - 0.0725) / 2
fy = float([l for l in links["fork_carriage_link"].findall("collision")
            if abs(float(l.find("origin").get("xyz").split()[0]) - 0.415) < 1e-6][0].find("origin").get("xyz").split()[1])
check("forks centred in EUR pockets with >5 cm side clearance", abs(abs(fy) - pocket_c) < 0.01 and pocket_half - 0.06 > 0.05,
      f"fork y={fy}, pocket centre {pocket_c}, clearance {pocket_half - 0.06:.3f} m")

# ----------------------------------------------------------------------------- bridge
bridge = yaml.safe_load((PKG / "config" / "ros_gz_bridge.yaml").read_text())
by_ros = {b["ros_topic_name"]: b for b in bridge}
expect = {  # ros topic -> (gz topic, ros type, gz type, direction)  - from README sec. 6
    "/scan": ("/lidar/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
    "/camera/image_raw": ("/camera/image", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
    "/camera/depth/image_raw": ("/camera/depth", "sensor_msgs/msg/Image", "gz.msgs.Image", "GZ_TO_ROS"),
    "/imu": ("/imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
    "/clock": ("/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS"),
    "/odom": ("/model/wafl/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
    "/front_steer_cmd_pos": ("/model/wafl/front_steer/cmd_pos", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    "/castor_cmd_pos": ("/model/wafl/joint/caster_joint/cmd_pos", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    "/front_left_wheel_cmd_vel": ("/model/wafl/joint/front_left_wheel_joint/cmd_vel", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    "/front_right_wheel_cmd_vel": ("/model/wafl/joint/front_right_wheel_joint/cmd_vel", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    "/fork_cmd_vel": ("/model/wafl/joint/fork_joint/cmd_vel", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"),
    "/joint_states": ("/world/warehouse/model/wafl/joint_state", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"),
}
mism = []
for t, (g, rt, gt, dr) in expect.items():
    b = by_ros.get(t)
    if not b or (b["gz_topic_name"], b["ros_type_name"], b["gz_type_name"], b["direction"]) != (g, rt, gt, dr):
        mism.append(t)
check("bridge entries match the README sec. 6 mapping (names, types, directions)", not mism, ",".join(mism))
extra = set(by_ros) - set(expect)
check("bridge has no unexpected entries; /cmd_vel and TF are not bridged", not extra and "/cmd_vel" not in by_ros
      and not any("/tf" in b["gz_topic_name"] for b in bridge), ",".join(sorted(extra)))
gz_in_plugins = {"/lidar/scan", "/camera/image", "/camera/depth", "/imu", "/model/wafl/odometry",
                 "/model/wafl/front_steer/cmd_pos",
                 "/model/wafl/joint/front_left_wheel_joint/cmd_vel",
                 "/model/wafl/joint/front_right_wheel_joint/cmd_vel",
                 "/model/wafl/joint/caster_joint/cmd_pos", "/model/wafl/joint/fork_joint/cmd_vel"}
not_in = [g for g in gz_in_plugins if g not in plug_txt and g.replace("front_left_wheel_joint", "${joint}").replace("front_right_wheel_joint", "${joint}") not in plug_txt]
check("every bridged gz topic is produced/consumed by wafl_gz_plugins.xacro", not not_in, ",".join(not_in))
check("OdometryPublisher TF goes to an unbridged topic (EKF owns odom->base_footprint)",
      "<tf_topic>/model/wafl/tf_unused" in plug_txt)
check("sensor frame ids match the README (laser_link, camera_link)",
      plug_txt.count("<gz_frame_id>laser_link<") == 1 and plug_txt.count("<gz_frame_id>camera_link<") == 2)
hfov = float(re.search(r"<horizontal_fov>([\d.]+)<", plug_txt).group(1))
check("camera HFOV reproduces README fx=526.607 at 640 px", abs(320 / math.tan(hfov / 2) - 526.607) < 0.2,
      f"fx = {320 / math.tan(hfov / 2):.2f}")

# ----------------------------------------------------------------------------- world <-> locations.yaml
loc = yaml.safe_load((PERC / "config" / "locations.yaml").read_text())
inc = {i.findtext("name"): [float(t) for t in i.findtext("pose").split()] for i in world.iter("include")}
spawn = loc["spawn"]["world"]
ok = True
detail = []
for l in loc["locations"]:
    if l["marker_id"] is None:
        continue
    pose = inc.get(f"pallet_{l['label']}")
    if pose is None or abs(pose[0] - l["pallet_pose"]["world"][0]) > 1e-3 or abs(pose[1] - l["pallet_pose"]["world"][1]) > 1e-3 \
            or abs(pose[5] - l["pallet_pose"]["world"][2]) > 1e-3:
        ok = False
        detail.append(l["label"])
    mx, my, _ = l["pallet_pose"]["map"]
    wx, wy, _ = l["pallet_pose"]["world"]
    if abs(mx - (wx - spawn[0])) > 1e-3 or abs(my - (wy - spawn[1])) > 1e-3:
        ok = False
        detail.append(l["label"] + "-map")
    mdl = ET.parse(PKG / "models" / f"wafl_pallet_{l['marker_id']}" / "model.sdf").getroot().find("model").get("name")
    if mdl != f"wafl_pallet_{l['marker_id']}" or not any(f"wafl_pallet_{l['marker_id']}" in (e.findtext("uri") or "")
                                                          and e.findtext("name") == f"pallet_{l['label']}" for e in world.iter("include")):
        ok = False
        detail.append(l["label"] + "-model")
check("locations.yaml pallet poses/IDs match the world includes (world and map frames)", ok, ",".join(detail))
ids = sorted(l["marker_id"] for l in loc["locations"] if l["marker_id"] is not None)
check("locations.yaml: pallet IDs 0-3 + dock 99, DICT_4X4_100", ids == [0, 1, 2, 3] and loc["charging_dock"]["marker_id"] == 99
      and loc["aruco"]["dictionary"] == "DICT_4X4_100")

# clearance: approach poses inside the aisle (|y| < 1.75 - 0.5), pallets inside their bay
aisle_ok = all(abs(l["approach_pose"]["world"][1]) < 1.75 - 0.2 for l in loc["locations"])
check("approach poses are inside the aisle", aisle_ok)
spawn_clear = abs(spawn[1]) < 1.0 and abs(spawn[0]) < 6.5
check("robot spawn is inside the hall, in the aisle", spawn_clear)

# ----------------------------------------------------------------------------- launch file
lt = (PKG / "launch" / "gazebo.launch.py").read_text()
tree = ast.parse(lt)
declared = re.findall(r"DeclareLaunchArgument\('(\w+)'", lt)
used = re.findall(r"LaunchConfiguration\('(\w+)'\)", lt)
check("launch: every LaunchConfiguration is declared", set(used) <= set(declared), str(set(used) - set(declared)))
check("launch: use_sim_time argument exists (team rule 5)", "use_sim_time" in declared)
m = re.search(r"WORLD_NAME = '(\w+)'.*MODEL_NAME = '(\w+)'", lt, re.S)
check("launch spawns world/model names that match the bridge", m and m.groups() == ("warehouse", "wafl"))
sp = [re.search(rf"'spawn_{k}', default_value='([^']+)'", lt).group(1) for k in ("x", "y", "yaw")]
check("launch spawn defaults equal the layout spawn pose in locations.yaml",
      [float(x) for x in sp] == [float(spawn[0]), float(spawn[1]), float(spawn[2])], str(sp))
check("launch: bridge node does not use use_sim_time (it provides /clock)",
      "parameters=[{'config_file': LaunchConfiguration('bridge_config')}]" in lt
      or "parameters=[{'config_file': bridge_config}]" in lt)

# ----------------------------------------------------------------------------- summary
fails = [r for r in results if not r[0]]
print(f"\n{len(results) - len(fails)}/{len(results)} checks passed")
sys.exit(1 if fails else 0)
