"""Builds the simulation copy of the robot description IN MEMORY. WAFL2025.xacro is never modified.

What the patch does (old 2025 URDF -> README section 6 sim contract):
  * front webs (leftweb/rightweb) -> revolute steering joints, axis +z  (positive = wheel points left)
  * castorjoint                   -> revolute rear steering joint, wheel axis made lateral at beta = 0
  * wheel collisions              -> cylinders (mesh-vs-ground contact is the main source of jitter)
  * base_footprint                -> moved to the front-axle midpoint (command point of base_controller)
  * joint dynamics                -> low friction on wheels, moderate damping on steering joints
"""
import math
import os
import xml.etree.ElementTree as ET

import xacro

LEGACY_SENSORS = {'rplidar_a2', 'kinect_sensor', 'imu_sensor'}   # Gazebo-Classic blocks in WAFL2025.xacro
WEBS = (('leftweb', 'leftwheel'), ('rightweb', 'rightwheel'))    # (front steering joint, front wheel joint)
CASTOR, CASTOR_WHEEL = 'castorjoint', 'castorwheel'
WHEEL_LINKS = ('left_wheel', 'right_wheel', 'castor_wheel')

WHEEL_RADIUS = 0.10
# link -> (axial width along local z, collision z-offset), measured from the STL files
WHEEL_DIMS = {'left_wheel': (0.25, 0.0), 'right_wheel': (0.25, 0.0), 'castor_wheel': (0.13, -0.056)}
STEER_LIMIT = 1.65            # rad (~94 deg): just beyond the +-90 deg the controller can command
COM_X = 0.20          # m in base_link. CAD value 0.4476 is AHEAD of the front axle (0.375): the weight sits on the fork tips
CHASSIS_BOTTOM_Z = -0.45   # m in base_link: the box starts ~0.16 m above the floor (base_link sits 0.609 m up)
CHASSIS_X_MAX = 0.60       # m in base_link: the box stops before the forks (forks get their own collision later)
CHASSIS_MU = 0.02     # near-frictionless chassis/fork contact, so a touching fork tip cannot drag the robot
FOOTPRINT_AT_FRONT_AXLE = True


def _floats(text):
    return [float(x) for x in text.split()]


def _replace(joint, tag, **attrs):
    for e in joint.findall(tag):
        joint.remove(e)
    ET.SubElement(joint, tag, {k: str(v) for k, v in attrs.items()})

def _chassis_box():
    """Bounding box (size, centre) of the upper chassis mesh, in the base_link frame."""
    import numpy as np
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory('wafl_description'), 'meshes', 'base_link.STL')
    raw = open(path, 'rb').read()
    if raw[:5] == b'solid' and b'facet' in raw[:1000]:
        v = np.array([[float(x) for x in l.split()[1:4]]
                      for l in raw.decode().splitlines() if l.strip().startswith('vertex')])
    else:
        cnt = int(np.frombuffer(raw[80:84], '<u4')[0])
        dt = np.dtype([('n', '<f4', (3,)), ('v', '<f4', (3, 3)), ('a', '<u2')])
        v = np.frombuffer(raw[84:84 + 50 * cnt], dtype=dt)['v'].reshape(-1, 3)
    keep = v[(v[:, 2] >= CHASSIS_BOTTOM_Z) & (v[:, 0] <= CHASSIS_X_MAX)]
    lo, hi = keep.min(0), keep.max(0)
    return hi - lo, (hi + lo) / 2.0

def build_robot_description(xacro_path):
    root = ET.fromstring(xacro.process_file(xacro_path).toxml().encode('utf-8'))
    joints = {j.get('name'): j for j in root.findall('joint')}
    links = {l.get('name'): l for l in root.findall('link')}

    # 1) front steering joints. The web is rotated so the wheel axis is lateral at alpha = 0.
    for web, wheel in WEBS:
        psi = _floats(joints[wheel].find('origin').get('rpy'))[2]
        j = joints[web]
        j.set('type', 'revolute')
        j.find('origin').set('rpy', '0 0 %.6f' % (math.pi - psi))
        _replace(j, 'axis', xyz='0 0 1')
        _replace(j, 'limit', lower=-STEER_LIMIT, upper=STEER_LIMIT, effort=60, velocity=3.0)
        _replace(j, 'dynamics', damping=0.8, friction=0.05)
        _replace(joints[wheel], 'dynamics', damping=0.02, friction=0.0)

    # 2) rear steering joint. Same trick: the 2025 file leaves the wheel axis 25 deg off lateral at beta = 0.
    cj, cw = joints[CASTOR], joints[CASTOR_WHEEL]
    psi_c = _floats(cw.find('origin').get('rpy'))[2]
    cj.set('type', 'revolute')
    cj.find('origin').set('rpy', '0 0 %.6f' % (-psi_c))
    _replace(cj, 'axis', xyz='0 0 1')
    _replace(cj, 'limit', lower=-STEER_LIMIT, upper=STEER_LIMIT, effort=40, velocity=3.0)
    _replace(cj, 'dynamics', damping=0.5, friction=0.02)
    _replace(cw, 'dynamics', damping=0.01, friction=0.0)          # passive: must roll freely

    # 3) base_footprint at the midpoint of the front axle
    if FOOTPRINT_AT_FRONT_AXLE:
        lx, ly, _ = _floats(joints['leftweb'].find('origin').get('xyz'))
        rx, ry, _ = _floats(joints['rightweb'].find('origin').get('xyz'))
        fj = joints['chassis_base_footprint']
        x, y, z = _floats(fj.find('origin').get('xyz'))
        fj.find('origin').set('xyz', '%.6f %.6f %.6f' % (x - (lx + rx) / 2, y - (ly + ry) / 2, z))

    # 3b) sim workaround: CoM between the axles, and no friction on the chassis/fork mesh
    com = links['base_link'].find('inertial').find('origin')
    cx, cy, cz = _floats(com.get('xyz'))
    com.set('xyz', '%.5f %.5f %.5f' % (COM_X, cy, cz))
    gb = ET.SubElement(root, 'gazebo', reference='base_link')
    ET.SubElement(gb, 'mu1').text = str(CHASSIS_MU)
    ET.SubElement(gb, 'mu2').text = str(CHASSIS_MU)

    # 4) drop legacy Gazebo-Classic / ros_control content
    for g in list(root.findall('gazebo')):
        for s in list(g.findall('sensor')):
            if s.get('name') in LEGACY_SENSORS:
                g.remove(s)
        for p in list(g.findall('plugin')):
            if 'libgazebo_ros' in (p.get('filename') or ''):
                g.remove(p)
        if len(g) == 0:
            root.remove(g)
    for t in root.findall('transmission'):
        root.remove(t)

       # 5) wheel collisions -> cylinders + friction
    for name in WHEEL_LINKS:
        link = links[name]
        for c in link.findall('collision'):
            link.remove(c)
        width, zoff = WHEEL_DIMS[name]
        col = ET.SubElement(link, 'collision')
        ET.SubElement(col, 'origin', xyz='0 0 %s' % zoff, rpy='0 0 0')
        geo = ET.SubElement(col, 'geometry')
        ET.SubElement(geo, 'cylinder', radius=str(WHEEL_RADIUS), length=str(width))
        g = ET.SubElement(root, 'gazebo', reference=name)
        ET.SubElement(g, 'mu1').text = '2.0'
        ET.SubElement(g, 'mu2').text = '2.0'
            # simple chassis collision: one box clear of the floor instead of the meshes
    for name in ('base_link', 'left_web', 'right_web', 'castor'):
        for c in links[name].findall('collision'):
            links[name].remove(c)
    size, ctr = _chassis_box()
    col = ET.SubElement(links['base_link'], 'collision')
    ET.SubElement(col, 'origin', xyz='%.4f %.4f %.4f' % tuple(ctr), rpy='0 0 0')
    geo = ET.SubElement(col, 'geometry')
    ET.SubElement(geo, 'box', size='%.4f %.4f %.4f' % tuple(size))
    return ET.tostring(root, encoding='unicode')