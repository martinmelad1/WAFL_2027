"""Builds the simulation copy of the robot description IN MEMORY. WAFL2025.xacro is never modified."""
import math
import xml.etree.ElementTree as ET

import xacro

LEGACY_SENSORS = {'rplidar_a2', 'kinect_sensor', 'imu_sensor'}   # Gazebo-Classic blocks in WAFL2025.xacro
WEBS = (('leftweb', 'leftwheel'), ('rightweb', 'rightwheel'))


def build_robot_description(xacro_path):
    root = ET.fromstring(xacro.process_file(xacro_path).toxml().encode('utf-8'))
    joints = {j.get('name'): j for j in root.findall('joint')}

    # 1) webs -> fixed, rotated so the wheel axis is lateral (+Y = forward rolling)
    if all(web in joints and wheel in joints for web, wheel in WEBS):
        for web, wheel in WEBS:
            psi = float(joints[wheel].find('origin').get('rpy').split()[2])
            j = joints[web]
            j.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                for e in j.findall(tag):
                    j.remove(e)
            j.find('origin').set('rpy', '0 0 %.6f' % (math.pi - psi))

    # 2) drop legacy Gazebo-Classic / ros_control content
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

    return ET.tostring(root, encoding='unicode')