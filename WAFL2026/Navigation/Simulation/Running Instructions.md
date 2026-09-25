# WAFL2026 — Running Instructions

**Stack:** ROS 2 Humble + Ignition Gazebo

---

## 1. Open a Terminal and Source the Workspace

```bash
cd ~/GP2_nav/ws_lidar
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 2. Kill any Stale Gazebo Instances

> Important — do this before every launch to avoid hangs.

```bash
killall -9 ign
pkill -f gazebo
```

---

## 3. Launch the Simulation

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch wafl2026 humble4.launch.py
```

Wait until Gazebo finishes loading before moving on.

---

## 4. Open RViz (new terminal)

```bash
cd ~/GP2_nav/ws_lidar
source install/setup.bash

ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

---

## 5. If the Map Doesn't Appear

Check the `map_server` lifecycle state:

```bash
ros2 lifecycle get /map_server
```

If it returns `unconfigured`, bring it up manually:

```bash
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

---

## 6. Localization (AMCL)

In RViz:

1. Click **2D Pose Estimate** in the top toolbar.
2. Click on the robot's actual position on the map.
3. Drag the arrow in the direction the robot is facing.

When the green particle cloud appears, AMCL is running correctly.

---

## 7. Send a Navigation Goal

In RViz, from the top toolbar pick **Nav2 Goal** (or **2D Goal Pose**), then:

1. Click the goal location on the map.
2. Drag the arrow to set the desired heading.

The robot will start moving toward the goal automatically.

### Alternative — Bishoy's Navigation Launch

```bash
cd ~/GP2_nav/ws_lidar
source install/setup.bash
ros2 launch wafl_navigation navigation.launch.py
```

### Caster Controller (Bishoy)

```bash
cd ~/GP2_nav/ws_lidar
source install/setup.bash
ros2 run wafl_navigation caster_controller
```

---

## 8. If Gazebo Freezes

```bash
killall -9 ign
LIBGL_ALWAYS_SOFTWARE=1 ign gazebo -r
```

If Gazebo then opens normally, re-launch the project:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch wafl2026 humble4.launch.py
```

---

## Recommended VM Settings (If you are not using Dualboot)

| Setting                 | Value   |
| ----------------------- | ------- |
| RAM                     | 16 GB   |
| Processors              | 4       |
| Graphics Memory         | 128 MB  |
| Accelerate 3D Graphics  | ON      |
