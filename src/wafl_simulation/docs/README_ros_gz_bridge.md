# `ros_gz_bridge.yaml` — Gazebo Harmonic <-> ROS 2 Jazzy topic bridge

Configuration for `ros_gz_bridge`'s `parameter_bridge`, in `wafl_simulation/config/`. It is loaded by `gazebo.launch.py`:

```
parameter_bridge --ros-args -p config_file:=<this file>
```

## Name contracts
| Item | Value |
|------|-------|
| World name | `warehouse` (`/world/warehouse/...`) |
| Model name | `wafl` (`/model/wafl/...`) |

The Gazebo-side topics are produced or consumed by the robot plugins (`urdf/wafl_gz_plugins.xacro`) and `worlds/warehouse.world`. If you rename a topic here, change it there too.

## Gazebo -> ROS (`GZ_TO_ROS`)

| ROS topic | Gazebo topic | ROS type | Notes |
|-----------|--------------|----------|-------|
| `/clock` | `/clock` | `rosgraph_msgs/Clock` | Required: every `use_sim_time` node stalls without it. |
| `/scan` | `/lidar/scan` | `sensor_msgs/LaserScan` | RPLiDAR A2M8-like, frame `laser_link`. |
| `/camera/image_raw` | `/camera/image` | `sensor_msgs/Image` | Kinect v1 RGB, 640x480 rgb8. |
| `/camera/depth/image_raw` | `/camera/depth` | `sensor_msgs/Image` | Depth, 32FC1 in **metres**. |
| `/imu` | `/imu` | `sensor_msgs/Imu` | Raw sim IMU. Hardware publishes `/imu/data` instead. In sim it goes through `odom_covariance_fix` to `/imu_fixed`. |
| `/odom` | `/model/wafl/odometry` | `nav_msgs/Odometry` | `frame_id: odom`, `child_frame_id: base_footprint`, **zero covariance**, so it goes through `odom_covariance_fix` to `/odom_fixed` and then the EKF. |
| `/joint_states` | `/world/warehouse/model/wafl/joint_state` | `sensor_msgs/JointState` | Gazebo type is `gz.msgs.Model`. Feeds `robot_state_publisher` and `sim_fork_adapter`. |

## ROS -> Gazebo (`ROS_TO_GZ`)

| ROS topic | Gazebo topic | ROS type | Notes |
|-----------|--------------|----------|-------|
| `/front_steer_cmd_pos` | `/model/wafl/front_steer/cmd_pos` | `std_msgs/Float64` | Front steering angle alpha in radians. Both front steering joints listen to this one gz topic, so they stay parallel. Written by `base_controller`. |
| `/front_left_wheel_cmd_vel` | `/model/wafl/joint/front_left_wheel_joint/cmd_vel` | `std_msgs/Float64` | Front-left wheel angular velocity in rad/s. Written by `base_controller`. |
| `/front_right_wheel_cmd_vel` | `/model/wafl/joint/front_right_wheel_joint/cmd_vel` | `std_msgs/Float64` | Front-right wheel angular velocity in rad/s. Written by `base_controller`. |
| `/castor_cmd_pos` | `/model/wafl/joint/caster_joint/cmd_pos` | `std_msgs/Float64` | Rear steering angle beta in radians. The "castor" spelling is kept from 2026. Written by `base_controller`. |
| `/fork_cmd_vel` | `/model/wafl/joint/fork_joint/cmd_vel` | `std_msgs/Float64` | Fork joint velocity in m/s, + = up. Written by `sim_fork_adapter`. |

## Deliberately NOT bridged
- **`/model/wafl/tf*` (OdometryPublisher TF).** The EKF is the only publisher of `odom -> base_footprint`; bridging this would create a second publisher.
- **`/cmd_vel`.** Gazebo must never read raw `/cmd_vel`. Only the per-joint commands published by `base_controller` after arbitration reach the robot, so manual, fork and dock commands cannot bypass the e-stop and priority logic.

## Adding a topic
Append an entry:

```yaml
- ros_topic_name: "/my_topic"
  gz_topic_name: "/model/wafl/my_topic"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: GZ_TO_ROS      # or ROS_TO_GZ (or BIDIRECTIONAL)
```

Then update the Master Topic Index in the main README, and rebuild and relaunch.

## Debugging
```bash
gz topic -l                    # topics on the Gazebo side
ros2 topic list -t             # topics on the ROS side
ros2 topic hz /scan            # check sensor data flows
ros2 topic echo /clock --once  # check the clock bridge
```
If a ROS topic is missing, check that the Gazebo topic name matches the plugin config exactly (`gz topic -l`) and that the ROS/Gazebo message type pair is a supported combination.
