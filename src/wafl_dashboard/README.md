# WAFL 2027 dashboard

This is the browser UI and ROS 2 bridge for the WAFL autonomous forklift. FastAPI runs on the ROS machine (sim host or Jetson); the operator browser does not need ROS or DDS.

## Request and data flow

```text
Browser: static HTML/CSS/JS
  ├─ WebSocket /ws: status, pose, mission and health updates; joystick commands
  ├─ HTTP /api/*: snapshots, map PNG, safety/mode/manual commands
  └─ HTTP /stream/*: MJPEG camera bytes
             ↕
FastAPI routes + DashboardRosNode (rclpy spins in a background thread)
             ↕ local ROS 2 topics / DDS
```

FastAPI owns the browser-facing protocols and serves `static/`. `DashboardRosNode` owns ROS subscriptions and publishers. ROS callbacks update the latest snapshot and forward events to connected browsers using the asyncio event loop. ROS never runs on FastAPI's event-loop thread. Run one Uvicorn worker because each worker would create a separate ROS node and duplicate publishers.

## Run

Build and source the workspace (ROS 2 Jazzy must be installed):

```bash
cd <workspace>
rosdep install --from-paths src --ignore-src -r -y
sudo apt update
sudo apt install python3-fastapi python3-uvicorn python3-pil
colcon build --packages-select wafl_dashboard
source install/setup.bash
ros2 launch wafl_dashboard dashboard.launch.py port:=8000
```

Open `http://localhost:8000`. Ubuntu 24.04 marks its system Python as externally managed, so install the dashboard libraries with `apt` rather than system-wide `pip`. Ubuntu Noble provides these Python packages in its repositories. The page and its CSS/JS must be served by FastAPI; opening `index.html` directly or with `python -m http.server` does not provide the API/WebSocket.

## Routes and topics

| Browser endpoint | ROS connection |
|---|---|
| `GET /` and `/css/*`, `/js/*` | Serves the dashboard files |
| `WS /ws` | Snapshot + live status/pose/battery/map/health/log updates; incoming `{"type":"joy","v":0.3,"w":0}` |
| `GET /api/snapshot`, `/api/health`, `/api/logs` | Latest ROS data and log ring buffer |
| `GET /api/map.png`, `/api/map/meta` | `/map` occupancy grid rendered as PNG, plus geometry |
| `GET /stream/camera`, `/stream/debug` | `/camera/stream/compressed`, `/pallet/debug_image` as MJPEG |
| `POST /api/estop` | `/e_stop`; activation also publishes `/gui/mode = MANUAL` |
| `POST /api/mode` | `/gui/mode`; refuses AUTO if E-stop is latched |
| WebSocket joystick, `POST /api/lift`, `/api/lights` | `/manual_cmd_vel`, `/lift_cmd`, `/lights_cmd`; only accepted in MANUAL and without E-stop |
| `POST /api/mission` | JSON on `/wms/manual_mission` (`std_msgs/String`) |

Subscriptions include `/map`, TF `map → base_footprint`, `/odometry/filtered`, `/plan`, `/robot/state`, `/fork_height`, `/battery/state`, `/wms/mission_status`, `/wms/package_list`, `/wms/location_grid`, `/wms/connection_status`, `/rosout`, `/camera/stream/compressed`, and `/pallet/debug_image`. ESP32 heartbeat timestamps use `/encoder_count`, `/imu/data`, and `/fork_height`.

The browser sends joystick commands at 10 Hz while a control is held and sends zero on release. The server publishes zero after 0.3 seconds without a joystick update and when a WebSocket closes. The base controller's 0.5 second command timeout is another layer. This web E-stop is a software command over the network; it is not a substitute for the physical power-cut E-stop described in the root README.

This checkout does not yet contain `wafl_interfaces` or the other ROS application packages described in the root README. The dashboard therefore treats the custom mission message as an optional runtime dependency: mission status appears once `wafl_interfaces` is built and sourced. The ordinary ROS message subscriptions and browser server can still start without it.

## Current page limitations

The checked-in HTML currently has the Robot view wired for telemetry, map, camera and manual controls. Logs and WMS tabs are placeholders even though the API carries log and WMS messages. The current subscribed topics do not provide package quantity, payload weight, or ETA, so those fields cannot yet show live values. Mission status is a custom `wafl_interfaces/WmsMissionStatus` message; package and location snapshots from the WMS bridge are JSON strings as specified by the root README.
