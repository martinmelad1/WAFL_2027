"""ROS 2 node that bridges dashboard HTTP/WebSocket clients to ROS topics."""
import json
import math
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import BatteryState, CompressedImage
from std_msgs.msg import Bool, Float32, Int32, Int32MultiArray, String
try:
    from wafl_interfaces.msg import WmsMissionStatus
except ImportError:
    WmsMissionStatus = None
from tf2_ros import Buffer, TransformListener

from .map_render import render_occupancy_grid


class DashboardRosNode(Node):
    """Owns ROS publishers/subscribers; callbacks update a thread-safe snapshot."""

    def __init__(self, loop, broadcast):
        super().__init__("dashboard_server")
        self.loop, self.broadcast = loop, broadcast
        self.lock = threading.RLock()
        self.data = {
            "mode": "MANUAL", "estop": False, "state": "UNKNOWN",
            "battery": None, "speed": 0.0, "fork_height_mm": None,
            "pose": None, "mission_status": None, "package_list": None,
            "location_grid": None, "wms_connected": False,
        }
        self.logs = deque(maxlen=500)
        self.last_seen = {}
        self.map_png, self.map_meta = None, None
        self.camera = {"camera": None, "debug": None}
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_estop = self.create_publisher(Bool, "/e_stop", latched)
        self.pub_mode = self.create_publisher(String, "/gui/mode", latched)
        self.pub_cmd = self.create_publisher(Twist, "/manual_cmd_vel", 10)
        self.pub_lift = self.create_publisher(Int32, "/lift_cmd", 10)
        self.pub_lights = self.create_publisher(Int32MultiArray, "/lights_cmd", 10)
        self.pub_mission = self.create_publisher(String, "/wms/manual_mission", 10)

        self.create_subscription(OccupancyGrid, "/map", self.on_map, 1)
        self.create_subscription(Odometry, "/odometry/filtered", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(Path, "/plan", self.on_plan, 5)
        self.create_subscription(String, "/robot/state", self.on_state, latched)
        self.create_subscription(Float32, "/fork_height", self.on_fork, 10)
        self.create_subscription(BatteryState, "/battery/state", self.on_battery, 10)
        self.create_subscription(String, "/gui/mode", self.on_mode, latched)
        self.create_subscription(Bool, "/e_stop", self.on_estop, latched)
        if WmsMissionStatus is not None:
            self.create_subscription(WmsMissionStatus, "/wms/mission_status", self.on_mission_status, 10)
        else:
            self.get_logger().warning("wafl_interfaces is not installed; WMS mission status subscription is disabled")
        self.create_subscription(String, "/wms/package_list", lambda m: self.save_json("package_list", m.data), 10)
        self.create_subscription(String, "/wms/location_grid", lambda m: self.save_json("location_grid", m.data), 10)
        self.create_subscription(Bool, "/wms/connection_status", self.on_wms, 10)
        self.create_subscription(CompressedImage, "/camera/stream/compressed", lambda m: self.on_image("camera", m), qos_profile_sensor_data)
        self.create_subscription(CompressedImage, "/pallet/debug_image", lambda m: self.on_image("debug", m), qos_profile_sensor_data)
        self.create_subscription(Log, "/rosout", self.on_log, 100)
        for topic, typ in (("/encoder_count", Float32), ("/imu/data", object), ("/fork_height", Float32)):
            if typ is object:
                from sensor_msgs.msg import Imu
                typ = Imu
            self.create_subscription(typ, topic, lambda _m, t=topic: self.heartbeat(t), qos_profile_sensor_data)
        self.create_timer(0.1, self.publish_tf_pose)
        self.create_timer(0.1, self.deadman)
        self.create_timer(2.0, self.publish_health)
        self.create_timer(0.5, self.publish_status)

    def emit(self, kind, data):
        with self.lock:
            if kind in ("status", "mission_status", "health", "wms", "pose"):
                self.data[kind] = data
        if self.loop and not self.loop.is_closed():
            self.loop.call_soon_threadsafe(lambda: __import__("asyncio").create_task(self.broadcast({"type": kind, "data": data})))

    def heartbeat(self, topic): self.last_seen[topic] = time.monotonic()
    def on_state(self, msg): self.data["state"] = msg.data; self.emit("status", self.status_payload())
    def on_mode(self, msg): self.data["mode"] = msg.data.upper(); self.emit("status", self.status_payload())
    def on_estop(self, msg): self.data["estop"] = bool(msg.data); self.emit("status", self.status_payload())
    def on_fork(self, msg): self.data["fork_height_mm"] = float(msg.data); self.heartbeat("/fork_height")

    def on_battery(self, msg):
        self.data["battery"] = max(0.0, min(100.0, float(msg.percentage) * 100.0)) if math.isfinite(msg.percentage) else None
        self.emit("battery", {"percentage": self.data["battery"]})

    def on_odom(self, msg):
        v = msg.twist.twist.linear.x
        self.data["speed"] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.emit("status", self.status_payload())

    def publish_tf_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            pose = {"x": p.x, "y": p.y, "heading_deg": yaw}
            self.data["pose"] = pose
            self.emit("pose", pose)
        except Exception:
            pass  # TF may not be ready during startup.

    def on_map(self, msg):
        png = render_occupancy_grid(msg)
        if png:
            self.map_png = png
            self.map_meta = {"width": msg.info.width, "height": msg.info.height,
                "resolution": msg.info.resolution, "origin": {"x": msg.info.origin.position.x, "y": msg.info.origin.position.y}}
            self.emit("map_updated", {"stamp": time.time()})

    def on_plan(self, msg):
        points = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
        self.emit("plan", {"frame_id": msg.header.frame_id, "points": points})

    def on_image(self, key, msg): self.camera[key] = bytes(msg.data)
    def save_json(self, key, raw):
        try: self.data[key] = json.loads(raw)
        except json.JSONDecodeError: self.data[key] = raw
        self.emit("wms", {key: self.data[key]})
    @staticmethod
    def mission_dict(m):
        return {"id": m.mission_id, "type": m.type, "pickup": m.pickup_location,
                "dropoff": m.dropoff_location, "priority": m.priority, "timestamp": m.timestamp}

    def on_mission_status(self, msg):
        status = {"active": self.mission_dict(msg.active_mission) if msg.active_mission.mission_id else None,
                  "queued": [self.mission_dict(m) for m in msg.queued_missions],
                  "completed": [self.mission_dict(m) for m in msg.completed_missions]}
        self.data["mission_status"] = status
        self.emit("mission_status", status)
    def on_wms(self, msg): self.data["wms_connected"] = bool(msg.data); self.emit("wms", {"connected": bool(msg.data)})

    def on_log(self, msg):
        item = {"stamp": time.time(), "level": int(msg.level), "name": msg.name, "message": msg.msg}
        self.logs.append(item)
        self.emit("log", item)

    def status_payload(self):
        return {k: self.data.get(k) for k in ("mode", "estop", "state", "battery", "speed", "fork_height_mm", "pose")}

    def snapshot(self):
        with self.lock:
            return {**self.data, "status": self.status_payload(), "logs": list(self.logs),
                    "health": self.health_payload()}

    def health_payload(self):
        now = time.monotonic()
        return {"nodes": sorted(self.get_node_names()), "esp32": {
            topic: now - self.last_seen.get(topic, 0) < 1.0 for topic in ("/encoder_count", "/imu/data", "/fork_height")},
            "wms_connected": self.data["wms_connected"]}

    def publish_status(self): self.emit("status", self.status_payload())
    def publish_health(self): self.emit("health", self.health_payload())
    def deadman(self):
        if self.data.get("mode") != "MANUAL": return
        if self.last_joy_time is not None and time.monotonic() - self.last_joy_time > 0.3:
            self.pub_cmd.publish(Twist()); self.last_joy_time = None
    last_joy_time = None

    def joy(self, v, w):
        if self.data["mode"] != "MANUAL": raise ValueError("Manual drive requires MANUAL mode")
        msg = Twist(); msg.linear.x = float(v); msg.angular.z = float(w)
        self.pub_cmd.publish(msg); self.last_joy_time = time.monotonic()

    def stop(self): self.pub_cmd.publish(Twist()); self.last_joy_time = None
