#!/usr/bin/env python3
"""
ROS2 Node: pallet_pose_node
============================
Detects ArUco markers in a camera image, estimates the 3D pose of the
marker relative to the camera using solvePnP, and publishes:

  /pallet_pose        (geometry_msgs/PoseStamped)  – full 6-DOF pose in camera_frame
  /fork_target        (geometry_msgs/PointStamped) – approach point offset in front of pallet
  /pallet_location    (std_msgs/String)             – human-readable location name
  /pallet_location_id (std_msgs/String)             – raw marker ID string
  /pallet_marker      (visualization_msgs/Marker)  – RViz cube + text label

Design notes
------------
* Only the *first* detected marker is processed per frame.  If multi-marker
  support is needed, iterate over corners/ids and publish a marker array.
* Camera intrinsics are declared as ROS2 parameters so they can be set from
  a launch file or YAML without recompiling.
* tf2 broadcasting is left to a dedicated tf broadcaster node (or via
  robot_localization / camera driver).  The pose message already carries the
  source frame, so downstream nodes can transform it when needed.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker


# ─────────────────────────────────────────────────────────────────────────────
# Constants / defaults  –  Kinect v1 RGB camera (640×480)
# ─────────────────────────────────────────────────────────────────────────────
#
#  Camera matrix (3×3 row-major):
#      fx     0    cx        526.607    0       318.525
#       0    fy    cy    =     0      526.607   241.181
#       0     0     1          0        0         1
#
#  Distortion model: plumb_bob → [k1, k2, p1, p2, k3]
#      k1 =  0.10265184   (barrel distortion, primary radial term)
#      k2 = -0.18646532   (correction at larger radii)
#      p1 =  0.0          (tangential – negligible for Kinect)
#      p2 =  0.0
#      k3 =  0.0
#
# These are used when no launch file overrides the parameters.
# Always prefer setting values from the launch file; these are fallbacks only.

DEFAULT_CAMERA_MATRIX = [
    526.60717328,   0.0,          318.52510740,
      0.0,        526.60717328,   241.18145973,
      0.0,          0.0,            1.0,
]
DEFAULT_DIST_COEFFS = [0.10265184, -0.18646532, 0.0, 0.0, 0.0]

# How far in front of the marker (along camera +Z) to place the fork target.
FORK_APPROACH_OFFSET_M = 0.5   # metres

# Marker ID → named factory location mapping.
# Extend this or load it from a YAML file for larger warehouses.
ID_TO_LOCATION: dict[int, dict] = {
    0: {"name": "Location_A1", "x":  5.0, "y": 2.0, "yaw": 0.0},
    1: {"name": "Location_A2", "x":  5.0, "y": 5.5, "yaw": 0.0},
    2: {"name": "Location_B1", "x": 12.0, "y": 2.0, "yaw": 1.5707},
    3: {"name": "Location_B2", "x": 12.0, "y": 5.5, "yaw": 1.5707},
}

# QoS for sensor data: best-effort, keep last 1 (same as most camera drivers)
SENSOR_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)


# ─────────────────────────────────────────────────────────────────────────────
# Pure math helpers
# ─────────────────────────────────────────────────────────────────────────────

def build_object_points(half: float) -> np.ndarray:
    """
    Return the four 3-D corners of a square marker of side 2*half,
    centred at the origin in the XY plane (Z=0).

    Corner order must match OpenCV's ArUco convention:
        top-left, top-right, bottom-right, bottom-left
    (i.e. the same order detectMarkers() returns image corners).

    Having the origin at the *centre* of the square means tvec will
    point from the camera to the **centre** of the marker – exactly
    what we want for a fork-approach target.
    """
    return np.array(
        [
            [-half,  half, 0.0],   # top-left
            [ half,  half, 0.0],   # top-right
            [ half, -half, 0.0],   # bottom-right
            [-half, -half, 0.0],   # bottom-left
        ],
        dtype=np.float32,
    )


def estimate_marker_pose(
    corners: np.ndarray,
    marker_length: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """
    Estimate the 6-DOF pose of a single ArUco marker.

    Parameters
    ----------
    corners      : (1,4,2) or (4,2) float32 – image-plane corners from detectMarkers.
    marker_length: physical side length of the printed marker in metres.
    camera_matrix: 3×3 intrinsic matrix.
    dist_coeffs  : distortion coefficients (4, 5, 8, or 12 elements).

    Returns
    -------
    rvec : (3,1) rotation vector  – encodes R such that  p_cam = R*p_marker + tvec
    tvec : (3,1) translation vector in metres (camera → marker centre, camera frame)
    Both are None if solvePnP fails.

    Coordinate convention
    ---------------------
    OpenCV camera frame:  +X right, +Y down, +Z away from camera (into the scene).
    So tvec[2] (tz) is the *depth* – how far away the marker is.
    tvec[0] (tx) is the lateral offset (right is positive).
    tvec[1] (ty) is the vertical offset (down is positive).
    """
    obj_pts = build_object_points(marker_length / 2.0)
    img_pts = corners.reshape((4, 2)).astype(np.float32)

    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,   # best closed-form solver for squares
    )
    if not ok:
        return None, None
    return rvec, tvec


def rvec_to_quaternion(rvec: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Convert an OpenCV rotation vector to a unit quaternion (w, x, y, z).

    Steps
    -----
    1. cv2.Rodrigues converts the compact 3-vector rvec into a 3×3 rotation
       matrix R.  The vector's direction is the rotation axis; its magnitude
       is the rotation angle in radians.

    2. We use Shepperd's method to convert R → quaternion.  This avoids the
       singularity that naive trace-based formulas hit when the rotation is
       close to 180°.

    Returns (w, x, y, z) in ROS/geometry_msgs convention.
    """
    R, _ = cv2.Rodrigues(rvec)              # (3,3) float64

    # Shepperd's method – pick the largest diagonal term to maximise
    # numerical precision when squarerooting.
    trace = float(np.trace(R))

    if trace > 0.0:
        s = 2.0 * math.sqrt(trace + 1.0)   # s = 4w
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])  # s = 4x
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])  # s = 4y
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])  # s = 4z
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    return (w, x, y, z)


def quaternion_to_euler(
    w: float, x: float, y: float, z: float
) -> Tuple[float, float, float]:
    """
    Convert unit quaternion (w,x,y,z) to (roll, pitch, yaw) in radians.

    Uses the ZYX / RPY convention (same as tf2 and ROS nav stack).

    Yaw interpretation in the camera frame
    ---------------------------------------
    OpenCV camera frame: X right, Y down, Z forward.
    "Yaw around Z" therefore means: rotation of the marker about the optical
    axis of the camera.  A flat marker facing the camera squarely has yaw≈0.
    Rotating the marker clockwise (when viewed from the camera) gives a
    positive yaw (right-hand rule around +Z forward).

    After tf2 transform to the robot/map frame, yaw has the usual
    "heading" interpretation.
    """
    # roll  (rotation around x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (rotation around y-axis) – clamp to avoid domain error
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))        # numerical guard
    pitch = math.asin(sinp)

    # yaw   (rotation around z-axis)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 Node
# ─────────────────────────────────────────────────────────────────────────────

class PalletPoseNode(Node):
    """
    Detect a single ArUco marker per frame and publish its pose.

    Topic graph
    -----------
    Subscribes: <image_topic>  (sensor_msgs/Image)
    Publishes : /pallet_pose        (geometry_msgs/PoseStamped)
                /fork_target        (geometry_msgs/PointStamped)
                /pallet_location    (std_msgs/String)
                /pallet_location_id (std_msgs/String)
                /pallet_marker      (visualization_msgs/Marker)
    """

    # ------------------------------------------------------------------
    def __init__(self) -> None:
        super().__init__("pallet_pose_node")
        self._declare_parameters()
        self._load_parameters()
        self._build_aruco_detector()
        self.bridge = CvBridge()
        self._create_subscribers()
        self._create_publishers()
        # Pre-allocate object points once (never changes at runtime)
        self._obj_pts = build_object_points(self.marker_length / 2.0)
        self.get_logger().info(
            f"PalletPoseNode ready – listening on [{self.image_topic}]"
        )

    # ------------------------------------------------------------------
    # Parameter management
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("image_topic",    "/image_raw")              # kinect_ros2 default
        self.declare_parameter("camera_frame",   "camera_rgb_optical_frame")  # Kinect v1 optical frame
        self.declare_parameter("marker_length",  0.15)   # metres
        self.declare_parameter("camera_matrix",  DEFAULT_CAMERA_MATRIX)
        self.declare_parameter("dist_coeffs",    DEFAULT_DIST_COEFFS)
        self.declare_parameter("aruco_dict_id",  0)      # DICT_4X4_50 = 0
        self.declare_parameter("fork_offset_m",  FORK_APPROACH_OFFSET_M)

    def _load_parameters(self) -> None:
        self.image_topic   = self.get_parameter("image_topic").value
        self.camera_frame  = self.get_parameter("camera_frame").value
        self.marker_length = float(self.get_parameter("marker_length").value)
        self.fork_offset   = float(self.get_parameter("fork_offset_m").value)
        self.camera_matrix = np.array(
            self.get_parameter("camera_matrix").value, dtype=np.float64
        ).reshape((3, 3))
        self.dist_coeffs = np.array(
            self.get_parameter("dist_coeffs").value, dtype=np.float64
        )
        dict_id = int(self.get_parameter("aruco_dict_id").value)
        self._aruco_dict_id = dict_id

    # ------------------------------------------------------------------
    # ArUco
    # ------------------------------------------------------------------

    def _build_aruco_detector(self) -> None:
        aruco_dict = cv2.aruco.getPredefinedDictionary(self._aruco_dict_id)
        params = cv2.aruco.DetectorParameters()
        # Tighten defaults slightly for warehouse lighting
        params.minMarkerPerimeterRate = 0.03
        params.adaptiveThreshWinSizeStep = 10
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)

    # ------------------------------------------------------------------
    # ROS infrastructure
    # ------------------------------------------------------------------

    def _create_subscribers(self) -> None:
        self.create_subscription(
            Image, self.image_topic, self._image_cb, SENSOR_QOS
        )

    def _create_publishers(self) -> None:
        # Reliable QoS for downstream consumers (nav stack, UI)
        self.pub_pose    = self.create_publisher(PoseStamped,  "/pallet_pose",       10)
        self.pub_fork    = self.create_publisher(PointStamped, "/fork_target",        10)
        self.pub_loc     = self.create_publisher(String,       "/pallet_location",    10)
        self.pub_loc_id  = self.create_publisher(String,       "/pallet_location_id", 10)
        self.pub_marker  = self.create_publisher(Marker,       "/pallet_marker",      10)

    # ------------------------------------------------------------------
    # Image callback – hot path, keep allocations minimal
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image) -> None:
        # Decode image (no copy needed; cv_bridge manages lifetime)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is None or len(ids) == 0:
            return

        # Only process the first detected marker
        marker_corners = corners[0]
        marker_id      = int(ids[0][0])

        rvec, tvec = estimate_marker_pose(
            marker_corners, self.marker_length,
            self.camera_matrix, self.dist_coeffs,
        )
        if rvec is None:
            self.get_logger().warn("solvePnP failed – skipping frame.")
            return

        qw, qx, qy, qz = rvec_to_quaternion(rvec)
        tx, ty, tz = tvec.flatten()
        roll, pitch, yaw = quaternion_to_euler(qw, qx, qy, qz)

        stamp  = msg.header.stamp
        header = self._make_header(stamp)

        self._publish_pose(header, tx, ty, tz, qw, qx, qy, qz)
        self._publish_fork_target(header, tx, ty, tz)
        self._publish_location(marker_id)
        self._publish_rviz_marker(stamp, tx, ty, tz, marker_id)

        self.get_logger().info(
            f"ID {marker_id:2d} | "
            f"z={tz:+.3f}m  x={tx:+.3f}m  y={ty:+.3f}m | "
            f"roll={math.degrees(roll):+.1f}°  "
            f"pitch={math.degrees(pitch):+.1f}°  "
            f"yaw={math.degrees(yaw):+.1f}°",
            throttle_duration_sec=0.5,   # cap at 2 Hz to avoid log flooding
        )

    # ------------------------------------------------------------------
    # Publish helpers
    # ------------------------------------------------------------------

    def _make_header(self, stamp) -> Header:
        h = Header()
        h.stamp    = stamp
        h.frame_id = self.camera_frame
        return h

    def _publish_pose(
        self, header: Header,
        tx: float, ty: float, tz: float,
        qw: float, qx: float, qy: float, qz: float,
    ) -> None:
        msg = PoseStamped()
        msg.header = header
        msg.pose.position.x    = tx
        msg.pose.position.y    = ty
        msg.pose.position.z    = tz
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        self.pub_pose.publish(msg)

    def _publish_fork_target(
        self, header: Header, tx: float, ty: float, tz: float
    ) -> None:
        """
        Publish an approach point offset in front of the pallet.

        In the OpenCV camera frame +Z is forward (depth), so adding
        fork_offset along Z gives a point 'fork_offset' metres in front
        of the marker – a sensible fork-insertion approach target.
        """
        msg = PointStamped()
        msg.header  = header
        msg.point.x = tx
        msg.point.y = ty
        msg.point.z = tz - self.fork_offset   # approach from in front
        self.pub_fork.publish(msg)

    def _publish_location(self, marker_id: int) -> None:
        loc = ID_TO_LOCATION.get(marker_id)
        name = loc["name"] if loc else f"UNKNOWN_ID_{marker_id}"
        self.pub_loc.publish(String(data=name))
        self.pub_loc_id.publish(String(data=str(marker_id)))

    def _publish_rviz_marker(
        self, stamp, tx: float, ty: float, tz: float, marker_id: int
    ) -> None:
        loc  = ID_TO_LOCATION.get(marker_id, {})
        label = loc.get("name", f"ID {marker_id}")

        base = Marker()
        base.header.stamp    = stamp
        base.header.frame_id = self.camera_frame
        base.action          = Marker.ADD
        base.lifetime.sec    = 1

        # ── Cube ──
        cube = Marker()
        cube.header          = base.header
        cube.ns              = "pallet"
        cube.id              = 0
        cube.type            = Marker.CUBE
        cube.action          = Marker.ADD
        cube.pose.position.x = tx
        cube.pose.position.y = ty
        cube.pose.position.z = tz
        cube.pose.orientation.w = 1.0
        cube.scale.x = cube.scale.y = cube.scale.z = 0.2
        cube.color.r = 0.2; cube.color.g = 0.8
        cube.color.b = 0.2; cube.color.a = 0.8
        cube.lifetime.sec = 1
        self.pub_marker.publish(cube)

        # ── Text label ──
        text = Marker()
        text.header             = base.header
        text.ns                 = "pallet_label"
        text.id                 = 1
        text.type               = Marker.TEXT_VIEW_FACING
        text.action             = Marker.ADD
        text.pose.position.x    = tx
        text.pose.position.y    = ty
        text.pose.position.z    = tz + 0.25
        text.pose.orientation.w = 1.0
        text.scale.z            = 0.1
        text.color.r = text.color.g = text.color.b = 1.0
        text.color.a            = 1.0
        text.text               = label
        text.lifetime.sec       = 1
        self.pub_marker.publish(text)


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PalletPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
