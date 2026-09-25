#!/usr/bin/env python3
"""
ROS2 Node: pallet_front_angle_node
====================================
YOLO + Kinect depth based fine-alignment helper.

Detects the pallet, its pockets, and four keypoints using TensorRT
engines on the Jetson GPU. Combines the keypoint pixel coordinates
with the Kinect depth stream to compute three angles useful for
forklift fine-alignment:

  /pallet/rotate_to_front_rad    – command to rotate forklift to face pallet square-on
  /pallet/pallet_yaw_offset_rad  – pallet's yaw angle in the camera frame
  /pallet/center_yaw_rad         – bearing of pallet centre from camera optical axis
  /pallet/final_z_m              – horizontal distance to pallet (after camera_height correction)

Each angle channel also has two averaging variants (simple and
time-weighted circular means over a 3-second window) on matching
'_avg_rad' / '_weighted_avg_rad' topics.

Author : teammate's original code, integrated into pallet_vision package.
"""

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from ultralytics import YOLO


# Height of the camera above ground (metres) – used to project Z into horizontal distance
CAMERA_HEIGHT_M = 0.57


class PalletFrontAngleNode(Node):
    def __init__(self):
        super().__init__("pallet_front_angle_node")

        self.rgb_count = 0
        self.depth_count = 0
        self.avg_window_sec = 3.0
        self.angle_history: deque = deque()

        # ---------------- PARAMETERS ----------------
        # Topics under /kinect/ – the launch file remaps the driver into this namespace
        self.declare_parameter("rgb_topic",         "/image_raw")
        self.declare_parameter("depth_topic",       "/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera_info")

        # TensorRT engine paths (relative to working directory)
        self.declare_parameter("model_pallet_path",   "models/bestt.engine")
        self.declare_parameter("model_pocket_path",   "models/besttt.engine")
        self.declare_parameter("model_keypoint_path", "models/best_pose12.engine")

        # Intrinsics – default to Kinect v1 RGB; overridden if camera_info is published
        self.declare_parameter("fx", 526.60717328)
        self.declare_parameter("fy", 526.60717328)
        self.declare_parameter("cx", 318.5251074)
        self.declare_parameter("cy", 241.18145973)

        self.declare_parameter("depth_window",  5)
        self.declare_parameter("min_depth_m",   0.35)
        self.declare_parameter("max_depth_m",   6.0)
        self.declare_parameter("keypoint_conf", 0.10)
        self.declare_parameter("debug_view",    True)

        pallet_model_path   = self.get_parameter("model_pallet_path").value
        pocket_model_path   = self.get_parameter("model_pocket_path").value
        keypoint_model_path = self.get_parameter("model_keypoint_path").value

        self.fx = float(self.get_parameter("fx").value)
        self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value)
        self.cy = float(self.get_parameter("cy").value)
        self.depth_window  = int(self.get_parameter("depth_window").value)
        self.min_depth_m   = float(self.get_parameter("min_depth_m").value)
        self.max_depth_m   = float(self.get_parameter("max_depth_m").value)
        self.keypoint_conf = float(self.get_parameter("keypoint_conf").value)
        self.debug_view    = bool(self.get_parameter("debug_view").value)

        # ---------------- LOAD TensorRT ENGINES ----------------
        self.get_logger().info("Loading TensorRT engines …")
        self.model_pallet   = YOLO(pallet_model_path,   task="detect")
        self.model_pocket   = YOLO(pocket_model_path,   task="detect")
        self.model_keypoint = YOLO(keypoint_model_path, task="pose")

        # GPU warmup so the first real frame doesn't pay JIT cost
        dummy = np.zeros((640, 640, 3), dtype=np.uint8)
        self.model_pallet  (dummy, verbose=False, device="cuda")
        self.model_pocket  (dummy, verbose=False, device="cuda")
        self.model_keypoint(dummy, verbose=False, device="cuda")
        self.get_logger().info("GPU warmup complete – YOLO engines loaded.")

        # ---------------- ROS COMMUNICATIONS ----------------
        self.bridge = CvBridge()
        self.latest_depth = None
        self.latest_depth_stamp = None

        self.sub_depth = self.create_subscription(
            Image, self.get_parameter("depth_topic").value,
            self.depth_callback, qos_profile_sensor_data,
        )
        self.sub_rgb = self.create_subscription(
            Image, self.get_parameter("rgb_topic").value,
            self.image_callback, qos_profile_sensor_data,
        )
        if self.get_parameter("camera_info_topic").value:
            self.sub_camera_info = self.create_subscription(
                CameraInfo, self.get_parameter("camera_info_topic").value,
                self.camera_info_callback, qos_profile_sensor_data,
            )

        # Instantaneous publishers
        self.pub_rotate_to_front = self.create_publisher(Float32, "/pallet/rotate_to_front_rad",    10)
        self.pub_pallet_yaw      = self.create_publisher(Float32, "/pallet/pallet_yaw_offset_rad", 10)
        self.pub_center_yaw      = self.create_publisher(Float32, "/pallet/center_yaw_rad",        10)
        self.pub_final_z         = self.create_publisher(Float32, "/pallet/final_z_m",             10)

        # Averaging publishers
        self.pub_rotate_to_front_avg          = self.create_publisher(Float32, "/pallet/rotate_to_front_avg_rad",           10)
        self.pub_rotate_to_front_weighted_avg = self.create_publisher(Float32, "/pallet/rotate_to_front_weighted_avg_rad",  10)
        self.pub_pallet_yaw_avg               = self.create_publisher(Float32, "/pallet/pallet_yaw_offset_avg_rad",          10)
        self.pub_pallet_yaw_weighted_avg      = self.create_publisher(Float32, "/pallet/pallet_yaw_offset_weighted_avg_rad", 10)
        self.pub_center_yaw_avg               = self.create_publisher(Float32, "/pallet/center_yaw_avg_rad",                 10)
        self.pub_center_yaw_weighted_avg      = self.create_publisher(Float32, "/pallet/center_yaw_weighted_avg_rad",        10)

        # Debug image publisher – subscribe with rqt_image_view on /pallet/debug_image
        self.pub_debug_image = self.create_publisher(Image, "/pallet/debug_image", 1)

        self.last_log_time = self.get_clock().now()
        self.get_logger().info("PalletFrontAngleNode (TensorRT) ready.")

    # ──────────────────────────────────────────────────────────────────
    # Topic callbacks
    # ──────────────────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        """Live-update intrinsics from the camera driver."""
        self.fx, self.fy = float(msg.k[0]), float(msg.k[4])
        self.cx, self.cy = float(msg.k[2]), float(msg.k[5])

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth_stamp = msg.header.stamp
            self.depth_count += 1
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    # ──────────────────────────────────────────────────────────────────
    # Depth & 3-D helpers
    # ──────────────────────────────────────────────────────────────────

    def get_depth(self, depth_image, u, v):
        """Median depth (metres) over a small patch around pixel (u, v)."""
        if depth_image is None:
            return None
        h, w = depth_image.shape[:2]
        u, v = int(u), int(v)
        if not (0 <= u < w and 0 <= v < h):
            return None

        half = max(1, self.depth_window // 2)
        patch = depth_image[
            max(0, v - half):min(h, v + half + 1),
            max(0, u - half):min(w, u + half + 1),
        ]

        if patch.dtype == np.uint16:
            valid = patch[patch > 0].astype(np.float32) / 1000.0
        else:
            valid = patch[np.isfinite(patch)]

        valid = valid[(valid >= self.min_depth_m) & (valid <= self.max_depth_m)]
        return float(np.median(valid)) if valid.size > 0 else None

    def pixel_to_3d(self, u, v, z_m):
        """Back-project a pixel to a 3-D point in the camera frame."""
        x = (float(u) - self.cx) * z_m / self.fx
        y = (float(v) - self.cy) * z_m / self.fy
        return np.array([x, y, z_m], dtype=np.float32)

    # ──────────────────────────────────────────────────────────────────
    # YOLO post-processing
    # ──────────────────────────────────────────────────────────────────

    def calculate_iou(self, box1, box2):
        x1, y1, x2, y2 = box1[:4]
        x3, y3, x4, y4 = box2[:4]
        xi1, yi1 = max(x1, x3), max(y1, y3)
        xi2, yi2 = min(x2, x4), min(y2, y4)
        inter = max(0, xi2 - xi1) * max(0, yi2 - yi1)
        if inter == 0:
            return 0.0
        area1 = max(1, (x2 - x1) * (y2 - y1))
        area2 = max(1, (x4 - x3) * (y4 - y3))
        return inter / float(min(area1, area2))

    def filter_pockets(self, pockets, threshold=0.5):
        if not pockets:
            return []
        pockets = sorted(pockets, key=lambda x: x[4], reverse=True)
        keep = []
        while pockets:
            curr = pockets.pop(0)
            keep.append(curr)
            pockets = [p for p in pockets if self.calculate_iou(curr, p) < threshold]
        return keep

    def extract_top4_keypoints(self, results_kp, offset_x=0, offset_y=0):
        if not results_kp or results_kp[0].keypoints is None:
            return None
        kpts = results_kp[0].keypoints
        if len(kpts.xy) == 0:
            return None

        xy, confs = kpts.xy[0].cpu().numpy(), kpts.conf[0].cpu().numpy()
        indices = confs.argsort()[::-1]
        final = []
        for i in indices:
            if confs[i] < self.keypoint_conf:
                continue
            final.append((int(xy[i][0]) + offset_x, int(xy[i][1]) + offset_y, int(i), confs[i]))
            if len(final) == 4:
                break
        return final if len(final) == 4 else None

    def draw_keypoints(self, output, points):
        for (rx, ry, ri, rc) in points:
            cv2.circle(output, (rx, ry), 6, (0, 0, 255), -1)
            cv2.putText(output, f"{ri}:{rc:.2f}", (rx + 5, ry - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # ──────────────────────────────────────────────────────────────────
    # Geometry: 4 keypoints + depth → pose & angles
    # ──────────────────────────────────────────────────────────────────

    def compute_pose_and_angle(self, points, depth_image):
        pts = sorted(points, key=lambda x: x[2])  # sort by keypoint index
        u1, v1, _, _ = pts[0]
        u2, v2, _, _ = pts[1]
        u3, v3, _, _ = pts[2]
        u4, v4, _, _ = pts[3]

        z1 = self.get_depth(depth_image, u1, v1)
        z2 = self.get_depth(depth_image, u2, v2)
        z3 = self.get_depth(depth_image, u3, v3)
        z4 = self.get_depth(depth_image, u4, v4)
        if None in (z1, z2, z3, z4):
            return None

        p1 = self.pixel_to_3d(u1, v1, z1)
        p2 = self.pixel_to_3d(u2, v2, z2)
        p3 = self.pixel_to_3d(u3, v3, z3)
        p4 = self.pixel_to_3d(u4, v4, z4)

        edge_x = p2 - p1
        edge_y_seed = p4 - p1
        if np.linalg.norm(edge_x) < 1e-6 or np.linalg.norm(edge_y_seed) < 1e-6:
            return None

        x_axis = edge_x / np.linalg.norm(edge_x)
        y_axis_seed = edge_y_seed / np.linalg.norm(edge_y_seed)
        z_axis = np.cross(x_axis, y_axis_seed)
        norm_z = np.linalg.norm(z_axis)
        if norm_z < 1e-6:
            return None
        z_axis /= norm_z
        if z_axis[2] < 0:
            z_axis *= -1.0  # normal must point forward

        y_axis = np.cross(z_axis, x_axis)
        norm_y = np.linalg.norm(y_axis)
        if norm_y < 1e-6:
            return None
        y_axis /= norm_y

        R = np.column_stack((x_axis, y_axis, z_axis))
        center_3d = (p1 + p2 + p3 + p4) / 4.0
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[:3, 3] = center_3d

        pallet_yaw = math.atan2(float(z_axis[0]), float(z_axis[2]))
        center_yaw = math.atan2(float(center_3d[0]), float(center_3d[2]))
        rotate_cmd = -pallet_yaw

        return {
            "center_3d": center_3d,
            "R": R, "T": T,
            "pallet_yaw_offset_rad": pallet_yaw,
            "center_yaw_rad":        center_yaw,
            "rotate_to_front_rad":   rotate_cmd,
        }

    # ──────────────────────────────────────────────────────────────────
    # Angle averaging (3 s circular window)
    # ──────────────────────────────────────────────────────────────────

    def prune_angle_history(self, now_sec):
        while self.angle_history and (now_sec - self.angle_history[0]["t"]) > self.avg_window_sec:
            self.angle_history.popleft()

    def add_angle_sample(self, now_sec, rotate, pallet_yaw, center_yaw):
        self.angle_history.append({
            "t": now_sec,
            "rotate_to_front_rad":   float(rotate),
            "pallet_yaw_offset_rad": float(pallet_yaw),
            "center_yaw_rad":        float(center_yaw),
        })
        self.prune_angle_history(now_sec)

    def circular_mean(self, angles):
        if not angles:
            return None
        s = sum(math.sin(a) for a in angles)
        c = sum(math.cos(a) for a in angles)
        if abs(s) < 1e-12 and abs(c) < 1e-12:
            return None
        return math.atan2(s, c)

    def weighted_circular_mean(self, timed_angles, now_sec):
        if not timed_angles:
            return None
        w_sin, w_cos = 0.0, 0.0
        for st, ang in timed_angles:
            w = max(1e-6, self.avg_window_sec - (now_sec - st))
            w_sin += w * math.sin(ang)
            w_cos += w * math.cos(ang)
        if abs(w_sin) < 1e-12 and abs(w_cos) < 1e-12:
            return None
        return math.atan2(w_sin, w_cos)

    def compute_angle_averages(self, now_sec):
        self.prune_angle_history(now_sec)
        if not self.angle_history:
            return None

        rotate_angles = [s["rotate_to_front_rad"]   for s in self.angle_history]
        pallet_angles = [s["pallet_yaw_offset_rad"] for s in self.angle_history]
        center_angles = [s["center_yaw_rad"]        for s in self.angle_history]

        rotate_timed = [(s["t"], s["rotate_to_front_rad"])   for s in self.angle_history]
        pallet_timed = [(s["t"], s["pallet_yaw_offset_rad"]) for s in self.angle_history]
        center_timed = [(s["t"], s["center_yaw_rad"])        for s in self.angle_history]

        return {
            "rotate_to_front_avg_rad":            self.circular_mean(rotate_angles),
            "rotate_to_front_weighted_avg_rad":   self.weighted_circular_mean(rotate_timed, now_sec),
            "pallet_yaw_offset_avg_rad":          self.circular_mean(pallet_angles),
            "pallet_yaw_offset_weighted_avg_rad": self.weighted_circular_mean(pallet_timed, now_sec),
            "center_yaw_avg_rad":                 self.circular_mean(center_angles),
            "center_yaw_weighted_avg_rad":        self.weighted_circular_mean(center_timed, now_sec),
        }

    # ──────────────────────────────────────────────────────────────────
    # Main RGB image callback
    # ──────────────────────────────────────────────────────────────────

    def image_callback(self, msg: Image):
        try:
            self.rgb_count += 1
            if self.latest_depth is None:
                return

            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            depth_image = cv2.resize(
                self.latest_depth,
                (image.shape[1], image.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )

            output = image.copy()
            h, w = image.shape[:2]
            cv2.line(output, (w // 2, 0), (w // 2, h - 1), (255, 255, 0), 1)

            # --- Pallet detection ---
            res_p = self.model_pallet(image, verbose=False, device="cuda")
            pallet_boxes = []
            if res_p and res_p[0].boxes:
                for b in res_p[0].boxes:
                    px1, py1, px2, py2 = map(int, b.xyxy[0])
                    cv2.rectangle(output, (px1, py1), (px2, py2), (255, 0, 0), 2)
                    pallet_boxes.append((px1, py1, px2, py2))

            # ── STEP 1: keypoints inside pockets ───────────────────────
            pose_result = None
            source_name = ""
            all_pockets = []

            for (px1, py1, px2, py2) in pallet_boxes:
                roi = image[py1:py2, px1:px2]
                if roi.size == 0:
                    continue
                res_pkt = self.model_pocket(roi, verbose=False, device="cuda")
                if res_pkt and res_pkt[0].boxes:
                    pkts = [[*map(int, b.xyxy[0]), float(b.conf[0])] for b in res_pkt[0].boxes]
                    for pk in self.filter_pockets(pkts):
                        pk_x1, pk_y1 = pk[0] + px1, pk[1] + py1
                        pk_x2, pk_y2 = pk[2] + px1, pk[3] + py1
                        all_pockets.append((pk_x1, pk_y1, pk_x2, pk_y2))
                        cv2.rectangle(output, (pk_x1, pk_y1), (pk_x2, pk_y2), (0, 255, 0), 2)

            for (pk_x1, pk_y1, pk_x2, pk_y2) in all_pockets:
                pad_w = int((pk_x2 - pk_x1) * 0.22)
                pad_h = int((pk_y2 - pk_y1) * 0.22)
                x1 = max(0, pk_x1 - pad_w)
                y1 = max(0, pk_y1 - pad_h)
                x2 = min(w, pk_x2 + pad_w)
                y2 = min(h, pk_y2 + pad_h)
                crop = image[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                res_kp = self.model_keypoint(crop, conf=self.keypoint_conf,
                                             verbose=False, device="cuda")
                pts = self.extract_top4_keypoints(res_kp, x1, y1)
                if pts:
                    temp = self.compute_pose_and_angle(pts, depth_image)
                    if temp:
                        pose_result = temp
                        self.draw_keypoints(output, pts)
                        source_name = "pocket"
                        break

            # ── STEP 2: fallback to pallet ROI ─────────────────────────
            if pose_result is None:
                for (px1, py1, px2, py2) in pallet_boxes:
                    crop = image[py1:py2, px1:px2]
                    if crop.size == 0:
                        continue
                    res_kp = self.model_keypoint(crop, conf=self.keypoint_conf,
                                                 verbose=False, device="cuda")
                    pts = self.extract_top4_keypoints(res_kp, px1, py1)
                    if pts:
                        temp = self.compute_pose_and_angle(pts, depth_image)
                        if temp:
                            pose_result = temp
                            self.draw_keypoints(output, pts)
                            source_name = "pallet"
                            break

            # ── STEP 3: last fallback – full image ─────────────────────
            if pose_result is None:
                res_kp = self.model_keypoint(image, conf=self.keypoint_conf,
                                             verbose=False, device="cuda")
                pts = self.extract_top4_keypoints(res_kp, 0, 0)
                if pts:
                    temp = self.compute_pose_and_angle(pts, depth_image)
                    if temp:
                        pose_result = temp
                        self.draw_keypoints(output, pts)
                        source_name = "full_image"

            # ── Publish + draw ─────────────────────────────────────────
            if pose_result:
                now_sec = time.monotonic()
                self.add_angle_sample(
                    now_sec,
                    pose_result["rotate_to_front_rad"],
                    pose_result["pallet_yaw_offset_rad"],
                    pose_result["center_yaw_rad"],
                )
                avg = self.compute_angle_averages(now_sec)

                self.pub_rotate_to_front.publish(Float32(data=pose_result["rotate_to_front_rad"]))
                self.pub_pallet_yaw     .publish(Float32(data=pose_result["pallet_yaw_offset_rad"]))
                self.pub_center_yaw     .publish(Float32(data=pose_result["center_yaw_rad"]))

                Z = float(pose_result["center_3d"][2])
                final_z = math.sqrt(max(0.0, Z ** 2 - CAMERA_HEIGHT_M ** 2))
                self.pub_final_z.publish(Float32(data=final_z))

                if avg:
                    self.pub_rotate_to_front_avg         .publish(Float32(data=avg["rotate_to_front_avg_rad"]))
                    self.pub_rotate_to_front_weighted_avg.publish(Float32(data=avg["rotate_to_front_weighted_avg_rad"]))
                    self.pub_pallet_yaw_avg              .publish(Float32(data=avg["pallet_yaw_offset_avg_rad"]))
                    self.pub_pallet_yaw_weighted_avg     .publish(Float32(data=avg["pallet_yaw_offset_weighted_avg_rad"]))
                    self.pub_center_yaw_avg              .publish(Float32(data=avg["center_yaw_avg_rad"]))
                    self.pub_center_yaw_weighted_avg     .publish(Float32(data=avg["center_yaw_weighted_avg_rad"]))

                center_3d = pose_result["center_3d"]
                u_c = int((self.fx * center_3d[0] / center_3d[2]) + self.cx)
                v_c = int((self.fy * center_3d[1] / center_3d[2]) + self.cy)
                if 0 <= u_c < w and 0 <= v_c < h:
                    cv2.circle(output, (u_c, v_c), 7, (255, 0, 255), -1)

                deg = lambda r: math.degrees(r) if r is not None else None
                yaw_deg  = deg(pose_result["pallet_yaw_offset_rad"])
                rot_deg  = deg(pose_result["rotate_to_front_rad"])
                cent_deg = deg(pose_result["center_yaw_rad"])

                cv2.putText(output, f"source: {source_name}",           (20,  30), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2)
                cv2.putText(output, f"pallet_yaw: {yaw_deg:+.2f} deg",  (20,  60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,   255, 255), 2)
                cv2.putText(output, f"rotate_cmd: {rot_deg:+.2f} deg",  (20,  90), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0,   255, 0  ), 2)
                cv2.putText(output, f"center_yaw: {cent_deg:+.2f} deg", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 200, 0  ), 2)
                cv2.putText(output, f"Z: {Z:.3f}m  Final Z: {final_z:.3f}m", (20, 150),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

                if avg:
                    cv2.putText(output,
                                f'rot avg/wavg: {deg(avg["rotate_to_front_avg_rad"]):+.2f} / {deg(avg["rotate_to_front_weighted_avg_rad"]):+.2f} deg',
                                (20, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
                    cv2.putText(output,
                                f'pallet avg/wavg: {deg(avg["pallet_yaw_offset_avg_rad"]):+.2f} / {deg(avg["pallet_yaw_offset_weighted_avg_rad"]):+.2f} deg',
                                (20, 210), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                    cv2.putText(output,
                                f'center avg/wavg: {deg(avg["center_yaw_avg_rad"]):+.2f} / {deg(avg["center_yaw_weighted_avg_rad"]):+.2f} deg',
                                (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 200, 0), 2)

                now = self.get_clock().now()
                if (now - self.last_log_time).nanoseconds > 1e9:
                    self.last_log_time = now
                    self.get_logger().info(
                        f"rot={rot_deg:+.2f} | yaw={yaw_deg:+.2f} | center={cent_deg:+.2f} | "
                        f"Z={Z:.3f} final_Z={final_z:.3f} | "
                        f"side_walk={final_z * math.cos(math.radians(abs(rot_deg))):.3f}"
                    )
            else:
                cv2.putText(output, "No valid 3D pallet angle found",
                            (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Always publish annotated frame as ROS topic (view with rqt_image_view on /pallet/debug_image)
            debug_msg = self.bridge.cv2_to_imgmsg(output, encoding="bgr8")
            debug_msg.header = msg.header
            self.pub_debug_image.publish(debug_msg)

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PalletFrontAngleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
