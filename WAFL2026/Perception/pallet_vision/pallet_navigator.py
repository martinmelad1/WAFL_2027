#!/usr/bin/env python3
"""
ROS2 Node: pallet_navigator
============================
State machine that:
  1. Waits in IDLE until a pallet marker is detected (/pallet_location_id).
  2. Looks up the dropoff destination from the marker ID.
  3. Waits until the YOLO pipeline reports final_z_m ≈ 0 on /pallet/final_z_m,
     which indicates the forklift has reached the pallet and lifted it
     (distance to pallet = zero).
  4. Sends a NavigateToPose goal to Nav2 and monitors it.
  5. Resets to IDLE after delivery (or on failure).
"""

from __future__ import annotations

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, String


# ─────────────────────────────────────────────────────────────────────────────
# Location DB  (move to a YAML parameter for production)
# ─────────────────────────────────────────────────────────────────────────────

ID_TO_LOCATION: dict[int, dict] = {
    0: {"name": "Location_A1", "x":  5.0, "y": 2.0, "yaw": 0.0},
    1: {"name": "Location_A2", "x":  5.0, "y": 5.5, "yaw": 0.0},
    2: {"name": "Location_B1", "x": 12.0, "y": 2.0, "yaw": math.pi / 2},
    3: {"name": "Location_B2", "x": 12.0, "y": 5.5, "yaw": math.pi / 2},
}

# Distance threshold (metres) below which we consider the pallet lifted.
# The YOLO node publishes final_z_m = 0 when distance = 0, but a small
# tolerance handles sensor noise near zero.
LIFTED_THRESHOLD_M = 0.05


# ─────────────────────────────────────────────────────────────────────────────
# State machine
# ─────────────────────────────────────────────────────────────────────────────

class State(Enum):
    IDLE        = auto()   # waiting for a marker detection
    MARKER_READ = auto()   # marker seen, waiting for pallet to be lifted
    NAVIGATING  = auto()   # Nav2 goal in flight


# ─────────────────────────────────────────────────────────────────────────────
# Node
# ─────────────────────────────────────────────────────────────────────────────

class PalletNavigator(Node):
    """
    Listens for pallet detections and autonomously navigates to deliver them.

    State transitions
    -----------------
    IDLE        --[marker detected]--> MARKER_READ
    MARKER_READ --[final_z_m ≈ 0   --> NAVIGATING   (pallet lifted)
    NAVIGATING  --[arrived / error]--> IDLE
    """

    def __init__(self) -> None:
        super().__init__("pallet_navigator")

        self._state: State = State.IDLE
        self._destination: Optional[dict] = None
        self._goal_handle: Optional[ClientGoalHandle] = None

        # Action client – Nav2
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        reliable_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        # ArUco detection → tells us which pallet and where to deliver it
        self.create_subscription(
            String, "/pallet_location_id", self._on_detection, reliable_qos
        )

        # YOLO distance → final_z_m = 0 means forklift reached the pallet (lifted)
        self.create_subscription(
            Float32, "/pallet/final_z_m", self._on_distance, reliable_qos
        )

        self.get_logger().info("PalletNavigator ready – state: IDLE")

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_detection(self, msg: String) -> None:
        """
        Handle a new ArUco detection.
        Decodes the marker ID, looks up the dropoff destination, and
        transitions to MARKER_READ to wait for the pallet to be lifted.
        Only acts when IDLE.
        """
        if self._state != State.IDLE:
            return

        try:
            mid = int(msg.data)
        except ValueError:
            self.get_logger().warn(f"Could not parse marker ID: '{msg.data}'")
            return

        loc = ID_TO_LOCATION.get(mid)
        if loc is None:
            self.get_logger().warn(f"Marker ID {mid} not in location DB – ignoring.")
            return

        self._destination = loc
        self._state       = State.MARKER_READ
        self.get_logger().info(
            f"Marker {mid} → dropoff destination '{loc['name']}' "
            f"({loc['x']}, {loc['y']}). Waiting for pallet to be lifted …"
        )

    def _on_distance(self, msg: Float32) -> None:
        """
        Monitor the YOLO pipeline's final_z_m output.
        When the horizontal distance to the pallet drops to zero (within
        LIFTED_THRESHOLD_M), the forklift has reached and lifted the pallet —
        trigger navigation to the dropoff destination.
        Only acts when in MARKER_READ state.
        """
        if self._state != State.MARKER_READ:
            return

        if msg.data <= LIFTED_THRESHOLD_M:
            self._state = State.NAVIGATING
            self.get_logger().info(
                f"Pallet lifted (final_z_m={msg.data:.3f}m) – "
                f"sending Nav2 goal to '{self._destination['name']}' …"
            )
            self._send_nav_goal(self._destination)

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def _send_nav_goal(self, loc: dict) -> None:
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 action server unavailable – resetting.")
            self._reset()
            return

        yaw = float(loc["yaw"])
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(loc["x"])
        pose.pose.position.y = float(loc["y"])
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose.pose.orientation.z = math.sin(yaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose

        send_future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future) -> None:
        handle: ClientGoalHandle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the goal – resetting.")
            self._reset()
            return

        self._goal_handle = handle
        self.get_logger().info("Nav2 accepted goal – forklift moving to dropoff …")
        handle.get_result_async().add_done_callback(self._on_arrived)

    def _on_arrived(self, future) -> None:
        self.get_logger().info(
            f"Forklift arrived at '{self._destination['name']}' – delivery complete!"
        )
        self._reset()

    def _on_feedback(self, feedback_msg) -> None:
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f"Distance remaining: {dist:.2f} m",
            throttle_duration_sec=2.0,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _reset(self) -> None:
        self._destination  = None
        self._goal_handle  = None
        self._state        = State.IDLE
        self.get_logger().info("State reset → IDLE")


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = PalletNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
