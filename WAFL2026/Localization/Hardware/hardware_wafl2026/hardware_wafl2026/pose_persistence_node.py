#!/usr/bin/env python3
"""
Pose Persistence Node

Solves: AMCL forgets its position between runs, forcing manual re-localization.

What it does:
  1. Subscribes to /amcl_pose and writes the latest pose to disk every 2s.
  2. On startup, reads the saved pose and republishes it to /initialpose
     three times (1s apart) to overcome AMCL's volatile-QoS subscription
     and the race condition with AMCL's lifecycle activation.

Save file:  ~/.ros/amcl_last_pose.yaml

To wipe and start fresh:
  rm ~/.ros/amcl_last_pose.yaml
"""

import os
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


SAVE_PATH = os.path.expanduser('~/.ros/amcl_last_pose.yaml')


class PosePersistenceNode(Node):

    def __init__(self):
        super().__init__('pose_persistence_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('save_period_s',     2.0)
        self.declare_parameter('restore_delay_s',   0.5)
        self.declare_parameter('restore_attempts',  3)
        self.declare_parameter('restore_spacing_s', 1.0)

        save_period       = float(self.get_parameter('save_period_s').value)
        restore_delay     = float(self.get_parameter('restore_delay_s').value)
        self.attempts_left = int(self.get_parameter('restore_attempts').value)
        self.attempt_spacing = float(
            self.get_parameter('restore_spacing_s').value)

        # ── State ────────────────────────────────────────────────────────────
        self.last_pose = None
        self.restore_msg = None  # built once on startup if file exists

        # ── Pub/Sub ──────────────────────────────────────────────────────────
        self.init_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # ── Timers ───────────────────────────────────────────────────────────
        # Periodic save of latest pose
        self.create_timer(save_period, self._save_to_disk)

        # One-shot restore kickoff after a short delay (master launch already
        # delays starting this node, so 0.5s here is enough)
        self.restore_kick = self.create_timer(
            restore_delay, self._begin_restore)

        # Repeated restore publish (created on first kick, see _begin_restore)
        self.restore_repeat = None

        self.get_logger().info(
            f'Pose persistence ready. Save file: {SAVE_PATH}')

    # ── Subscriber callback ─────────────────────────────────────────────────

    def _amcl_cb(self, msg):
        self.last_pose = msg

    # ── Save to disk ────────────────────────────────────────────────────────

    def _save_to_disk(self):
        if self.last_pose is None:
            return
        p = self.last_pose.pose.pose
        data = {
            'position': {
                'x': float(p.position.x),
                'y': float(p.position.y),
                'z': float(p.position.z),
            },
            'orientation': {
                'x': float(p.orientation.x),
                'y': float(p.orientation.y),
                'z': float(p.orientation.z),
                'w': float(p.orientation.w),
            },
            'covariance': [float(c) for c in self.last_pose.pose.covariance],
        }
        try:
            os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
            tmp_path = SAVE_PATH + '.tmp'
            with open(tmp_path, 'w') as f:
                yaml.safe_dump(data, f)
            os.replace(tmp_path, SAVE_PATH)   # atomic
        except Exception as e:
            self.get_logger().warn(f'Save failed: {e}')

    # ── Restore from disk ───────────────────────────────────────────────────

    def _begin_restore(self):
        # one-shot — disable this kickoff timer
        self.restore_kick.cancel()

        if not os.path.exists(SAVE_PATH):
            self.get_logger().info(
                'No saved pose found. Using default initial pose.')
            return

        try:
            with open(SAVE_PATH) as f:
                data = yaml.safe_load(f)

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = data['position']['x']
            msg.pose.pose.position.y = data['position']['y']
            msg.pose.pose.position.z = data['position']['z']
            msg.pose.pose.orientation.x = data['orientation']['x']
            msg.pose.pose.orientation.y = data['orientation']['y']
            msg.pose.pose.orientation.z = data['orientation']['z']
            msg.pose.pose.orientation.w = data['orientation']['w']
            msg.pose.covariance = data['covariance']

            self.restore_msg = msg
            self.get_logger().info(
                f'Restoring pose x={msg.pose.pose.position.x:.2f} '
                f'y={msg.pose.pose.position.y:.2f} — publishing '
                f'{self.attempts_left} times to ensure AMCL receives it')

            # Publish repeatedly; AMCL's /initialpose sub is volatile QoS,
            # so a single message can be missed if timing is unlucky.
            self.restore_repeat = self.create_timer(
                self.attempt_spacing, self._publish_restore)

        except Exception as e:
            self.get_logger().warn(f'Restore failed: {e}')

    def _publish_restore(self):
        if self.restore_msg is None or self.attempts_left <= 0:
            if self.restore_repeat is not None:
                self.restore_repeat.cancel()
            return
        # refresh timestamp each publish
        self.restore_msg.header.stamp = self.get_clock().now().to_msg()
        self.init_pub.publish(self.restore_msg)
        self.attempts_left -= 1
        if self.attempts_left == 0:
            self.get_logger().info('Restore publishes done.')
            self.restore_repeat.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = PosePersistenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final save on shutdown — catches the last pose without waiting
        # for the next timer tick.
        node._save_to_disk()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
