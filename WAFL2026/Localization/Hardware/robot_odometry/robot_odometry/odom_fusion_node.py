#!/usr/bin/env python3
"""
Fuses /wheel_velocity (linear m/s) and /imu_yaw (absolute heading, degrees)
into /odom (nav_msgs/Odometry).

IMPORTANT — NO TF broadcast here.
The EKF node (robot_localization) subscribes to /odom and is the sole
publisher of the  odom -> base_footprint  TF edge. Having two nodes publish
the same TF edge causes a conflict that makes AMCL jump.

Run on: Laptop A (robot hardware laptop)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from tf_transformations import quaternion_from_euler


class OdomFusionNode(Node):

    def __init__(self):
        super().__init__('odom_fusion_node')

        # ── Parameters ───────────────────────────────────────────────────────
        self.declare_parameter('odom_frame',        'odom')
        self.declare_parameter('base_frame',        'base_footprint')
        self.declare_parameter('publish_rate_hz',   50.0)
        # Clamp max dt so a startup delay doesn't cause a huge position jump
        self.declare_parameter('max_dt_s',          0.2)
        # Treat velocity as zero if no encoder message for this long
        self.declare_parameter('velocity_timeout_s', 0.5)

        self.odom_frame      = self.get_parameter('odom_frame').value
        self.base_frame      = self.get_parameter('base_frame').value
        self.max_dt          = float(self.get_parameter('max_dt_s').value)
        self.vel_timeout     = float(self.get_parameter('velocity_timeout_s').value)
        rate                 = float(self.get_parameter('publish_rate_hz').value)

        # ── State ─────────────────────────────────────────────────────────────
        self.x    = 0.0
        self.y    = 0.0
        self.yaw  = 0.0   # radians — set from IMU

        self.linear_velocity = 0.0
        self.last_vel_time   = None
        self.have_yaw        = False

        self.last_time       = None   # set on first timer tick after yaw arrives

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Float32, '/wheel_velocity', self._vel_cb, 10)
        self.create_subscription(
            Float32, '/imu_yaw', self._yaw_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.create_timer(1.0 / rate, self._update)

        self.get_logger().info(
            'OdomFusionNode ready — publishing /odom only (no TF). '
            'EKF will broadcast odom → base_footprint.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _vel_cb(self, msg: Float32):
        self.linear_velocity = float(msg.data)
        self.last_vel_time   = self.get_clock().now()

    def _yaw_cb(self, msg: Float32):
        self.yaw     = math.radians(float(msg.data))
        self.have_yaw = True

    # ── Integration loop ──────────────────────────────────────────────────────

    def _update(self):
        # Don't publish until we have at least one real yaw measurement;
        # prevents AMCL from latching onto a bogus zero-yaw odom at startup.
        if not self.have_yaw:
            return

        now = self.get_clock().now()

        if self.last_time is None:
            self.last_time = now
            self._publish(now, v=0.0)   # publish pose-only frame on first tick
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        if dt > self.max_dt:
            self.get_logger().warn(
                f'dt={dt:.3f}s > max_dt={self.max_dt}s — skipping step '
                f'(startup gap or timer stall)')
            self.last_time = now
            return

        # If the encoder has gone silent, assume the robot is stopped.
        if (self.last_vel_time is None or
                (now - self.last_vel_time).nanoseconds * 1e-9 > self.vel_timeout):
            v = 0.0
        else:
            v = self.linear_velocity

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.last_time = now
        self._publish(now, v)

    # ── Publisher ─────────────────────────────────────────────────────────────

    def _publish(self, now: Time, v: float):
        q = quaternion_from_euler(0.0, 0.0, self.yaw)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Non-zero covariance — EKF cannot invert an all-zero matrix.
        # x,y are reasonably trusted; z/roll/pitch are fixed (2D robot);
        # yaw has moderate uncertainty (phone IMU).
        pose_cov = [0.0] * 36
        pose_cov[0 * 6 + 0] = 0.05   # x
        pose_cov[1 * 6 + 1] = 0.05   # y
        pose_cov[2 * 6 + 2] = 1e6    # z  (not observable in 2D)
        pose_cov[3 * 6 + 3] = 1e6    # roll
        pose_cov[4 * 6 + 4] = 1e6    # pitch
        pose_cov[5 * 6 + 5] = 0.08   # yaw

        twist_cov = [0.0] * 36
        twist_cov[0 * 6 + 0] = 0.05  # vx
        twist_cov[5 * 6 + 5] = 0.08  # vyaw

        odom.pose.covariance  = pose_cov
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = 0.0
        odom.twist.covariance = twist_cov

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
