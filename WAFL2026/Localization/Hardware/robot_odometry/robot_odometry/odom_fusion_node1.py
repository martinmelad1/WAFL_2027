#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler


class OdomFusionNode(Node):

    def __init__(self):

        super().__init__('odom_fusion_node')

        # ---------------- STATE ----------------
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.linear_velocity = 0.0

        self.last_time = time.time()

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            Float32,
            '/wheel_velocity',
            self.velocity_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/imu_yaw',
            self.yaw_callback,
            10
        )

        # ---------------- PUBLISHER ----------------
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # ---------------- TF ----------------
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- TIMER ----------------
        self.timer = self.create_timer(
            0.02,
            self.update_odom
        )

        self.get_logger().info("Odometry Fusion Node Started")

    # ------------------------------------------------
    # CALLBACKS
    # ------------------------------------------------

    def velocity_callback(self, msg):
        self.linear_velocity = msg.data

    def yaw_callback(self, msg):

        # Convert degrees to radians
        self.yaw = math.radians(msg.data)

    # ------------------------------------------------
    # MAIN ODOM UPDATE
    # ------------------------------------------------

    def update_odom(self):

        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 0:
            return

        # ---------------- POSITION UPDATE ----------------

        self.x += self.linear_velocity * math.cos(self.yaw) * dt
        self.y += self.linear_velocity * math.sin(self.yaw) * dt

        # ---------------- QUATERNION ----------------

        q = quaternion_from_euler(
            0,
            0,
            self.yaw
        )

        # ---------------- ODOM MESSAGE ----------------

        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        # ---------------- TF ----------------

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time


def main(args=None):

    rclpy.init(args=args)

    node = OdomFusionNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()