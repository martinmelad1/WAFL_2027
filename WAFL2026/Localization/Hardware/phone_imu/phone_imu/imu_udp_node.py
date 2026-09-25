#!/usr/bin/env python3

import socket
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32


UDP_IP = "0.0.0.0"
UDP_PORT = 5005


class PhoneIMUNode(Node):

    def __init__(self):

        super().__init__('phone_imu_node')

        # ROS publisher
        self.yaw_pub = self.create_publisher(
            Float32,
            '/imu_yaw',
            10
        )

        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.sock.setblocking(False)

        # timer
        self.timer = self.create_timer(
            0.01,
            self.read_udp
        )

        self.get_logger().info("Phone IMU UDP Node Started")

    def read_udp(self):

        try:
            data, addr = self.sock.recvfrom(4096)

        except BlockingIOError:
            return

        text = data.decode(
            "utf-8",
            errors="ignore"
        ).strip()

        parts = text.split(",")

        if len(parts) < 5:
            return

        try:
            # your current axis mapping
            yaw = float(parts[4])

        except ValueError:
            return

        msg = Float32()
        msg.data = yaw

        self.yaw_pub.publish(msg)

        self.get_logger().info(
            f'Yaw: {yaw:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PhoneIMUNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()