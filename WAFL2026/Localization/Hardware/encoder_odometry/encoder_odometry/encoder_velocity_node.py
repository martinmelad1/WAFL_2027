#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import time
import math


class EncoderVelocityNode(Node):

    def __init__(self):
        super().__init__('encoder_velocity_node')

        # -----------------------------
        # PARAMETERS
        # -----------------------------
        self.wheel_radius = 0.1
        self.pulses_per_rev = 600
        self.gear_ratio = 1885.0 / 600.0

        # -----------------------------
        # VARIABLES
        # -----------------------------
        self.previous_ticks = None
        self.previous_time = time.time()

        # -----------------------------
        # SUBSCRIBER
        # -----------------------------
        self.subscription = self.create_subscription(
            Float32,
            '/encoder_count',
            self.encoder_callback,
            10
        )

        # -----------------------------
        # PUBLISHER
        # -----------------------------
        self.velocity_publisher = self.create_publisher(
            Float32,
            '/wheel_velocity',
            10
        )

        self.get_logger().info('Encoder Velocity Node Started')

    def encoder_callback(self, msg):

        current_ticks = msg.data
        current_time = time.time()

        # First message
        if self.previous_ticks is None:
            self.previous_ticks = current_ticks
            self.previous_time = current_time
            return

        # -----------------------------
        # Compute delta
        # -----------------------------
        delta_ticks = current_ticks - self.previous_ticks
        delta_time = current_time - self.previous_time

        if delta_time <= 0:
            return

        # -----------------------------
        # Wheel velocity calculation
        # -----------------------------
        wheel_velocity = (
            (2.0 * math.pi * self.wheel_radius * delta_ticks)
            /
            (self.pulses_per_rev * self.gear_ratio * delta_time)
        )

        # -----------------------------
        # Publish velocity
        # -----------------------------
        velocity_msg = Float32()
        velocity_msg.data = float(wheel_velocity)

        self.velocity_publisher.publish(velocity_msg)

        # -----------------------------
        # Debug print
        # -----------------------------
        self.get_logger().info(
            f'Velocity: {wheel_velocity:.4f} m/s'
        )

        # Update previous values
        self.previous_ticks = current_ticks
        self.previous_time = current_time


def main(args=None):

    rclpy.init(args=args)

    node = EncoderVelocityNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()