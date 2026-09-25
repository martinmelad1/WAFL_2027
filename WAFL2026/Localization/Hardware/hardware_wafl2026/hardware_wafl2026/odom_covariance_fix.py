#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SensorFix(Node):
    def __init__(self):
        super().__init__('sensor_fix')

        # --- Odom relay: /odom -> /odom_fixed ---
        # Ignition DiffDrive publishes all-zero covariance. Inject non-zero
        # diagonal so the EKF can invert the measurement noise matrix.
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/odom_fixed', 10)

        # --- IMU relay: /imu -> /imu_fixed ---
        # Ignition IMU publishes all-zero covariance AND a Gazebo-scoped
        # frame_id (wafl2025/base_footprint/imu_sensor). Fix both.
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)
        self.imu_pub = self.create_publisher(
            Imu, '/imu_fixed', 10)

        self.get_logger().info(
            'sensor_fix started: /odom->/odom_fixed, /imu->/imu_fixed')

    def odom_callback(self, msg):
        cov = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.01, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.01,
        ]
        msg.pose.covariance = cov
        msg.twist.covariance = cov
        self.odom_pub.publish(msg)

    def imu_callback(self, msg):
        # Fix frame_id — Gazebo scopes it as 'model/link/sensor_name'
        msg.header.frame_id = 'imu_link'

        # Inject realistic covariances for a basic IMU
        # orientation: ~0.01 rad std dev
        msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]
        # angular velocity: ~0.005 rad/s std dev
        msg.angular_velocity_covariance = [
            0.005, 0.0,   0.0,
            0.0,   0.005, 0.0,
            0.0,   0.0,   0.005,
        ]
        # linear acceleration: ~0.1 m/s^2 std dev
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
