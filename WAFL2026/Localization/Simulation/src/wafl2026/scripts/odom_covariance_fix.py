#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomCovarianceFix(Node):
    def __init__(self):
        super().__init__('odom_covariance_fix')
        self.sub = self.create_subscription(
            Odometry, '/odom', self.callback, 10)
        self.pub = self.create_publisher(
            Odometry, '/odom_fixed', 10)

    def callback(self, msg):
        # Inject non-zero diagonal covariance so EKF can invert the matrix.
        # Values: [x, y, z, roll, pitch, yaw] variance
        pose_cov = [0.01, 0, 0, 0, 0, 0,
                    0, 0.01, 0, 0, 0, 0,
                    0, 0, 0.01, 0, 0, 0,
                    0, 0, 0, 0.01, 0, 0,
                    0, 0, 0, 0, 0.01, 0,
                    0, 0, 0, 0, 0, 0.01]
        twist_cov = [0.01, 0, 0, 0, 0, 0,
                     0, 0.01, 0, 0, 0, 0,
                     0, 0, 0.01, 0, 0, 0,
                     0, 0, 0, 0.01, 0, 0,
                     0, 0, 0, 0, 0.01, 0,
                     0, 0, 0, 0, 0, 0.01]
        msg.pose.covariance = pose_cov
        msg.twist.covariance = twist_cov
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(OdomCovarianceFix())

if __name__ == '__main__':
    main()
