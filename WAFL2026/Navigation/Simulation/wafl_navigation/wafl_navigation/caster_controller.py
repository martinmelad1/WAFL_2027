#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math


class CasterController(Node):

    def __init__(self):
        super().__init__('caster_controller')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.caster_pub = self.create_publisher(
            Float64,
            '/castor_cmd_pos',
            10
        )

        # تكبير تأثير الدوران (لأن التجربة أثبتت إن الزوايا الصغيرة
        # كانت تخلي الروبوت يكمل مستقيم)
        self.L = 0.5

        self.get_logger().info(
            "Caster Controller Started"
        )

    def cmd_vel_callback(self, msg):

        v = msg.linear.x
        w = msg.angular.z

        EPS = 0.01

        # robot stopped
        if abs(v) < EPS and abs(w) < EPS:

            angle = 0.0

        # rotate in place
        elif abs(v) < EPS:

            # لأن الـ caster خلف الروبوت
            angle = -math.pi/2 if w > 0 else math.pi/2

        else:

            # IMPORTANT:
            # caster خلف الروبوت → نعكس الإشارة
            angle = math.atan2(
                -w * self.L,
                v
            )

        max_angle = math.radians(80)

        angle = max(
            -max_angle,
            min(max_angle, angle)
        )

        out = Float64()
        out.data = angle

        self.caster_pub.publish(out)

        self.get_logger().info(
            f"v={v:.2f} "
            f"w={w:.2f} "
            f"caster={math.degrees(angle):.1f} deg"
        )


def main(args=None):

    rclpy.init(args=args)

    node = CasterController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
