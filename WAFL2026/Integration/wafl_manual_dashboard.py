#!/usr/bin/env python3

import sys
import subprocess

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Int32

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel,
    QVBoxLayout, QHBoxLayout, QGroupBox,
    QLineEdit, QMessageBox
)
from PyQt5.QtCore import QTimer


# ================= ROS NODE =================
class WaflControlNode(Node):
    def __init__(self):
        super().__init__('wafl_manual_control')

        self.move_pub = self.create_publisher(Float32, '/move_distance', 10)
        self.steer_pub = self.create_publisher(Float32, '/target_angle', 10)
        self.lift_pub = self.create_publisher(Int32, '/lift_cmd', 10)

        self.move_timer = None
        self.move_value = 0.0

    # ---------- Move ----------
    def publish_move_once(self, value):
        msg = Float32()
        msg.data = value
        self.move_pub.publish(msg)

    def start_move_continuous(self, value, rate_hz=10):
        self.move_value = value
        self.move_timer = self.create_timer(
            1.0 / rate_hz, self.move_timer_cb
        )

    def stop_move_continuous(self):
        if self.move_timer:
            self.move_timer.cancel()
            self.move_timer = None

    def move_timer_cb(self):
        msg = Float32()
        msg.data = self.move_value
        self.move_pub.publish(msg)

    # ---------- Steering ----------
    def publish_steering(self, value):
        msg = Float32()
        msg.data = value
        self.steer_pub.publish(msg)

    # ---------- Lifting ----------
    def publish_lift(self, cmd):
        msg = Int32()
        msg.data = cmd
        self.lift_pub.publish(msg)


# ================= GUI =================
class WaflDashboard(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.node = ros_node
        self.agent_process = None

        self.setWindowTitle("WAFL - Manual Control Dashboard")
        self.setGeometry(200, 200, 600, 500)

        self.init_ui()

        # ROS spinning timer
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0))
        self.ros_timer.start(50)

    # ---------- UI ----------
    def init_ui(self):
        main_layout = QVBoxLayout()

        # Agent button
        self.agent_btn = QPushButton("Start micro-ROS Agent")
        self.agent_btn.clicked.connect(self.start_agent)
        main_layout.addWidget(self.agent_btn)

        # Top section
        top_layout = QHBoxLayout()

        # -------- Move Control --------
        move_group = QGroupBox("Move Control")
        move_layout = QVBoxLayout()

        self.move_input = QLineEdit()
        self.move_input.setPlaceholderText("Distance (Float)")
        move_layout.addWidget(self.move_input)

        move_once_btn = QPushButton("Publish Once")
        move_once_btn.clicked.connect(self.move_once)
        move_layout.addWidget(move_once_btn)

        move_cont_btn = QPushButton("Start Continuous")
        move_cont_btn.clicked.connect(self.move_continuous)
        move_layout.addWidget(move_cont_btn)

        move_stop_btn = QPushButton("Stop Continuous")
        move_stop_btn.clicked.connect(self.node.stop_move_continuous)
        move_layout.addWidget(move_stop_btn)

        move_group.setLayout(move_layout)
        top_layout.addWidget(move_group)

        # -------- Steering --------
        steer_group = QGroupBox("Steering")
        steer_layout = QVBoxLayout()

        self.steer_input = QLineEdit()
        self.steer_input.setPlaceholderText("Angle (+/- Float)")
        steer_layout.addWidget(self.steer_input)

        steer_btn = QPushButton("Publish Steering")
        steer_btn.clicked.connect(self.publish_steering)
        steer_layout.addWidget(steer_btn)

        steer_group.setLayout(steer_layout)
        top_layout.addWidget(steer_group)

        main_layout.addLayout(top_layout)

        # -------- Lifting --------
        lift_group = QGroupBox("Lifting Mechanism")
        lift_layout = QHBoxLayout()

        stop_btn = QPushButton("STOP")
        stop_btn.clicked.connect(lambda: self.node.publish_lift(0))

        up_btn = QPushButton("UP")
        up_btn.clicked.connect(lambda: self.node.publish_lift(1))

        down_btn = QPushButton("DOWN")
        down_btn.clicked.connect(lambda: self.node.publish_lift(2))

        lift_layout.addWidget(stop_btn)
        lift_layout.addWidget(up_btn)
        lift_layout.addWidget(down_btn)

        lift_group.setLayout(lift_layout)
        main_layout.addWidget(lift_group)

        # Exit
        exit_btn = QPushButton("EXIT")
        exit_btn.clicked.connect(self.close)
        main_layout.addWidget(exit_btn)

        self.setLayout(main_layout)

    # ---------- Callbacks ----------
    def start_agent(self):
        if self.agent_process is None:
            self.agent_process = subprocess.Popen([
                "ros2", "run", "micro_ros_agent",
                "micro_ros_agent", "udp4", "--port", "8888"
            ])
            QMessageBox.information(self, "Agent", "micro-ROS Agent Started")

    def move_once(self):
        try:
            value = float(self.move_input.text())
            self.node.publish_move_once(value)
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid move distance")

    def move_continuous(self):
        try:
            value = float(self.move_input.text())
            self.node.start_move_continuous(value)
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid move distance")

    def publish_steering(self):
        try:
            value = float(self.steer_input.text())
            self.node.publish_steering(value)
        except ValueError:
            QMessageBox.warning(self, "Error", "Invalid steering angle")

    def closeEvent(self, event):
        self.node.stop_move_continuous()
        if self.agent_process:
            self.agent_process.terminate()
        rclpy.shutdown()
        event.accept()


# ================= MAIN =================
def main():
    rclpy.init()
    node = WaflControlNode()

    app = QApplication(sys.argv)
    dashboard = WaflDashboard(node)
    dashboard.show()

    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
