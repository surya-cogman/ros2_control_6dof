#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (
    QApplication, QWidget, QSlider, QLabel, QVBoxLayout, QHBoxLayout
)
from PyQt5.QtCore import Qt
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class ArmGUI(Node):
    def __init__(self):
        super().__init__("arm_gui")

        self.joint_names = [
            "joint_1", "joint_2", "joint_3",
            "joint_4", "joint_5", "joint_6"
        ]

        self.current_pos = np.zeros(6)

        # ROS2 subscriber + publisher
        self.create_subscription(
            JointState, "/joint_states",
            self.joint_state_callback, 10
        )

        self.pub = self.create_publisher(
            Float64MultiArray,
            "/forward_command_controller/commands",
            10
        )

        # ---- GUI ----
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle("6DOF Arm Controller")

        layout = QVBoxLayout()

        self.sliders = []
        self.labels = []

        for i in range(6):
            hbox = QHBoxLayout()

            label = QLabel(f"Joint {i+1}: 0.00 rad")
            self.labels.append(label)
            hbox.addWidget(label)

            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-314)   # -3.14 rad
            slider.setMaximum(314)    # 3.14 rad
            slider.setValue(0)
            slider.valueChanged.connect(self.slider_changed)
            self.sliders.append(slider)

            hbox.addWidget(slider)
            layout.addLayout(hbox)

        self.window.setLayout(layout)
        self.window.show()

    def joint_state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.current_pos[idx] = pos

        # Update label text
        for i in range(6):
            self.labels[i].setText(
                f"Joint {i+1}: {self.current_pos[i]:.2f} rad")

    def slider_changed(self):
        cmd = Float64MultiArray()
        cmd.data = [s.value() / 100.0 for s in self.sliders]  # scale down
        self.pub.publish(cmd)

    def run(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.app.processEvents()


def main():
    rclpy.init()
    gui = ArmGUI()

    while True:
        gui.run()


if __name__ == "__main__":
    main()
