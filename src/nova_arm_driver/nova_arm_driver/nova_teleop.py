#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QVBoxLayout,
)

from PyQt5.QtCore import QTimer


class NovaTeleop(Node):

    def __init__(self):

        super().__init__("nova_teleop")

        self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.publisher = self.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        self.page = 0
        self.fine_mode = False

        self.previous_buttons = []

        self.step = 0.02
        self.fine_step = 0.005

        self.positions = [0.0] * 6

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "gripper",
        ]

        self.axes = [0.0] * 8

        self.timer = self.create_timer(
            0.05,
            self.publish_command
        )

        self.get_logger().info("Nova Teleop Started")

    def joy_callback(self, msg):

        self.axes = list(msg.axes)

        if len(self.previous_buttons) == 0:
            self.previous_buttons = [0] * len(msg.buttons)

        # L1
        if msg.buttons[4] and not self.previous_buttons[4]:
            self.page = (self.page - 1) % 3

        # R1
        if msg.buttons[5] and not self.previous_buttons[5]:
            self.page = (self.page + 1) % 3

        # X
        if msg.buttons[2] and not self.previous_buttons[2]:
            self.fine_mode = not self.fine_mode

        self.previous_buttons = list(msg.buttons)

        step = self.fine_step if self.fine_mode else self.step

        left = -msg.axes[1]
        right = -msg.axes[3]

        j1 = self.page * 2
        j2 = j1 + 1

        self.positions[j1] += left * step
        self.positions[j2] += right * step

    def publish_command(self):

        msg = JointState()

        msg.name = self.joint_names
        msg.position = self.positions

        self.publisher.publish(msg)


class Window(QWidget):

    def __init__(self, node):

        super().__init__()

        self.node = node

        self.setWindowTitle("Nova Arm Teleop")

        layout = QVBoxLayout()

        self.page_label = QLabel()
        layout.addWidget(self.page_label)

        self.mode_label = QLabel()
        layout.addWidget(self.mode_label)

        self.labels = []

        for i in range(6):

            lbl = QLabel()

            layout.addWidget(lbl)

            self.labels.append(lbl)

        self.help = QLabel(
            "\n"
            "L1/R1 : Change Joint Pair\n"
            "Left Stick : Joint A\n"
            "Right Stick : Joint B\n"
            "X : Toggle Fine Mode"
        )

        layout.addWidget(self.help)

        self.setLayout(layout)

        timer = QTimer(self)

        timer.timeout.connect(self.update_gui)

        timer.start(50)

    def update_gui(self):

        rclpy.spin_once(
            self.node,
            timeout_sec=0.0
        )

        pages = [
            "Joint1 / Joint2",
            "Joint3 / Joint4",
            "Joint5 / Gripper",
        ]

        self.page_label.setText(
            f"<h2>{pages[self.node.page]}</h2>"
        )

        mode = "Fine" if self.node.fine_mode else "Normal"

        self.mode_label.setText(
            f"<b>Mode:</b> {mode}"
        )

        for i, value in enumerate(self.node.positions):

            self.labels[i].setText(
                f"{self.node.joint_names[i]} : {value:.3f} rad"
            )


def main(args=None):

    rclpy.init(args=args)

    node = NovaTeleop()

    app = QApplication(sys.argv)

    window = Window(node)

    window.show()

    app.exec()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()