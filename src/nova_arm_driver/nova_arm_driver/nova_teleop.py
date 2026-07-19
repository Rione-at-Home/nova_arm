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

        self.save_pub = self.create_publisher(
            String,
            "/save_pose",
            10
        )

        self.save_pub = self.create_publisher(
            String,
            "/save_pose",
            10
        )

        # New publishers
        self.move_pub = self.create_publisher(
            String,
            "/move_to_pose",
            10
        )

        self.update_pub = self.create_publisher(
            String,
            "/update_pose",
            10
        )

        self.delete_pub = self.create_publisher(
            String,
            "/delete_pose",
            10
        )

        self.page = 0
        self.fine_mode = False

        self.previous_buttons = []

        self.step = 0.05
        self.fine_step = 0.01

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
    
    def selected_pose(self):

        item = self.pose_list.currentItem()

        if item is None:
            return None

        return item.text()


    def move_pose(self):

        pose = self.selected_pose()

        if pose is None:
            return

        msg = String()
        msg.data = pose

        self.node.move_pub.publish(msg)


    def update_pose(self):

        pose = self.selected_pose()

        if pose is None:
            return

        msg = String()
        msg.data = pose

        self.node.update_pub.publish(msg)


    def delete_pose(self):

        pose = self.selected_pose()

        if pose is None:
            return

        msg = String()
        msg.data = pose

        self.node.delete_pub.publish(msg)

    def publish_command(self):

        msg = JointState()

        msg.name = self.joint_names
        msg.position = self.positions

        self.publisher.publish(msg)


from PyQt5.QtWidgets import (
    QWidget,
    QLabel,
    QVBoxLayout,
    QPushButton,
    QLineEdit,
    QListWidget,
)

from std_msgs.msg import String

import yaml
import os


class Window(QWidget):

    def __init__(self, node):

        super().__init__()

        self.node = node

        self.setWindowTitle("Nova Arm Teach Pendant")
        self.resize(450, 600)

        layout = QVBoxLayout()

        ########################################################
        # Current Pair
        ########################################################

        self.page_label = QLabel()
        layout.addWidget(self.page_label)

        self.mode_label = QLabel()
        layout.addWidget(self.mode_label)

        ########################################################
        # Joint Values
        ########################################################

        self.labels = []

        for _ in range(6):

            lbl = QLabel()

            layout.addWidget(lbl)

            self.labels.append(lbl)


        layout.addWidget(QLabel("Pose Name"))

        self.pose_name = QLineEdit()

        self.pose_name.setPlaceholderText(
            "pickup_bag"
        )

        layout.addWidget(self.pose_name)


        self.save_button = QPushButton(
            "Save Pose"
        )

        self.move_button = QPushButton("Move To")
        self.update_button = QPushButton("Update")
        self.delete_button = QPushButton("Delete")
        self.rename_button = QPushButton("Rename")

        layout.addWidget(self.move_button)
        layout.addWidget(self.update_button)
        layout.addWidget(self.delete_button)
        layout.addWidget(self.rename_button)

        self.move_button.clicked.connect(self.move_pose)
        self.update_button.clicked.connect(self.update_pose)
        self.delete_button.clicked.connect(self.delete_pose)
        self.rename_button.clicked.connect(self.rename_pose)

        layout.addWidget(
            self.save_button
        )

        self.save_button.clicked.connect(
            self.save_pose
        )


        # Saved Poses
        layout.addWidget(QLabel("Saved Poses"))

        self.pose_list = QListWidget()

        self.pose_list.itemDoubleClicked.connect(
            lambda _: self.move_pose()
        )

        layout.addWidget(
            self.pose_list
        )




        self.setLayout(layout)


        self.update_timer = QTimer()

        self.update_timer.timeout.connect(
            self.update_gui
        )

        self.update_timer.start(50)
        self.pose_timer = QTimer()

        self.pose_timer.timeout.connect(
            self.refresh_pose_list
        )

        self.pose_timer.start(1000)


    def save_pose(self):

        name = self.pose_name.text().strip()

        if name == "":
            return

        msg = String()

        msg.data = name

        self.node.save_pub.publish(msg)


    def refresh_pose_list(self):

        pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        if not os.path.exists(
            pose_file
        ):
            return

        with open(
            pose_file,
            "r"
        ) as f:

            poses = yaml.safe_load(f)

        self.pose_list.clear()

        if poses:

            for name in poses.keys():

                self.pose_list.addItem(name)


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

        mode = (
            "Fine"
            if self.node.fine_mode
            else "Normal"
        )

        self.mode_label.setText(
            f"<b>Mode:</b> {mode}"
        )

        for i, value in enumerate(
            self.node.positions
        ):

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