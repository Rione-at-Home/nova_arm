#!/usr/bin/env python3

import os
import time
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class NovaDemo(Node):

    def __init__(self):

        super().__init__("nova_demo")

        self.publisher = self.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        with open(pose_file, "r") as f:
            self.poses = yaml.safe_load(f)

        self.sequence = [
            "home",
            "ready",
            "approach",
            "grasp",
            "carry",
            "place",
            "home",
        ]

    def move_to_pose(self, pose_name):

        pose = self.poses[pose_name]

        msg = JointState()

        msg.name = pose["names"]
        msg.position = pose["positions"]

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Moving to {pose_name}"
        )

    def run(self):

        for pose in self.sequence:

            self.move_to_pose(pose)

            time.sleep(3)


def main(args=None):

    rclpy.init(args=args)

    node = NovaDemo()

    time.sleep(1)

    node.run()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()