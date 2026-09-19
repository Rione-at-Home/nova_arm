#!/usr/bin/env python3
"""
move_to_saved_pose.py

Moves the arm to a pose saved in poses.yaml (default: tray_hold) so you can
check that it is correct. Talks to ArmDriver via /arm_speed and /arm_command,
so ArmDriver must be running.

Usage:
    python3 move_to_saved_pose.py
    python3 move_to_saved_pose.py --name tray_hold --speed 15
"""

import argparse
import os
import sys
import time

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


class MoveToSavedPose(Node):

    def __init__(self):
        super().__init__("move_to_saved_pose")
        self.cmd_pub = self.create_publisher(JointState, "/arm_command", 10)
        self.speed_pub = self.create_publisher(Int32, "/arm_speed", 10)

    def wait_for_driver(self, timeout=5.0):
        start = time.time()
        while self.cmd_pub.get_subscription_count() == 0:
            if time.time() - start > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def set_speed(self, percent):
        msg = Int32()
        msg.data = int(percent)
        self.speed_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        time.sleep(0.3)

    def send_pose(self, names, positions):
        msg = JointState()
        msg.name = list(names)
        msg.position = [float(p) for p in positions]
        self.cmd_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--name", default="tray_hold")
    ap.add_argument("--speed", type=int, default=20, help="percent, 1-100")
    ap.add_argument(
        "--pose-file",
        default=os.path.expanduser("~/nova_arm_ws/poses.yaml"),
    )
    args = ap.parse_args()

    with open(args.pose_file, "r") as f:
        poses = yaml.safe_load(f) or {}

    if args.name not in poses:
        sys.exit(f"Pose '{args.name}' not found. Available: {list(poses.keys())}")

    pose = poses[args.name]

    rclpy.init()
    node = MoveToSavedPose()

    try:
        if not node.wait_for_driver():
            sys.exit("No subscriber on /arm_command. Is ArmDriver running?")

        print(f"Setting speed to {args.speed}%")
        node.set_speed(args.speed)

        print(f"Moving to '{args.name}':")
        for n, p in zip(pose["names"], pose["positions"]):
            print(f"  {n:<8} {p:7.3f} rad")

        # Send a few times to make sure the driver receives it.
        for _ in range(5):
            node.send_pose(pose["names"], pose["positions"])
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.1)

        print("Command sent. The driver keeps torque on, so the arm will hold the pose.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()