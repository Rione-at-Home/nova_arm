#!/usr/bin/env python3

import os
import time
import yaml
import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from .trajectory import TrajectoryPlanner 


class NovaDemoPrecision(Node):

    def __init__(self):
        super().__init__("nova_demo_precision")


        self.speed_pub = self.create_publisher(
            Int32,
            "/arm_speed",
            10
        )

        pose_file = os.path.expanduser("~/nova_arm_ws/poses.yaml")
        with open(pose_file, "r") as f:
            self.poses = yaml.safe_load(f)

        self.joint_names = self.poses["home"]["names"]

        self.dt = 0.02  # 50 Hz control loop

        # ROS 2 High-Precision Timer
        self.timer = None

        self.planner = TrajectoryPlanner(
            node=self,
            poses=self.poses,
            joint_names=self.joint_names
        )

    def set_speed(self, percent):
        msg = Int32()
        msg.data = percent
        self.speed_pub.publish(msg)
        time.sleep(0.1)


   
    def run_full_mission(self):

        self.set_speed(30)

        self.get_logger().info(
            "Phase A: Moving to target bag..."
        )

        self.planner.execute_segment(
            ["home", "ready", "approach", "grasp"]
        )

        self.get_logger().info(
            "Grasping object..."
        )

        time.sleep(1.0)

        self.get_logger().info(
            "Phase B: Carrying object..."
        )

        self.planner.execute_segment(
            ["grasp", "carry", "place"]
        )

        self.get_logger().info(
            "Releasing object..."
        )

        time.sleep(1.0)

        self.get_logger().info(
            "Phase C: Returning Home..."
        )

        self.planner.execute_segment(
            ["place", "home"]
        )

        self.get_logger().info(
            "Mission Completed Successfully!"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NovaDemoPrecision()
    
    time.sleep(1.0)
    node.run_full_mission()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
