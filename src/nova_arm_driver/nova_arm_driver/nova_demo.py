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

    def spin_wait(self, duration_sec, dt=0.02):
        # spin_once instead of a
        # blind sleep, so e-stop (and anything else) is still live
        # during dwell periods like a grasp/release pause, not just
        # during active motion.
        start_nano = self.get_clock().now().nanoseconds

        while rclpy.ok():
            elapsed_sec = (
                self.get_clock().now().nanoseconds - start_nano
            ) / 1e9

            if elapsed_sec >= duration_sec:
                break

            if self.planner.estop_triggered:
                return False

            rclpy.spin_once(self, timeout_sec=dt)

        return True

   
    def run_full_mission(self):

        self.set_speed(30)

        self.get_logger().info(
            "Phase A: Moving to target bag..."
        )

        if not self.planner.execute_segment(
            ["home", "ready", "approach", "grasp"]
        ):
            self.get_logger().error(
                "Mission aborted during Phase A"
            )
            return

        self.get_logger().info(
            "Grasping object..."
        )

        if not self.spin_wait(1.0):
            self.get_logger().error(
                "Mission aborted during grasp dwell"
            )
            return

        self.get_logger().info(
            "Phase B: Carrying object..."
        )

        if not self.planner.execute_segment(
            ["grasp", "carry", "place"]
        ):
            self.get_logger().error(
                "Mission aborted during Phase B"
            )
            return

        self.get_logger().info(
            "Releasing object..."
        )

        if not self.spin_wait(1.0):
            self.get_logger().error(
                "Mission aborted during release dwell"
            )
            return

        self.get_logger().info(
            "Phase C: Returning Home..."
        )

        if not self.planner.execute_segment(
            ["place", "home"]
        ):
            self.get_logger().error(
                "Mission aborted during Phase C"
            )
            return

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