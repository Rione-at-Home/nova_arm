#!/usr/bin/env python3

import os
import time
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

import numpy as np
from scipy.interpolate import CubicSpline


class NovaDemo(Node):

    def __init__(self):

        super().__init__("nova_demo")

        self.pose_pub = self.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        self.speed_pub = self.create_publisher(
            Int32,
            "/arm_speed",
            10
        )

        pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        with open(pose_file, "r") as f:

            self.poses = yaml.safe_load(f)

        self.sequence = [
            ("home", 50),
            ("ready", 50),
            ("approach", 60),
            ("grasp", 45),
            ("carry", 45),
            ("place", 40),
            ("home", 50),
        ]

        self.current_position = None


    def set_speed(self, percent):

        msg = Int32()

        msg.data = percent

        self.speed_pub.publish(msg)

        self.get_logger().info(
            f"Speed: {percent}%"
        )

        time.sleep(0.2)


    def publish_pose(self, names, positions):

        msg = JointState()

        msg.name = names
        msg.position = list(map(float, positions))

        self.pose_pub.publish(msg)

    def execute_spline_sequence(self, sequence_subset, segment_duration=2.5, dt=0.02):
        """Execute a sequence of poses using cubic spline interpolation."""

        joint_names = self.poses["home"]["names"]

        waypoints = [self.poses[pose_name]["positions"] for pose_name in sequence_subset]


        num_waypoints = len(waypoints)
        time_points = np.linspace(0, segment_duration * (num_waypoints - 1), num_waypoints)

        cs = CubicSpline(time_points, waypoints, axis=0, bc_type='clamped')

        sample_times = np.arange(0, time_points[-1], dt)

        for t in sample_times:
            interpolated_positions = cs(t)

            self.publish_pose(joint_names, interpolated_positions)

            time.sleep(dt)

        self.publish_pose(joint_names, waypoints[-1])

    def run_spline_sequence(self, segment_duration=2.5, dt=0.02):


        self.get_logger().info("Starting spline sequence...")
        self.execute_spline_sequence(["home", "ready", "approach", "grasp"], segment_duration)

        self.get_logger().info("Grasping object...")
        time.sleep(1)

        self.get_logger().info("Carrying object...")
        self.execute_spline_sequence(["grasp", "carry", "place"], segment_duration)

        self.get_logger().info("Placing object...")
        time.sleep(1)

        self.get_logger().info("Returning to home position...")
        self.execute_spline_sequence(["place", "home"], segment_duration)

    def move_to_pose(
        self,
        pose_name,
        steps=60,
        dt=0.05
    ):

        pose = self.poses[pose_name]

        goal = pose["positions"]

        if self.current_position is None:

            self.current_position = goal.copy()

            self.publish_pose(
                pose["names"],
                goal
            )

            time.sleep(2)

            return

        start = self.current_position.copy()

        self.get_logger().info(
            f"Moving -> {pose_name}"
        )

        for step in range(steps + 1):

            raw_alpha = step / steps

            # Quintic Easing
            alpha = raw_alpha ** 3 * (raw_alpha * (raw_alpha * 6 - 15) + 10)

            interp = []

            for s, g in zip(start, goal):

                interp.append(
                    s + alpha * (g - s)
                )

            self.publish_pose(
                pose["names"],
                interp
            )

            time.sleep(dt)

        self.current_position = goal.copy()

        time.sleep(0.5)


    def run(self):

        for pose_name, speed in self.sequence:

            self.set_speed(speed)

            self.move_to_pose(
                pose_name
            )




def main(args=None):

    rclpy.init(args=args)

    node = NovaDemo()

    time.sleep(1)

    node.run_spline_sequence(segment_duration=2.5, dt=0.02)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":

    main()