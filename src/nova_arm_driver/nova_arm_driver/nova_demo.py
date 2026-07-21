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


class NovaDemoPrecision(Node):

    def __init__(self):
        super().__init__("nova_demo_precision")

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

        pose_file = os.path.expanduser("~/nova_arm_ws/poses.yaml")
        with open(pose_file, "r") as f:
            self.poses = yaml.safe_load(f)

        self.joint_names = self.poses["home"]["names"]

        self.active_spline = None
        self.total_duration = 0.0
        self.start_time = None
        self.dt = 0.02  # 50 Hz control loop

        # ROS 2 High-Precision Timer
        self.timer = None

    def set_speed(self, percent):
        msg = Int32()
        msg.data = percent
        self.speed_pub.publish(msg)
        time.sleep(0.1)

    def publish_pose(self, positions):
        msg = JointState()
        msg.name = self.joint_names
        msg.position = positions
        self.pose_pub.publish(msg)

    def prepare_spline_segment(self, sequence_subset, segment_duration=2.0):
        """Generates the cubic spline function for a set of poses."""
        
        waypoints = [self.poses[pose_name]["positions"] for pose_name in sequence_subset]
        num_waypoints = len(waypoints)
        
        time_points = np.linspace(0, (num_waypoints - 1) * segment_duration, num_waypoints)
        
        # 'clamped' enforces zero velocity at start and end of segment
        self.active_spline = CubicSpline(time_points, waypoints, axis=0, bc_type='clamped')
        self.total_duration = time_points[-1]
        self.start_time = self.get_clock().now()

    def run_full_mission(self):
        # Set arm speed
        self.set_speed(30)

        # Segment 1: Approach & Grasp
        self.get_logger().info("Phase A: Moving to target bag...")
        self.execute_segment_blocking(["home", "ready", "approach", "grasp"], segment_duration=2.0)

        # Gripper Pause
        self.get_logger().info("Grasping object...")
        time.sleep(1.0)

        # Segment 2: Carry & Place
        self.get_logger().info("Phase B: Carrying object to place destination...")
        self.execute_segment_blocking(["grasp", "carry", "place"], segment_duration=2.0)

        # Gripper Pause
        self.get_logger().info("Releasing object...")
        time.sleep(1.0)

        # Segment 3: Return Home
        self.get_logger().info("Phase C: Returning Home...")
        self.execute_segment_blocking(["place", "home"], segment_duration=2.0)

        self.get_logger().info("Mission Completed Successfully!")

    def execute_segment_blocking(self, sequence_subset, segment_duration=2.0):
        self.prepare_spline_segment(sequence_subset, segment_duration)
        
        # Run a 50Hz streaming loop using ROS Clock
        start_nano = self.get_clock().now().nanoseconds
        total_nano = int(self.total_duration * 1e9)

        while rclpy.ok():
            now_nano = self.get_clock().now().nanoseconds
            elapsed_sec = (now_nano - start_nano) / 1e9

            if elapsed_sec >= self.total_duration:
                # Final position publish
                final_pos = self.poses[sequence_subset[-1]]["positions"]
                self.publish_pose(final_pos)
                break

            # Evaluate spline at exact current timestamp
            current_positions = self.active_spline(elapsed_sec).tolist()
            self.publish_pose(current_positions)
            
            time.sleep(self.dt)


def main(args=None):
    rclpy.init(args=args)
    node = NovaDemoPrecision()
    
    time.sleep(1.0)
    node.run_full_mission()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
