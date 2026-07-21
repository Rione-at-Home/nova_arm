#!/usr/bin/env python3

import os
import time
import yaml

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class NovaDemoTrajectory(Node):

    def __init__(self):
        super().__init__("nova_demo_trajectory")

        # Publisher
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )

        pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        with open(pose_file, "r") as f:
            self.poses = yaml.safe_load(f)


    def build_trajectory_message(self, sequence_subset, segment_duration=2.0):
        """
        Creates a JointTrajectory message for a sequence of keyframe poses.
        
        :param sequence_subset: List of pose names from poses.yaml
        :param segment_duration: Time in seconds between each pose waypoint
        """
        msg = JointTrajectory()
        joint_names = self.poses["home"]["names"]
        msg.joint_names = joint_names

        cumulative_time = 0.0

        for pose_name in sequence_subset:
            
            point = JointTrajectoryPoint()
            point.positions = self.poses[pose_name]["positions"]

            # Convert target time into ROS 2 Duration message
            sec = int(cumulative_time)
            nanosec = int((cumulative_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(point)

            # Add timestamp for the next waypoint
            cumulative_time += segment_duration

        return msg, cumulative_time


    def execute_sequence(self):

        time.sleep(1.0)

        # Segment 1: Home -> Ready -> Approach -> Grasp
        self.get_logger().info("Phase A: Moving to target bag...")
        msg_a, duration_a = self.build_trajectory_message(
            ["home", "ready", "approach", "grasp"], 
            segment_duration=2.0
        )
        self.trajectory_pub.publish(msg_a)
        time.sleep(duration_a + 0.5) # Wait for hardware execution to finish

        # Action Pause: Gripper closing time
        self.get_logger().info("Grasping object...")
        time.sleep(1.0)

        # Segment 2: Grasp -> Carry -> Place
        self.get_logger().info("Phase B: Carrying object to place destination...")
        msg_b, duration_b = self.build_trajectory_message(
            ["grasp", "carry", "place"], 
            segment_duration=2.0
        )
        self.trajectory_pub.publish(msg_b)
        time.sleep(duration_b + 0.5)

        # Action Pause: Gripper releasing time
        self.get_logger().info("Releasing object...")
        time.sleep(1.0)

        # Segment 3: Place -> Home
        self.get_logger().info("Phase C: Returning Home...")
        msg_c, duration_c = self.build_trajectory_message(
            ["place", "home"], 
            segment_duration=2.0
        )
        self.trajectory_pub.publish(msg_c)
        time.sleep(duration_c + 0.5)

        self.get_logger().info("Full competition trajectory completed!")


def main(args=None):
    rclpy.init(args=args)
    node = NovaDemoTrajectory()
    
    node.execute_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
