import time

import numpy as np
import rclpy

from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState


class TrajectoryPlanner:

    def __init__(self, node, poses, joint_names):

        self.node = node
        self.poses = poses
        self.joint_names = joint_names

        self.pose_pub = node.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        self.active_spline = None
        self.total_duration = 0.0

    def prepare_segment(
        self,
        sequence_subset,
        segment_duration=2.0
    ):
        waypoints = [
            self.poses[name]["positions"]
            for name in sequence_subset
        ]

        num = len(waypoints)

        time_points = np.linspace(
            0,
            (num - 1) * segment_duration,
            num
        )

        self.active_spline = CubicSpline(
            time_points,
            waypoints,
            axis=0,
            bc_type="clamped",
        )

        self.total_duration = time_points[-1]

    def evaluate(self, t):

        return self.active_spline(t).tolist()

    def publish(self, positions):

        msg = JointState()
        msg.name = self.joint_names
        msg.position = positions
        self.pose_pub.publish(msg)
    
    def execute_segment(
    self,
    sequence_subset,
    segment_duration=2.0,
    dt=0.02,):
        
        
        self.prepare_segment(
            sequence_subset,
            segment_duration
        )
                        
        # Run a 50Hz streaming loop using ROS Clock
        start_nano = self.node.get_clock().now().nanoseconds

        while rclpy.ok():
            now_nano = self.node.get_clock().now().nanoseconds
            elapsed_sec = (now_nano - start_nano) / 1e9

            if elapsed_sec >= self.total_duration:
                # Final position publish
                final_pos = self.poses[sequence_subset[-1]]["positions"]
                self.publish(final_pos)
                time.sleep(dt)
                break
            
            # Evaluate spline at exact current timestamp
            current_positions = self.evaluate(elapsed_sec)

            self.publish(
                current_positions
            )
                        
            time.sleep(dt)
