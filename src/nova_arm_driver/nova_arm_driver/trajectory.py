import time

import numpy as np
import rclpy

from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


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

        # E-stop: latches True the moment a stop is requested.
        # Checked every cycle inside execute_segment so a fault can
        # halt an in-progress two-arm motion immediately, rather
        # than letting the current spline run to completion.
        self.estop_triggered = False

        self.estop_sub = node.create_subscription(
            Bool,
            "/estop",
            self.estop_callback,
            10
        )

        self.active_spline = None
        self.total_duration = 0.0

    def estop_callback(self, msg):

        if msg.data and not self.estop_triggered:
            self.node.get_logger().error(
                "E-STOP received - aborting trajectory execution"
            )

        self.estop_triggered = bool(msg.data)

    def reset_estop(self):

        # Explicit, separate call so clearing an e-stop is always a
        # deliberate action, never an accidental side effect of
        # starting the next segment.
        self.estop_triggered = False

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

            if self.estop_triggered:
                self.node.get_logger().error(
                    "Segment aborted: E-STOP active"
                )
                return False

            now_nano = self.node.get_clock().now().nanoseconds
            elapsed_sec = (now_nano - start_nano) / 1e9

            if elapsed_sec >= self.total_duration:
                # Final position publish
                final_pos = self.poses[sequence_subset[-1]]["positions"]
                self.publish(final_pos)
                # spin_once (not a blind sleep) so any pending
                # callback - e-stop, joint_states feedback, future
                # subscriptions - gets a chance to run during the
                # wait, instead of being silently deferred.
                rclpy.spin_once(self.node, timeout_sec=dt)
                break
            
            # Evaluate spline at exact current timestamp
            current_positions = self.evaluate(elapsed_sec)

            self.publish(
                current_positions
            )

            rclpy.spin_once(self.node, timeout_sec=dt)

        return True