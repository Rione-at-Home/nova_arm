from sensor_msgs.msg import JointState
import numpy as np
from scipy.interpolate import CubicSpline

class TrajectoryPlanner:

    def __init__(self, node):

        self.node = node

        self.pose_pub = node.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        self.active_spline = None
        self.total_duration = 0.0

    def prepare_segment(
        self,
        poses,
        sequence_subset,
        segment_duration=2.0,
    ):

        waypoints = [
            poses[name]["positions"]
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

    def publish(self, names, positions):

        msg = JointState()

        msg.name = names
        msg.position = positions

        self.pose_pub.publish(msg)