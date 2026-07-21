#!/usr/bin/env python3

import time
import numpy as np

from sensor_msgs.msg import JointState


class TrajectoryPlanner:

    def __init__(self, node):

        self.node = node

        self.publisher = node.create_publisher(
            JointState,
            "/arm_command",
            10
        )

    ###########################################################

    def cubic_segment(self):
        """
        Computes one cubic spline segment.
        """

        pass

    ###########################################################

    def interpolate_segment(self):
        """
        Generates interpolated points between
        two joint configurations.
        """

        pass

    ###########################################################

    def publish(self, names, positions):

        msg = JointState()

        msg.name = names
        msg.position = positions

        self.publisher.publish(msg)

    ###########################################################

    def execute_segment(self):
        """
        Publish every point in a trajectory.
        """

        pass

    ###########################################################

    def execute_path(self):
        """
        Execute multiple spline segments.
        """

        pass