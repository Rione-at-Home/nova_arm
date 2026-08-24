#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PoseManager(Node):

    def __init__(self):

        super().__init__("nova_pose_manager")

        self.current_pose = JointState()
        self.current_positions = []

        self.pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        # Load existing poses at startup
        self.poses = {}

        if os.path.exists(self.pose_file):

            with open(self.pose_file, "r") as f:

                loaded = yaml.safe_load(f)

            if loaded is not None:
                self.poses = loaded

        # Publisher so this node can command the arm directly
        # (needed for move_callback)
        self.arm_pub = self.create_publisher(
            JointState,
            "/arm_command",
            10
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self.pose_callback,
            10
        )

        self.create_subscription(
            String,
            "/save_pose",
            self.save_callback,
            10
        )

        self.create_subscription(
            String,
            "/move_to_pose",
            self.move_callback,
            10
        )

        self.create_subscription(
            String,
            "/update_pose",
            self.update_callback,
            10
        )

        self.create_subscription(
            String,
            "/delete_pose",
            self.delete_callback,
            10
        )

        self.get_logger().info(
            "Nova Pose Manager Started"
        )

    def pose_callback(self, msg):

        # This now comes from /joint_states (ArmDriver's sync-read
        # feedback), i.e. the arm's actual measured position — not
        # an echo of the last commanded /arm_command message. That
        # means Save/Update below capture where the arm truly is,
        # even if it stalled short of a commanded goal.
        self.current_pose = msg
        self.current_positions = list(msg.position)

    def move_callback(self, msg):

        name = msg.data

        if name not in self.poses:

            self.get_logger().warning(
                f"Pose '{name}' not found."
            )

            return

        pose = self.poses[name]

        joint_msg = JointState()

        joint_msg.name = pose["names"]
        joint_msg.position = pose["positions"]

        self.arm_pub.publish(joint_msg)

        self.get_logger().info(
            f"Moved to '{name}'"
        )

    def delete_callback(self, msg):

        name = msg.data

        if name not in self.poses:
            return

        del self.poses[name]

        self.save_yaml()

        self.get_logger().info(
            f"Deleted '{name}'"
        )

    def update_callback(self, msg):

        name = msg.data

        if name not in self.poses:
            return

        if not self.current_positions:

            self.get_logger().warning(
                "No current position received yet; can't update."
            )

            return

        self.poses[name]["positions"] = self.current_positions.copy()

        self.save_yaml()

        self.get_logger().info(
            f"Updated '{name}'"
        )

    def save_callback(self, msg):

        pose_name = msg.data.strip()

        if pose_name == "":

            self.get_logger().warn(
                "Pose name is empty."
            )

            return

        # Refresh from current in-memory state
        # (self.poses is kept up to date across calls)
        self.poses[pose_name] = {

            "names": list(self.current_pose.name),
            "positions": list(self.current_pose.position),

        }

        self.save_yaml()

        self.get_logger().info(
            f"Saved pose '{pose_name}'"
        )

    def save_yaml(self):

        with open(self.pose_file, "w") as f:

            yaml.dump(
                self.poses,
                f,
                sort_keys=False
            )


############################################################


def main(args=None):

    rclpy.init(args=args)

    node = PoseManager()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()