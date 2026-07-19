#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs import msg
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class PoseManager(Node):

    def __init__(self):

        super().__init__("nova_pose_manager")

        self.current_pose = JointState()

        self.pose_file = os.path.expanduser(
            "~/nova_arm_ws/poses.yaml"
        )

        self.create_subscription(
            JointState,
            "/arm_command",
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

        self.current_pose = msg

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
        
    def save_callback(self, msg):

        pose_name = msg.data.strip()

        if pose_name == "":

            self.get_logger().warn(
                "Pose name is empty."
            )

            return

        poses = {}

        if os.path.exists(self.pose_file):

            with open(self.pose_file, "r") as f:

                loaded = yaml.safe_load(f)

                if loaded is not None:
                    poses = loaded

        poses[pose_name] = {

            "names":
                list(self.current_pose.name),

            "positions":
                list(self.current_pose.position),

        }

        with open(self.pose_file, "w") as f:

            yaml.dump(
                poses,
                f,
                sort_keys=False
            )

        self.get_logger().info(
            f"Saved pose '{pose_name}'"
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