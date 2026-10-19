#!/usr/bin/env python3
"""
fake_gripper: bench stand-in for nova_arm_driver's two gripper joints, for testing grasp_node on a
laptop together with `tactile_node --ros-args -p demo:=true`.

Subscribes /arm_command, follows "<side>_gripper" goals at `speed` rad/s, and publishes
/joint_states at 50 Hz. A simulated object stops the finger slightly past `object_rad` (matching
tactile_node's demo_contact_rad), so the pad sees pressure while the finger angle barely moves,
like the real servo pushing on an object. Set object_rad to a value past the closed limit for an
"empty gripper" run. Do NOT run it alongside the real driver (both would publish /joint_states).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeGripper(Node):
    def __init__(self) -> None:
        super().__init__("fake_gripper")
        self.declare_parameter("sides", ["right", "left"])
        self.declare_parameter("speed", 3.0)
        self.declare_parameter("object_rad", 0.45)
        self.declare_parameter("closing_sign", 1.0)
        self.joints = [f"{s}_gripper" for s in self.get_parameter("sides").value]
        self.goal = {j: 0.0 for j in self.joints}
        self.virtual = {j: 0.0 for j in self.joints}
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(JointState, "/arm_command", self._on_cmd, 10)
        self.create_timer(0.02, self._tick)
        self.get_logger().info(f"fake grippers {self.joints}, object at "
                               f"{self.get_parameter('object_rad').value} rad")

    def _on_cmd(self, m: JointState) -> None:
        for n, p in zip(m.name, m.position):
            if n in self.goal:
                self.goal[n] = float(p)

    def _tick(self) -> None:
        speed = float(self.get_parameter("speed").value) * 0.02
        obj = float(self.get_parameter("object_rad").value)
        sign = float(self.get_parameter("closing_sign").value)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for j in self.joints:
            d = self.goal[j] - self.virtual[j]
            self.virtual[j] += max(-speed, min(speed, d))
            pos = self.virtual[j]
            past = sign * (pos - obj)
            if past > 0:                       # finger stopped by the object (2 cm-ish of give)
                pos = obj + sign * min(past, 0.02)
            msg.name.append(j)
            msg.position.append(pos)
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeGripper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()