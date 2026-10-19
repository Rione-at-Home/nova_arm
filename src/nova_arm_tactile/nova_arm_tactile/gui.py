#!/usr/bin/env python3
"""
tactile_gui: live view of the gripper pads. Subscribes to /arm/<side>/tactile/frame and shows the
pads side by side, with the grasp state when grasp_node is running. Works live or with a bag.

Needs a display and python3-tk (matplotlib's TkAgg backend).
"""
from __future__ import annotations

import threading

import numpy as np
import rclpy
from nova_arm_tactile_msgs.msg import GraspState, TactileFrame
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from .viewer import COLS, ROWS, DualViewer, ViewFrame


class GuiNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_tactile_gui")
        self.declare_parameter("sides", ["right", "left"])
        self.declare_parameter("window_s", 15.0)
        self.declare_parameter("delta_range", 120.0)
        self.declare_parameter("contact_threshold", 15.0)
        self.names = [str(s) for s in self.get_parameter("sides").value]
        self.viewer = None  # set by main()
        self.grasp_state = {}
        self.rezero_clients = {}
        for side in self.names:
            self.create_subscription(TactileFrame, f"/arm/{side}/tactile/frame",
                                     lambda m, s=side: self._on_frame(s, m), qos_profile_sensor_data)
            self.create_subscription(GraspState, f"/arm/{side}/grasp_state",
                                     lambda m, s=side: self.grasp_state.__setitem__(s, m), 10)
            self.rezero_clients[side] = self.create_client(Trigger, f"/arm/{side}/tactile/rezero")
        self.rezero_clients[None] = self.create_client(Trigger, "/arm/tactile/rezero_all")

    def _on_frame(self, side: str, m: TactileFrame) -> None:
        if self.viewer is None:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        raw = np.frombuffer(bytes(m.raw), dtype=np.uint8).astype(np.float32)
        delta = np.asarray(m.delta, dtype=np.float32)
        if raw.size != ROWS * COLS or delta.size != ROWS * COLS:
            return
        self.viewer.views[side].push(ViewFrame(
            t=t, seq=m.seq, raw=raw.reshape(ROWS, COLS), delta=delta.reshape(ROWS, COLS),
            calibrated=m.calibrated, peak=m.peak, total=m.total, contact_taxels=m.contact_taxels,
            in_contact=m.in_contact, centroid_row=m.centroid_row, centroid_col=m.centroid_col,
            bad_frames=m.bad_frames, dropped_bytes=m.dropped_bytes, rate_hz=m.rate_hz))

    def rezero(self, side) -> None:
        cli = self.rezero_clients[side]
        if not cli.service_is_ready():
            self.get_logger().warning("re-zero service not available (is tactile_node running?)")
            return
        cli.call_async(Trigger.Request())

    def subtitle(self) -> str:
        parts = []
        for side in self.names:
            g = self.grasp_state.get(side)
            if g is not None:
                parts.append(f"{side}: {g.state_name}  ({g.detail})")
        return "   |   ".join(parts) if parts else "grasp_node not running"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GuiNode()
    node.viewer = DualViewer(
        node.names, node.rezero,
        window_s=float(node.get_parameter("window_s").value),
        delta_range=float(node.get_parameter("delta_range").value),
        threshold=float(node.get_parameter("contact_threshold").value))
    node.viewer.subtitle_fn = node.subtitle
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    try:
        node.viewer.run()
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()