#!/usr/bin/env python3
"""
grasp_node: tactile-guided grasping for both nova_arm grippers (option B).

Per side (<side> = right | left):
  action   /arm/<side>/grasp          nova_arm_tactile_msgs/action/Grasp   GRASP or OPEN
  pub      /arm/<side>/grasp_state    nova_arm_tactile_msgs/GraspState     state_rate_hz, always
  sub      /arm/<side>/tactile/frame  from tactile_node
  sub      /arm/<side>/motion_allowed std_msgs/Bool (latched), same gate as nova_arm_driver
  client   /arm/<side>/tactile/rezero called after a successful OPEN (auto_rezero_on_open)
Shared:
  pub      /arm_command               sensor_msgs/JointState, only "<side>_gripper"
  sub      /joint_states              measured gripper angle

The node only ever moves the two gripper joints, and only while a goal is active or while
holding (small re-tightening steps). Between goals it never opens on its own: a fault or a
lost object leaves the gripper where it is for the task layer to decide.

All controller logic lives in grasp_controller.py (ROS-free, unit tested).
"""
from __future__ import annotations

import threading
import time
from typing import Dict, Optional

import rclpy
from nova_arm_tactile_msgs.action import Grasp
from nova_arm_tactile_msgs.msg import GraspState, TactileFrame
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .grasp_controller import (CANCELED, OPENED, GoalResult, GraspConfig, GraspController,
                               TactileSample)

NAN = float("nan")


class _Waiter:
    """Hands a controller result from the control timer to the action's execute callback."""

    def __init__(self) -> None:
        self.event = threading.Event()
        self.result: Optional[GoalResult] = None

    def finish(self, result: GoalResult) -> None:
        self.result = result
        self.event.set()


class _Side:
    def __init__(self, name: str, cfg: GraspConfig) -> None:
        self.name = name
        self.joint = f"{name}_gripper"
        self.ctl = GraspController(cfg)
        self.sample: Optional[TactileSample] = None
        self.position = NAN
        self.position_stamp = -1e9
        self.motion_allowed = True
        self.waiter: Optional[_Waiter] = None


class GraspNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_grasp")
        d = GraspConfig()
        self.declare_parameter("sides", ["right", "left"])
        self.declare_parameter("control_rate_hz", 25.0)
        self.declare_parameter("state_rate_hz", 10.0)
        self.declare_parameter("auto_rezero_on_open", True)
        for key in ("close_speed", "squeeze_speed", "target_total", "max_squeeze_rad", "settle_s",
                    "slip_ratio", "overshoot_ratio", "stall_rad", "stall_s", "open_tol_rad",
                    "open_timeout_s", "goal_timeout_s", "calib_wait_s", "tactile_stale_s",
                    "joint_stale_s"):
            self.declare_parameter(key, float(getattr(d, key)))
        self.declare_parameter("contact_confirm_ticks", d.contact_confirm_ticks)
        self.declare_parameter("lost_ticks", d.lost_ticks)

        self.lock = threading.Lock()
        self.sides: Dict[str, _Side] = {}
        self.auto_rezero = bool(self.get_parameter("auto_rezero_on_open").value)
        control_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.cmd_pub = self.create_publisher(JointState, "/arm_command", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10,
                                 callback_group=control_group)
        self.state_pubs, self.rezero_clients, self.action_servers = {}, {}, {}

        for side in [str(s) for s in self.get_parameter("sides").value]:
            # per-side open/closed angles: the left gripper may close in the other direction
            self.declare_parameter(f"{side}.open_rad", d.open_rad)
            self.declare_parameter(f"{side}.closed_rad", d.closed_rad)
            cfg = self._config(side)
            s = self.sides[side] = _Side(side, cfg)
            self.get_logger().info(
                f"[{side}] gripper '{s.joint}': open {cfg.open_rad:.2f} rad, closed limit "
                f"{cfg.closed_rad:.2f} rad, target total {cfg.target_total:.0f}")

            self.create_subscription(TactileFrame, f"/arm/{side}/tactile/frame",
                                     lambda m, s=s: self._on_tactile(s, m),
                                     qos_profile_sensor_data, callback_group=control_group)
            self.create_subscription(Bool, f"/arm/{side}/motion_allowed",
                                     lambda m, s=s: self._on_motion_allowed(s, m), latched,
                                     callback_group=control_group)
            self.state_pubs[side] = self.create_publisher(GraspState, f"/arm/{side}/grasp_state", 10)
            self.rezero_clients[side] = self.create_client(
                Trigger, f"/arm/{side}/tactile/rezero", callback_group=action_group)
            self.action_servers[side] = ActionServer(
                self, Grasp, f"/arm/{side}/grasp",
                execute_callback=lambda gh, s=s: self._execute(s, gh),
                goal_callback=lambda _req: GoalResponse.ACCEPT,
                cancel_callback=lambda _gh: CancelResponse.ACCEPT,
                callback_group=action_group)

        rate = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate, self._control, callback_group=control_group)
        srate = max(1.0, float(self.get_parameter("state_rate_hz").value))
        self.create_timer(1.0 / srate, self._publish_states, callback_group=control_group)
        self.get_logger().info("arm_grasp up: actions " +
                               ", ".join(f"/arm/{n}/grasp" for n in self.sides))

    def _config(self, side: str) -> GraspConfig:
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        cfg = GraspConfig(open_rad=float(p(f"{side}.open_rad")),
                          closed_rad=float(p(f"{side}.closed_rad")))
        for key in ("close_speed", "squeeze_speed", "target_total", "max_squeeze_rad", "settle_s",
                    "slip_ratio", "overshoot_ratio", "stall_rad", "stall_s", "open_tol_rad",
                    "open_timeout_s", "goal_timeout_s", "calib_wait_s", "tactile_stale_s",
                    "joint_stale_s"):
            setattr(cfg, key, float(p(key)))
        cfg.contact_confirm_ticks = int(p("contact_confirm_ticks"))
        cfg.lost_ticks = int(p("lost_ticks"))
        if abs(cfg.closed_rad - cfg.open_rad) < 0.05:
            raise ValueError(f"{side}: open_rad and closed_rad are too close together")
        return cfg

    # ------------------------------------------------------------------ inputs
    def _on_tactile(self, s: _Side, m: TactileFrame) -> None:
        sample = TactileSample(stamp=time.monotonic(), calibrated=m.calibrated,
                               in_contact=m.in_contact, total=m.total,
                               contact_taxels=m.contact_taxels, centroid_row=m.centroid_row,
                               centroid_col=m.centroid_col)
        with self.lock:
            s.sample = sample

    def _on_joint_states(self, m: JointState) -> None:
        now = time.monotonic()
        with self.lock:
            for name, pos in zip(m.name, m.position):
                for s in self.sides.values():
                    if name == s.joint:
                        s.position, s.position_stamp = float(pos), now

    def _on_motion_allowed(self, s: _Side, m: Bool) -> None:
        with self.lock:
            if bool(m.data) != s.motion_allowed:
                self.get_logger().warning(
                    f"[{s.name}] gripper motion " + ("ALLOWED" if m.data else "INHIBITED"))
            s.motion_allowed = bool(m.data)

    # ------------------------------------------------------------------ control loop
    def _control(self) -> None:
        now = time.monotonic()
        finished = []
        with self.lock:
            for s in self.sides.values():
                cmd, result = s.ctl.tick(now, s.sample, s.position, s.position_stamp,
                                         s.motion_allowed)
                if cmd is not None and s.motion_allowed:
                    self._send(s, cmd)
                if result is not None:
                    # take the waiter now, under the lock, so a goal accepted right after this
                    # tick cannot receive the previous goal's result
                    finished.append((s, result, s.waiter))
                    s.waiter = None
        for s, result, waiter in finished:
            self._finish(s, result, waiter)

    def _send(self, s: _Side, cmd: float) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [s.joint]
        msg.position = [float(cmd)]
        self.cmd_pub.publish(msg)

    def _finish(self, s: _Side, result: GoalResult, waiter: Optional[_Waiter]) -> None:
        level = self.get_logger().info if result.success else self.get_logger().warning
        level(f"[{s.name}] {result.outcome_name}: {result.message}")
        if result.outcome == OPENED and self.auto_rezero:
            cli = self.rezero_clients[s.name]
            if cli.service_is_ready():
                cli.call_async(Trigger.Request())
            else:
                self.get_logger().warning(f"[{s.name}] tactile rezero service not available")
        if waiter is not None:
            waiter.finish(result)

    def _publish_states(self) -> None:
        now = time.monotonic()
        with self.lock:
            for s in self.sides.values():
                c, smp = s.ctl, s.sample
                m = GraspState()
                m.header.stamp = self.get_clock().now().to_msg()
                m.side = s.name
                m.state = c.state
                m.state_name = c.state_name
                m.detail = c.detail
                m.tactile_ok = c.tactile_ok(now)
                if smp is not None:
                    m.in_contact = smp.in_contact
                    m.total = float(smp.total)
                    m.contact_taxels = int(smp.contact_taxels)
                    m.centroid_row = float(smp.centroid_row)
                    m.centroid_col = float(smp.centroid_col)
                else:
                    m.centroid_row = m.centroid_col = NAN
                m.gripper_command = float(c.command)
                m.gripper_position = float(s.position)
                m.contact_position = float(c.contact_command)
                m.target_total = float(c.target)
                self.state_pubs[s.name].publish(m)

    # ------------------------------------------------------------------ action
    def _execute(self, s: _Side, gh):
        req = gh.request
        waiter = _Waiter()
        now = time.monotonic()
        with self.lock:
            if req.command == Grasp.Goal.OPEN:
                prev = s.ctl.start_open(now, req.timeout_s)
                what = "open"
            else:
                prev = s.ctl.start_grasp(now, req.target_total, req.close_speed, req.timeout_s)
                what = f"grasp (target {s.ctl.target:.0f})"
            old, s.waiter = s.waiter, waiter
        if old is not None and prev is not None:
            old.finish(prev)                   # the preempted goal returns CANCELED
        self.get_logger().info(f"[{s.name}] goal: {what}")

        fb = Grasp.Feedback()
        while rclpy.ok() and not waiter.event.wait(0.1):
            if gh.is_cancel_requested:
                with self.lock:
                    res = s.ctl.cancel(time.monotonic(), "canceled by client")
                    if s.waiter is waiter:
                        s.waiter = None
                if res is not None:
                    waiter.finish(res)
                break
            with self.lock:
                c = s.ctl
                fb.state = c.state
                fb.gripper_command = float(c.command)
                fb.gripper_position = float(s.position)
                if s.sample is not None:
                    fb.total = float(s.sample.total)
                    fb.contact_taxels = int(s.sample.contact_taxels)
                    fb.in_contact = bool(s.sample.in_contact)
            gh.publish_feedback(fb)

        r = waiter.result
        out = Grasp.Result()
        if r is None:                          # shutting down
            out.success, out.outcome, out.message = False, CANCELED, "node shutting down"
            out.contact_position = NAN
            gh.abort()
            return out
        out.success = r.success
        out.outcome = r.outcome
        out.message = r.message
        out.gripper_position = float(r.gripper_position)
        out.contact_position = float(r.contact_position)
        out.total = float(r.total)
        out.contact_taxels = int(r.contact_taxels)
        if r.outcome == CANCELED and gh.is_cancel_requested:
            gh.canceled()
        elif r.success:
            gh.succeed()
        else:
            gh.abort()
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraspNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()