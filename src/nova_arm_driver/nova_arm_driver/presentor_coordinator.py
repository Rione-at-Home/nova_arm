#!/usr/bin/env python3
"""
presenter_coordinator.py

Startup : moves the arm to the saved holding pose (default "tray_hold") and
          keeps re-sending it so it stays there.
Trigger : on /presenter_trigger (from qr_trigger.py) runs, in order:
            1. DRIVE  - Kobuki drives forward a short distance (open-loop, timed)
            2. SCAN   - head pans/tilts through a few waypoints (looks at crowd)
            3. SPEAK  - publishes Japanese text for the TTS node
Abort   : /presenter_abort (std_msgs/Empty) stops the base, centers the head
          and returns to IDLE.

Topics
  subscribes: /presenter_trigger  std_msgs/String
              /presenter_abort    std_msgs/Empty
  publishes : /arm_command        sensor_msgs/JointState  (arm hold pose)
              /arm_speed          std_msgs/Int32
              <cmd_vel_topic>     geometry_msgs/Twist     (default /commands/velocity)
              /head/pan_target    std_msgs/Float32        (degrees)
              /head/tilt_target   std_msgs/Float32        (degrees)
              /tts/say            std_msgs/String         (text to speak, Japanese)
              /presenter_state    std_msgs/String         (IDLE / DRIVE / SCAN / SPEAK)

Test without the QR code:
  ros2 topic pub --once /presenter_trigger std_msgs/msg/String "{data: start}"
"""

import os
from collections import deque

import yaml

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float32, Int32, String


class PresenterCoordinator(Node):

    def __init__(self):
        super().__init__("presenter_coordinator")

        # ---- parameters -------------------------------------------------
        self.declare_parameter("pose_file", os.path.expanduser("~/nova_arm_ws/poses.yaml"))
        self.declare_parameter("hold_pose", "tray_hold")
        self.declare_parameter("arm_speed", 20)

        self.declare_parameter("cmd_vel_topic", "/commands/velocity")
        self.declare_parameter("forward_speed", 0.10)      # m/s
        self.declare_parameter("forward_distance", 0.40)   # m (open loop)

        # Head waypoints in degrees (driver limits: pan +-60, tilt +-45).
        # Head node needs ~3-4 s to finish each move, so dwell defaults to 4 s.
        self.declare_parameter("scan_pan", [-35.0, 35.0, -20.0, 0.0])
        self.declare_parameter("scan_tilt", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("scan_dwell", 4.0)

        self.declare_parameter("tts_topic", "/tts/say")
        self.declare_parameter("speech_text", "どうぞ、名刺をお取りください。")
        self.declare_parameter("speech_duration", 5.0)     # s to wait for speech

        p = self.get_parameter
        self.arm_speed = int(p("arm_speed").value)
        self.forward_speed = max(abs(p("forward_speed").value), 0.01)
        self.forward_distance = max(p("forward_distance").value, 0.0)
        self.scan_pan = list(p("scan_pan").value)
        self.scan_tilt = list(p("scan_tilt").value)
        self.scan_dwell = p("scan_dwell").value
        self.speech_text = p("speech_text").value
        self.speech_duration = p("speech_duration").value

        if len(self.scan_pan) != len(self.scan_tilt):
            raise ValueError("scan_pan and scan_tilt must have the same length")

        # ---- load the hold pose ----------------------------------------
        with open(p("pose_file").value, "r") as f:
            poses = yaml.safe_load(f) or {}
        name = p("hold_pose").value
        if name not in poses:
            raise RuntimeError(f"Pose '{name}' not in poses.yaml: {list(poses.keys())}")
        self.hold_names = list(poses[name]["names"])
        self.hold_positions = [float(x) for x in poses[name]["positions"]]

        # ---- pub/sub ----------------------------------------------------
        self.arm_pub = self.create_publisher(JointState, "/arm_command", 10)
        self.arm_speed_pub = self.create_publisher(Int32, "/arm_speed", 10)
        self.vel_pub = self.create_publisher(Twist, p("cmd_vel_topic").value, 10)
        self.pan_pub = self.create_publisher(Float32, "/head/pan_target", 10)
        self.tilt_pub = self.create_publisher(Float32, "/head/tilt_target", 10)
        self.tts_pub = self.create_publisher(String, p("tts_topic").value, 10)
        self.state_pub = self.create_publisher(String, "/presenter_state", 10)

        self.create_subscription(String, "/presenter_trigger", self.trigger_cb, 10)
        self.create_subscription(Empty, "/presenter_abort", self.abort_cb, 10)

        # ---- state machine ---------------------------------------------
        self.state = "IDLE"
        self.queue = deque()
        self.current = None
        self.step_end = 0.0

        self.hold_count = 0
        self.create_timer(1.0, self.hold_arm)      # keep arm at hold pose
        self.create_timer(0.05, self.tick)         # 20 Hz sequence runner

        self.get_logger().info(
            f"Coordinator ready. Holding '{name}'. Waiting for /presenter_trigger."
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------------------------------------------------------------- arm

    def hold_arm(self):
        if self.hold_count < 3:
            msg = Int32()
            msg.data = self.arm_speed
            self.arm_speed_pub.publish(msg)
            self.hold_count += 1

        msg = JointState()
        msg.name = self.hold_names
        msg.position = self.hold_positions
        self.arm_pub.publish(msg)

    # ---------------------------------------------------------------- actions

    def drive_tick(self):
        t = Twist()
        t.linear.x = self.forward_speed
        self.vel_pub.publish(t)

    def stop_base(self):
        self.vel_pub.publish(Twist())

    def look(self, pan, tilt):
        a = Float32()
        a.data = float(pan)
        b = Float32()
        b.data = float(tilt)
        self.pan_pub.publish(a)
        self.tilt_pub.publish(b)

    def speak(self):
        msg = String()
        msg.data = self.speech_text
        self.tts_pub.publish(msg)
        self.get_logger().info(f"TTS -> {self.speech_text}")

    # ---------------------------------------------------------------- sequence

    def set_state(self, s):
        if s != self.state:
            self.state = s
            msg = String()
            msg.data = s
            self.state_pub.publish(msg)

    def trigger_cb(self, msg):
        if self.state != "IDLE":
            self.get_logger().info("Trigger ignored: sequence already running.")
            return

        steps = [dict(
            name="DRIVE",
            dur=self.forward_distance / self.forward_speed,
            enter=None, tick=self.drive_tick, exit=self.stop_base,
        )]
        for pan, tilt in zip(self.scan_pan, self.scan_tilt):
            steps.append(dict(
                name="SCAN", dur=self.scan_dwell,
                enter=lambda a=pan, b=tilt: self.look(a, b),
                tick=None, exit=None,
            ))
        steps.append(dict(
            name="SPEAK", dur=self.speech_duration,
            enter=self.speak, tick=None, exit=None,
        ))

        self.queue = deque(steps)
        self.current = None
        self.get_logger().info("Trigger received. Starting sequence.")

    def abort_cb(self, _msg):
        self.get_logger().warn("ABORT: stopping base, centering head.")
        self.stop_base()
        self.look(0.0, 0.0)
        self.queue.clear()
        self.current = None
        self.set_state("IDLE")

    def tick(self):
        if self.current is None:
            if not self.queue:
                self.set_state("IDLE")
                return
            self.current = self.queue.popleft()
            self.step_end = self.now() + self.current["dur"]
            self.set_state(self.current["name"])
            self.get_logger().info(
                f"Step {self.current['name']} ({self.current['dur']:.1f}s)"
            )
            if self.current["enter"]:
                self.current["enter"]()
            return

        if self.current["tick"]:
            self.current["tick"]()

        if self.now() >= self.step_end:
            if self.current["exit"]:
                self.current["exit"]()
            self.current = None
            if not self.queue:
                self.get_logger().info("Sequence complete.")

    # ---------------------------------------------------------------- shutdown

    def destroy_node(self):
        try:
            self.stop_base()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PresenterCoordinator()
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