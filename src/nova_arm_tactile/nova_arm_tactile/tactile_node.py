#!/usr/bin/env python3
"""
tactile_node: reads the FlexiTac pad on the fixed finger of each nova_arm gripper and publishes it.

Follows the nova_arm_driver conventions:
  * one independent serial link per arm, persistent udev names (/dev/tactile_right, /dev/tactile_left)
  * a missing or unplugged pad never stops the node; it is retried and picked up when it returns
  * read-only health on /arm_tactile_status (diagnostic_msgs/DiagnosticArray), one entry per pad

Topics (per side, <side> = right | left):
  /arm/<side>/tactile/frame   nova_arm_tactile_msgs/TactileFrame   raw + delta + features + link health
  /arm/<side>/tactile/image   sensor_msgs/Image (mono8)            delta clipped to 0..255
  /arm_tactile_status         diagnostic_msgs/DiagnosticArray      status_rate_hz
Services:
  /arm/<side>/tactile/rezero  std_srvs/Trigger   new baseline for that pad (gripper must be open)
  /arm/tactile/rezero_all     std_srvs/Trigger

Demo mode (demo:=true) needs no pads. With demo_follow_gripper:=true (default) the simulated
pressure comes from the gripper goal on /arm_command: an object sits at demo_contact_rad and
pressure grows as the goal is driven past it (on the real gripper, force comes from that same
goal-vs-object error). Pair it with fake_gripper to test grasping on a laptop.
"""
from __future__ import annotations

import array
import math
import time
from typing import Dict

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nova_arm_tactile_msgs.msg import TactileFrame
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

from .core import (COLS, DEFAULT_BAUD, ROWS, DemoStream, FingerChannel, ProcessedFrame,
                   parse_mask, parse_region)

DEFAULT_PORTS = {"right": "/dev/tactile_right", "left": "/dev/tactile_left"}


def _stamp(ns: int) -> TimeMsg:
    msg = TimeMsg()
    msg.sec = int(ns // 1_000_000_000)
    msg.nanosec = int(ns % 1_000_000_000)
    return msg


class TactileNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_tactile")
        self.declare_parameter("sides", ["right", "left"])
        self.declare_parameter("baud", DEFAULT_BAUD)
        self.declare_parameter("baseline_frames", 30)
        self.declare_parameter("contact_threshold", 15.0)
        self.declare_parameter("contact_min_taxels", 2)
        self.declare_parameter("publish_image", True)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("reconnect_period_s", 3.0)
        self.declare_parameter("stale_s", 0.5)          # no frame for this long -> ERROR
        self.declare_parameter("min_rate_hz", 50.0)      # below this -> WARN
        self.declare_parameter("demo", False)
        self.declare_parameter("demo_glitch", False)
        self.declare_parameter("demo_follow_gripper", True)
        self.declare_parameter("demo_contact_rad", 0.45)     # simulated object face, gripper goal in rad
        self.declare_parameter("demo_gain", 600.0)           # peak counts per rad the goal is driven past it
        self.declare_parameter("demo_closing_sign", 1.0)     # +1 if closing increases the angle

        self.sides = [str(s) for s in self.get_parameter("sides").value]
        baud = int(self.get_parameter("baud").value)
        demo = bool(self.get_parameter("demo").value)
        glitch = bool(self.get_parameter("demo_glitch").value)
        self.publish_image = bool(self.get_parameter("publish_image").value)
        self.stale_s = float(self.get_parameter("stale_s").value)
        self.min_rate = float(self.get_parameter("min_rate_hz").value)
        reconnect = max(0.5, float(self.get_parameter("reconnect_period_s").value))

        self.channels: Dict[str, FingerChannel] = {}
        self.pub_frame, self.pub_image, self.frame_ids = {}, {}, {}
        self._last_warn: Dict[str, float] = {}
        self._gripper_pos: Dict[str, float] = {}
        self._demo_cfg = (float(self.get_parameter("demo_contact_rad").value),
                          float(self.get_parameter("demo_gain").value),
                          float(self.get_parameter("demo_closing_sign").value))

        for i, side in enumerate(self.sides):
            self.declare_parameter(f"{side}.port", DEFAULT_PORTS.get(side, f"/dev/tactile_{side}"))
            self.declare_parameter(f"{side}.mask", "")
            self.declare_parameter(f"{side}.active_region", "")
            self.declare_parameter(f"{side}.frame_id", f"{side}_gripper_tactile")
            port = str(self.get_parameter(f"{side}.port").value)
            mask = (parse_mask(str(self.get_parameter(f"{side}.mask").value))
                    | parse_region(str(self.get_parameter(f"{side}.active_region").value)))
            self.frame_ids[side] = str(self.get_parameter(f"{side}.frame_id").value)

            opener = None
            if demo:
                press = None
                if bool(self.get_parameter("demo_follow_gripper").value):
                    press = (lambda s=side: self._demo_press(s))
                stream = DemoStream(glitch=glitch, seed=i, phase=1.7 * i, press=press)
                opener = (lambda s=stream: s)
            ch = FingerChannel(
                side, port, baud,
                baseline_frames=int(self.get_parameter("baseline_frames").value),
                threshold=float(self.get_parameter("contact_threshold").value),
                min_taxels=int(self.get_parameter("contact_min_taxels").value),
                mask=mask, opener=opener, log=self.get_logger().info)
            ch.max_backoff = reconnect
            self.channels[side] = ch

            base = f"/arm/{side}/tactile"
            self.pub_frame[side] = self.create_publisher(TactileFrame, f"{base}/frame",
                                                         qos_profile_sensor_data)
            if self.publish_image:
                self.pub_image[side] = self.create_publisher(Image, f"{base}/image",
                                                             qos_profile_sensor_data)
            self.create_service(Trigger, f"{base}/rezero",
                                lambda req, res, s=side: self._rezero([s], res))
            self._last_warn[side] = 0.0

        self.create_service(Trigger, "/arm/tactile/rezero_all",
                            lambda req, res: self._rezero(list(self.channels), res))

        if demo:
            self.create_subscription(JointState, "/arm_command", self._on_arm_command, 10)

        self.status_pub = self.create_publisher(DiagnosticArray, "/arm_tactile_status", 10)
        rate = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.2), self._publish_status)

        for ch in self.channels.values():
            ch.start()
        self.create_timer(0.002, self._publish)        # drain the reader threads
        self.create_timer(2.0, self._watchdog)
        mode = "DEMO (simulated pads)" if demo else "serial"
        self.get_logger().info(
            f"arm_tactile up ({mode}): " +
            ", ".join(f"{s}={c.port}" for s, c in self.channels.items()) +
            f" @ {baud}. Keep the grippers open while the baselines are collected.")

    # -- demo
    def _on_arm_command(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name.endswith("_gripper"):
                self._gripper_pos[name[:-len("_gripper")]] = float(pos)

    def _demo_press(self, side: str):
        # runs on the reader thread: only touches the cached floats and a dict read
        pos = self._gripper_pos.get(side)
        if pos is None:
            return 0.0, 8.0, 16.0
        contact, gain, sign = self._demo_cfg
        amp = min(220.0, max(0.0, sign * (pos - contact)) * gain)
        return amp, 8.0, 16.0

    # -- services
    def _rezero(self, sides, res):
        res.success = True
        res.message = " | ".join(self.channels[s].rezero() for s in sides)
        self.get_logger().info(res.message)
        return res

    # -- publishing
    def _publish(self) -> None:
        for side, ch in self.channels.items():
            for pf in ch.drain():
                self.pub_frame[side].publish(self._frame_msg(side, pf))
                if self.publish_image:
                    self.pub_image[side].publish(self._image_msg(side, pf))

    def _frame_msg(self, side: str, pf: ProcessedFrame) -> TactileFrame:
        m = TactileFrame()
        m.header.stamp = _stamp(pf.stamp_ns)
        m.header.frame_id = self.frame_ids[side]
        m.finger = side
        m.seq = pf.seq
        m.rows = ROWS
        m.cols = COLS
        m.raw = pf.raw.tobytes()
        m.calibrated = pf.calibrated
        m.delta = array.array("f", pf.feat.delta.astype(np.float32).ravel().tolist())
        m.peak = pf.feat.peak
        m.total = pf.feat.total
        m.contact_taxels = pf.feat.contact_taxels
        m.in_contact = pf.feat.in_contact
        m.centroid_row = pf.feat.centroid_row
        m.centroid_col = pf.feat.centroid_col
        m.bad_frames = pf.bad_frames
        m.dropped_bytes = pf.dropped_bytes
        m.rate_hz = pf.rate_hz
        return m

    def _image_msg(self, side: str, pf: ProcessedFrame) -> Image:
        img = Image()
        img.header.stamp = _stamp(pf.stamp_ns)
        img.header.frame_id = self.frame_ids[side]
        img.height, img.width = ROWS, COLS
        img.encoding = "mono8"
        img.is_bigendian = 0
        img.step = COLS
        img.data = np.clip(pf.feat.delta, 0, 255).astype(np.uint8).tobytes()
        return img

    # -- health
    def _publish_status(self) -> None:
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        now = time.time()
        for side, ch in self.channels.items():
            info = ch.info()
            age = (now - info["last_frame_time"]) if info["last_frame_time"] else math.inf
            st = DiagnosticStatus()
            st.name = f"arm_{side}_tactile"
            st.hardware_id = side
            if not info["connected"]:
                st.level, st.message = DiagnosticStatus.ERROR, "port not available"
            elif age > self.stale_s:
                st.level = DiagnosticStatus.ERROR
                st.message = ("port open, no frames yet" if math.isinf(age)
                              else f"no frames for {age:.1f} s")
            elif info["calibrating"]:
                st.level, st.message = DiagnosticStatus.WARN, "collecting baseline"
            elif info["rate_hz"] < self.min_rate:
                st.level, st.message = DiagnosticStatus.WARN, f"low rate {info['rate_hz']:.0f} Hz"
            else:
                st.level, st.message = DiagnosticStatus.OK, "ok"
            st.values = [
                KeyValue(key="kind", value="tactile"),
                KeyValue(key="arm", value=side),
                KeyValue(key="connected", value="1" if info["connected"] else "0"),
                KeyValue(key="port", value=str(info["port"])),
                KeyValue(key="connect_attempts", value=str(info["connect_attempts"])),
                KeyValue(key="last_error", value=str(info["last_error"])),
                KeyValue(key="frames", value=str(info["frames"])),
                KeyValue(key="frame_age_s", value="inf" if math.isinf(age) else f"{age:.3f}"),
                KeyValue(key="rate_hz", value=f"{info['rate_hz']:.1f}"),
                KeyValue(key="calibrated", value="1" if info["calibrated"] else "0"),
                KeyValue(key="bad_frames", value=str(info["bad_frames"])),
                KeyValue(key="dropped_bytes", value=str(info["dropped_bytes"])),
                KeyValue(key="noisy_taxels", value=str(info["noisy_taxels"])),
                KeyValue(key="baseline_note", value=str(info["baseline_note"])),
            ]
            msg.status.append(st)
        self.status_pub.publish(msg)

    def _watchdog(self) -> None:
        now = time.time()
        for side, ch in self.channels.items():
            if now - self._last_warn[side] < 10.0:
                continue
            if not ch.connected:
                self._last_warn[side] = now
                self.get_logger().warning(
                    f"[{side}] tactile pad not detected on {ch.port}: "
                    f"{ch.last_error or 'opening...'}; running without it")
            elif ch.last_frame_time and now - ch.last_frame_time > 1.0:
                self._last_warn[side] = now
                self.get_logger().warning(
                    f"[{side}] port open but no frames for {now - ch.last_frame_time:.1f} s "
                    "(check power, firmware, baud)")

    def destroy_node(self):
        for ch in self.channels.values():
            ch.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TactileNode()
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