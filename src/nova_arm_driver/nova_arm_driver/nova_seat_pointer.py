#!/usr/bin/env python3
"""
nova_seat_pointer
=================

Find a free chair with the arm-mounted RealSense and point at it.

  1. SCAN    point_start -> point_left -> ... -> point_right, stopping at
             several angles and asking receptionist_cv for free seats.
  2. CENTER  Once a free seat is confirmed, turn joint1 (base yaw) in a
             closed loop until the seat is in the middle of the image.
             Because the camera rides on the arm, the arm is now facing it.
  3. POINT   Optionally move to a 'point_seat' pose (keeping the centred
             joint1), hold, publish the result, and go back to point_start.

Result is published as JSON on /seat_pointer/result (std_msgs/String).

Run WITHOUT the GUI streaming (close nova_gui or leave "Enable Motion" off):
only one program may publish /arm_command at a time.

Test the motion logic without the camera:
    ros2 run nova_arm_driver seat_pointer --ros-args -p fake:=true

Real run (needs the receptionist_cv virtualenv, see bottom of file).
"""

import json
import math
import os
import sys
import time

import numpy as np
import yaml
from scipy.interpolate import CubicSpline

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String


POSE_FILE = os.path.expanduser("~/nova_arm_ws/poses.yaml")
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
YAW = 0                 # index of joint1 (base rotation)
JOINT_LIMIT = 2.6       # ArmDriver clamps at +/-150 deg
DT = 0.02               # 50 Hz command stream
MAX_JOINT_VEL = 0.6     # rad/s, used to scale move durations
COLOR_HFOV_DEG = 69.0   # D435 colour camera horizontal field of view


# ---------------------------------------------------------------------------
# Seat detectors
# ---------------------------------------------------------------------------
# Both return a list of free seats in the CAMERA frame (metres):
#     x = right, y = down, z = forward   (camera_color_optical_frame)
#     [{"x": .., "y": .., "z": .., "conf": ..}, ...]


class RealSeatDetector:
    """
    Wraps receptionist_cv: camera.Camera + seat_finder.find_seats_cuboid,
    called the same way test/test_live_seats.py does.
    """

    def __init__(self, cv_root, camera_tilt_deg, logger):
        self.log = logger
        cv_root = os.path.expanduser(cv_root)
        for path in (cv_root, os.path.join(cv_root, "modules")):
            if path not in sys.path:
                sys.path.insert(0, path)

        import camera
        import seat_finder

        self._seat_finder = seat_finder

        # align_every=1: depth on every frame (the default 2 leaves every
        # other frame with depth=None). No webcam fallback: without depth
        # there are no 3D seat positions, so better to fail loudly.
        self._camera = camera.Camera(align_every=1)
        self.log.info(f"Camera open: {self._camera.name}")

        # The D435 has no IMU, so tell seat_finder which way is down.
        # Camera pitched DOWN by `tilt` -> down in the optical frame is
        # (0, cos(tilt), sin(tilt)). Same formula as test_live_seats.py.
        t = math.radians(camera_tilt_deg)
        self._assumed_gravity = [0.0, math.cos(t), math.sin(t)]

    def _grab(self):
        for _ in range(5):
            frame = self._camera.read(depth=True)
            if frame is not None and frame.get("depth") is not None:
                return frame
        if self._camera.error:
            raise RuntimeError(f"RealSense error: {self._camera.error}")
        return None

    def flush(self, n=3):
        """Throw away frames captured while the arm was still moving."""
        for _ in range(n):
            self._grab()

    def detect(self):
        frame = self._grab()
        if frame is None:
            self.log.warn("No depth frame from the RealSense.")
            return []

        gravity = frame.get("gravity")          # only set on cameras with an IMU
        if gravity is None:
            gravity = self._assumed_gravity

        res = self._seat_finder.find_seats_cuboid(
            frame["rgb"], frame["depth"], frame["intrinsics"], gravity=gravity
        )

        if res.get("status") == "unavailable":
            raise RuntimeError(
                f"seat_finder unavailable ({res.get('reason')}). "
                "Did you run download_models.py in receptionist_cv?"
            )

        out = []
        for s in res.get("seats", []):           # free seats, nearest first
            pos = s.get("position")
            if not pos:
                continue
            out.append({
                "x": float(pos["x"]), "y": float(pos["y"]), "z": float(pos["z"]),
                "conf": float(s.get("det_score") or 0.0),
                "type": s.get("type"),
            })
        return out

    def close(self):
        self._camera.close()


class FakeSeatDetector:
    """
    Pretends there is one free chair at a fixed direction in the room so the
    scan / centre / point logic can be tested on the real arm with no camera.
    """

    def __init__(self, node, chair_deg, distance=2.0):
        self.node = node
        self.chair = math.radians(chair_deg)   # + = left of joint1 = 0
        self.distance = distance

    def flush(self, n=5):
        pass

    def detect(self):
        heading = self.node.yaw_sign * float(self.node.q[YAW])   # + = left
        rel = self.chair - heading                                 # + = chair is left
        if abs(rel) > math.radians(COLOR_HFOV_DEG / 2 - 4):
            return []
        noise = np.random.normal(0, 0.01, 2)
        x = -self.distance * math.sin(rel) + noise[0]   # optical x = right
        z = self.distance * math.cos(rel) + noise[1]
        return [{"x": x, "y": 0.35, "z": z, "conf": 0.9}]

    def close(self):
        pass


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class SeatPointer(Node):

    def __init__(self):
        super().__init__("nova_seat_pointer")

        # dynamic_typing: accept 30 as well as 30.0 (values are cast below)
        any_type = ParameterDescriptor(dynamic_typing=True)

        def p(name, default):
            return self.declare_parameter(name, default, any_type).value

        self.fake = bool(p("fake", False))
        self.fake_chair_deg = float(p("fake_chair_deg", 20.0))
        self.cv_root = str(p("cv_root", "~/receptionist_cv"))
        self.camera_tilt_deg = float(p("camera_tilt_deg", 0.0))

        self.speed = int(p("speed", 25))                 # % for /arm_speed
        self.scan_steps = int(p("scan_steps", 6))        # stops from left to right
        self.min_move_time = float(p("min_move_time", 1.0))
        self.settle_time = float(p("settle_time", 0.6))  # wait before grabbing frames
        self.frames_per_look = int(p("frames_per_look", 3))
        self.frames_needed = int(p("frames_needed", 2))

        # +1: increasing joint1 turns the camera LEFT. Flip to -1 if
        # centring turns the wrong way (the node detects this and stops).
        self.yaw_sign = float(p("yaw_sign", 1.0))
        self.center_tol_deg = float(p("center_tolerance_deg", 2.0))
        self.center_gain = float(p("center_gain", 0.8))
        self.max_center_step = float(p("max_center_step", 0.35))   # rad
        self.max_center_iters = int(p("max_center_iters", 6))

        self.point_hold = float(p("point_hold_time", 3.0))
        self.return_to_start = bool(p("return_to_start", True))

        self.cmd_pub = self.create_publisher(JointState, "/arm_command", 10)
        self.speed_pub = self.create_publisher(Int32, "/arm_speed", 10)
        self.result_pub = self.create_publisher(String, "/seat_pointer/result", 10)

        self.poses = self._load_poses()
        self.q = None   # last commanded joint vector (unknown at start)

        if self.fake:
            self.detector = FakeSeatDetector(self, self.fake_chair_deg)
            self.get_logger().warn(
                f"FAKE mode: pretending a free chair is at {self.fake_chair_deg:+.0f} deg"
            )
        else:
            self.detector = RealSeatDetector(
                self.cv_root, self.camera_tilt_deg, self.get_logger()
            )
            self.get_logger().info("RealSense + seat_finder ready")

    # ---------------- poses ----------------

    def _load_poses(self):
        with open(POSE_FILE, "r") as f:
            poses = yaml.safe_load(f) or {}
        missing = [n for n in ("point_start", "point_left", "point_right") if n not in poses]
        if missing:
            raise RuntimeError(f"Missing poses in {POSE_FILE}: {', '.join(missing)}")
        return poses

    def pose(self, name):
        pose = self.poses[name]
        lookup = dict(zip(pose.get("names", JOINT_NAMES), pose["positions"]))
        return np.array([float(lookup[j]) for j in JOINT_NAMES])

    # ---------------- motion ----------------

    def _publish(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [float(v) for v in q]
        self.cmd_pub.publish(msg)

    def set_speed(self, percent):
        msg = Int32()
        msg.data = int(percent)
        self.speed_pub.publish(msg)
        time.sleep(0.1)

    def move_to(self, target, duration=None):
        target = np.clip(np.asarray(target, float), -JOINT_LIMIT, JOINT_LIMIT)

        if self.q is None:
            # Real position unknown: send the goal and let the servos get there
            # at the /arm_speed limit.
            self.get_logger().info("Moving to start pose (position unknown, going slowly)")
            t_end = time.monotonic() + 3.0
            while rclpy.ok() and time.monotonic() < t_end:
                self._publish(target)
                time.sleep(DT)
            self.q = target.copy()
            return

        if duration is None:
            duration = max(self.min_move_time, np.max(np.abs(target - self.q)) / MAX_JOINT_VEL)

        spline = CubicSpline([0.0, duration], np.vstack([self.q, target]),
                             axis=0, bc_type="clamped")
        t0 = time.monotonic()
        while rclpy.ok():
            t = time.monotonic() - t0
            if t >= duration:
                break
            self.q = spline(t)
            self._publish(self.q)
            time.sleep(DT)

        self.q = target.copy()
        self._publish(self.q)

    # ---------------- vision ----------------

    @staticmethod
    def _angle(seat):
        """Horizontal angle of the seat from the image centre, + = right."""
        return math.atan2(seat["x"], seat["z"])

    @staticmethod
    def _dist(seat):
        return math.hypot(seat["x"], seat["z"])

    def look(self):
        """
        Let the arm settle, grab several frames and return the nearest free seat
        that shows up in at least `frames_needed` of them (or None).
        """
        time.sleep(self.settle_time)
        self.detector.flush()

        frames = [self.detector.detect() for _ in range(self.frames_per_look)]

        confirmed = []
        for i, frame in enumerate(frames):
            for seat in frame:
                p = np.array([seat["x"], seat["y"], seat["z"]])
                hits = 1 + sum(
                    any(np.linalg.norm(p - np.array([o["x"], o["y"], o["z"]])) < 0.3 for o in other)
                    for j, other in enumerate(frames) if j != i
                )
                if hits >= self.frames_needed:
                    confirmed.append(seat)

        if not confirmed:
            return None
        return min(confirmed, key=self._dist)

    def look_for(self, previous):
        """Re-find the same seat after a small turn: similar distance, closest to centre."""
        time.sleep(self.settle_time)
        self.detector.flush()
        candidates = []
        for _ in range(self.frames_per_look):
            for s in self.detector.detect():
                if abs(self._dist(s) - self._dist(previous)) < 0.5:
                    candidates.append(s)
        if not candidates:
            return None
        return min(candidates, key=lambda s: abs(self._angle(s)))

    # ---------------- behaviour ----------------

    def scan(self):
        start = self.pose("point_start")
        left = self.pose("point_left")
        right = self.pose("point_right")

        n = max(2, self.scan_steps)
        stops = [("point_start", start)]
        for i in range(n):
            a = i / (n - 1)
            label = "point_left" if i == 0 else "point_right" if i == n - 1 else f"scan {i}/{n - 1}"
            stops.append((label, left + (right - left) * a))

        for label, q in stops:
            self.get_logger().info(f"Scanning at {label} (joint1 = {q[YAW]:+.2f} rad)")
            self.move_to(q)
            seat = self.look()
            if seat:
                self.get_logger().info(
                    f"Free seat: {self._dist(seat):.2f} m away, "
                    f"{math.degrees(self._angle(seat)):+.1f} deg from centre"
                )
                return seat
        return None

    def center(self, seat):
        left_yaw = self.pose("point_left")[YAW]
        right_yaw = self.pose("point_right")[YAW]
        lo, hi = min(left_yaw, right_yaw) - 0.3, max(left_yaw, right_yaw) + 0.3

        tol = math.radians(self.center_tol_deg)
        for it in range(1, self.max_center_iters + 1):
            err = self._angle(seat)        # + = seat is right of centre
            self.get_logger().info(f"Centering {it}: error {math.degrees(err):+.1f} deg")
            if abs(err) < tol:
                return seat

            # Seat to the right -> turn right -> joint1 moves by -yaw_sign.
            step = -self.yaw_sign * self.center_gain * err
            step = float(np.clip(step, -self.max_center_step, self.max_center_step))

            target = self.q.copy()
            target[YAW] = float(np.clip(target[YAW] + step, lo, hi))
            self.move_to(target)

            new = self.look_for(seat)
            if new is None:
                hint = ""
                if it == 1:
                    hint = (" If the arm turned AWAY from the chair, joint1 direction is "
                            "reversed: re-run with  -p yaw_sign:=%g" % (-self.yaw_sign))
                self.get_logger().warn("Lost the seat while centering." + hint)
                return None

            if abs(self._angle(new)) > abs(err) + math.radians(1.0):
                self.get_logger().error(
                    "Turning made the error bigger, joint1 direction is probably "
                    "reversed. Re-run with  -p yaw_sign:=%g" % (-self.yaw_sign)
                )
                return None
            seat = new

        if abs(self._angle(seat)) < 3 * tol:
            self.get_logger().warn("Close enough to centre, continuing.")
            return seat
        return None

    def point(self):
        if "point_seat" in self.poses:
            target = self.pose("point_seat")
            target[YAW] = self.q[YAW]      # keep the centred direction
            self.get_logger().info("Pointing (point_seat pose with centred joint1)")
            self.move_to(target)
        else:
            self.get_logger().info("Pointing (holding centred pose; save a 'point_seat' pose to customise)")
        time.sleep(self.point_hold)

    def publish_result(self, found, seat=None):
        data = {"found": found}
        if found and seat is not None:
            data.update({
                "joint1": round(float(self.q[YAW]), 4),
                "distance_m": round(self._dist(seat), 3),
                "seat_camera_xyz": [round(float(seat[k]), 3) for k in ("x", "y", "z")],
            })
        msg = String()
        msg.data = json.dumps(data)
        self.result_pub.publish(msg)
        self.get_logger().info(f"Result: {msg.data}")

    def run(self):
        self.set_speed(self.speed)

        seat = self.scan()
        if seat is None:
            self.get_logger().warn("No free seat found in the scan range.")
            self.publish_result(False)
        else:
            seat = self.center(seat)
            if seat is None:
                self.publish_result(False)
            else:
                self.publish_result(True, seat)
                self.point()

        if self.return_to_start:
            self.move_to(self.pose("point_start"))
        self.get_logger().info("Done.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SeatPointer()
        time.sleep(1.0)          # let publishers connect to arm_driver
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.detector.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


# ---------------------------------------------------------------------------
# Real run, with the receptionist_cv virtualenv (it has onnxruntime,
# pyrealsense2 etc. that system Python doesn't):
#
#   source /opt/ros/humble/setup.bash
#   source ~/nova_arm_ws/install/setup.bash
#   source ~/receptionist_cv/.venv/bin/activate
#   pip install pyyaml scipy        # if the venv doesn't have them
#   python -m nova_arm_driver.nova_seat_pointer --ros-args -p camera_tilt_deg:=0.0
#
# (`ros2 run` uses system Python and won't see the venv's packages.)
# ---------------------------------------------------------------------------
