#!/usr/bin/env python3
"""
arm_skills: the arm side of the receptionist (server of receptionist_interfaces/PointAt).

    /arm/point_at   receptionist_interfaces/action/PointAt   point the Nova arm at a 3D target
    /arm/tuck       std_srvs/srv/Trigger                     arm to the tuck (driving) pose

The camera is on the HEAD, so a target from the CV node (PointStamped in camera_color_optical_frame,
stamped with the capture time) is turned into an arm direction the same way gaze_controller does it:

  1. head angles AT THE IMAGE TIME  (/joint_states from sudohead, else the commanded /head/*_target)
  2. r_anchor_from_camera(pan, tilt) -> the direction with the head at pan = tilt = 0
     (gaze_controller.head_state, same conventions: pan + = left, tilt + = down)
  3. + the measured offsets in robot_geometry.yaml (head pivot, camera on the head, arm shoulder)
  4. base turned since the image? (/odom yaw, like gaze base_compensation) -> rotate the point back
  5. arm yaw = direction from the shoulder -> joint1;  the other joints come from the 'point_seat'
     pose (optionally a pitch joint tilts by the target's elevation)

The arm stays pointing until a Tuck call or a new PointAt goal (a new goal pre-empts the old one).
This node is the only publisher of /arm_command while it runs: close nova_gui / seat_pointer first.

Arm feedback: ArmDriver does not publish the arm's joints on /joint_states, but its /arm_servo_status
(DiagnosticArray, 2 Hz: per servo comm_ok, goal, position in raw ticks) gives them. They are used for the
start of each move (smooth spline from where the arm really is) and for "arrived" (every arrive_joints
within arrive_tol_rad, or stopped close to it for still_s: a joint that sags under load). A servo with
comm_ok=0 fails the motion; /arm_motion_allowed False (arm health node recovering) refuses it.

Result messages that the manager acts on start with a keyword: "out_of_range: ..." (the target is beyond
joint1's range: turn the base), "servo_error: ...", "not_allowed: ...".

Run (both workspaces sourced, receptionist_control first):
    ros2 run nova_arm_driver arm_skills --ros-args --params-file ~/nova_arm_ws/robot_geometry.yaml
"""

import math
import os
import threading
import time

import numpy as np
import yaml
from scipy.interpolate import CubicSpline

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Int32
from std_srvs.srv import Trigger

from receptionist_interfaces.action import PointAt

# Same helpers and conventions as the gaze controller (one definition of the head geometry).
from gaze_controller.head_state import HeadBuffer, joint_state_to_degrees, r_anchor_from_camera
from gaze_controller.tracking import YawBuffer

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
ARM_IDS = [1, 2, 3, 4, 5, 6]          # Dynamixel IDs of ARM_JOINTS (ArmDriver.JOINT_TO_ID)
YAW = 0
JOINT_LIMIT = 2.6          # ArmDriver clamps at +/-150 deg
DT = 0.02                  # 50 Hz command stream


# ---------------------------------------------------------------------------
# Geometry (no ROS: testable anywhere)
# ---------------------------------------------------------------------------
# base_link: x forward, y left, z up (metres), origin = the base's rotation centre on the floor.

def _rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry_down(deg):
    """Rotation that pitches the forward axis DOWN by deg (tilt + = down)."""
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def camera_point_to_base(p_cam, pan_deg, tilt_deg, geo):
    """Point in the camera optical frame (seen at head angles pan, tilt) -> base_link."""
    pan = pan_deg - geo["head_pan_offset_deg"]
    tilt = tilt_deg - geo["head_tilt_offset_deg"]
    # direction in the optical frame of the head at pan = tilt = 0 (gaze_controller convention)
    xa, ya, za = r_anchor_from_camera(pan, tilt) @ np.asarray(p_cam, float)
    v_body = np.array([za, -xa, -ya])                     # optical (right, down, fwd) -> (fwd, left, up)
    cam_pos = np.asarray(geo["head_pivot_xyz"], float) + \
        _rz(pan) @ _ry_down(tilt) @ np.asarray(geo["camera_offset_xyz"], float)
    return cam_pos + v_body


def base_point_to_arm(p_base, geo, base_turned_deg=0.0):
    """(yaw_rad, elevation_rad, distance_m) of a base_link point as seen from the arm shoulder.
    base_turned_deg: how far the base has turned LEFT since the point was measured."""
    p = _rz(-base_turned_deg) @ np.asarray(p_base, float)      # base turned left -> target moved right
    rel = p - np.asarray(geo["arm_shoulder_xyz"], float)
    horiz = math.hypot(rel[0], rel[1])
    return math.atan2(rel[1], rel[0]), math.atan2(rel[2], horiz), float(np.linalg.norm(rel))


def dxl_to_rad(ticks):
    """Raw AX-12A position (0..1023 over 300 deg, 512 = centre) -> radians. Inverse of ArmDriver.rad_to_dxl."""
    return math.radians(float(ticks) / 1023.0 * 300.0 - 150.0)


def parse_servo_status(msg, ids=ARM_IDS):
    """/arm_servo_status DiagnosticArray -> (positions {id: rad}, dead [ids with comm_ok=0])."""
    positions, dead = {}, []
    for st in msg.status:
        try:
            sid = int(st.hardware_id or str(st.name).rsplit("_", 1)[-1])
        except ValueError:
            continue
        if sid not in ids:
            continue
        kv = {k.key: k.value for k in st.values}
        if kv.get("comm_ok") != "1":
            dead.append(sid)
            continue
        try:
            positions[sid] = dxl_to_rad(int(kv["position"]))
        except (KeyError, ValueError):
            pass
    return positions, dead


def arrival(q_meas, target, joints_idx, tol, close):
    """'there' (every checked joint within tol), 'close' (within close) or 'far'; plus the max error."""
    err = max(abs(float(q_meas[i]) - float(target[i])) for i in joints_idx)
    return ("there" if err < tol else "close" if err < close else "far"), err


def arm_joint_goal(point_pose, yaw, elevation, geo):
    """Joint vector for pointing: point_pose with joint1 (and optionally a pitch joint) replaced.
    Returns (q, message, ok)."""
    q = np.array(point_pose, float)
    j1 = geo["joint1_forward"] + geo["joint1_sign"] * yaw
    lo, hi = geo["joint1_min"], geo["joint1_max"]
    margin = math.radians(geo["reach_margin_deg"])
    if j1 < lo - margin or j1 > hi + margin:
        return None, (f"out_of_range: target is {math.degrees(yaw):+.0f} deg from forward, outside the arm's range "
                      f"(joint1 would be {j1:+.2f} rad, limits [{lo:+.2f}, {hi:+.2f}]): turn the base"), False
    msg = ""
    if j1 < lo or j1 > hi:
        msg = f"joint1 clamped to its limit ({j1:+.2f} -> {min(max(j1, lo), hi):+.2f} rad)"
    q[YAW] = min(max(j1, lo), hi)
    pj = geo.get("pitch_index")
    if pj is not None:
        dp = geo["pitch_sign"] * elevation
        dp = max(-geo["pitch_max_rad"], min(geo["pitch_max_rad"], dp))
        q[pj] = q[pj] + dp
    return np.clip(q, -JOINT_LIMIT, JOINT_LIMIT), msg, True


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class ArmSkills(Node):

    def __init__(self):
        super().__init__("arm_skills")
        any_type = ParameterDescriptor(dynamic_typing=True)   # 30 or 30.0, [0, 0, 1] or [0.0, 0.0, 1.0]

        def p(name, default):
            self.declare_parameter(name, default, any_type)

        # topics / names
        p("point_action", "/arm/point_at")
        p("tuck_service", "/arm/tuck")
        p("arm_command_topic", "/arm_command")
        p("arm_speed_topic", "/arm_speed")
        p("joint_state_topic", "/joint_states")
        p("pan_joint", "head_pan_joint")
        p("tilt_joint", "head_tilt_joint")
        p("pan_target_topic", "/head/pan_target")
        p("tilt_target_topic", "/head/tilt_target")
        p("odom_topic", "/odom")
        p("camera_frame", "camera_color_optical_frame")
        p("base_frame", "base_link")
        # geometry (robot_geometry.yaml)
        p("head_pivot_xyz", [0.0, 0.0, 1.10])
        p("camera_offset_xyz", [0.0, 0.0, 0.0])
        p("head_pan_offset_deg", 0.0)
        p("head_tilt_offset_deg", 0.0)
        p("arm_shoulder_xyz", [0.10, -0.20, 0.80])
        p("arm_side", "right")
        p("joint1_forward", 0.0)
        p("joint1_sign", 1.0)
        p("joint1_min", -2.5)
        p("joint1_max", 2.5)
        p("reach_margin_deg", 10.0)
        p("pitch_joint", "")
        p("pitch_sign", 1.0)
        p("pitch_max_rad", 0.6)
        # poses
        p("poses_file", "~/nova_arm_ws/poses.yaml")
        p("point_pose", "point_seat")
        p("tuck_pose", "home")
        p("zero_before_point", True)           # every PointAt: all joints to 0 first, then point
        # motion
        p("speed_percent", 30)
        p("max_joint_vel", 0.8)
        p("min_move_time", 0.8)
        p("unknown_start_time", 3.0)
        p("arrive_tol_rad", 0.05)
        p("arrive_timeout_s", 3.0)
        p("settle_s", 0.5)
        # feedback
        p("history_s", 120.0)
        p("head_feedback", "auto")             # auto | measured | commanded
        p("measured_fresh_s", 0.5)
        p("base_compensation", True)
        p("odom_fresh_s", 0.5)
        p("arm_feedback_fresh_s", 0.5)
        p("servo_status_topic", "/arm_servo_status")   # ArmDriver's 2 Hz per-servo status (arm feedback)
        p("motion_allowed_topic", "/arm_motion_allowed")
        p("status_fresh_s", 1.5)               # servo status older than this: no arm feedback
        p("arrive_joints", ["joint1", "joint2", "joint3", "joint4", "joint5"])   # gripper not checked
        p("still_s", 1.2)                      # stopped this long within close_rad: accept (sagging joint)
        p("close_rad", 0.12)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self.camera_frame, self.base_frame = str(g("camera_frame")), str(g("base_frame"))
        self.pan_joint, self.tilt_joint = str(g("pan_joint")), str(g("tilt_joint"))
        self.arm_side = str(g("arm_side"))
        self.head_feedback = str(g("head_feedback"))
        if self.head_feedback not in ("auto", "measured", "commanded"):
            raise ValueError("head_feedback must be auto, measured or commanded")
        self.measured_fresh = float(g("measured_fresh_s"))
        self.base_comp = bool(g("base_compensation"))
        self.odom_fresh = float(g("odom_fresh_s"))
        self.arm_fresh = float(g("arm_feedback_fresh_s"))
        self.status_fresh = float(g("status_fresh_s"))
        self.arrive_idx = [ARM_JOINTS.index(j) for j in g("arrive_joints")]
        self.still = float(g("still_s"))
        self.close = float(g("close_rad"))

        self.geo = {k: [float(v) for v in g(k)] for k in ("head_pivot_xyz", "camera_offset_xyz", "arm_shoulder_xyz")}
        for k in ("head_pan_offset_deg", "head_tilt_offset_deg", "joint1_forward", "joint1_sign", "joint1_min",
                  "joint1_max", "reach_margin_deg", "pitch_sign", "pitch_max_rad"):
            self.geo[k] = float(g(k))
        pitch = str(g("pitch_joint"))
        if pitch and pitch not in ARM_JOINTS:
            raise ValueError(f"pitch_joint must be one of {ARM_JOINTS} or empty")
        self.geo["pitch_index"] = ARM_JOINTS.index(pitch) if pitch else None

        self.poses_file = os.path.expanduser(str(g("poses_file")))
        self.point_pose_name, self.tuck_pose_name = str(g("point_pose")), str(g("tuck_pose"))
        self.zero_first = bool(g("zero_before_point"))
        self.speed = int(g("speed_percent"))
        self.max_vel = float(g("max_joint_vel"))
        self.min_move = float(g("min_move_time"))
        self.unknown_start = float(g("unknown_start_time"))
        self.arrive_tol = float(g("arrive_tol_rad"))
        self.arrive_timeout = float(g("arrive_timeout_s"))
        self.settle = float(g("settle_s"))

        history = float(g("history_s"))
        self.head_measured = HeadBuffer(keep_s=history)
        self.head_commanded = HeadBuffer(keep_s=history)
        self._cmd_pan = self._cmd_tilt = None
        self.yaw = YawBuffer(keep_s=history)
        self.odom_rx = None
        self.arm_q = None            # measured arm joints (if the driver publishes /joint_states)
        self.arm_rx = None
        self.status_q = None         # measured arm joints from /arm_servo_status
        self.status_rx = None
        self.dead = []               # servo IDs reporting comm_ok=0
        self.status_any_rx = None    # receive time of the last /arm_servo_status (even with dead servos)
        self.motion_allowed = True
        self.measured_rx = None      # receive time of the last head /joint_states
        self.q = None                # last commanded arm joints
        self._warned = set()

        self._motion_lock = threading.Lock()   # one motion at a time
        self._owner_lock = threading.Lock()
        self._owner = None                     # goal handle (or "tuck") that may move the arm

        cb = ReentrantCallbackGroup()
        self.cmd_pub = self.create_publisher(JointState, g("arm_command_topic"), 10)
        self.speed_pub = self.create_publisher(Int32, g("arm_speed_topic"), 10)
        self.create_subscription(JointState, g("joint_state_topic"), self._on_joints, 50, callback_group=cb)
        self.create_subscription(Float32, g("pan_target_topic"), self._on_pan_target, 20, callback_group=cb)
        self.create_subscription(Float32, g("tilt_target_topic"), self._on_tilt_target, 20, callback_group=cb)
        if self.base_comp:
            from nav_msgs.msg import Odometry
            self.create_subscription(Odometry, g("odom_topic"), self._on_odom, 20, callback_group=cb)
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.create_subscription(DiagnosticArray, g("servo_status_topic"), self._on_status, 10, callback_group=cb)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, g("motion_allowed_topic"), self._on_allowed, latched, callback_group=cb)

        self._server = ActionServer(
            self, PointAt, g("point_action"), execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            handle_accepted_callback=self._on_accepted, callback_group=cb)
        self.create_service(Trigger, g("tuck_service"), self._on_tuck, callback_group=cb)

        self._load_poses()           # fail early if the poses are missing
        self.get_logger().info(
            f"arm_skills ready: {g('point_action')} (PointAt), {g('tuck_service')} (Trigger); "
            f"point pose '{self.point_pose_name}', tuck pose '{self.tuck_pose_name}'")

    # ------------------------------------------------------------------ poses
    def _load_poses(self):
        with open(self.poses_file) as f:
            poses = yaml.safe_load(f) or {}
        missing = [n for n in (self.point_pose_name, self.tuck_pose_name) if n not in poses]
        if missing:
            raise RuntimeError(f"Missing poses in {self.poses_file}: {', '.join(missing)} "
                               "(save them with nova_gui)")
        return poses

    def _pose(self, name):
        pose = self._load_poses()[name]              # re-read: poses can be re-taught while running
        lookup = dict(zip(pose.get("names", ARM_JOINTS), pose["positions"]))
        return np.array([float(lookup[j]) for j in ARM_JOINTS])

    # ------------------------------------------------------------------ inputs
    def _on_joints(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = t if t > 0 else time.time()
        deg = joint_state_to_degrees(msg.name, msg.position, self.pan_joint, self.tilt_joint)
        if deg is not None:
            self.head_measured.add(t, *deg)
            self.measured_rx = time.monotonic()
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            self.arm_q = np.array([float(msg.position[names.index(j)]) for j in ARM_JOINTS])
            self.arm_rx = time.monotonic()

    def _on_status(self, msg):
        positions, dead = parse_servo_status(msg)
        self.dead = dead
        self.status_any_rx = time.monotonic()
        if all(sid in positions for sid in ARM_IDS):
            self.status_q = np.array([positions[sid] for sid in ARM_IDS])
            self.status_rx = time.monotonic()

    def _on_allowed(self, msg):
        self.motion_allowed = bool(msg.data)

    def _on_pan_target(self, msg):
        self._cmd_pan = float(msg.data)
        self._add_commanded()

    def _on_tilt_target(self, msg):
        self._cmd_tilt = float(msg.data)
        self._add_commanded()

    def _add_commanded(self):
        if self._cmd_pan is not None and self._cmd_tilt is not None:
            self.head_commanded.add(time.time(), self._cmd_pan, self._cmd_tilt)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.yaw.add(t if t > 0 else time.time(), yaw)
        self.odom_rx = time.monotonic()

    # ------------------------------------------------------------------ head / base at a given time
    def _head_at(self, t):
        """(pan, tilt, source) at time t, or None."""
        if self.head_feedback in ("auto", "measured"):
            latest = self.head_measured.latest()
            fresh = self.measured_rx is not None and time.monotonic() - self.measured_rx <= self.measured_fresh
            if latest is not None and fresh:          # freshness by RECEIVE time (any publisher clock)
                angles = self.head_measured.at(t)
                if angles is None and abs(t - latest[0]) > 5.0:
                    self._warn_once("joint_clock", "/joint_states stamps don't match the camera's clock: using "
                                    "the newest head angle (less accurate if the head moved since the image)")
                    angles = latest[1:]
                if angles is not None:
                    return (*angles, "measured")
            if self.head_feedback == "measured":
                return None
        angles = self.head_commanded.at(t)
        return None if angles is None else (*angles, "commanded")

    def _base_turned_since(self, t):
        """Degrees the base turned LEFT since time t (0 without fresh odometry)."""
        if not self.base_comp or self.odom_rx is None or time.monotonic() - self.odom_rx > self.odom_fresh:
            self._warn_once("no_odom", "no fresh /odom: assuming the base has not turned since the image")
            return 0.0
        s = self.yaw.samples
        if not (s and s[0][0] - 0.5 <= t <= s[-1][0] + 0.5):
            self._warn_once("odom_clock", "/odom stamps don't match the image time: not correcting for a base turn")
            return 0.0
        now, then = self.yaw.latest(), self.yaw.at(t)
        if now is None or then is None:
            return 0.0
        return now[1] - then

    # ------------------------------------------------------------------ target -> joints
    def _goal_joints(self, target):
        """PointStamped -> (q, info, error). info / error are strings for the log and the result."""
        frame = target.header.frame_id or self.camera_frame
        p = (target.point.x, target.point.y, target.point.z)
        if not all(math.isfinite(v) for v in p):
            return None, "", "target has non-finite coordinates"
        t = target.header.stamp.sec + target.header.stamp.nanosec * 1e-9

        if frame == self.base_frame:
            p_base, src = np.array(p), "base"
            turned = self._base_turned_since(t) if t > 0 else 0.0
        elif frame == self.camera_frame:
            if p[2] <= 0.05:
                return None, "", "target is not in front of the camera (z <= 0.05 m)"
            if t <= 0:
                self._warn_once("no_stamp", "target has no capture stamp: using the head angles NOW")
                t = time.time()
            head = self._head_at(t)
            if head is None:
                return None, "", ("no head angle for the image time (no /joint_states from sudohead and no "
                                  "/head/*_target, or the target is older than history_s)")
            pan, tilt, src = head
            p_base = camera_point_to_base(p, pan, tilt, self.geo)
            turned = self._base_turned_since(t)
        else:
            return None, "", f"unsupported frame '{frame}' (use {self.camera_frame} or {self.base_frame})"

        yaw, elev, dist = base_point_to_arm(p_base, self.geo, turned)
        q, note, ok = arm_joint_goal(self._pose(self.point_pose_name), yaw, elev, self.geo)
        info = (f"target {dist:.2f} m, yaw {math.degrees(yaw):+.1f} deg, elevation {math.degrees(elev):+.1f} deg "
                f"(head from {src}, base turned {turned:+.1f} deg)")
        if not ok:
            return None, info, note
        return q, (info + ("; " + note if note else "")), ""

    # ------------------------------------------------------------------ motion
    def _publish(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(ARM_JOINTS)
        msg.position = [float(v) for v in q]
        self.cmd_pub.publish(msg)

    def _arm_fresh(self):
        return self._arm_now() is not None

    def _servo_error(self):
        """A recent /arm_servo_status lists servos with comm_ok=0."""
        return bool(self.dead) and self.status_any_rx is not None and \
            time.monotonic() - self.status_any_rx <= self.status_fresh

    def _arm_now(self):
        """Measured arm joints (rad): the driver's /joint_states if it publishes them, else /arm_servo_status."""
        now = time.monotonic()
        if self.arm_q is not None and self.arm_rx is not None and now - self.arm_rx <= self.arm_fresh:
            return self.arm_q
        if self.status_q is not None and self.status_rx is not None and now - self.status_rx <= self.status_fresh:
            return self.status_q
        return None

    def _move(self, target, should_stop):
        """Stream a smooth move to `target`, then wait until the arm is there.
        Returns "done", "stopped" (should_stop() became true: the arm holds where it is) or "timeout"."""
        if self._servo_error():
            return "servo_error"
        self.speed_pub.publish(Int32(data=self.speed))
        measured = self._arm_now()
        start = measured.copy() if measured is not None else self.q

        if start is None:
            # Never commanded and no feedback: send the goal and let the servos go at /arm_speed.
            self._warn_once("unknown_start", "arm position unknown (no arm /joint_states): first move goes "
                            "straight to the goal at the /arm_speed limit")
            t_end = time.monotonic() + self.unknown_start
            while time.monotonic() < t_end:
                if should_stop():
                    return "stopped"
                self._publish(target)
                time.sleep(DT)
            self.q = target.copy()
            return "done"

        duration = max(self.min_move, float(np.max(np.abs(target - start))) / self.max_vel)
        spline = CubicSpline([0.0, duration], np.vstack([start, target]), axis=0, bc_type="clamped")
        t0 = time.monotonic()
        while True:
            t = time.monotonic() - t0
            if should_stop():
                self._publish(self.q)          # hold where the command is now
                return "stopped"
            if t >= duration:
                break
            self.q = spline(t)
            self._publish(self.q)
            time.sleep(DT)
        self.q = target.copy()
        self._publish(self.q)

        # "done": the measured joints reach the goal (driver /joint_states or /arm_servo_status), else settle
        if self._arm_fresh():
            t_end = time.monotonic() + self.arrive_timeout
            still_since, last_q = None, None
            while time.monotonic() < t_end:
                if should_stop():
                    return "stopped"
                if self._servo_error():
                    return "servo_error"
                q = self._arm_now()
                if q is not None:
                    state, _ = arrival(q, target, self.arrive_idx, self.arrive_tol, self.close)
                    if state == "there":
                        return "done"
                    moving = last_q is None or float(np.max(np.abs(q - last_q))) > 0.01
                    if state == "close" and not moving:
                        still_since = still_since or time.monotonic()
                        if time.monotonic() - still_since >= self.still:
                            return "done_close"
                    else:
                        still_since = None
                    last_q = q.copy()
                time.sleep(0.1)
            return "timeout"
        time.sleep(self.settle)
        return "done"

    def _take_ownership(self, owner):
        with self._owner_lock:
            self._owner = owner

    def _owns(self, owner):
        with self._owner_lock:
            return self._owner is owner

    # ------------------------------------------------------------------ PointAt
    def _on_goal(self, goal):
        if goal.arm not in ("", PointAt.Goal.ARM_AUTO, self.arm_side):
            self.get_logger().warn(f"PointAt rejected: arm '{goal.arm}' requested, this robot has '{self.arm_side}'")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_accepted(self, goal_handle):
        self._take_ownership(goal_handle)          # pre-empts a running goal or tuck
        goal_handle.execute()

    def _execute(self, goal_handle):
        result = PointAt.Result(arm_used=self.arm_side)

        def stop():
            return goal_handle.is_cancel_requested or not self._owns(goal_handle)

        with self._motion_lock:                    # waits until a pre-empted motion has stopped
            if stop():
                goal_handle.abort()
                result.message = "pre-empted before it started"
                return result
            if not self.motion_allowed:
                goal_handle.abort()
                result.message = "not_allowed: the arm health node is recovering a servo"
                self.get_logger().warn(f"PointAt refused: {result.message}")
                return result
            q, info, error = self._goal_joints(goal_handle.request.target)
            if q is None:
                self.get_logger().warn(f"PointAt failed: {error}" + (f" ({info})" if info else ""))
                goal_handle.abort()
                result.message = error
                return result
            self.get_logger().info(f"PointAt: {info}")
            goal_handle.publish_feedback(PointAt.Feedback(phase="moving"))
            outcome = "done"
            if self.zero_first:
                outcome = self._move(np.zeros(len(ARM_JOINTS)), stop)
                if outcome in ("timeout", "done_close"):
                    outcome = "done"           # carry on: the point move gets its own arrival check
            if outcome == "done":
                outcome = self._move(q, stop)

        if outcome == "servo_error":
            goal_handle.abort()
            result.message = f"servo_error: servo(s) {self.dead} not answering"
            self.get_logger().error(f"PointAt failed: {result.message}")
            return result

        if outcome == "stopped":
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = "cancelled"
            else:
                goal_handle.abort()
                result.message = "pre-empted by a newer goal or tuck"
            return result

        goal_handle.publish_feedback(PointAt.Feedback(phase="pointing"))
        goal_handle.succeed()
        result.success = True
        result.message = {"done": info,
                          "done_close": info + "; stopped slightly short of the goal (sagging joint?)",
                          }.get(outcome, info + "; arm did not report reaching the goal in time")
        return result

    # ------------------------------------------------------------------ Tuck
    def _on_tuck(self, _request, response):
        token = object()
        self._take_ownership(token)                # stops any PointAt
        try:
            target = self._pose(self.tuck_pose_name)
        except Exception as exc:  # noqa: BLE001
            response.success, response.message = False, str(exc)
            return response
        if not self.motion_allowed:
            response.success, response.message = False, "not_allowed: the arm health node is recovering a servo"
            return response
        with self._motion_lock:
            outcome = self._move(target, lambda: not self._owns(token))
        response.success = outcome in ("done", "done_close")
        response.message = {"done": "tucked", "done_close": "tucked (slightly short of the pose)",
                            "stopped": "interrupted by a new PointAt goal",
                            "servo_error": f"servo_error: servo(s) {self.dead} not answering",
                            "timeout": "arm did not report reaching the tuck pose in time"}[outcome]
        self.get_logger().info(f"Tuck: {response.message}")
        return response

    def _warn_once(self, key, text):
        if key not in self._warned:
            self._warned.add(key)
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSkills()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
