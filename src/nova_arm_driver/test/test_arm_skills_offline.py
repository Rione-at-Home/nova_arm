#!/usr/bin/env python3
"""Offline test of arm_skills (PointAt / Tuck server). No ROS, robot or arm: the ROS modules are replaced
by stand-ins; the real geometry functions and the real _move / _execute run against a simulated arm whose
/arm_servo_status follows the commands. Needs numpy + scipy and gaze_controller on the path. ~10 s.

    cd ~/nova_arm      # (or ~/nova_arm_ws)
    PYTHONPATH=src/nova_arm_driver:$HOME/receptionist_control/src/gaze_controller \\
        python3 src/nova_arm_driver/test/test_arm_skills_offline.py
"""

import math
import sys
import threading
import time
import types

import numpy as np

# ---- stand-ins for the ROS modules arm_skills imports at module level
STUBS = {
    "rclpy": {}, "rclpy.node": {"Node": object}, "rclpy.action": {
        "ActionServer": object, "CancelResponse": types.SimpleNamespace(ACCEPT=1),
        "GoalResponse": types.SimpleNamespace(ACCEPT=1, REJECT=0)},
    "rclpy.callback_groups": {"ReentrantCallbackGroup": object}, "rclpy.executors": {"MultiThreadedExecutor": object},
    "rcl_interfaces": {}, "rcl_interfaces.msg": {"ParameterDescriptor": object},
    "sensor_msgs": {}, "sensor_msgs.msg": {"JointState": lambda: types.SimpleNamespace(header=types.SimpleNamespace())},
    "std_msgs": {}, "std_msgs.msg": {"Bool": object, "Float32": object, "Int32": lambda data=0: types.SimpleNamespace(data=data)},
    "std_srvs": {}, "std_srvs.srv": {"Trigger": object},
    "receptionist_interfaces": {}, "receptionist_interfaces.action": {},
}
for name, attrs in STUBS.items():
    mod = sys.modules.setdefault(name, types.ModuleType(name))
    for k, v in attrs.items():
        setattr(mod, k, v)


class _PointAt:
    class Goal:
        ARM_AUTO = "auto"

    class Result:
        def __init__(self, arm_used=""):
            self.success, self.arm_used, self.message = False, arm_used, ""

    class Feedback:
        def __init__(self, phase=""):
            self.phase = phase


sys.modules["receptionist_interfaces.action"].PointAt = _PointAt

from nova_arm_driver import arm_skills as A                    # noqa: E402
from gaze_controller.head_state import head_angles_to_point     # noqa: E402

RESULTS = []


def check(label, cond, detail=""):
    RESULTS.append(bool(cond))
    print(f"{'PASS' if cond else 'FAIL'}  {label}" + (f"  -- {detail}" if detail else ""))


def rad_to_dxl(rad):                                            # copy of ArmDriver.rad_to_dxl
    value = int(((math.degrees(rad) + 150) / 300) * 1023)
    return max(0, min(1023, value))


GEO = {"head_pivot_xyz": [0.0, 0.0, 1.0], "camera_offset_xyz": [0.0, 0.0, 0.0], "arm_shoulder_xyz": [0.0, 0.0, 1.0],
       "head_pan_offset_deg": 0.0, "head_tilt_offset_deg": 0.0, "joint1_forward": 1.546, "joint1_sign": 1.0,
       "joint1_min": 0.2, "joint1_max": 2.9, "reach_margin_deg": 10.0, "pitch_sign": 1.0, "pitch_max_rad": 0.6,
       "pitch_index": None}


def kv(k, v):
    return types.SimpleNamespace(key=k, value=v)


def status_msg(q, dead=()):
    out = []
    for i, sid in enumerate(A.ARM_IDS):
        vals = [kv("comm_ok", "0")] if sid in dead else [kv("comm_ok", "1"), kv("position", str(rad_to_dxl(q[i])))]
        out.append(types.SimpleNamespace(name=f"arm_servo_{sid}", hardware_id=str(sid), values=vals))
    return types.SimpleNamespace(status=out)


# ------------------------------------------------------------------ pure functions
def test_pure():
    worst = max(abs(A.dxl_to_rad(rad_to_dxl(r)) - r) for r in np.linspace(-2.5, 2.5, 101))
    check("dxl_to_rad inverts ArmDriver.rad_to_dxl (within one tick, 0.29 deg)", worst < math.radians(0.3),
          f"worst {math.degrees(worst):.2f} deg")
    q = [0.1, -0.5, 0.3, 0.0, 0.2, 0.0]
    pos, dead = A.parse_servo_status(status_msg(q, dead={3}))
    check("parse_servo_status: positions in rad, comm_ok=0 servos listed as dead",
          dead == [3] and 3 not in pos and abs(pos[2] - (-0.5)) < 0.01)
    st, err = A.arrival(np.array(q), np.array(q) + 0.03, [0, 1, 2, 3, 4], 0.05, 0.12)
    check("arrival: 0.03 rad off with tol 0.05 -> there", st == "there", f"{err:.2f}")
    st, _ = A.arrival(np.array(q), np.array(q) + 0.08, [0, 1, 2, 3, 4], 0.05, 0.12)
    check("arrival: 0.08 rad off -> close (accepted only if it stays still)", st == "close")

    pose = np.array([1.546, -1.523, -0.401, 0.0, 0.0, 0.0])
    qg, note, ok = A.arm_joint_goal(pose, math.radians(30), 0.0, GEO)
    check("arm_joint_goal: 30 deg left -> joint1 = forward + 0.52, other joints from point_start",
          ok and abs(qg[0] - (1.546 + math.radians(30))) < 1e-9 and np.allclose(qg[1:], pose[1:]))
    qg, note, ok = A.arm_joint_goal(pose, math.radians(-80), 0.0, GEO)    # j1 = 0.15: 3 deg past the limit 0.2
    check("just past the limit (< reach_margin_deg): clamped to the limit, success with a note",
          ok and abs(qg[0] - 0.2) < 1e-9 and "clamped" in note)
    qg, note, ok = A.arm_joint_goal(pose, math.radians(-120), 0.0, GEO)
    check("far outside the range: fails without moving, message starts with 'out_of_range:'",
          not ok and qg is None and note.startswith("out_of_range:"), note[:60])

    # camera -> base -> arm yaw agrees with the gaze controller's head maths (head and shoulder at the pivot)
    worst = 0.0
    rnd = np.random.default_rng(2)
    for _ in range(300):
        p = (rnd.uniform(-1.5, 1.5), rnd.uniform(-0.5, 1.0), rnd.uniform(0.5, 3.0))
        pan, tilt = rnd.uniform(-60, 60), rnd.uniform(-30, 40)
        yaw, _, _ = A.base_point_to_arm(A.camera_point_to_base(p, pan, tilt, GEO), GEO)
        want = head_angles_to_point(p, pan, tilt)[0]
        worst = max(worst, abs(math.degrees(yaw) - want))
    check("camera->arm yaw = gaze_controller.head_angles_to_point pan (300 random cases)", worst < 1e-6,
          f"worst {worst:.1e} deg")
    yaw, _, _ = A.base_point_to_arm(A.camera_point_to_base((0.0, 0.0, 2.0), 20.0, 0.0, GEO), GEO, base_turned_deg=15.0)
    check("base turned 15 deg left since the image: target at 20 deg now at 5 deg", abs(math.degrees(yaw) - 5.0) < 1e-6)


# ------------------------------------------------------------------ node logic against a simulated arm
class Log:
    def __init__(self):
        self.lines = []

    def info(self, m): self.lines.append(("info", m))
    def warn(self, m): self.lines.append(("warn", m))
    def error(self, m): self.lines.append(("error", m))


class Pub:
    def __init__(self, sink):
        self.sink = sink

    def publish(self, msg):
        self.sink(msg)


class SimArm:
    """Servos move toward the last command at `rate` rad/s; a status message every 0.5 s."""

    def __init__(self, node, start, rate=2.0, sag=None, dead=(), feedback=True):
        self.node, self.q = node, np.array(start, float)
        self.goal = self.q.copy()
        self.rate, self.sag, self.dead, self.feedback = rate, dict(sag or {}), set(dead), feedback
        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)

    def command(self, msg):
        self.goal = np.array(msg.position, float)

    def _run(self):
        t_status, dt = 0.0, 0.02
        while self.running:
            target = self.goal.copy()
            for i, d in self.sag.items():
                target[i] += d
            step = np.clip(target - self.q, -self.rate * dt, self.rate * dt)
            self.q = self.q + step
            t_status += dt
            if self.feedback and t_status >= 0.5:
                t_status = 0.0
                self.node._on_status(status_msg(self.q, self.dead))
            time.sleep(dt)

    def stop(self):
        self.running = False


def make_node(**over):
    n = object.__new__(A.ArmSkills)
    log = Log()
    n.get_logger = lambda: log
    n.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: None))
    n.speed, n.max_vel, n.min_move, n.unknown_start = 30, 1.5, 0.4, 0.5
    n.arrive_tol, n.arrive_timeout, n.settle = 0.05, 3.0, 0.3
    n.arm_fresh, n.status_fresh, n.still, n.close = 0.5, 1.5, 0.8, 0.12
    n.arrive_idx = [0, 1, 2, 3, 4]
    n.arm_q = n.arm_rx = n.status_q = n.status_rx = n.q = None
    n.dead, n.motion_allowed, n._warned, n.status_any_rx = [], True, set(), None
    n.speed_pub = Pub(lambda m: None)
    n.arm_side, n.zero_first = "right", False
    n._motion_lock, n._owner_lock, n._owner = threading.Lock(), threading.Lock(), None
    for k, v in over.items():
        setattr(n, k, v)
    return n, log


def run_move(sim_kw, target, **node_kw):
    n, log = make_node(**node_kw)
    sim = SimArm(n, sim_kw.pop("start", np.zeros(6)), **sim_kw)
    n.cmd_pub = Pub(sim.command)
    sim.thread.start()
    time.sleep(0.6)                                   # first status arrives
    t0 = time.monotonic()
    out = n._move(np.array(target, float), lambda: False)
    dt = time.monotonic() - t0
    sim.stop()
    return out, dt, sim, n, log


def test_moves():
    target = [1.2, -1.0, -0.4, 0.0, 0.0, 0.0]
    out, dt, sim, n, _ = run_move({}, target)
    check("move with /arm_servo_status: starts from the measured pose, 'done' once the servos are there",
          out == "done" and np.max(np.abs(sim.q[:5] - np.array(target[:5]))) < 0.05, f"{dt:.1f} s")

    out, dt, sim, n, _ = run_move({"sag": {1: 0.09}}, target)
    check("joint2 sags 0.09 rad (still, within close_rad): 'done_close' after still_s",
          out == "done_close", f"{out}, {dt:.1f} s")

    out, dt, sim, n, _ = run_move({"sag": {1: 0.4}}, target)
    check("blocked 0.4 rad short: 'timeout' after arrive_timeout_s", out == "timeout", f"{dt:.1f} s")

    out, dt, sim, n, _ = run_move({"dead": {4}}, target)
    check("a servo reports comm_ok=0: 'servo_error' (no move sent)", out == "servo_error" and n.dead == [4]
          and np.allclose(sim.q, 0.0))

    out, dt, sim, n, log = run_move({"feedback": False}, target)
    check("no arm feedback at all (old driver): unknown start, goes straight there, 'done'",
          out == "done" and any("unknown" in m for s, m in log.lines if s == "warn"))


class GoalHandle:
    def __init__(self, target):
        self.request = types.SimpleNamespace(target=target)
        self.is_cancel_requested = False
        self.state, self.feedback = None, []

    def abort(self): self.state = "aborted"
    def succeed(self): self.state = "succeeded"
    def canceled(self): self.state = "canceled"
    def publish_feedback(self, fb): self.feedback.append(fb.phase)


def test_execute():
    pose = np.array([1.546, -1.523, -0.401, 0.0, 0.0, 0.0])
    n, log = make_node()
    n._goal_joints = lambda target: (pose.copy(), "target 2.00 m, yaw +0.0 deg", "")
    n.motion_allowed = False
    gh = GoalHandle(None)
    n._take_ownership(gh)
    res = n._execute(gh)
    check("motion not allowed (arm health node recovering): aborted, message 'not_allowed: ...'",
          gh.state == "aborted" and res.message.startswith("not_allowed:"))

    n, log = make_node()
    n._goal_joints = lambda target: (None, "target 2.0 m, yaw -120 deg", "out_of_range: target is -120 deg ...")
    gh = GoalHandle(None)
    n._take_ownership(gh)
    res = n._execute(gh)
    check("out of range: aborted without moving, message passed on for the manager",
          gh.state == "aborted" and res.message.startswith("out_of_range:") and not res.success)

    n, log = make_node()
    sim = SimArm(n, np.zeros(6))
    n.cmd_pub = Pub(sim.command)
    sim.thread.start()
    time.sleep(0.6)
    n._goal_joints = lambda target: (pose.copy(), "target 2.00 m, yaw +0.0 deg", "")
    gh = GoalHandle(None)
    n._take_ownership(gh)
    res = n._execute(gh)
    sim.stop()
    check("PointAt end to end: moving -> pointing feedback, succeeded, arm at the goal",
          gh.state == "succeeded" and res.success and gh.feedback == ["moving", "pointing"]
          and np.max(np.abs(sim.q[:5] - pose[:5])) < 0.05)


if __name__ == "__main__":
    test_pure()
    test_moves()
    test_execute()
    print(f"\n{sum(RESULTS)}/{len(RESULTS)} passed")
    sys.exit(0 if all(RESULTS) else 1)
