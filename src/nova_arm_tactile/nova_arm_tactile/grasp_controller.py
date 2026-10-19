r"""
ROS-free grasp state machine for one nova_arm gripper (one moving finger, FlexiTac pad on the
fixed finger).

The AX-12A gripper servo is position-controlled, so force is applied indirectly: the controller
walks the gripper goal toward "closed" in small steps, stops at tactile contact, then keeps
stepping (more slowly) while the pad's contact signal `total` is below the target. Driving the goal
a little past the object makes the servo's P-control push against it; the pad closes the loop.

    IDLE --grasp--> CLOSING --contact--> SQUEEZING --total >= target--> HOLDING (result GRASPED)
                       |                     \--squeeze limit--------> HOLDING (GRASPED_WEAK)
                       |--closed limit, no contact--> IDLE (NO_OBJECT)
                       |--finger blocked, no contact--> IDLE (STALLED)
    HOLDING: re-tightens on slip (total < slip_ratio * target), backs off on large overshoot,
             goes LOST if contact disappears. It never opens on its own.
    any --open--> OPENING --at open position--> IDLE (OPENED)
    sensor stale / arm unavailable during a goal --> FAULT (goal fails, gripper held where it is)

tick() is called at a fixed rate with the latest pad sample and measured gripper position and
returns the goal to send (or None for "no change") plus a GoalResult when a goal finishes.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

# states (match nova_arm_tactile_msgs/GraspState)
IDLE, OPENING, CLOSING, SQUEEZING, HOLDING, LOST, FAULT = range(7)
STATE_NAMES = ["IDLE", "OPENING", "CLOSING", "SQUEEZING", "HOLDING", "LOST", "FAULT"]

# outcomes (match nova_arm_tactile_msgs/Grasp result)
(GRASPED, GRASPED_WEAK, OPENED, NO_OBJECT, STALLED, SENSOR_FAULT, ARM_UNAVAILABLE, TIMEOUT,
 CANCELED) = range(9)
OUTCOME_NAMES = ["GRASPED", "GRASPED_WEAK", "OPENED", "NO_OBJECT", "STALLED", "SENSOR_FAULT",
                 "ARM_UNAVAILABLE", "TIMEOUT", "CANCELED"]

NAN = float("nan")


@dataclass
class GraspConfig:
    open_rad: float = 0.0            # gripper angle when fully open
    closed_rad: float = 0.79         # furthest the controller may drive (grasp pose in poses.yaml)
    close_speed: float = 0.4         # rad/s while approaching the object
    squeeze_speed: float = 0.08      # rad/s while tightening against it
    target_total: float = 300.0      # contact signal to squeeze to (tune per object class)
    max_squeeze_rad: float = 0.15    # never drive the goal more than this past first contact
    contact_confirm_ticks: int = 2   # consecutive ticks with in_contact before treating it as real
    settle_s: float = 0.3            # wait at the squeeze/closed limit for the servo to catch up
    slip_ratio: float = 0.6          # holding: re-tighten when total falls below ratio * target
    overshoot_ratio: float = 2.5     # holding: back off when total exceeds ratio * target
    lost_ticks: int = 10             # holding: ticks without contact before declaring LOST
    stall_rad: float = 0.12          # goal ahead of the measured angle by more than this ...
    stall_s: float = 0.4             # ... for this long, without contact -> STALLED
    open_tol_rad: float = 0.05
    open_timeout_s: float = 3.0
    goal_timeout_s: float = 8.0
    calib_wait_s: float = 2.0        # wait this long for a baseline before failing a grasp
    tactile_stale_s: float = 0.25
    joint_stale_s: float = 0.5

    @property
    def direction(self) -> float:
        return 1.0 if self.closed_rad >= self.open_rad else -1.0


@dataclass
class TactileSample:
    stamp: float                     # host seconds when the frame was received
    calibrated: bool
    in_contact: bool
    total: float
    contact_taxels: int
    centroid_row: float = NAN
    centroid_col: float = NAN


@dataclass
class GoalResult:
    outcome: int
    success: bool
    message: str
    gripper_position: float
    contact_position: float
    total: float
    contact_taxels: int

    @property
    def outcome_name(self) -> str:
        return OUTCOME_NAMES[self.outcome]


@dataclass
class _Goal:
    kind: str                        # "grasp" | "open"
    target: float
    speed: float
    deadline: float
    started: float


class GraspController:
    def __init__(self, cfg: GraspConfig) -> None:
        self.cfg = cfg
        self.state = IDLE
        self.detail = "idle"
        self.command: float = NAN          # last goal sent (rad)
        self.position: float = NAN         # last measured position (rad)
        self.contact_command: float = NAN  # goal at first confirmed contact
        self.target = cfg.target_total
        self.goal: Optional[_Goal] = None
        self._contact_ticks = 0
        self._no_contact_ticks = 0
        self._stall_since: Optional[float] = None
        self._settle_since: Optional[float] = None
        self._last_tick: Optional[float] = None
        self.sample: Optional[TactileSample] = None

    # ------------------------------------------------------------------ helpers
    @property
    def state_name(self) -> str:
        return STATE_NAMES[self.state]

    @property
    def busy(self) -> bool:
        return self.goal is not None

    def _clamp(self, x: float) -> float:
        lo, hi = sorted((self.cfg.open_rad, self.cfg.closed_rad))
        return min(hi, max(lo, x))

    def _past(self, a: float, b: float) -> float:
        """How far a is past b in the closing direction."""
        return self.cfg.direction * (a - b)

    def _result(self, outcome: int, success: bool, message: str) -> GoalResult:
        s = self.sample
        self.goal = None
        self.detail = message
        return GoalResult(outcome, success, message, self.position, self.contact_command,
                          s.total if s else 0.0, s.contact_taxels if s else 0)

    def tactile_ok(self, now: float) -> bool:
        s = self.sample
        return s is not None and s.calibrated and now - s.stamp <= self.cfg.tactile_stale_s

    # ------------------------------------------------------------------ goals
    def start_grasp(self, now: float, target_total: float = 0.0, close_speed: float = 0.0,
                    timeout_s: float = 0.0) -> Optional[GoalResult]:
        """Start a grasp. Returns the CANCELED result of a goal it preempts, if any."""
        prev = self.cancel(now, "preempted by a new grasp") if self.goal else None
        cfg = self.cfg
        self.target = target_total if target_total > 0 else cfg.target_total
        speed = close_speed if close_speed > 0 else cfg.close_speed
        timeout = timeout_s if timeout_s > 0 else cfg.goal_timeout_s
        self.goal = _Goal("grasp", self.target, speed, now + timeout, now)
        self.contact_command = NAN
        self._contact_ticks = self._no_contact_ticks = 0
        self._stall_since = self._settle_since = None
        if not math.isnan(self.position):
            self.command = self._clamp(self.position)
        self.state, self.detail = CLOSING, "approaching"
        return prev

    def start_open(self, now: float, timeout_s: float = 0.0) -> Optional[GoalResult]:
        prev = self.cancel(now, "preempted by open") if self.goal else None
        timeout = timeout_s if timeout_s > 0 else self.cfg.open_timeout_s
        self.goal = _Goal("open", 0.0, 0.0, now + timeout, now)
        self.contact_command = NAN
        self.state, self.detail = OPENING, "opening"
        return prev

    def cancel(self, now: float, why: str = "canceled") -> Optional[GoalResult]:
        if self.goal is None:
            return None
        in_contact = bool(self.sample and self.sample.in_contact)
        if self.state in (SQUEEZING, HOLDING) and in_contact:
            self.state = HOLDING              # keep whatever grip we already have
        else:
            if not math.isnan(self.position) and self.state == CLOSING:
                self.command = self._clamp(self.position)
            self.state = IDLE
        return self._result(CANCELED, False, why)

    # ------------------------------------------------------------------ tick
    def tick(self, now: float, sample: Optional[TactileSample], position: float,
             position_stamp: float, motion_allowed: bool = True
             ) -> Tuple[Optional[float], Optional[GoalResult]]:
        cfg = self.cfg
        dt = 0.0 if self._last_tick is None else max(0.0, min(0.1, now - self._last_tick))
        self._last_tick = now
        if sample is not None:
            self.sample = sample
        joint_ok = not math.isnan(position) and now - position_stamp <= cfg.joint_stale_s
        if joint_ok:
            self.position = position
        before = self.command
        result = None

        if self.goal is not None:
            result = self._tick_goal(now, dt, joint_ok, motion_allowed)
        elif self.state in (HOLDING, LOST) and motion_allowed:
            self._tick_hold(now, dt)

        changed = (not math.isnan(self.command)
                   and (math.isnan(before) or abs(self.command - before) > 1e-5))
        return (self.command if changed else None), result

    def _tick_goal(self, now, dt, joint_ok, motion_allowed) -> Optional[GoalResult]:
        cfg, g = self.cfg, self.goal
        if not motion_allowed:
            self.state = FAULT
            return self._result(ARM_UNAVAILABLE, False, "motion_allowed is false for this arm")
        if not joint_ok:
            self.state = FAULT
            return self._result(ARM_UNAVAILABLE, False,
                                "no fresh gripper position on /joint_states (arm not connected?)")
        if math.isnan(self.command):
            self.command = self._clamp(self.position)

        if g.kind == "open":
            self.command = cfg.open_rad
            if abs(self.position - cfg.open_rad) <= cfg.open_tol_rad:
                self.state = IDLE
                return self._result(OPENED, True, "open")
            if now > g.deadline:
                self.state = IDLE
                return self._result(TIMEOUT, False,
                                    f"gripper did not reach open (at {self.position:.2f} rad)")
            return None

        # ---- grasp
        s = self.sample
        if not self.tactile_ok(now):
            calibrating = s is not None and not s.calibrated and now - s.stamp <= cfg.tactile_stale_s
            if calibrating and now - g.started < cfg.calib_wait_s and self.state == CLOSING \
                    and math.isnan(self.contact_command):
                return None                    # baseline still being collected: wait, don't move
            self.command = self._clamp(self.position) if self.state == CLOSING else self.command
            self.state = FAULT
            why = ("pad not calibrated (re-zero with the gripper open)" if calibrating
                   else "no fresh tactile frames")
            return self._result(SENSOR_FAULT, False, why)
        if now > g.deadline:
            if self.state == CLOSING:
                self.command = self._clamp(self.position)
            self.state = HOLDING if (s.in_contact and self.state == SQUEEZING) else IDLE
            return self._result(TIMEOUT, False, f"timed out in {STATE_NAMES[self.state]}")

        self._contact_ticks = self._contact_ticks + 1 if s.in_contact else 0

        if self.state == CLOSING:
            if self._contact_ticks >= cfg.contact_confirm_ticks:
                self.contact_command = self.command
                self.state, self.detail = SQUEEZING, "contact"
                self._settle_since = None
                return None
            at_limit = self._past(self.command, cfg.closed_rad) >= -1e-6
            if at_limit:
                if self._settle_since is None:
                    self._settle_since = now
                if now - self._settle_since >= cfg.settle_s:
                    self.state = IDLE
                    return self._result(NO_OBJECT, False, "closed fully without tactile contact")
                return None
            lag = self._past(self.command, self.position)
            if lag > cfg.stall_rad:
                if self._stall_since is None:
                    self._stall_since = now
                if now - self._stall_since >= cfg.stall_s:
                    self.command = self._clamp(self.position)
                    self.state = IDLE
                    return self._result(STALLED, False,
                                        "finger blocked but the pad sees no contact "
                                        "(object off the pad, or pad not working)")
                return None                    # let the servo catch up before advancing
            self._stall_since = None
            self.command = self._clamp(self.command + cfg.direction * g.speed * dt)
            return None

        if self.state == SQUEEZING:
            if not s.in_contact:
                self._no_contact_ticks += 1
                if self._no_contact_ticks >= cfg.lost_ticks:
                    self.state, self.detail = CLOSING, "contact lost while squeezing, re-approaching"
                    self.contact_command = NAN
                    self._no_contact_ticks = 0
                return None
            self._no_contact_ticks = 0
            if s.total >= self.target:
                self.state = HOLDING
                return self._result(GRASPED, True, f"holding at total {s.total:.0f}")
            squeezed = self._past(self.command, self.contact_command)
            limit = cfg.max_squeeze_rad
            at_closed = self._past(self.command, cfg.closed_rad) >= -1e-6
            if squeezed >= limit - 1e-6 or at_closed:
                if self._settle_since is None:
                    self._settle_since = now
                if now - self._settle_since >= cfg.settle_s:
                    self.state = HOLDING
                    return self._result(GRASPED_WEAK, True,
                                        f"squeeze limit reached at total {s.total:.0f} "
                                        f"(target {self.target:.0f})")
                return None
            step = cfg.direction * cfg.squeeze_speed * dt
            nxt = self.command + step
            if self._past(nxt, self.contact_command) > limit:
                nxt = self.contact_command + cfg.direction * limit
            self.command = self._clamp(nxt)
            return None
        return None

    def _tick_hold(self, now: float, dt: float) -> None:
        """No goal active: keep the grip, re-tighten on slip, report drops. Never opens."""
        cfg, s = self.cfg, self.sample
        if math.isnan(self.contact_command) or self.state not in (HOLDING, LOST):
            return
        if not self.tactile_ok(now):
            self.detail = "holding blind: tactile stale"
            return
        if not s.in_contact:
            self._no_contact_ticks += 1
            if self._no_contact_ticks >= cfg.lost_ticks and self.state != LOST:
                self.state, self.detail = LOST, "contact lost while holding (dropped?)"
            return
        self._no_contact_ticks = 0
        if self.state == LOST:
            self.state, self.detail = HOLDING, "contact regained"
        squeezed = self._past(self.command, self.contact_command)
        if s.total < cfg.slip_ratio * self.target and squeezed < cfg.max_squeeze_rad - 1e-6:
            nxt = self.command + cfg.direction * cfg.squeeze_speed * dt
            if self._past(nxt, self.contact_command) > cfg.max_squeeze_rad:
                nxt = self.contact_command + cfg.direction * cfg.max_squeeze_rad
            self.command = self._clamp(nxt)
            self.detail = "slip: re-tightening"
        elif s.total > cfg.overshoot_ratio * self.target and squeezed > 1e-6:
            nxt = self.command - cfg.direction * cfg.squeeze_speed * dt
            if self._past(nxt, self.contact_command) < 0.0:
                nxt = self.contact_command
            self.command = self._clamp(nxt)
            self.detail = "over target: easing off"