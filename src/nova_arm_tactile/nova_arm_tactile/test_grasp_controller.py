"""Grasp state machine against a simulated AX-12A gripper and pad (no ROS needed)."""
import math

import pytest

from nova_arm_tactile.grasp_controller import (
    ARM_UNAVAILABLE, CANCELED, CLOSING, FAULT, GRASPED, GRASPED_WEAK, HOLDING, IDLE, LOST,
    NO_OBJECT, OPENED, SENSOR_FAULT, SQUEEZING, STALLED, GraspConfig, GraspController,
    TactileSample)

DT = 0.02


class Sim:
    """
    Servo follows its goal at `servo_speed`; an object whose near face is at `contact` stops the
    finger (a little compliance) and the pad reads gain * how far the servo is driven past it.
    `blocked` stops the finger without any pad signal (object touching only off-pad).
    """

    def __init__(self, cfg, contact=None, gain=2000.0, blocked=None, servo_speed=3.0):
        self.cfg, self.contact, self.gain, self.blocked = cfg, contact, gain, blocked
        self.servo_speed = servo_speed
        self.virtual = cfg.open_rad      # where the servo is trying to be
        self.goal = cfg.open_rad
        self.t = 0.0
        self.calibrated = True
        self.fresh = True
        self.extra_total = 0.0
        self.ctl = GraspController(cfg)
        self.sent = []

    @property
    def d(self):
        return self.cfg.direction

    def position(self):
        p = self.virtual
        for lim in (self.contact, self.blocked):
            if lim is not None and self.d * (p - lim) > 0:
                p = lim + self.d * min(0.02, self.d * (p - lim))
        return p

    def total(self):
        if self.contact is None:
            return 0.0
        return max(0.0, self.gain * self.d * (self.virtual - self.contact)) + self.extra_total

    def step(self, motion_allowed=True):
        self.t += DT
        diff = self.goal - self.virtual
        self.virtual += math.copysign(min(abs(diff), self.servo_speed * DT), diff)
        tot = self.total() if self.calibrated else 0.0
        sample = TactileSample(stamp=self.t if self.fresh else -10.0, calibrated=self.calibrated,
                               in_contact=tot > 5.0, total=tot, contact_taxels=int(tot > 5) * 6)
        cmd, res = self.ctl.tick(self.t, sample, self.position(), self.t, motion_allowed)
        if cmd is not None:
            self.goal = cmd
            self.sent.append(cmd)
        return res

    def run(self, seconds=10.0, **kw):
        for _ in range(int(seconds / DT)):
            res = self.step(**kw)
            if res is not None:
                return res
        return None


@pytest.fixture
def cfg():
    return GraspConfig(open_rad=0.0, closed_rad=0.79, target_total=300.0)


def test_grasp_reaches_target_and_holds(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.ctl.start_grasp(sim.t)
    res = sim.run()
    assert res.outcome == GRASPED and res.success
    assert sim.ctl.state == HOLDING
    assert 0.45 <= res.contact_position <= 0.45 + 0.05
    # never driven past the squeeze limit
    assert max(sim.sent) <= res.contact_position + cfg.max_squeeze_rad + 1e-6


def test_no_object(cfg):
    sim = Sim(cfg, contact=None)
    sim.ctl.start_grasp(sim.t)
    res = sim.run()
    assert res.outcome == NO_OBJECT and not res.success
    assert max(sim.sent) <= cfg.closed_rad + 1e-9


def test_soft_object_hits_squeeze_limit(cfg):
    sim = Sim(cfg, contact=0.4, gain=300.0)    # 0.15 rad * 300 = 45 < target
    sim.ctl.start_grasp(sim.t)
    res = sim.run()
    assert res.outcome == GRASPED_WEAK and res.success
    assert max(sim.sent) <= res.contact_position + cfg.max_squeeze_rad + 1e-6


def test_blocked_without_pad_contact_is_a_stall(cfg):
    sim = Sim(cfg, contact=None, blocked=0.3)
    sim.ctl.start_grasp(sim.t)
    res = sim.run()
    assert res.outcome == STALLED
    assert sim.ctl.state == IDLE
    assert abs(sim.goal - sim.position()) < 0.05      # goal pulled back, servo not left pushing


def test_stale_pad_aborts_and_holds_position(cfg):
    sim = Sim(cfg, contact=0.5)
    sim.ctl.start_grasp(sim.t)
    for _ in range(10):
        assert sim.step() is None
    sim.fresh = False
    res = sim.run(1.0)
    assert res.outcome == SENSOR_FAULT and sim.ctl.state == FAULT


def test_waits_for_baseline_then_grasps(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.calibrated = False
    sim.ctl.start_grasp(sim.t)
    for _ in range(25):                     # 0.5 s calibrating: no motion, no result
        assert sim.step() is None
    assert sim.virtual == pytest.approx(0.0)
    sim.calibrated = True
    assert sim.run().outcome == GRASPED


def test_uncalibrated_for_too_long_fails(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.calibrated = False
    sim.ctl.start_grasp(sim.t)
    assert sim.run(5.0).outcome == SENSOR_FAULT


def test_motion_inhibited(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.ctl.start_grasp(sim.t)
    assert sim.run(1.0, motion_allowed=False).outcome == ARM_UNAVAILABLE


def test_open_after_grasp(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.ctl.start_grasp(sim.t)
    assert sim.run().outcome == GRASPED
    sim.ctl.start_open(sim.t)
    res = sim.run()
    assert res.outcome == OPENED and sim.ctl.state == IDLE
    assert sim.position() == pytest.approx(0.0, abs=cfg.open_tol_rad)


def test_slip_retightens_then_drop_is_lost(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.ctl.start_grasp(sim.t)
    assert sim.run().outcome == GRASPED
    held = sim.ctl.command
    sim.extra_total = -200.0                 # object slipping: less signal for the same squeeze
    for _ in range(50):
        sim.step()
    assert sim.ctl.command > held            # tightened
    assert sim.ctl.command <= sim.ctl.contact_command + cfg.max_squeeze_rad + 1e-6
    sim.contact = None                       # object gone
    for _ in range(30):
        sim.step()
    assert sim.ctl.state == LOST
    goal_when_lost = sim.ctl.command
    for _ in range(50):
        sim.step()
    assert sim.ctl.command == goal_when_lost  # does not chase or open on its own


def test_cancel_while_squeezing_keeps_grip(cfg):
    sim = Sim(cfg, contact=0.45, gain=400.0)
    sim.ctl.start_grasp(sim.t)
    while sim.ctl.state != SQUEEZING:
        assert sim.step() is None
    sim.step()
    res = sim.ctl.cancel(sim.t)
    assert res.outcome == CANCELED and sim.ctl.state == HOLDING


def test_reversed_direction(cfg):
    rev = GraspConfig(open_rad=0.2, closed_rad=-0.6, target_total=300.0)
    sim = Sim(rev, contact=-0.3)
    sim.virtual = sim.goal = 0.2
    sim.ctl.start_grasp(sim.t)
    res = sim.run()
    assert res.outcome == GRASPED
    assert min(sim.sent) >= res.contact_position - rev.max_squeeze_rad - 1e-6


def test_new_grasp_preempts_open(cfg):
    sim = Sim(cfg, contact=0.45)
    sim.ctl.start_open(sim.t)
    prev = sim.ctl.start_grasp(sim.t)
    assert prev.outcome == CANCELED
    assert sim.ctl.state == CLOSING
    assert sim.run().outcome == GRASPED