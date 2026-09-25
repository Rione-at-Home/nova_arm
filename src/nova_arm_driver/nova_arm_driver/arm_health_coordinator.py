#!/usr/bin/env python3
"""
arm_health_coordinator.py

Monitors the AX-12A arm servos (via the ArmDriver's /arm_servo_status), and
recovers the known fault "Torque Limit reduced by an alarm shutdown" through
the driver's /arm/servo_<id>/restore_torque_limit services. It never opens
the serial port and never publishes /arm_command.

States : DIAGNOSTIC -> HEALTHY -> DIAGNOSTIC -> RECOVERING -> VERIFYING -> HEALTHY
         any failure -> FAULT (latched; leave with /arm_health/reset)

Publishes
  /arm_health_state   std_msgs/String  state name (latched, re-sent at 1 Hz)
  /arm_health_event   std_msgs/String  ARM_PROBLEM_DETECTED, ARM_RECOVERING,
                                       ARM_RECOVERED, ARM_PROBLEM_CLEARED, ARM_FAULT
  /arm_health_detail  std_msgs/String  JSON: state, reason, attempts, affected servos
  /arm_motion_allowed std_msgs/Bool    True only in HEALTHY (latched)
Subscribes
  /arm_servo_status   diagnostic_msgs/DiagnosticArray
Services
  /arm_health/reset   std_srvs/Trigger  (human acknowledges a FAULT)
Clients
  /arm/servo_<id>/restore_torque_limit  std_srvs/Trigger

Note: float parameters must be given as floats on the CLI (e.g. 60.0, not 60).
"""

import json
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

HEALTHY = "HEALTHY"
DIAGNOSTIC = "DIAGNOSTIC"
RECOVERING = "RECOVERING"
VERIFYING = "VERIFYING"
FAULT = "FAULT"

ERR_VOLTAGE = 1 << 0
ERR_OVERHEAT = 1 << 2
ERR_OVERLOAD = 1 << 5
ERR_NAMES = {0: "input voltage", 1: "angle limit", 2: "OVERHEATING", 3: "range",
             4: "checksum", 5: "OVERLOAD", 6: "instruction"}

VOLT_MIN, VOLT_MAX = 95, 125     # 0.1 V units, same window as arm_servo_status.py


def decode_err(err):
    return [n for b, n in ERR_NAMES.items() if err & (1 << b)]


@dataclass
class Servo:
    id: int
    ok: bool = False
    err: int = 0
    torque_enable: int = 0
    goal: int = 0
    position: int = 0
    load: int = 0
    voltage: int = 0
    temp: int = 0
    torque_limit: int = 0
    max_torque: int = 0

    @property
    def load_pct(self):
        return (self.load & 0x3FF) * 100.0 / 1023.0

    def describe(self):
        flags = decode_err(self.err)
        return (f"ID {self.id}: temp={self.temp}C torque_limit={self.torque_limit} "
                f"max_torque={self.max_torque} torque_enable={self.torque_enable} "
                f"pos={self.position} goal={self.goal} load={self.load_pct:.0f}% "
                f"volt={self.voltage / 10:.1f}V flags={flags or '-'}")


class ArmHealthCoordinator(Node):

    def __init__(self):
        super().__init__("arm_health_coordinator")

        d = self.declare_parameter
        d("servo_ids", [1, 2, 3, 4, 5, 6])
        d("status_topic", "/arm_servo_status")
        d("restore_service_pattern", "/arm/servo_{id}/restore_torque_limit")
        d("recover_max_temp_c", 50)          # same as restore_torque_limit.py
        d("warn_temp_c", 55)                 # same as arm_servo_status.py
        d("overload_clear_pct", 50.0)        # load below this = overload cleared
        d("max_recovery_attempts", 3)        # per servo
        d("detect_confirm_s", 1.0)           # issue must persist this long
        d("comm_timeout_s", 2.0)             # status older than this = stale
        d("comm_fault_s", 15.0)              # comm loss this long -> FAULT
        d("cooldown_timeout_s", 180.0)       # waiting for cool-down -> FAULT
        d("retry_delay_s", 3.0)              # pause between attempts
        d("service_timeout_s", 5.0)
        d("verify_settle_s", 1.0)
        d("verify_duration_s", 3.0)          # must stay good this long
        d("hold_tolerance_ticks", 30)        # ~8.8 deg allowed drift after recovery
        d("attempt_reset_s", 300.0)          # healthy this long -> attempts reset

        g = lambda n: self.get_parameter(n).value
        self.servo_ids = [int(i) for i in g("servo_ids")]
        self.recover_max_temp = int(g("recover_max_temp_c"))
        self.warn_temp = int(g("warn_temp_c"))
        self.overload_clear = float(g("overload_clear_pct"))
        self.max_attempts = int(g("max_recovery_attempts"))
        self.detect_confirm = float(g("detect_confirm_s"))
        self.comm_timeout = float(g("comm_timeout_s"))
        self.comm_fault = float(g("comm_fault_s"))
        self.cooldown_timeout = float(g("cooldown_timeout_s"))
        self.retry_delay = float(g("retry_delay_s"))
        self.service_timeout = float(g("service_timeout_s"))
        self.verify_settle = float(g("verify_settle_s"))
        self.verify_duration = float(g("verify_duration_s"))
        self.hold_tol = int(g("hold_tolerance_ticks"))
        self.attempt_reset = float(g("attempt_reset_s"))

        # ---- pub/sub/services ------------------------------------------
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, "/arm_health_state", latched)
        self.detail_pub = self.create_publisher(String, "/arm_health_detail", latched)
        self.motion_pub = self.create_publisher(Bool, "/arm_motion_allowed", latched)
        self.event_pub = self.create_publisher(String, "/arm_health_event", 10)

        self.create_subscription(DiagnosticArray, g("status_topic"), self.status_cb, 10)
        self.create_service(Trigger, "/arm_health/reset", self.reset_cb)

        pattern = g("restore_service_pattern")
        self.restore_clients = {i: self.create_client(Trigger, pattern.format(id=i))
                        for i in self.servo_ids}

        # ---- state -------------------------------------------------------
        self.servos = {}
        self.status_rx = None            # our clock time of last status message
        self.attempts = {}
        self.affected = []
        self.incident = False            # a problem has been announced
        self.announced_recovering = False
        self.issue_since = None
        self.retry_after = 0.0
        self.queue = []
        self.hold_pos = {}
        self.pending = None
        self.pending_id = None
        self.call_start = 0.0
        self.verify_start = 0.0
        self.good_since = None

        self.state = "START"
        self.state_since = self.now()
        self.reason = ""
        self.set_state(DIAGNOSTIC, "startup: waiting for first servo status")

        self.create_timer(0.2, self.tick)
        self.create_timer(1.0, self.heartbeat)

    # ------------------------------------------------------------ utilities

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def emit(self, event):
        self.event_pub.publish(String(data=event))
        self.get_logger().info(f"event: {event}")

    def set_state(self, new, reason=""):
        old = self.state
        self.state, self.state_since, self.reason = new, self.now(), reason
        text = f"ARM HEALTH {old} -> {new}" + (f": {reason}" if reason else "")
        if new == FAULT:
            self.get_logger().error(text)
        elif new == HEALTHY:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self.publish_state()
        self.motion_pub.publish(Bool(data=(new == HEALTHY)))
        self.publish_detail()

    def publish_state(self):
        self.state_pub.publish(String(data=self.state))

    def publish_detail(self):
        detail = {
            "state": self.state,
            "reason": self.reason,
            "attempts": self.attempts,
            "affected": [
                {"id": s.id, "temp": s.temp, "torque_limit": s.torque_limit,
                 "max_torque": s.max_torque, "torque_enable": s.torque_enable,
                 "position": s.position, "goal": s.goal,
                 "flags": decode_err(s.err)}
                for i in self.affected if (s := self.servos.get(i)) and s.ok
            ],
        }
        self.detail_pub.publish(String(data=json.dumps(detail)))

    def heartbeat(self):
        self.publish_state()

    # ------------------------------------------------------------ status in

    def status_cb(self, msg):
        for st in msg.status:
            try:
                sid = int(st.hardware_id)
                kv = {k.key: k.value for k in st.values}
                if kv.get("comm_ok") != "1":
                    self.servos[sid] = Servo(id=sid, ok=False)
                    continue
                self.servos[sid] = Servo(
                    id=sid, ok=True, err=int(kv["err"]),
                    torque_enable=int(kv["torque_enable"]),
                    goal=int(kv["goal"]), position=int(kv["position"]),
                    load=int(kv["load"]), voltage=int(kv["voltage"]),
                    temp=int(kv["temp"]), torque_limit=int(kv["torque_limit"]),
                    max_torque=int(kv["max_torque"]))
            except (ValueError, KeyError):
                self.get_logger().warn(f"Malformed status for '{st.name}'",
                                       throttle_duration_sec=10.0)
        self.status_rx = self.now()

    # ------------------------------------------------------- classification

    def hw_active(self, s):
        """Alarm conditions that are flagged AND whose cause has not cleared."""
        a = []
        if s.err & ERR_OVERHEAT and s.temp > self.recover_max_temp:
            a.append(f"overheating flag, temp {s.temp}C > {self.recover_max_temp}C")
        if s.err & ERR_OVERLOAD and s.load_pct > self.overload_clear:
            a.append(f"overload flag, load {s.load_pct:.0f}%")
        if s.err & ERR_VOLTAGE and not (VOLT_MIN <= s.voltage <= VOLT_MAX):
            a.append(f"voltage flag, {s.voltage / 10:.1f} V out of range")
        return a

    def evaluate(self, now):
        """List of (servo_id, KIND, text). Empty list = arm looks healthy."""
        if self.status_rx is None or now - self.status_rx > self.comm_timeout:
            return [(0, "COMM", "no fresh status from arm driver")]
        issues = []
        for sid in self.servo_ids:
            s = self.servos.get(sid)
            if s is None or not s.ok:
                issues.append((sid, "COMM", f"ID {sid}: no response"))
                continue
            if s.max_torque == 0:
                issues.append((sid, "MAX_TORQUE_ZERO",
                               f"ID {sid}: EEPROM Max Torque is 0 (use Dynamixel Wizard)"))
                continue
            if s.torque_enable == 0:
                issues.append((sid, "TORQUE_DISABLED",
                               f"ID {sid}: Torque Enable = 0 (not a torque-limit fault)"))
            for text in self.hw_active(s):
                issues.append((sid, "HW_ALARM", f"ID {sid}: {text}"))
            if s.torque_limit < s.max_torque:
                issues.append((sid, "TORQUE_LIMIT_LOW",
                               f"ID {sid}: Torque Limit {s.torque_limit} < Max Torque {s.max_torque}"))
        return issues

    # ---------------------------------------------------------------- tick

    def tick(self):
        now = self.now()
        if self.state == HEALTHY:
            self.step_healthy(now)
        elif self.state == DIAGNOSTIC:
            self.step_diagnostic(now)
        elif self.state == RECOVERING:
            self.step_recovering(now)
        elif self.state == VERIFYING:
            self.step_verifying(now)
        else:
            self.get_logger().error(
                f"HUMAN INTERVENTION REQUIRED: {self.reason}  "
                f"(fix it, then: ros2 service call /arm_health/reset std_srvs/srv/Trigger)",
                throttle_duration_sec=30.0)

    # -------------------------------------------------------------- HEALTHY

    def step_healthy(self, now):
        issues = self.evaluate(now)
        if not issues:
            self.issue_since = None
            for s in self.servos.values():
                if s.ok and s.temp >= self.warn_temp:
                    self.get_logger().warn(f"ID {s.id}: temperature {s.temp}C is high",
                                           throttle_duration_sec=30.0)
            if self.attempts and now - self.state_since > self.attempt_reset:
                self.attempts.clear()
            return

        if self.issue_since is None:
            self.issue_since = now
        if now - self.issue_since < self.detect_confirm:
            return

        for _, _, text in issues:
            self.get_logger().warn(f"Servo fault detected: {text}")
        for sid, _, _ in issues:
            if sid in self.servos:
                self.get_logger().warn(self.servos[sid].describe())
        self.affected = sorted({sid for sid, _, _ in issues if sid})
        self.incident = True
        self.announced_recovering = False
        self.emit("ARM_PROBLEM_DETECTED")
        self.set_state(DIAGNOSTIC, "; ".join(t for _, _, t in issues))

    # ----------------------------------------------------------- DIAGNOSTIC

    def step_diagnostic(self, now):
        if now < self.retry_after:
            return
        issues = self.evaluate(now)

        if not issues:
            if self.incident:
                self.emit("ARM_PROBLEM_CLEARED")
            self.incident = False
            self.issue_since = None
            self.set_state(HEALTHY, "no fault present")
            return

        kinds = {k for _, k, _ in issues}
        self.affected = sorted({sid for sid, _, _ in issues if sid})

        # Not something we may fix automatically -> human.
        fatal = [t for _, k, t in issues if k in ("MAX_TORQUE_ZERO", "TORQUE_DISABLED")]
        if fatal:
            self.to_fault("not auto-recoverable: " + "; ".join(fatal))
            return

        # Communication or an alarm condition that has not cleared: wait.
        targets = sorted(sid for sid, k, _ in issues if k == "TORQUE_LIMIT_LOW")
        hot = [sid for sid in targets
               if self.servos[sid].temp > self.recover_max_temp]
        if "COMM" in kinds or "HW_ALARM" in kinds or hot:
            limit = self.comm_fault if "COMM" in kinds else self.cooldown_timeout
            waited = now - self.state_since
            if waited > limit:
                self.to_fault(f"condition did not clear within {limit:.0f}s: "
                              + "; ".join(t for _, _, t in issues))
                return
            why = ("no communication" if "COMM" in kinds else
                   "alarm condition active" if "HW_ALARM" in kinds else
                   f"servo(s) {hot} above {self.recover_max_temp}C")
            self.get_logger().warn(
                f"Waiting ({why}); NOT restoring torque yet. {waited:.0f}/{limit:.0f}s",
                throttle_duration_sec=10.0)
            return

        # Only TORQUE_LIMIT_LOW remains and the servos are cool enough.
        exhausted = [sid for sid in targets
                     if self.attempts.get(sid, 0) >= self.max_attempts]
        if exhausted:
            self.to_fault(f"servo(s) {exhausted} still faulty after "
                          f"{self.max_attempts} recovery attempts")
            return
        self.begin_recovery(targets)

    # ----------------------------------------------------------- RECOVERING

    def begin_recovery(self, targets):
        self.incident = True
        self.queue = list(targets)
        self.pending = None
        for sid in targets:
            self.attempts[sid] = self.attempts.get(sid, 0) + 1
            s = self.servos[sid]
            self.hold_pos[sid] = s.position
            self.get_logger().warn(
                f"Recovery attempt {self.attempts[sid]}/{self.max_attempts} on ID {sid}: "
                f"hold at {s.position} ticks, then restore Torque Limit "
                f"{s.torque_limit} -> {s.max_torque}. " + s.describe())
        if not self.announced_recovering:
            self.announced_recovering = True
            self.emit("ARM_RECOVERING")
        self.set_state(RECOVERING, f"servo(s) {targets}")

    def recovery_failed(self, text):
        self.get_logger().error(f"Recovery step failed: {text}")
        self.pending = None
        self.queue = []
        self.retry_after = self.now() + self.retry_delay
        self.set_state(DIAGNOSTIC, f"recovery failed: {text}")

    def step_recovering(self, now):
        if self.pending is not None:
            sid = self.pending_id
            if self.pending.done():
                try:
                    res = self.pending.result()
                except Exception as exc:          # noqa: BLE001
                    res = None
                    err = str(exc)
                if res is not None and res.success:
                    self.get_logger().info(f"ID {sid}: driver reports: {res.message}")
                    self.pending = None
                else:
                    self.recovery_failed(f"ID {sid}: " + (res.message if res else err))
                    return
            elif now - self.call_start > self.service_timeout:
                try:
                    self.restore_clients[sid].remove_pending_request(self.pending)
                except Exception:                 # noqa: BLE001
                    pass
                self.recovery_failed(f"ID {sid}: restore service timed out")
                return
            else:
                return

        if self.queue:
            sid = self.queue.pop(0)
            client = self.restore_clients[sid]
            if not client.service_is_ready():
                self.to_fault(f"restore service for ID {sid} not available; "
                              f"is the modified ArmDriver running?")
                return
            self.pending = client.call_async(Trigger.Request())
            self.pending_id, self.call_start = sid, now
            return

        self.verify_start, self.good_since = now, None
        self.set_state(VERIFYING, "re-checking servo status")

    # ------------------------------------------------------------ VERIFYING

    def step_verifying(self, now):
        # only judge statuses received after the writes had time to settle
        if self.status_rx is None or self.status_rx < self.verify_start + self.verify_settle:
            return

        problems = [t for _, _, t in self.evaluate(now)]
        for sid, held in self.hold_pos.items():
            s = self.servos.get(sid)
            if s and s.ok and (abs(s.position - held) > self.hold_tol
                               or abs(s.goal - held) > self.hold_tol):
                problems.append(f"ID {sid}: moved away from hold position {held} "
                                f"(pos={s.position}, goal={s.goal})")
        if problems:
            self.get_logger().error("Verification FAILED: " + "; ".join(problems))
            self.retry_after = now + self.retry_delay
            self.set_state(DIAGNOSTIC, "verification failed")
            return

        if self.good_since is None:
            self.good_since = now
            return
        if now - self.good_since >= self.verify_duration:
            for sid in self.hold_pos:
                self.get_logger().info("Verified: " + self.servos[sid].describe())
            self.hold_pos.clear()
            self.incident = False
            self.issue_since = None
            self.emit("ARM_RECOVERED")
            self.set_state(HEALTHY, "recovery verified")

    # ---------------------------------------------------------------- FAULT

    def to_fault(self, reason):
        self.queue, self.pending = [], None
        self.emit("ARM_FAULT")
        self.set_state(FAULT, reason)

    def reset_cb(self, _req, res):
        if self.state != FAULT:
            res.success, res.message = False, f"not in FAULT (state is {self.state})"
            return res
        self.attempts.clear()
        self.incident = False
        self.retry_after = 0.0
        self.set_state(DIAGNOSTIC, "operator reset")
        res.success, res.message = True, "re-diagnosing"
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ArmHealthCoordinator()
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