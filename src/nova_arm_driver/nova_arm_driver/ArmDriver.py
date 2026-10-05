#!/usr/bin/env python3
"""
ArmDriver node: coordinates two independent DYNAMIXEL buses (one per
physical arm, each on its own USB2Dynamixel) behind the same
/arm_command, /arm_speed, /joint_states topics used previously.

[HEALTH] additions (marked in the code):
  * An arm that is not plugged in / not powered NEVER stops the node. Its bus
    stays down, the other arm runs normally, and the node retries the port
    every `reconnect_period_s`. The bus state is published for the health
    coordinator, which raises the alert.
  * /arm_servo_status   diagnostic_msgs/DiagnosticArray, 2 Hz, read-only.
      one "arm_<side>_bus" entry per arm (kind=bus) and one
      "arm_<side>_servo_<id>" entry per servo (kind=servo); both carry an
      `arm` key. Servo entries carry the raw register values.
  * /arm/<side>/servo_<id>/restore_torque_limit  std_srvs/Trigger
  * /arm/<side>/motion_allowed  std_msgs/Bool (transient_local): while False,
    /arm_command goals for THAT arm are dropped (the other arm still moves).
    Default True, so nothing changes if no coordinator is running.
  * /joint_states only contains joints that have been read at least once, so
    an absent arm is not reported at 0 rad. Set `fill_absent_joints:=true` to
    get the old fixed 12-joint layout (absent joints at 0.0).
  * Torque is enabled with Goal = Present Position first (no jump on startup
    or when an arm comes back); set `hold_on_enable:=false` for the old
    behaviour.

The default single-threaded executor serialises every callback, so timers,
subscriptions and services never touch a serial port at the same time.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

from nova_arm_driver.ArmBus import ArmBus


# Right side arm
RIGHT_JOINT_TO_ID = {
    "right_joint1": 1,
    "right_joint2": 2,
    "right_joint3": 3,
    "right_joint4": 4,
    "right_joint5": 5,
    "right_gripper": 6,
}

# Left side arm
LEFT_JOINT_TO_ID = {
    "left_joint1": 11,
    "left_joint2": 12,
    "left_joint3": 13,
    "left_joint4": 14,
    "left_joint5": 15,
    "left_gripper": 16,
}

JOINT_ORDER = list(RIGHT_JOINT_TO_ID.keys()) + list(LEFT_JOINT_TO_ID.keys())

DEFAULT_RIGHT_PORT = "/dev/dxl_right"
DEFAULT_LEFT_PORT = "/dev/dxl_left"
DEFAULT_BAUDRATE = 1000000

# How often to poll actual motor positions and publish /joint_states.
FEEDBACK_PERIOD_SEC = 0.02


class ArmDriver(Node):

    def __init__(self):

        super().__init__("arm_driver")

        # PARAMETERS
        self.declare_parameter("right_port", DEFAULT_RIGHT_PORT)
        self.declare_parameter("left_port", DEFAULT_LEFT_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)

        # [HEALTH]
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("reconnect_period_s", 3.0)
        self.declare_parameter("recovery_max_temp_c", 50)
        self.declare_parameter("require_motion_inhibit_for_restore", True)
        self.declare_parameter("hold_on_enable", True)
        self.declare_parameter("fill_absent_joints", False)

        right_port = self.get_parameter("right_port").value
        left_port = self.get_parameter("left_port").value
        baudrate = self.get_parameter("baudrate").value
        hold = bool(self.get_parameter("hold_on_enable").value)
        self.fill_absent = bool(self.get_parameter("fill_absent_joints").value)

        # ARM BUSES: one per physical arm, no shared state.
        self.right_bus = ArmBus(
            "right", right_port, RIGHT_JOINT_TO_ID, baudrate,
            self.get_logger(), hold_on_enable=hold,
        )
        self.left_bus = ArmBus(
            "left", left_port, LEFT_JOINT_TO_ID, baudrate,
            self.get_logger(), hold_on_enable=hold,
        )
        self.buses = [self.right_bus, self.left_bus]
        self.bus_by_name = {bus.name: bus for bus in self.buses}

        # Joint name -> owning bus, used to route incoming commands.
        self.joint_to_bus = {}
        for bus in self.buses:
            for joint_name in bus.joint_to_id:
                self.joint_to_bus[joint_name] = bus

        # [HEALTH] per-arm motion gate and throttled-warning bookkeeping
        self.motion_allowed = {bus.name: True for bus in self.buses}
        self._last_warn = {}

        # CONNECT + ENABLE TORQUE
        #
        # A missing arm is logged and left disconnected; the node keeps
        # running and retries in reconnect_tick().
        for bus in self.buses:
            self._bring_up(bus)

        # SUBSCRIPTIONS
        self.command_sub = self.create_subscription(
            JointState,
            "/arm_command",
            self.command_callback,
            10,
        )

        self.speed_sub = self.create_subscription(
            Int32,
            "/arm_speed",
            self.speed_callback,
            10,
        )

        # FEEDBACK
        #
        # /joint_states carries the arm's ACTUAL measured position.
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.feedback_timer = self.create_timer(
            FEEDBACK_PERIOD_SEC,
            self.read_callback,
        )

        # [HEALTH] motion gates, status publisher, restore services, reconnect
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        for bus in self.buses:
            self.create_subscription(
                Bool,
                f"/arm/{bus.name}/motion_allowed",
                lambda msg, b=bus: self.motion_allowed_callback(b, msg),
                latched,
            )
            for dxl_id in bus.joint_to_id.values():
                self.create_service(
                    Trigger,
                    f"/arm/{bus.name}/servo_{dxl_id}/restore_torque_limit",
                    lambda req, res, b=bus, i=dxl_id:
                        self.restore_callback(b, i, res),
                )

        self.status_pub = self.create_publisher(
            DiagnosticArray, "/arm_servo_status", 10
        )
        rate = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.2), self.publish_status)

        period = float(self.get_parameter("reconnect_period_s").value)
        self.create_timer(max(period, 0.5), self.reconnect_tick)



    def _warn_throttled(self, key, text, period=5.0):
        """
        Per-key throttle (rclpy's own throttle is per call site, which
        would let one arm's warning hide the other's).
        """
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_warn.get(key, -1e9) >= period:
            self._last_warn[key] = now
            self.get_logger().warning(text)

    def _bring_up(self, bus):
        """
        Connect one bus and enable torque. Returns True if it is up.
        """
        if not bus.connect():
            self.get_logger().warning(
                f"[{bus.name}] arm not detected on {bus.port_name}; running "
                f"without it, retrying every "
                f"{self.get_parameter('reconnect_period_s').value} s"
            )
            return False
        bus.enable_torque()
        return True

    def reconnect_tick(self):
        for bus in self.buses:
            if not bus.connected:
                if self._bring_up(bus):
                    self.get_logger().info(f"[{bus.name}] arm detected")

    # JOINT CALLBACK
    def command_callback(self, msg):

        # Route each joint's goal to its owning bus's sync-write buffer.
        # Both arms' packets get flushed back-to-back at the end.
        queued_any = {bus.name: False for bus in self.buses}

        for joint_name, position_rad in zip(msg.name, msg.position):

            bus = self.joint_to_bus.get(joint_name)

            if bus is None:
                self.get_logger().warning(f"Unknown joint '{joint_name}'")
                continue

            # [HEALTH] arm being diagnosed/recovered: do not move it
            if not self.motion_allowed[bus.name]:
                self._warn_throttled(
                    f"inhibit_{bus.name}",
                    f"/arm_command for {bus.name} arm ignored: motion not "
                    f"allowed (arm health).",
                )
                continue

            # [HEALTH] arm not detected: drop its goals, keep the other arm
            if not bus.connected:
                self._warn_throttled(
                    f"absent_{bus.name}",
                    f"/arm_command for {bus.name} arm ignored: arm not "
                    f"connected.",
                )
                continue

            if bus.queue_goal(joint_name, position_rad):
                queued_any[bus.name] = True
                self.get_logger().info(
                    f"{joint_name} ({bus.name}) -> {position_rad:.2f} rad"
                )

        for bus in self.buses:
            if queued_any[bus.name]:
                bus.flush_writes()

    # FEEDBACK CALLBACK
    def read_callback(self):

        # Each bus reads only its own joints. A dead or missing arm costs
        # (almost) nothing and does not block feedback for the other arm.
        merged_positions = {}

        for bus in self.buses:
            merged_positions.update(
                bus.read_positions(include_unknown=self.fill_absent)
            )

        names = [n for n in JOINT_ORDER if n in merged_positions]
        if not names:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [merged_positions[n] for n in names]

        self.joint_state_pub.publish(msg)

    # SPEED CALLBACK
    def speed_callback(self, msg):

        speed = int(msg.data / 100.0 * 1023)
        speed = max(20, min(speed, 1023))

        for bus in self.buses:
            bus.set_speed(speed)

        self.get_logger().info(f"Speed set to {msg.data}%")

    # ------------------------------------------------------------------
    # [HEALTH] motion gate

    def motion_allowed_callback(self, bus, msg):
        if msg.data != self.motion_allowed[bus.name]:
            self.get_logger().warning(
                f"{bus.name} arm motion "
                + ("ALLOWED" if msg.data else "INHIBITED (arm health)")
            )
        self.motion_allowed[bus.name] = bool(msg.data)

    # [HEALTH] read-only status

    def publish_status(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        for bus in self.buses:
            servos = bus.read_all_status()
            total = len(servos)
            responding = sum(1 for v in servos.values() if v is not None)
            info = bus.info()

            st = DiagnosticStatus()
            st.name = f"arm_{bus.name}_bus"
            st.hardware_id = bus.name
            if not info["connected"]:
                st.level, st.message = DiagnosticStatus.ERROR, "port not available"
            elif responding == 0:
                st.level, st.message = DiagnosticStatus.ERROR, "no servo responds"
            elif responding < total:
                st.level = DiagnosticStatus.WARN
                st.message = f"{responding}/{total} servos respond"
            else:
                st.level, st.message = DiagnosticStatus.OK, "ok"
            st.values = [
                KeyValue(key="kind", value="bus"),
                KeyValue(key="arm", value=bus.name),
                KeyValue(key="connected", value="1" if info["connected"] else "0"),
                KeyValue(key="port", value=str(info["port"])),
                KeyValue(key="connect_attempts", value=str(info["connect_attempts"])),
                KeyValue(key="last_error", value=str(info["last_error"])),
                KeyValue(key="responding", value=str(responding)),
                KeyValue(key="total", value=str(total)),
            ]
            msg.status.append(st)

            for joint_name, dxl_id in bus.joint_to_id.items():
                d = servos[dxl_id]
                ss = DiagnosticStatus()
                ss.name = f"arm_{bus.name}_servo_{dxl_id}"
                ss.hardware_id = str(dxl_id)
                head = [
                    KeyValue(key="kind", value="servo"),
                    KeyValue(key="arm", value=bus.name),
                    KeyValue(key="id", value=str(dxl_id)),
                    KeyValue(key="joint", value=joint_name),
                ]
                if d is None:
                    ss.level, ss.message = DiagnosticStatus.ERROR, "no response"
                    ss.values = head + [KeyValue(key="comm_ok", value="0")]
                else:
                    ss.level, ss.message = DiagnosticStatus.OK, "ok"
                    ss.values = head + [KeyValue(key="comm_ok", value="1")] + [
                        KeyValue(key=k, value=str(v)) for k, v in d.items()
                    ]
                msg.status.append(ss)

        self.status_pub.publish(msg)

    # [HEALTH] recovery primitive

    def restore_callback(self, bus, dxl_id, response):
        log = self.get_logger()

        def fail(text):
            log.error(f"restore {bus.name} ID {dxl_id}: {text}")
            response.success = False
            response.message = text
            return response

        if (self.get_parameter("require_motion_inhibit_for_restore").value
                and self.motion_allowed[bus.name]):
            return fail(f"refused: /arm/{bus.name}/motion_allowed is still True "
                        f"(publish False first so nothing commands the arm)")

        max_temp = int(self.get_parameter("recovery_max_temp_c").value)
        ok, text = bus.restore_torque_limit(dxl_id, max_temp)
        if not ok:
            return fail(text)

        response.success = True
        response.message = text
        return response

    # SHUTDOWN
    def destroy_node(self):

        self.get_logger().info("Disabling torque...")

        for bus in self.buses:
            bus.disable_torque()
            bus.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ArmDriver()

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