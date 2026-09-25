#!/usr/bin/env python3
"""
ArmDriver.py  (+ arm-health interface)

Original behaviour is unchanged. Added (marked [HEALTH]):
  * /arm_servo_status  (diagnostic_msgs/DiagnosticArray, read-only, 2 Hz)
  * /arm/servo_<id>/restore_torque_limit (std_srvs/Trigger): the safe
    "hold here, then restore Torque Limit = Max Torque" sequence from
    restore_torque_limit.py, with every write verified.
  * /arm_motion_allowed (std_msgs/Bool, transient_local): while False, incoming
    /arm_command messages are dropped. This is what stops other nodes from
    fighting a recovery. Default is True, so nothing changes if no health
    node is running.

The default single-threaded executor serialises every callback, so timer,
subscriptions and services never touch the serial port at the same time.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler


ADDR_MAX_TORQUE = 14          # [HEALTH] EEPROM, 2 bytes
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34        # [HEALTH] RAM, 2 bytes
ADDR_PRESENT_POSITION = 36
ADDR_TEMPERATURE = 43         # [HEALTH]

# [HEALTH] one block read of addresses 24..43 (torque enable ... temperature)
ADDR_STATUS_BLOCK = 24
LEN_STATUS_BLOCK = 20

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# COMMUNICATION SETTINGS

PROTOCOL_VERSION = 1.0

PORT_NAME = "/dev/ttyUSB1"
BAUDRATE = 1000000

# CONTROL TABLE

JOINT_TO_ID = {
    "joint1": 1,
    "joint2": 2,
    "joint3": 3,
    "joint4": 4,
    "joint5": 5,
    "gripper": 6,
}


# HELPERS

def rad_to_dxl(rad):

    deg = math.degrees(rad)

    value = int(((deg + 150.0) / 300.0) * 1023.0)

    return max(0, min(1023, value))


def dxl_to_rad(value):

    deg = value * 300.0 / 1023.0 - 150.0

    return math.radians(deg)


# DRIVER

class ArmDriver(Node):

    def __init__(self):

        super().__init__("arm_driver")

        # [HEALTH] parameters / state
        self.declare_parameter("recovery_max_temp_c", 50)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("require_motion_inhibit_for_restore", True)
        self.motion_allowed = True

        self.port_handler = PortHandler(PORT_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # CONNECT TO DYNAMIXELS

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open {PORT_NAME}")

        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError("Failed to set baudrate")

        self.get_logger().info(
            f"Connected to Dynamixels on {PORT_NAME}"
        )

        # ENABLE TORQUE

        for dxl_id in JOINT_TO_ID.values():

            dxl_comm_result, dxl_error = \
                self.packet_handler.write1ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    ADDR_TORQUE_ENABLE,
                    TORQUE_ENABLE,
                )

            if dxl_comm_result != 0:
                self.get_logger().error(
                    f"Communication failed for ID {dxl_id}"
                )

            elif dxl_error != 0:
                self.get_logger().error(
                    f"Dynamixel error on ID {dxl_id}"
                )

            else:
                self.get_logger().info(
                    f"Torque enabled on ID {dxl_id}"
                )

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

        # [HEALTH] motion gate, status publisher, restore services
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, "/arm_motion_allowed", self.motion_allowed_callback, latched
        )

        self.status_pub = self.create_publisher(
            DiagnosticArray, "/arm_servo_status", 10
        )
        rate = float(self.get_parameter("status_rate_hz").value)
        self.create_timer(1.0 / max(rate, 0.2), self.publish_status)

        for dxl_id in JOINT_TO_ID.values():
            self.create_service(
                Trigger,
                f"/arm/servo_{dxl_id}/restore_torque_limit",
                lambda req, res, i=dxl_id: self.restore_callback(i, res),
            )

    # JOINT CALLBACK
    def command_callback(self, msg):

        # [HEALTH] arm is being diagnosed/recovered: do not move anything
        if not self.motion_allowed:
            self.get_logger().warn(
                "/arm_command ignored: motion not allowed (arm health).",
                throttle_duration_sec=5.0,
            )
            return

        for joint_name, position_rad in zip(
                msg.name,
                msg.position):

            if joint_name not in JOINT_TO_ID:

                self.get_logger().warn(
                    f"Unknown joint '{joint_name}'"
                )

                continue

            dxl_id = JOINT_TO_ID[joint_name]

            goal = rad_to_dxl(position_rad)

            dxl_comm_result, dxl_error = \
                self.packet_handler.write2ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    ADDR_GOAL_POSITION,
                    goal,
                )

            if dxl_comm_result != 0:

                self.get_logger().error(
                    f"Failed sending command to ID {dxl_id}"
                )

            elif dxl_error != 0:

                self.get_logger().error(
                    f"Dynamixel error on ID {dxl_id}"
                )

            else:

                self.get_logger().info(
                    f"{joint_name} -> "
                    f"{position_rad:.2f} rad "
                    f"({goal})"
                )

    # SPEED CALLBACK

    def speed_callback(self, msg):

        speed = int(msg.data / 100.0 * 1023)

        speed = max(20, min(speed, 1023))

        for dxl_id in JOINT_TO_ID.values():

            self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_MOVING_SPEED,
                speed,
            )

        self.get_logger().info(
            f"Speed set to {msg.data}%"
        )

    # ------------------------------------------------------------------
    # [HEALTH] low-level helpers

    def _r1(self, dxl_id, addr):
        v, c, _e = self.packet_handler.read1ByteTxRx(self.port_handler, dxl_id, addr)
        return v if c == 0 else None

    def _r2(self, dxl_id, addr):
        v, c, _e = self.packet_handler.read2ByteTxRx(self.port_handler, dxl_id, addr)
        return v if c == 0 else None

    def _w2_verified(self, dxl_id, addr, value, retries=5):
        for _ in range(retries):
            c, _e = self.packet_handler.write2ByteTxRx(
                self.port_handler, dxl_id, addr, int(value)
            )
            if c == 0 and self._r2(dxl_id, addr) == int(value):
                return True
            time.sleep(0.02)
        return False

    # [HEALTH] motion gate

    def motion_allowed_callback(self, msg):
        if msg.data != self.motion_allowed:
            self.get_logger().warn(
                "Arm motion " + ("ALLOWED" if msg.data else "INHIBITED (arm health)")
            )
        self.motion_allowed = bool(msg.data)

    # [HEALTH] read-only status

    def read_servo(self, dxl_id):
        """Return a dict of raw register values, or None if no valid response."""
        blk, comm, err = self.packet_handler.readTxRx(
            self.port_handler, dxl_id, ADDR_STATUS_BLOCK, LEN_STATUS_BLOCK
        )
        if comm != 0 or len(blk) != LEN_STATUS_BLOCK:
            return None
        max_torque, comm2, err2 = self.packet_handler.read2ByteTxRx(
            self.port_handler, dxl_id, ADDR_MAX_TORQUE
        )
        if comm2 != 0:
            return None

        def w(off):  # 16-bit little endian, offset relative to address 24
            return blk[off] | (blk[off + 1] << 8)

        return {
            "err": err | err2,          # hardware error flags (status packet)
            "torque_enable": blk[0],
            "cw_margin": blk[2], "ccw_margin": blk[3],
            "cw_slope": blk[4], "ccw_slope": blk[5],
            "goal": w(6),
            "moving_speed": w(8),
            "torque_limit": w(10),
            "position": w(12),
            "load": w(16),
            "voltage": blk[18],         # 0.1 V units
            "temp": blk[19],            # deg C
            "max_torque": max_torque,
        }

    def publish_status(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        for dxl_id in JOINT_TO_ID.values():
            st = DiagnosticStatus()
            st.name = f"arm_servo_{dxl_id}"
            st.hardware_id = str(dxl_id)
            d = self.read_servo(dxl_id)
            if d is None:
                st.level = DiagnosticStatus.ERROR
                st.message = "no response"
                st.values = [KeyValue(key="comm_ok", value="0")]
            else:
                st.level = DiagnosticStatus.OK
                st.message = "ok"
                st.values = [KeyValue(key="comm_ok", value="1")] + [
                    KeyValue(key=k, value=str(v)) for k, v in d.items()
                ]
            msg.status.append(st)
        self.status_pub.publish(msg)

    # [HEALTH] recovery primitive (restore_torque_limit.py, steps 1-11)

    def restore_callback(self, dxl_id, response):
        log = self.get_logger()

        def fail(text):
            log.error(f"restore ID {dxl_id}: {text}")
            response.success = False
            response.message = text
            return response

        if (self.get_parameter("require_motion_inhibit_for_restore").value
                and self.motion_allowed):
            return fail("refused: /arm_motion_allowed is still True "
                        "(publish False first so nothing commands the arm)")

        _, comm, _e = self.packet_handler.ping(self.port_handler, dxl_id)
        if comm != 0:
            return fail("no response to ping")

        temp = self._r1(dxl_id, ADDR_TEMPERATURE)
        limit = self._r2(dxl_id, ADDR_TORQUE_LIMIT)
        max_torque = self._r2(dxl_id, ADDR_MAX_TORQUE)
        pos = self._r2(dxl_id, ADDR_PRESENT_POSITION)
        if None in (temp, limit, max_torque, pos):
            return fail("a register read failed")

        log.warn(f"restore ID {dxl_id}: temp={temp}C torque_limit={limit} "
                 f"max_torque={max_torque} position={pos}")

        if max_torque == 0:
            return fail("EEPROM Max Torque is 0 (fix with Dynamixel Wizard)")

        if limit >= max_torque:
            response.success = True
            response.message = f"torque limit already {limit}, nothing to do"
            return response

        max_temp = int(self.get_parameter("recovery_max_temp_c").value)
        if temp > max_temp:
            return fail(f"still {temp}C (> {max_temp}C), let it cool")

        # hold where it is BEFORE torque comes back
        if not self._w2_verified(dxl_id, ADDR_GOAL_POSITION, pos):
            return fail("could not verify hold position; torque limit NOT restored")

        if not self._w2_verified(dxl_id, ADDR_TORQUE_LIMIT, max_torque):
            return fail("could not verify torque limit write")

        log.warn(f"restore ID {dxl_id}: holding at {pos} with torque limit {max_torque}")
        response.success = True
        response.message = f"held at {pos}, torque limit {max_torque}"
        return response

    # SHUTDOWN

    def destroy_node(self):

        self.get_logger().info("Disabling torque...")

        for dxl_id in JOINT_TO_ID.values():

            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
            )

        self.port_handler.closePort()

        super().destroy_node()


# Main

def main(args=None):

    rclpy.init(args=args)

    node = ArmDriver()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()