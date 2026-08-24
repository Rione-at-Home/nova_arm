#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import GroupSyncWrite
from dynamixel_sdk import GroupSyncRead




ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# COMMUNICATION SETTINGS

PROTOCOL_VERSION = 1.0

PORT_NAME = "/dev/ttyUSB0"
BAUDRATE = 1000000

# CONTROL TABLE

# Right Side Arm
RIGHT_JOINT_TO_ID = {
    "right_joint1": 1,
    "right_joint2": 2,
    "right_joint3": 3,
    "right_joint4": 4,
    "right_joint5": 5,
    "right_gripper": 6,
}

# Left Side Arm
LEFT_JOINT_TO_ID = {
    "left_joint1": 11,
    "left_joint2": 12,
    "left_joint3": 13,
    "left_joint4": 14,
    "left_joint5": 15,
    "left_gripper": 16,
}

JOINT_TO_ID = {**RIGHT_JOINT_TO_ID, **LEFT_JOINT_TO_ID}

# Fixed ordering used for /joint_states so consumers always get a
# consistent, predictable array layout across both arms.
JOINT_ORDER = list(JOINT_TO_ID.keys())

# How often to poll actual motor positions and publish /joint_states.
# Matched to the 50 Hz control loop used elsewhere in the package.
FEEDBACK_PERIOD_SEC = 0.02

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

        self.port_handler = PortHandler(PORT_NAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # Sync writer: batches goal positions for every motor into a
        # single broadcast packet so both arms move on the same tick,
        # instead of each motor being written one at a time.
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            2,  # goal position is a 2-byte value
        )

        # Sync reader: the read-side counterpart to the sync writer.
        # Polls PRESENT_POSITION on every motor in one broadcast
        # transaction, so both arms' actual state is sampled at
        # (as close as possible to) the same instant.
        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            2,
        )

        for dxl_id in JOINT_TO_ID.values():
            if not self.group_sync_read.addParam(dxl_id):
                self.get_logger().error(
                    f"Failed to add ID {dxl_id} to sync read"
                )

        # Last-known position per joint (radians), used to fill in
        # /joint_states if a given motor doesn't answer on a given
        # poll, so a single dropout doesn't collapse the whole message.
        self.last_known_positions = {
            joint_name: 0.0 for joint_name in JOINT_TO_ID
        }

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

        # FEEDBACK

        # /joint_states carries the arm's ACTUAL measured position,
        # as opposed to /arm_command which is only ever what was
        # asked for. Anything that needs to know where the arm
        # really is (pose saving, monitoring, future collision
        # checks) should subscribe here, not to /arm_command.
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.feedback_timer = self.create_timer(
            FEEDBACK_PERIOD_SEC,
            self.read_callback,
        )

    # JOINT CALLBACK
    def command_callback(self, msg):

        # Build up one sync-write packet covering every joint in this
        # message, then send it as a single broadcast transaction so
        # all motors (both arms) receive their goal at the same time.
        queued = []

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

            # Dynamixel SDK wants the 2-byte goal as a little-endian
            # byte array for sync write.
            param_goal = [
                goal & 0xFF,
                (goal >> 8) & 0xFF,
            ]

            add_ok = self.group_sync_write.addParam(
                dxl_id,
                bytes(param_goal),
            )

            if not add_ok:
                self.get_logger().error(
                    f"Failed to queue sync write for ID {dxl_id}"
                )
                continue

            queued.append((joint_name, dxl_id, position_rad, goal))

        if not queued:
            return

        dxl_comm_result = self.group_sync_write.txPacket()

        # Always clear queued params, even on failure, so a bad
        # transaction doesn't leak stale goals into the next cycle.
        self.group_sync_write.clearParam()

        if dxl_comm_result != 0:
            self.get_logger().error(
                "Sync write failed: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            return

        for joint_name, dxl_id, position_rad, goal in queued:
            self.get_logger().info(
                f"{joint_name} (ID {dxl_id}) -> "
                f"{position_rad:.2f} rad "
                f"({goal})"
            )

    # FEEDBACK CALLBACK
    def read_callback(self):

        dxl_comm_result = self.group_sync_read.txRxPacket()

        if dxl_comm_result != 0:
            self.get_logger().error(
                "Sync read failed: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )
            # Don't publish stale/garbage data on a failed poll;
            # last_known_positions will be reused next cycle.
            return

        positions = []

        for joint_name in JOINT_ORDER:

            dxl_id = JOINT_TO_ID[joint_name]

            if self.group_sync_read.isAvailable(
                    dxl_id, ADDR_PRESENT_POSITION, 2):

                raw = self.group_sync_read.getData(
                    dxl_id, ADDR_PRESENT_POSITION, 2
                )

                rad = dxl_to_rad(raw)
                self.last_known_positions[joint_name] = rad

            else:
                # This motor didn't answer this cycle (bus noise,
                # timing, etc). Fall back to its last known value
                # rather than dropping it from the message, which
                # would desync /joint_states' array length from
                # JOINT_ORDER.
                self.get_logger().warn(
                    f"No feedback for ID {dxl_id} "
                    f"({joint_name}) this cycle"
                )
                rad = self.last_known_positions[joint_name]

            positions.append(rad)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_ORDER)
        msg.position = positions

        self.joint_state_pub.publish(msg)

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