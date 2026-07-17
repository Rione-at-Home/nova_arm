#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler




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

    # JOINT CALLBACK
    def command_callback(self, msg):

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