#!/usr/bin/env python3
"""
ArmDriver node: coordinates two independent DYNAMIXEL buses (one per
physical arm, each on its own USB2Dynamixel) behind the same
/arm_command, /arm_speed, /joint_states topics used previously with the
single-bus OpenCR setup. External topic contracts are unchanged -- this
is an internal refactor from one shared bus to two independent ones.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Int32

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

# Fixed ordering used for /joint_states so consumers always get a
# consistent, predictable array layout across both arms.
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
        #
        # Ports are declared as parameters (not hardcoded constants) so
        # they can be overridden per-machine via launch file / YAML
        # without touching code -- useful now that there are two ports
        # instead of one, and their /dev names depend on the udev rules
        # set up on whatever machine is running this node.
        self.declare_parameter("right_port", DEFAULT_RIGHT_PORT)
        self.declare_parameter("left_port", DEFAULT_LEFT_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)

        right_port = self.get_parameter("right_port").value
        left_port = self.get_parameter("left_port").value
        baudrate = self.get_parameter("baudrate").value

        # ARM BUSES
        #
        # One ArmBus per physical arm. Each owns its own port handler,
        # packet handler, and sync-write buffer -- there is no shared
        # bus state between them, matching the isolated per-arm
        # electrical setup (independent USB2Dynamixel + SMPS2Dynamixel
        # + power supply per arm).
        self.right_bus = ArmBus(
            "right", right_port, RIGHT_JOINT_TO_ID, baudrate,
            self.get_logger(),
        )
        self.left_bus = ArmBus(
            "left", left_port, LEFT_JOINT_TO_ID, baudrate,
            self.get_logger(),
        )
        self.buses = [self.right_bus, self.left_bus]

        # Lookup from joint name -> owning bus, used to route incoming
        # commands to the correct arm without the node needing to know
        # ahead of time which arm a given joint belongs to.
        self.joint_to_bus = {}
        for bus in self.buses:
            for joint_name in bus.joint_to_id:
                self.joint_to_bus[joint_name] = bus

        # CONNECT
        #
        # Each bus connects independently. A failure on one arm's port
        # does not prevent the other arm from coming up -- it's logged
        # and that bus is left disconnected rather than raising and
        # killing the whole node.
        for bus in self.buses:
            try:
                bus.connect()
            except RuntimeError as exc:
                self.get_logger().error(str(exc))

        # ENABLE TORQUE
        for bus in self.buses:
            bus.enable_torque()

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
        # /joint_states carries the arm's ACTUAL measured position, as
        # opposed to /arm_command which is only ever what was asked
        # for. Anything that needs to know where the arm really is
        # (pose saving, monitoring, future collision checks) should
        # subscribe here, not to /arm_command.
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.feedback_timer = self.create_timer(
            FEEDBACK_PERIOD_SEC,
            self.read_callback,
        )

    # COMMAND CALLBACK
    def command_callback(self, msg):

        # Route each joint's goal to its owning bus's sync-write
        # buffer. Both arms' packets get flushed back-to-back at the
        # end of this callback, on the same node tick -- this keeps
        # both arms moving as close to the same instant as possible
        # without needing a separate coordinator node/topic hop in
        # between.
        queued_any = {bus.name: False for bus in self.buses}

        for joint_name, position_rad in zip(msg.name, msg.position):

            bus = self.joint_to_bus.get(joint_name)

            if bus is None:
                self.get_logger().warn(
                    f"Unknown joint '{joint_name}'"
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

        # Each bus reads only its own joints. A comm failure on one
        # arm's port does not block feedback for the other arm --
        # ArmBus.read_positions() already falls back to last-known
        # position per joint on a bad read.
        merged_positions = {}

        for bus in self.buses:
            merged_positions.update(bus.read_positions())

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_ORDER)
        msg.position = [merged_positions[name] for name in JOINT_ORDER]

        self.joint_state_pub.publish(msg)

    # SPEED CALLBACK
    def speed_callback(self, msg):

        speed = int(msg.data / 100.0 * 1023)
        speed = max(20, min(speed, 1023))

        for bus in self.buses:
            bus.set_speed(speed)

        self.get_logger().info(f"Speed set to {msg.data}%")

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
        rclpy.shutdown()


if __name__ == "__main__":
    main()