#!/usr/bin/env python3
"""
ArmBus wraps a single DYNAMIXEL serial bus (one USB2Dynamixel + one arm's
worth of AX-12A servos) behind a small, self-contained interface.

Each physical arm gets its own ArmBus instance, its own serial port, its
own PortHandler/PacketHandler, and its own GroupSyncWrite -- there is no
sharing of bus state between arms. This mirrors the electrical setup where
each arm has an independent USB2Dynamixel + SMPS2Dynamixel + power supply
chain (see the electrical-setup issue for wiring details).
"""

import math

from dynamixel_sdk import PortHandler
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import GroupSyncWrite


ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 1.0

GOAL_POSITION_LEN = 2  # bytes


def rad_to_dxl(rad):

    deg = math.degrees(rad)

    value = int(((deg + 150.0) / 300.0) * 1023.0)

    return max(0, min(1023, value))


def dxl_to_rad(value):

    deg = value * 300.0 / 1023.0 - 150.0

    return math.radians(deg)


class ArmBus:
    """
    One independent DYNAMIXEL bus: one serial port, one set of joints.

    Parameters
    ----------
    name : str
        Human-readable label used only for logging (e.g. "right", "left").
    port_name : str
        Serial device path. Should be a udev-mapped persistent symlink
        (e.g. "/dev/dxl_right"), not a raw /dev/ttyUSB* path, so arm
        identity survives reconnects/reboots.
    joint_to_id : dict[str, int]
        Mapping of joint name -> DYNAMIXEL ID for this arm only.
    baudrate : int
        Serial baud rate, must match what the servos are configured for.
    logger : rclpy logger
        Passed in from the node so ArmBus can log through the same
        node's logger rather than needing its own.
    """

    def __init__(self, name, port_name, joint_to_id, baudrate, logger):

        self.name = name
        self.port_name = port_name
        self.joint_to_id = joint_to_id
        self.id_to_joint = {v: k for k, v in joint_to_id.items()}
        self.baudrate = baudrate
        self.logger = logger

        self.port_handler = PortHandler(port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            GOAL_POSITION_LEN,
        )

        # Last-known position per joint (radians). Used to fill in
        # /joint_states when a given motor doesn't answer on a given
        # poll, so a single dropout on this arm doesn't collapse the
        # whole feedback message.
        self.last_known_positions = {
            joint_name: 0.0 for joint_name in joint_to_id
        }

        # Tracks whether this bus has anything queued for the current
        # command cycle, so the node only calls txPacket() on buses
        # that actually received a goal this tick.
        self._has_queued_write = False

    # CONNECTION

    def connect(self):
        """Open the serial port and set the baud rate. Raises RuntimeError on failure."""

        if not self.port_handler.openPort():
            raise RuntimeError(
                f"[{self.name}] Failed to open port {self.port_name}"
            )

        if not self.port_handler.setBaudRate(self.baudrate):
            raise RuntimeError(
                f"[{self.name}] Failed to set baudrate on {self.port_name}"
            )

        self.logger.info(
            f"[{self.name}] Connected to DYNAMIXELs on {self.port_name}"
        )

    def close(self):
        """Close the serial port. Safe to call even if never connected."""

        try:
            self.port_handler.closePort()
        except Exception as exc:  # pragma: no cover - defensive
            self.logger.warn(
                f"[{self.name}] Error closing {self.port_name}: {exc}"
            )

    # TORQUE

    def enable_torque(self):
        """Enable torque on every servo on this bus. Logs per-ID failures, does not raise."""

        for dxl_id in self.joint_to_id.values():

            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )

            if dxl_comm_result != 0:
                self.logger.error(
                    f"[{self.name}] Communication failed enabling torque "
                    f"on ID {dxl_id}"
                )
            elif dxl_error != 0:
                self.logger.error(
                    f"[{self.name}] DYNAMIXEL error enabling torque "
                    f"on ID {dxl_id}"
                )
            else:
                self.logger.info(
                    f"[{self.name}] Torque enabled on ID {dxl_id}"
                )

    def disable_torque(self):
        """Disable torque on every servo on this bus. Logs per-ID failures, does not raise."""

        for dxl_id in self.joint_to_id.values():

            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
            )

    # COMMAND (WRITE)

    def queue_goal(self, joint_name, position_rad):
        """
        Queue a goal position for one joint into this bus's sync-write
        buffer. Returns True if queued successfully, False otherwise
        (e.g. unknown joint for this arm, or SDK-level failure).
        """

        if joint_name not in self.joint_to_id:
            return False

        dxl_id = self.joint_to_id[joint_name]
        goal = rad_to_dxl(position_rad)

        param_goal = [
            goal & 0xFF,
            (goal >> 8) & 0xFF,
        ]

        add_ok = self.group_sync_write.addParam(dxl_id, bytes(param_goal))

        if not add_ok:
            self.logger.error(
                f"[{self.name}] Failed to queue sync write for ID {dxl_id}"
            )
            return False

        self._has_queued_write = True
        return True

    def flush_writes(self):
        """
        Send the queued sync-write packet for this bus, if anything was
        queued this cycle, then clear the buffer either way. No-op if
        nothing was queued (avoids sending empty packets every tick).
        """

        if not self._has_queued_write:
            return

        dxl_comm_result = self.group_sync_write.txPacket()

        self.group_sync_write.clearParam()
        self._has_queued_write = False

        if dxl_comm_result != 0:
            self.logger.error(
                f"[{self.name}] Sync write failed: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )

    # FEEDBACK (READ)

    def read_positions(self):
        """
        Poll present position for every joint on this bus.

        Returns dict[joint_name -> radians]. On a comm failure or a
        corrupted status packet for a given joint, falls back to that
        joint's last known position rather than dropping it from the
        result, so one bad read doesn't blank out the whole arm.
        """

        positions = {}

        for joint_name, dxl_id in self.joint_to_id.items():

            try:
                raw, dxl_comm_result, dxl_error = \
                    self.packet_handler.read2ByteTxRx(
                        self.port_handler,
                        dxl_id,
                        ADDR_PRESENT_POSITION,
                    )
            except IndexError:
                # dynamixel_sdk bug: a corrupted/truncated status packet
                # can report COMM_SUCCESS while the payload is short,
                # causing an IndexError inside the SDK itself. Treat it
                # the same as any other comm failure for this cycle.
                self.logger.warn(
                    f"[{self.name}] Corrupted packet from ID {dxl_id} "
                    f"({joint_name}) this cycle"
                )
                positions[joint_name] = self.last_known_positions[joint_name]
                continue

            if dxl_comm_result != 0 or dxl_error != 0:
                self.logger.warn(
                    f"[{self.name}] No feedback for ID {dxl_id} "
                    f"({joint_name}) this cycle"
                )
                positions[joint_name] = self.last_known_positions[joint_name]
            else:
                rad = dxl_to_rad(raw)
                self.last_known_positions[joint_name] = rad
                positions[joint_name] = rad

        return positions

    # SPEED

    def set_speed(self, speed_value):
        """Set moving speed on every servo on this bus. Logs per-ID failures, does not raise."""

        for dxl_id in self.joint_to_id.values():

            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_MOVING_SPEED,
                speed_value,
            )

            if dxl_comm_result != 0 or dxl_error != 0:
                self.logger.warn(
                    f"[{self.name}] Failed to set speed on ID {dxl_id}"
                )