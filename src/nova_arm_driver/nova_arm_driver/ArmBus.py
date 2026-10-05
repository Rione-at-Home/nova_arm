#!/usr/bin/env python3
"""
ArmBus wraps a single DYNAMIXEL serial bus (one USB2Dynamixel + one arm's
worth of AX-12A servos) behind a small, self-contained interface.

Each physical arm gets its own ArmBus instance, its own serial port, its
own PortHandler/PacketHandler, and its own GroupSyncWrite -- there is no
sharing of bus state between arms.

[HEALTH] additions (everything else behaves as before):
  * A missing / unplugged / unpowered arm never raises. connect() returns
    False, every I/O method becomes a no-op while the bus is down, and a
    serial error at runtime marks the bus down instead of propagating.
    NOTE: dynamixel_sdk's openPort() *raises* serial.SerialException when the
    device node does not exist (it does not return False), so connect()
    catches Exception, not just RuntimeError.
  * Fresh PortHandler/GroupSyncWrite on every connect() so reconnecting after
    a failed open / unplug is reliable.
  * read_all_status(): one block read per servo (Max Torque .. Temperature)
    for the health monitor. Also re-enables torque on a servo that answers
    again after having been silent (arm power-cycled) and has Torque Enable 0.
  * enable_servo(): optionally writes Goal = Present Position BEFORE torque
    comes on, and re-applies the last commanded speed, so a (re)connected arm
    never jumps to a stale goal or moves at full speed.
  * restore_torque_limit(): the "hold here, then restore Torque Limit =
    Max Torque" recovery, every write verified by read-back.
  * Backoff: if no servo on the bus answers, polling drops to a single ping per
    status cycle so a dead arm cannot starve the other arm's feedback loop.
"""

import math
import time

from dynamixel_sdk import GroupSyncWrite
from dynamixel_sdk import PacketHandler
from dynamixel_sdk import PortHandler


ADDR_MAX_TORQUE = 14          # EEPROM, 2 bytes
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_TORQUE_LIMIT = 34        # RAM, 2 bytes
ADDR_PRESENT_POSITION = 36
ADDR_TEMPERATURE = 43

# One block read, addresses 14..43 (Max Torque ... Present Temperature)
STATUS_BLOCK_ADDR = 14
STATUS_BLOCK_LEN = 30

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
        Label used for logging and topic names ("right", "left").
    port_name : str
        Serial device path. Should be a udev-mapped persistent symlink
        (e.g. "/dev/dxl_right"), not a raw /dev/ttyUSB* path.
    joint_to_id : dict[str, int]
        Mapping of joint name -> DYNAMIXEL ID for this arm only.
    baudrate : int
        Serial baud rate, must match what the servos are configured for.
    logger : rclpy logger
        Passed in from the node.
    hold_on_enable : bool
        Write Goal = Present Position before enabling torque on a servo.
    """

    def __init__(self, name, port_name, joint_to_id, baudrate, logger,
                 hold_on_enable=True):

        self.name = name
        self.port_name = port_name
        self.joint_to_id = joint_to_id
        self.id_to_joint = {v: k for k, v in joint_to_id.items()}
        self.baudrate = baudrate
        self.logger = logger
        self.hold_on_enable = hold_on_enable

        # Last-known position per joint (radians). Used to fill in
        # /joint_states when a motor doesn't answer on a given poll.
        self.last_known_positions = {j: 0.0 for j in joint_to_id}
        # True once a joint has been read successfully at least once, so a
        # never-seen joint is not reported as being at 0.0 rad.
        self.have_position = {j: False for j in joint_to_id}

        self.connected = False
        self.last_error = ""
        self.connect_attempts = 0
        self.speed_value = None            # last commanded raw speed
        self.servo_alive = {i: False for i in joint_to_id.values()}
        self._all_dead = False             # nothing on the bus answered

        self._has_queued_write = False
        self._build_handlers()

    # ------------------------------------------------------------------
    # internals

    def _build_handlers(self):
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            GOAL_POSITION_LEN,
        )
        self._has_queued_write = False

    def _close_quiet(self):
        try:
            self.port_handler.closePort()
        except Exception:  # noqa: BLE001  (ser may be None / already closed)
            pass

    def _mark_down(self, exc):
        """A serial-level failure: treat the whole bus as gone."""
        if self.connected:
            self.logger.error(
                f"[{self.name}] Lost {self.port_name}: "
                f"{type(exc).__name__}: {exc}"
            )
        self.connected = False
        self.last_error = f"{type(exc).__name__}: {exc}"
        self._has_queued_write = False
        for i in self.servo_alive:
            self.servo_alive[i] = False
        self._close_quiet()

    def _call(self, fn, *args):
        """
        Run one SDK transaction. Returns the SDK's result tuple, or None when
        there is no usable answer (bus down, corrupted packet, serial error).
        A serial-level exception marks the bus down.
        """
        if not self.connected:
            return None
        try:
            return fn(self.port_handler, *args)
        except IndexError:
            # dynamixel_sdk bug: a corrupted/truncated status packet can
            # report COMM_SUCCESS with a short payload -> IndexError inside
            # the SDK. Same as any other comm failure for this cycle.
            return None
        except Exception as exc:  # noqa: BLE001
            self._mark_down(exc)
            return None

    def _ping(self, dxl_id):
        r = self._call(self.packet_handler.ping, dxl_id)
        return r is not None and r[1] == 0

    def _r1(self, dxl_id, addr):
        r = self._call(self.packet_handler.read1ByteTxRx, dxl_id, addr)
        return r[0] if r is not None and r[1] == 0 else None

    def _r2(self, dxl_id, addr):
        r = self._call(self.packet_handler.read2ByteTxRx, dxl_id, addr)
        return r[0] if r is not None and r[1] == 0 else None

    def _w1(self, dxl_id, addr, value):
        """Returns (comm_ok, hw_error_byte). The error byte carries alarm
        flags and is non-zero on every packet while an alarm is active, so it
        is reported, not treated as write failure."""
        r = self._call(self.packet_handler.write1ByteTxRx, dxl_id, addr, int(value))
        return (r is not None and r[0] == 0), (r[1] if r is not None else 0)

    def _w2(self, dxl_id, addr, value):
        r = self._call(self.packet_handler.write2ByteTxRx, dxl_id, addr, int(value))
        return (r is not None and r[0] == 0), (r[1] if r is not None else 0)

    def _w2_verified(self, dxl_id, addr, value, retries=5):
        for _ in range(retries):
            ok, _err = self._w2(dxl_id, addr, value)
            if ok and self._r2(dxl_id, addr) == int(value):
                return True
            if not self.connected:
                return False
            time.sleep(0.02)
        return False

    # ------------------------------------------------------------------
    # CONNECTION

    def connect(self):
        """
        Try to open the serial port. Returns True on success, False if the
        arm's adapter is not there. Never raises.
        """
        self._close_quiet()
        self._build_handlers()
        self.connect_attempts += 1

        try:
            if not self.port_handler.openPort():
                raise RuntimeError("openPort() failed")
            if not self.port_handler.setBaudRate(self.baudrate):
                raise RuntimeError("setBaudRate() failed")
        except Exception as exc:  # noqa: BLE001  (SerialException, OSError...)
            self.connected = False
            msg = f"{type(exc).__name__}: {exc}"
            if msg != self.last_error:      # log each distinct failure once
                self.logger.error(
                    f"[{self.name}] Cannot open {self.port_name}: {msg}"
                )
            self.last_error = msg
            self._close_quiet()
            return False

        self.connected = True
        self.last_error = ""
        self._all_dead = False
        self.logger.info(
            f"[{self.name}] Connected to DYNAMIXELs on {self.port_name}"
        )
        return True

    def close(self):
        """Close the serial port. Safe to call even if never connected."""
        self.connected = False
        self._close_quiet()

    def info(self):
        return {
            "connected": self.connected,
            "port": self.port_name,
            "connect_attempts": self.connect_attempts,
            "last_error": self.last_error,
        }

    # TORQUE

    def enable_servo(self, dxl_id):
        """
        Hold at the present position (if hold_on_enable), re-apply the last
        commanded speed, then enable torque. Returns True on success.
        """
        if not self.connected:
            return False

        if self.hold_on_enable:
            pos = self._r2(dxl_id, ADDR_PRESENT_POSITION)
            if pos is None:
                self.logger.warning(
                    f"[{self.name}] ID {dxl_id}: no position reading, torque "
                    f"NOT enabled (will retry when it answers)"
                )
                return False
            if not self._w2_verified(dxl_id, ADDR_GOAL_POSITION, pos):
                self.logger.warning(
                    f"[{self.name}] ID {dxl_id}: could not set hold position, "
                    f"torque NOT enabled"
                )
                return False

        if self.speed_value is not None:
            self._w2(dxl_id, ADDR_MOVING_SPEED, self.speed_value)

        ok, _err = self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if not ok:
            self.logger.error(
                f"[{self.name}] Communication failed enabling torque "
                f"on ID {dxl_id}"
            )
            return False

        self.logger.info(f"[{self.name}] Torque enabled on ID {dxl_id}")
        return True

    def enable_torque(self):
        """Enable torque on every servo on this bus. Logs, does not raise."""
        if not self.connected:
            return
        for dxl_id in self.joint_to_id.values():
            self.enable_servo(dxl_id)

    def disable_torque(self):
        """Disable torque on every servo on this bus. Does not raise."""
        if not self.connected:
            return
        for dxl_id in self.joint_to_id.values():
            self._w1(dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    # COMMAND (WRITE)

    def queue_goal(self, joint_name, position_rad):
        """
        Queue a goal position for one joint into this bus's sync-write
        buffer. Returns True if queued, False otherwise (unknown joint,
        bus down, or SDK-level failure).
        """

        if not self.connected or joint_name not in self.joint_to_id:
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
        Send the queued sync-write packet, if anything was queued, then clear
        the buffer either way. No-op if nothing was queued.
        """

        if not self._has_queued_write:
            return

        try:
            dxl_comm_result = self.group_sync_write.txPacket()
        except Exception as exc:  # noqa: BLE001
            self.group_sync_write.clearParam()
            self._has_queued_write = False
            self._mark_down(exc)
            return

        self.group_sync_write.clearParam()
        self._has_queued_write = False

        if dxl_comm_result != 0:
            self.logger.error(
                f"[{self.name}] Sync write failed: "
                f"{self.packet_handler.getTxRxResult(dxl_comm_result)}"
            )

    # FEEDBACK (READ)

    def read_positions(self, include_unknown=False):
        """
        Poll present position for every joint on this bus.

        Returns dict[joint_name -> radians] for joints that have a real
        reading (current, or last known if this cycle's read failed). Joints
        never read successfully are left out unless include_unknown=True
        (then they are 0.0). When the bus is down or nothing answers, no bus
        traffic is generated and only last-known values are returned.
        """

        if self.connected and not self._all_dead:
            failures = 0
            for joint_name, dxl_id in self.joint_to_id.items():
                r = self._call(
                    self.packet_handler.read2ByteTxRx,
                    dxl_id,
                    ADDR_PRESENT_POSITION,
                )
                if not self.connected:
                    break
                if r is None or r[1] != 0:
                    failures += 1
                    self.logger.warning(
                        f"[{self.name}] No feedback for ID {dxl_id} "
                        f"({joint_name}) this cycle",
                        throttle_duration_sec=5.0,
                    )
                    continue
                rad = dxl_to_rad(r[0])
                self.last_known_positions[joint_name] = rad
                self.have_position[joint_name] = True
            if self.connected and failures == len(self.joint_to_id):
                self._all_dead = True
                self.logger.warning(
                    f"[{self.name}] No servo answers on {self.port_name} "
                    f"(arm powered off / cable?) - polling slowed"
                )

        return {
            j: self.last_known_positions[j]
            for j in self.joint_to_id
            if self.have_position[j] or include_unknown
        }

    def read_all_status(self):
        """
        Health poll. Returns dict[dxl_id -> dict | None] (None = no valid
        response) for every servo on this bus; never raises.
        """

        out = {i: None for i in self.joint_to_id.values()}
        if not self.connected:
            return out

        ids = list(self.joint_to_id.values())

        if self._all_dead:
            # one cheap probe instead of N timeouts
            if not self._ping(ids[0]):
                return out
            self._all_dead = False

        for dxl_id in ids:
            d = self._read_servo(dxl_id)
            if not self.connected:
                break
            out[dxl_id] = d
            was_alive = self.servo_alive[dxl_id]
            self.servo_alive[dxl_id] = d is not None
            if d is None:
                continue
            self._all_dead = False
            # Servo came back (arm power-cycled): its Torque Enable is 0 again.
            if not was_alive and d["torque_enable"] == 0:
                self.logger.warning(
                    f"[{self.name}] ID {dxl_id} answers again with torque off; "
                    f"re-enabling"
                )
                if self.enable_servo(dxl_id):
                    d["torque_enable"] = 1

        if self.connected and all(v is None for v in out.values()):
            self._all_dead = True

        return out

    def _read_servo(self, dxl_id):
        r = self._call(
            self.packet_handler.readTxRx,
            dxl_id,
            STATUS_BLOCK_ADDR,
            STATUS_BLOCK_LEN,
        )
        if r is None:
            return None
        blk, comm, err = r
        if comm != 0 or len(blk) != STATUS_BLOCK_LEN:
            return None

        def w(off):  # 16-bit little endian, offset relative to address 14
            return blk[off] | (blk[off + 1] << 8)

        return {
            "err": err,                  # hardware error flags (status packet)
            "torque_enable": blk[10],    # addr 24
            "cw_margin": blk[12], "ccw_margin": blk[13],
            "cw_slope": blk[14], "ccw_slope": blk[15],
            "goal": w(16),               # addr 30
            "moving_speed": w(18),       # addr 32
            "torque_limit": w(20),       # addr 34
            "position": w(22),           # addr 36
            "load": w(26),               # addr 40
            "voltage": blk[28],          # addr 42, 0.1 V units
            "temp": blk[29],             # addr 43, deg C
            "max_torque": w(0),          # addr 14
        }

    # RECOVERY

    def restore_torque_limit(self, dxl_id, max_temp_c):
        """
        Hold where the servo is, THEN restore Torque Limit = Max Torque, each
        write verified. Returns (success, message).
        """

        if not self.connected:
            return False, f"bus {self.name} not connected"

        if not self._ping(dxl_id):
            return False, "no response to ping"

        temp = self._r1(dxl_id, ADDR_TEMPERATURE)
        limit = self._r2(dxl_id, ADDR_TORQUE_LIMIT)
        max_torque = self._r2(dxl_id, ADDR_MAX_TORQUE)
        pos = self._r2(dxl_id, ADDR_PRESENT_POSITION)
        if None in (temp, limit, max_torque, pos):
            return False, "a register read failed"

        self.logger.warning(
            f"[{self.name}] restore ID {dxl_id}: temp={temp}C "
            f"torque_limit={limit} max_torque={max_torque} position={pos}"
        )

        if max_torque == 0:
            return False, "EEPROM Max Torque is 0 (fix with Dynamixel Wizard)"

        if limit >= max_torque:
            return True, f"torque limit already {limit}, nothing to do"

        if temp > max_temp_c:
            return False, f"still {temp}C (> {max_temp_c}C), let it cool"

        # hold where it is BEFORE torque comes back
        if not self._w2_verified(dxl_id, ADDR_GOAL_POSITION, pos):
            return False, "could not verify hold position; torque limit NOT restored"

        if not self._w2_verified(dxl_id, ADDR_TORQUE_LIMIT, max_torque):
            return False, "could not verify torque limit write"

        self.logger.warning(
            f"[{self.name}] restore ID {dxl_id}: holding at {pos} "
            f"with torque limit {max_torque}"
        )
        return True, f"held at {pos}, torque limit {max_torque}"

    # SPEED

    def set_speed(self, speed_value):
        """
        Set moving speed on every servo on this bus. The value is remembered
        and re-applied whenever a servo is (re)enabled, even if the bus is
        down right now. Does not raise.
        """

        self.speed_value = int(speed_value)

        if not self.connected:
            return

        for dxl_id in self.joint_to_id.values():
            ok, _err = self._w2(dxl_id, ADDR_MOVING_SPEED, self.speed_value)
            if not ok:
                self.logger.warning(
                    f"[{self.name}] Failed to set speed on ID {dxl_id}"
                )