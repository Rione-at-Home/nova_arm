#
## HeadDriver.py
#
#  Driver for the Dynamixel motors on ONE shared bus (e.g. /dev/ttyUSB2):
#    - Head pan/tilt : AX-12A       (Protocol 1.0), IDs 54, 55
#    - Cat head      : XM430-W210-T (Protocol 2.0), IDs 1, 2
#  Both run at 1,000,000 baud.
#

import time

from dynamixel_sdk import PortHandler, PacketHandler

# AX-12A control table (Protocol 1.0)
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

TICKS_PER_DEGREE = 1023.0 / 300.0
AX_MODEL = 12

# XM430-W210-T control table (Protocol 2.0)
XM_ADDR_OPERATING_MODE = 11
XM_ADDR_TORQUE_ENABLE = 64
XM_ADDR_PROFILE_ACCELERATION = 108
XM_ADDR_PROFILE_VELOCITY = 112
XM_ADDR_GOAL_POSITION = 116
XM_ADDR_PRESENT_POSITION = 132

XM_POSITION_MODE = 3
XM_TICKS_PER_DEGREE = 4096.0 / 360.0
XM_MAX_TICK = 4095
XM_MODEL = 1030


class DynamixelDriver:

    def __init__(
        self,
        device_name="/dev/ttyUSB2",
        baudrate=1000000,
        pan_id=54,
        tilt_id=55,
        cat_ids=(1, 2),
        use_cat=True,
        cat_profile_velocity=0,      # 0 = leave motor default (no extra limit)
        cat_profile_acceleration=0,  # 0 = leave motor default
    ):

        self.pan_id = pan_id
        self.tilt_id = tilt_id
        self.cat_ids = tuple(cat_ids)
        self.use_cat = use_cat
        self.cat_profile_velocity = cat_profile_velocity
        self.cat_profile_acceleration = cat_profile_acceleration

        # ONE port, TWO packet handlers (one per protocol version)
        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(1.0)      # AX-12A
        self.packet_handler_xm = PacketHandler(2.0)   # XM430

        if not self.port_handler.openPort():
            raise RuntimeError(
                f"Failed to open port {device_name}"
            )

        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(
                f"Failed to set baudrate {baudrate}"
            )

        print(f"Connected to {device_name}")

        self.pan_zero = 0
        self.tilt_zero = 0
        self.cat_zero = {i: 0 for i in self.cat_ids}

        self._last_warn = {}

        print("Pinging motors...")
        self._ping(self.packet_handler, self.pan_id, "AX pan", AX_MODEL)
        self._ping(self.packet_handler, self.tilt_id, "AX tilt", AX_MODEL)

        if self.use_cat:
            for i in self.cat_ids:
                ok = self._ping(
                    self.packet_handler_xm, i, f"XM cat{i}", XM_MODEL
                )
                if not ok:
                    raise RuntimeError(
                        f"Cat motor ID {i} did not answer on Protocol 2.0"
                    )

    # Communication Helpers
    def _ping(self, handler, dxl_id, label, expected_model):

        model, comm_result, error = handler.ping(
            self.port_handler,
            dxl_id
        )

        print(
            f"PING {label} ID={dxl_id} "
            f"MODEL={model} "
            f"COMM={comm_result} "
            f"ERROR={error}"
        )

        if comm_result == 0 and model != expected_model:
            print(
                f"  WARNING: expected model {expected_model}, got {model}"
            )

        return comm_result == 0

    def ping(self, dxl_id):
        # Kept for backward compatibility (AX motors)
        return self._ping(self.packet_handler, dxl_id, "AX", AX_MODEL)

    def _warn(self, key, text, period=1.0):
        # Print at most once per `period` seconds per key
        now = time.time()
        if now - self._last_warn.get(key, 0.0) >= period:
            self._last_warn[key] = now
            print(text)

    # AX-12A Torque
    def enable_torque(self, dxl_id):

        comm_result, error = (
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )
        )

        print(
            f"Enable Torque ID={dxl_id} "
            f"COMM={comm_result} "
            f"ERROR={error}"
        )

    def disable_torque(self, dxl_id):

        self.packet_handler.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

    # XM430 Torque
    def enable_torque_xm(self, dxl_id):
        """
        Safe start: set position mode, hold the CURRENT position as the goal,
        then enable torque so the motor does not jump to a stale goal.
        """

        # Operating mode can only be changed while torque is off
        self.packet_handler_xm.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            XM_ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

        self.packet_handler_xm.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            XM_ADDR_OPERATING_MODE,
            XM_POSITION_MODE,
        )

        if self.cat_profile_acceleration > 0:
            self.packet_handler_xm.write4ByteTxRx(
                self.port_handler,
                dxl_id,
                XM_ADDR_PROFILE_ACCELERATION,
                int(self.cat_profile_acceleration),
            )

        if self.cat_profile_velocity > 0:
            self.packet_handler_xm.write4ByteTxRx(
                self.port_handler,
                dxl_id,
                XM_ADDR_PROFILE_VELOCITY,
                int(self.cat_profile_velocity),
            )

        present = self.read_cat_position(dxl_id)

        if present is None:
            raise RuntimeError(
                f"Cannot read XM ID={dxl_id}; refusing to enable torque"
            )

        self.write_cat_position(dxl_id, present)

        comm_result, error = (
            self.packet_handler_xm.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                XM_ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )
        )

        print(
            f"Enable Torque XM ID={dxl_id} "
            f"COMM={comm_result} "
            f"ERROR={error}"
        )

    def disable_torque_xm(self, dxl_id):

        self.packet_handler_xm.write1ByteTxRx(
            self.port_handler,
            dxl_id,
            XM_ADDR_TORQUE_ENABLE,
            TORQUE_DISABLE,
        )

    def enable(self):

        self.enable_torque(self.pan_id)
        self.enable_torque(self.tilt_id)

        if self.use_cat:
            for i in self.cat_ids:
                self.enable_torque_xm(i)

    def disable(self):

        self.disable_torque(self.pan_id)
        self.disable_torque(self.tilt_id)

        if self.use_cat:
            for i in self.cat_ids:
                self.disable_torque_xm(i)

    # Raw Position Access (AX-12A)
    def read_position(self, dxl_id):

        position, comm_result, error = (
            self.packet_handler.read2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_POSITION,
            )
        )

        if comm_result != 0:
            print(
                f"Read Error ID={dxl_id} "
                f"COMM={comm_result}"
            )

        return position

    def write_position(self, dxl_id, position):

        comm_result, error = (
            self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_GOAL_POSITION,
                int(position),
            )
        )

    # Raw Position Access (XM430)
    def read_cat_position(self, dxl_id):
        """Returns ticks (0-4095) or None on failure."""

        position, comm_result, error = (
            self.packet_handler_xm.read4ByteTxRx(
                self.port_handler,
                dxl_id,
                XM_ADDR_PRESENT_POSITION,
            )
        )

        if comm_result != 0 or error != 0:
            self._warn(
                f"read{dxl_id}",
                f"Read Error XM ID={dxl_id} "
                f"COMM={comm_result} ERROR={error}"
            )
            return None

        return position

    def write_cat_position(self, dxl_id, position):

        position = max(0, min(XM_MAX_TICK, int(round(position))))

        comm_result, error = (
            self.packet_handler_xm.write4ByteTxRx(
                self.port_handler,
                dxl_id,
                XM_ADDR_GOAL_POSITION,
                position,
            )
        )

        if comm_result != 0 or error != 0:
            self._warn(
                f"write{dxl_id}",
                f"Write Error XM ID={dxl_id} "
                f"COMM={comm_result} ERROR={error}"
            )

    # Calibration
    def calibrate_zero(self):

        self.pan_zero = self.read_position(
            self.pan_id
        )

        self.tilt_zero = self.read_position(
            self.tilt_id
        )

        print()
        print("=== ZERO CALIBRATION ===")
        print(f"Pan Zero  : {self.pan_zero}")
        print(f"Tilt Zero : {self.tilt_zero}")

        if self.use_cat:
            for i in self.cat_ids:
                pos = self.read_cat_position(i)

                if pos is None:
                    raise RuntimeError(
                        f"Cannot calibrate cat motor ID {i}"
                    )

                self.cat_zero[i] = pos
                print(f"Cat{i} Zero : {pos}")

        print("========================")
        print()

    # Safe Limits
    PAN_MIN = -60
    PAN_MAX = 60

    TILT_MIN = -45
    TILT_MAX = 45

    # Cat joint limits in degrees from the startup position.
    # !! Conservative placeholders. Set these to the real mechanical range. !!
    CAT_MIN = {1: -45, 2: -45}
    CAT_MAX = {1: 45, 2: 45}

    # Pan Control
    def set_pan(self, angle):

        angle = max(
            self.PAN_MIN,
            min(self.PAN_MAX, angle)
        )

        position = int(
            self.pan_zero +
            angle * TICKS_PER_DEGREE
        )

        self.write_position(
            self.pan_id,
            position
        )

    def get_pan(self):

        position = self.read_position(
            self.pan_id
        )

        return (
            position - self.pan_zero
        ) / TICKS_PER_DEGREE

    # Tilt Control
    def set_tilt(self, angle):

        angle = max(
            self.TILT_MIN,
            min(self.TILT_MAX, angle)
        )

        position = int(
            self.tilt_zero +
            angle * TICKS_PER_DEGREE
        )

        self.write_position(
            self.tilt_id,
            position
        )

    def get_tilt(self):

        position = self.read_position(
            self.tilt_id
        )

        return (
            position - self.tilt_zero
        ) / TICKS_PER_DEGREE

    # Cat Control (joint = 1 or 2, angle in degrees from startup position)
    def set_cat(self, joint, angle):

        if not self.use_cat or joint not in self.cat_zero:
            return

        angle = max(
            self.CAT_MIN[joint],
            min(self.CAT_MAX[joint], angle)
        )

        position = (
            self.cat_zero[joint] +
            angle * XM_TICKS_PER_DEGREE
        )

        self.write_cat_position(joint, position)

    def get_cat(self, joint):

        position = self.read_cat_position(joint)

        if position is None:
            return None

        return (
            position - self.cat_zero[joint]
        ) / XM_TICKS_PER_DEGREE

    # Cleanup
    def close(self):

        self.port_handler.closePort()