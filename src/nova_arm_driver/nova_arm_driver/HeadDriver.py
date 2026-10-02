#
## HeadDriver.py
#
#  Driver for the head pan/tilt Dynamixel motors (e.g. /dev/ttyUSB2):
#    - Head pan/tilt : AX-12A (Protocol 1.0), IDs 54, 55
#  Runs at 1,000,000 baud.
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


class DynamixelDriver:

    def __init__(
        self,
        device_name="/dev/ttyUSB2",
        baudrate=1000000,
        pan_id=54,
        tilt_id=55,
    ):

        self.pan_id = pan_id
        self.tilt_id = tilt_id

        self.port_handler = PortHandler(device_name)
        self.packet_handler = PacketHandler(1.0)      # AX-12A

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

        print("Pinging motors...")
        self._ping(self.packet_handler, self.pan_id, "AX pan", AX_MODEL)
        self._ping(self.packet_handler, self.tilt_id, "AX tilt", AX_MODEL)

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
        # Kept for backward compatibility
        return self._ping(self.packet_handler, dxl_id, "AX", AX_MODEL)

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

    def _read_ax(self, dxl_id, retries=3):
        """Present position in ticks, or None if it could not be read."""

        for _ in range(retries):
            position, comm_result, error = (
                self.packet_handler.read2ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    ADDR_PRESENT_POSITION,
                )
            )

            if comm_result == 0 and error == 0:
                return position

            time.sleep(0.01)

        return None

    def hold_current_ax(self, dxl_id, retries=5):
        """
        Write the present position as the goal and VERIFY it by reading the
        goal register back, so torque-on cannot move the motor to a stale goal.
        """

        for _ in range(retries):

            position = self._read_ax(dxl_id)

            if position is None:
                continue

            comm_result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_GOAL_POSITION,
                int(position),
            )

            if comm_result == 0 and error == 0:

                goal, comm_result, error = self.packet_handler.read2ByteTxRx(
                    self.port_handler,
                    dxl_id,
                    ADDR_GOAL_POSITION,
                )

                if comm_result == 0 and error == 0 and goal == int(position):
                    return

            time.sleep(0.01)

        raise RuntimeError(
            f"Could not verify hold position on AX ID={dxl_id}; "
            f"refusing to enable torque"
        )

    def enable(self):

        for i in (self.pan_id, self.tilt_id):
            self.hold_current_ax(i)
            self.enable_torque(i)

    def disable(self):

        self.disable_torque(self.pan_id)
        self.disable_torque(self.tilt_id)

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

    # Calibration
    def calibrate_zero(self):

        self.pan_zero = self._read_ax(self.pan_id)
        self.tilt_zero = self._read_ax(self.tilt_id)

        if self.pan_zero is None or self.tilt_zero is None:
            raise RuntimeError(
                "Cannot read pan/tilt position for zero calibration"
            )

        print()
        print("=== ZERO CALIBRATION ===")
        print(f"Pan Zero  : {self.pan_zero}")
        print(f"Tilt Zero : {self.tilt_zero}")
        print("========================")
        print()

    # Safe Limits
    PAN_MIN = -60
    PAN_MAX = 60

    TILT_MIN = -45
    TILT_MAX = 45

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

    # Diagnostics
    def startup_drift(self):
        """Ticks each motor has moved away from its calibrated zero (None = read failed)."""

        drift = {}

        pan = self._read_ax(self.pan_id)
        tilt = self._read_ax(self.tilt_id)

        drift["pan"] = None if pan is None else pan - self.pan_zero
        drift["tilt"] = None if tilt is None else tilt - self.tilt_zero

        return drift

    # Cleanup
    def close(self):

        self.port_handler.closePort()