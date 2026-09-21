#!/usr/bin/env python3
"""
restore_torque_limit.py

Recovers an AX-12A whose Torque Limit (RAM, address 34) was set to 0 by an
alarm shutdown (overload / overheating).

For each selected servo, safely:
  1. reads temperature, torque limit, max torque and present position
  2. refuses if it is still hot (default limit 50 C) unless --force
  3. writes Goal Position = Present Position and VERIFIES it, so the servo
     holds where it is instead of snapping to an old goal
  4. writes Torque Limit = Max Torque (EEPROM, normally 1023) and VERIFIES it

SUPPORT THE ARM before running. ArmDriver must not be using the port.

Usage:
    python3 restore_torque_limit.py --port /dev/ttyUSB1            # ID 1 only
    python3 restore_torque_limit.py --port /dev/ttyUSB1 --ids 1 3
"""

import argparse
import sys
import time

import serial
from dynamixel_sdk import PacketHandler, PortHandler

ADDR_MAX_TORQUE = 14
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_TORQUE_LIMIT = 34
ADDR_PRESENT_POSITION = 36
ADDR_TEMPERATURE = 43


def r1(ph, port, i, addr):
    v, c, e = ph.read1ByteTxRx(port, i, addr)
    return v if c == 0 else None


def r2(ph, port, i, addr):
    v, c, e = ph.read2ByteTxRx(port, i, addr)
    return v if c == 0 else None


def w2_verified(ph, port, i, addr, value, retries=5):
    for _ in range(retries):
        c, e = ph.write2ByteTxRx(port, i, addr, int(value))
        if c == 0:
            back = r2(ph, port, i, addr)
            if back == int(value):
                return True
        time.sleep(0.02)
    return False


def _main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1])
    ap.add_argument("--max-temp", type=int, default=50,
                    help="refuse to restore above this temperature (C)")
    ap.add_argument("--force", action="store_true",
                    help="ignore the temperature check")
    args = ap.parse_args()

    port = PortHandler(args.port)
    ph = PacketHandler(1.0)

    if not port.openPort():
        sys.exit(f"Failed to open {args.port}")
    if not port.setBaudRate(args.baud):
        sys.exit("Failed to set baudrate")

    plan = []

    for i in args.ids:
        _, comm, _e = ph.ping(port, i)
        if comm != 0:
            print(f"ID {i}: no response, skipping")
            continue

        temp = r1(ph, port, i, ADDR_TEMPERATURE)
        limit = r2(ph, port, i, ADDR_TORQUE_LIMIT)
        max_torque = r2(ph, port, i, ADDR_MAX_TORQUE)
        pos = r2(ph, port, i, ADDR_PRESENT_POSITION)
        torque_on = r1(ph, port, i, ADDR_TORQUE_ENABLE)

        print(f"ID {i}: temp={temp} C  torque_limit={limit}  max_torque={max_torque}  "
              f"position={pos} ticks  torque_enable={torque_on}")

        if None in (temp, limit, max_torque, pos):
            print(f"ID {i}: a read failed, skipping")
            continue

        if max_torque == 0:
            print(f"ID {i}: EEPROM Max Torque is 0. Fix it with Dynamixel Wizard first. Skipping.")
            continue

        if limit >= max_torque:
            print(f"ID {i}: torque limit already {limit}, nothing to do")
            continue

        if temp > args.max_temp and not args.force:
            print(f"ID {i}: still {temp} C (> {args.max_temp} C). Let it cool, "
                  f"then run again (or use --force).")
            continue

        plan.append((i, pos, max_torque, torque_on))

    if not plan:
        port.closePort()
        print("\nNothing to restore.")
        return

    print("\nPlanned actions (each servo will HOLD its current position, then regain torque):")
    for i, pos, max_torque, _ in plan:
        print(f"  ID {i}: goal := {pos} ticks, torque limit := {max_torque}")

    answer = input("\nIs the arm SUPPORTED? Type YES to continue: ").strip()
    if answer != "YES":
        port.closePort()
        sys.exit("Aborted, nothing written.")

    for i, pos, max_torque, torque_on in plan:
        if not w2_verified(ph, port, i, ADDR_GOAL_POSITION, pos):
            print(f"ID {i}: could not verify hold position. NOT restoring torque.")
            continue
        if not w2_verified(ph, port, i, ADDR_TORQUE_LIMIT, max_torque):
            print(f"ID {i}: could not verify torque limit write.")
            continue
        print(f"ID {i}: restored. Holding at {pos} ticks with torque limit {max_torque}.")
        if torque_on == 0:
            print(f"ID {i}: note: Torque Enable is 0; the arm driver will enable it on start.")

    port.closePort()
    print("\nDone. Start the driver, then move the arm slowly (e.g. move_to_saved_pose.py --speed 20).")


def main():
    try:
        _main()
    except serial.SerialException as exc:
        sys.exit(f"Serial error: {exc}\nWrong port, or another program is using it?")


if __name__ == "__main__":
    main()