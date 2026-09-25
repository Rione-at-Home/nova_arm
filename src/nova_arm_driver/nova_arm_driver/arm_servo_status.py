#!/usr/bin/env python3
"""
arm_servo_status.py

READ-ONLY health check for the AX-12A arm servos (never writes anything).
Shows, per servo: torque state, position vs goal, temperature, voltage, load,
hardware error flags, and the torque / alarm / compliance settings.

IMPORTANT: the arm driver must NOT be using the port. But note:
  * Stopping ArmDriver with Ctrl+C DISABLES torque on every servo, which erases
    the evidence of which servos had torque. To keep the torque state as it
    was, stop it with:   pkill -9 -f ArmDriver      (then run this script)
  * SUPPORT THE ARM either way.

Usage:
    python3 arm_servo_status.py
    python3 arm_servo_status.py --port /dev/ttyUSB1
"""

import argparse
import sys

import serial
from dynamixel_sdk import PacketHandler, PortHandler

ERR_BITS = {
    0: "input voltage",
    1: "angle limit",
    2: "OVERHEATING",
    3: "range",
    4: "checksum",
    5: "OVERLOAD",
    6: "instruction",
}

# name: (address, size in bytes)   AX-12A control table
REGS = {
    "max_torque": (14, 2),
    "alarm_led": (17, 1),
    "alarm_shutdown": (18, 1),
    "torque_enable": (24, 1),
    "cw_margin": (26, 1),
    "ccw_margin": (27, 1),
    "cw_slope": (28, 1),
    "ccw_slope": (29, 1),
    "goal": (30, 2),
    "moving_speed": (32, 2),
    "torque_limit": (34, 2),
    "position": (36, 2),
    "load": (40, 2),
    "voltage": (42, 1),
    "temp": (43, 1),
    "punch": (48, 2),
}


def read_reg(ph, port, dxl_id, addr, size):
    if size == 1:
        value, comm, err = ph.read1ByteTxRx(port, dxl_id, addr)
    else:
        value, comm, err = ph.read2ByteTxRx(port, dxl_id, addr)
    return (value if comm == 0 else None), (err if comm == 0 else 0)


def decode_err(err):
    return [name for bit, name in ERR_BITS.items() if err & (1 << bit)]


def ticks_to_deg(v):
    return None if v is None else v * 300.0 / 1023.0 - 150.0


def fmt(v, spec="{}"):
    return "--" if v is None else spec.format(v)


def _main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = ap.parse_args()

    port = PortHandler(args.port)
    ph = PacketHandler(1.0)

    if not port.openPort():
        sys.exit(f"Failed to open {args.port}")
    if not port.setBaudRate(args.baud):
        sys.exit("Failed to set baudrate")

    data = {}
    errs = {}

    for i in args.ids:
        _, comm, _e = ph.ping(port, i)
        if comm != 0:
            print(f"ID {i}: NO RESPONSE")
            continue

        data[i] = {}
        errs[i] = 0
        for name, (addr, size) in REGS.items():
            value, err = read_reg(ph, port, i, addr, size)
            data[i][name] = value
            errs[i] |= err

    port.closePort()

    if not data:
        sys.exit(f"No servo answered on {args.port}. Wrong port?")

    # ---- table 1: state ----
    print()
    print("STATE")
    print(f"{'ID':<4}{'torque':<8}{'pos deg':>9}{'goal deg':>10}{'temp C':>8}"
          f"{'volt V':>8}{'load %':>8}  errors")
    for i, d in data.items():
        load = d["load"]
        load_pct = None if load is None else (load & 0x3FF) * 100.0 / 1023.0
        volt = None if d["voltage"] is None else d["voltage"] / 10.0
        torque = {1: "ON", 0: "OFF", None: "--"}[d["torque_enable"]]
        flags = decode_err(errs[i])
        print(
            f"{i:<4}{torque:<8}"
            f"{fmt(ticks_to_deg(d['position']), '{:9.1f}'):>9}"
            f"{fmt(ticks_to_deg(d['goal']), '{:10.1f}'):>10}"
            f"{fmt(d['temp'], '{:8d}'):>8}"
            f"{fmt(volt, '{:8.1f}'):>8}"
            f"{fmt(load_pct, '{:8.0f}'):>8}"
            f"  {', '.join(flags) if flags else '-'}"
        )

    # ---- table 2: settings ----
    print()
    print("SETTINGS")
    print(f"{'ID':<4}{'maxTq':>7}{'tqLim':>7}{'alarmSD':>9}{'alarmLED':>9}"
          f"{'margin':>8}{'slope':>8}{'punch':>7}{'speed':>7}")
    for i, d in data.items():
        margin = f"{fmt(d['cw_margin'])}/{fmt(d['ccw_margin'])}"
        slope = f"{fmt(d['cw_slope'])}/{fmt(d['ccw_slope'])}"
        alarm_sd = "--" if d["alarm_shutdown"] is None else f"0x{d['alarm_shutdown']:02X}"
        alarm_led = "--" if d["alarm_led"] is None else f"0x{d['alarm_led']:02X}"
        print(
            f"{i:<4}{fmt(d['max_torque']):>7}{fmt(d['torque_limit']):>7}"
            f"{alarm_sd:>9}{alarm_led:>9}{margin:>8}{slope:>8}"
            f"{fmt(d['punch']):>7}{fmt(d['moving_speed']):>7}"
        )

    # ---- warnings ----
    print()
    warnings = []
    off = [i for i, d in data.items() if d["torque_enable"] == 0]
    if off:
        warnings.append(
            f"Torque is OFF on IDs {off}. (Expected if you stopped the driver "
            f"with Ctrl+C; use pkill -9 to keep the real state.)"
        )
    for i, d in data.items():
        if d["temp"] is not None and d["temp"] >= 55:
            warnings.append(f"ID {i}: temperature {d['temp']} C is high (AX-12A shuts down near 70 C).")
        if d["voltage"] is not None and not (95 <= d["voltage"] <= 125):
            warnings.append(f"ID {i}: voltage {d['voltage'] / 10:.1f} V outside 9.5-12.5 V.")
        if d["max_torque"] is not None and d["max_torque"] < 1023:
            warnings.append(f"ID {i}: Max Torque (EEPROM) is {d['max_torque']}, below 1023.")
        if d["torque_limit"] is not None and d["torque_limit"] < 1023:
            warnings.append(f"ID {i}: Torque Limit is {d['torque_limit']}, below 1023.")
        flags = decode_err(errs[i])
        if flags:
            warnings.append(f"ID {i}: hardware error flags: {', '.join(flags)}.")

    if warnings:
        print("WARNINGS")
        for w in warnings:
            print(f"  - {w}")
    else:
        print("No warnings.")
    print()


def main():
    try:
        _main()
    except serial.SerialException as exc:
        sys.exit(f"Serial error: {exc}\nWrong port, or another program is using it?")


if __name__ == "__main__":
    main()