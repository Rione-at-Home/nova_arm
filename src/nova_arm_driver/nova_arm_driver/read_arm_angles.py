#!/usr/bin/env python3
"""
read_arm_angles.py

Hand-guide the arm into a pose and read what each motor's angle is.

- Disables torque on all joints so you can move the arm by hand
  (SUPPORT THE ARM: it will go limp and can fall under gravity).
- Prints live raw ticks, degrees and radians for every joint.
- Press ENTER to save the current pose into poses.yaml (same format
  used by nova_pose_manager / nova_demo: {names: [...], positions: [rad]}).
- Ctrl+C to quit.

IMPORTANT: Stop ArmDriver first. Only one program can use /dev/ttyUSB0.

Usage:
    python3 read_arm_angles.py                  # saves as "hold"
    python3 read_arm_angles.py --name tray_hold
    python3 read_arm_angles.py --keep-torque    # read only, don't touch torque
"""

import argparse
import math
import os
import select
import statistics
import sys
import time

import yaml
from dynamixel_sdk import PacketHandler, PortHandler

# Same control table / conversions as ArmDriver.py
ADDR_TORQUE_ENABLE = 24
ADDR_PRESENT_POSITION = 36
PROTOCOL_VERSION = 1.0

JOINT_TO_ID = {
    "joint1": 1,
    "joint2": 2,
    "joint3": 3,
    "joint4": 4,
    "joint5": 5,
    "gripper": 6,
}


def dxl_to_deg(value):
    return value * 300.0 / 1023.0 - 150.0


def read_all(ph, port):
    """Return {joint: ticks or None}."""
    out = {}
    for name, dxl_id in JOINT_TO_ID.items():
        value, comm, err = ph.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
        out[name] = value if (comm == 0 and err == 0) else None
    return out


def sample_median(ph, port, n=10, delay=0.03):
    """Median of n readings per joint to reduce jitter. None if any joint failed."""
    samples = {name: [] for name in JOINT_TO_ID}
    for _ in range(n):
        reading = read_all(ph, port)
        for name, v in reading.items():
            if v is not None:
                samples[name].append(v)
        time.sleep(delay)
    result = {}
    for name, vals in samples.items():
        if not vals:
            return None
        result[name] = statistics.median(vals)
    return result


def save_pose(pose_file, pose_name, ticks):
    poses = {}
    if os.path.exists(pose_file):
        with open(pose_file, "r") as f:
            poses = yaml.safe_load(f) or {}

    names = list(JOINT_TO_ID.keys())
    positions = [math.radians(dxl_to_deg(ticks[n])) for n in names]

    poses[pose_name] = {"names": names, "positions": positions}

    with open(pose_file, "w") as f:
        yaml.dump(poses, f, sort_keys=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--name", default="hold", help="pose name to save on ENTER")
    ap.add_argument(
        "--pose-file",
        default=os.path.expanduser("~/nova_arm_ws/poses.yaml"),
    )
    ap.add_argument(
        "--keep-torque",
        action="store_true",
        help="do not disable torque (read-only)",
    )
    args = ap.parse_args()

    port = PortHandler(args.port)
    ph = PacketHandler(PROTOCOL_VERSION)

    if not port.openPort():
        sys.exit(f"Failed to open {args.port} (is ArmDriver still running?)")
    if not port.setBaudRate(args.baud):
        sys.exit("Failed to set baudrate")

    if not args.keep_torque:
        print("\n*** Torque will be DISABLED in 3 seconds. SUPPORT THE ARM! ***")
        time.sleep(3.0)
        for dxl_id in JOINT_TO_ID.values():
            ph.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)
        print("Torque off. Move the arm to the holding pose.")

    print(f"Press ENTER to save pose '{args.name}' -> {args.pose_file}   (Ctrl+C to quit)\n")

    header = f"{'joint':<8}{'ticks':>7}{'deg':>9}{'rad':>9}"
    try:
        while True:
            reading = read_all(ph, port)

            lines = [header]
            for name, v in reading.items():
                if v is None:
                    lines.append(f"{name:<8}{'--':>7}{'--':>9}{'--':>9}  (read failed)")
                else:
                    deg = dxl_to_deg(v)
                    lines.append(
                        f"{name:<8}{v:>7d}{deg:>9.1f}{math.radians(deg):>9.3f}"
                    )

            # redraw the table in place
            sys.stdout.write("\033[H\033[J" if False else "")
            print("\n".join(lines))
            sys.stdout.write(f"\033[{len(lines)}A")
            sys.stdout.flush()

            if select.select([sys.stdin], [], [], 0.1)[0]:
                sys.stdin.readline()
                sys.stdout.write(f"\033[{len(lines)}B\n")
                ticks = sample_median(ph, port)
                if ticks is None:
                    print("Could not read every joint, pose NOT saved.")
                else:
                    save_pose(args.pose_file, args.name, ticks)
                    print(f"Saved '{args.name}':")
                    for n, t in ticks.items():
                        print(f"  {n:<8} {dxl_to_deg(t):8.1f} deg")
                    print()
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n" * (len(JOINT_TO_ID) + 2))
        port.closePort()


if __name__ == "__main__":
    main()