# Nova Arm Driver

ROS 2 driver for a dual-arm AX-12A manipulator, redesigned from the original single-bus OpenCR architecture to use **independent DYNAMIXEL communication buses for each arm**.

### Project Overview

The Nova Arm is a custom dual-arm manipulator used by the Ritsumeikan Ri-One@Home team. This driver provides a unified ROS 2 interface for controlling and monitoring both arms while keeping their underlying communication buses electrically and logically independent.

The driver was refactored to support the robot's revised hardware architecture:

* One USB2Dynamixel interface per arm
* Independent serial/packet handlers and sync-write buffers
* Persistent udev device names for reliable arm identification
* A shared ROS 2 interface preserved from the original driver
* Per-arm communication failure isolation
* Measured joint-state feedback at 50 Hz
* **Tolerance for a missing arm: an unplugged or unpowered arm never stops the node, and is picked up automatically when it comes back**
* **Read-only health status and a torque-limit recovery service for an external health monitor**

The main architectural change is the separation of **arm-level control from per-arm DYNAMIXEL bus communication**. `ArmDriver` coordinates the two arms, while `ArmBus` encapsulates communication with one physical arm.

### Architecture

```text
                          ROS 2
                            │
          ┌─────────────────┴─────────────────┐
          │            ArmDriver              │
          │  /arm_command   /arm_speed        │
          │  /joint_states                    │
          │  /arm_servo_status                │
          │  /arm/<side>/motion_allowed       │
          │  /arm/<side>/servo_<id>/restore_… │
          └─────────────────┬─────────────────┘
                            │
                   ┌────────┴────────┐
                   │                 │
              ArmBus (right)    ArmBus (left)
                   │                 │
             USB2Dynamixel     USB2Dynamixel
                   │                 │
              Right Arm          Left Arm
```

The existing ROS 2 topic interface (`/arm_command`, `/arm_speed`, `/joint_states`) remains in place, so higher-level nodes do not need to know that communication is split across two independent buses. The health interface is additive: with no health node running, the driver behaves as before.

---

## Package layout

```
nova_arm_driver/
  nova_arm_driver/
    ArmBus.py     # ArmBus: one independent DYNAMIXEL bus (one arm)
    ArmDriver.py  # ArmDriver node: owns two ArmBus instances
  launch/
    arm_driver.launch.py
  package.xml
  setup.py
  setup.cfg
```

`package.xml` must declare the message/service packages the driver imports:
`rclpy`, `sensor_msgs`, `std_msgs`, `diagnostic_msgs`, `std_srvs` (plus the `dynamixel_sdk` Python package).

## udev setup (required, do this first)

Raw `/dev/ttyUSB0` / `/dev/ttyUSB1` enumeration order is not guaranteed
to stay consistent across reboots or reconnects -- which arm ends up on
which device can silently swap. Create persistent symlinks keyed to
each USB2Dynamixel's serial number instead:

1. Plug in one USB2Dynamixel at a time and find its serial number:
   ```
   udevadm info -a -n /dev/ttyUSB0 | grep serial
   ```
2. Create or edit the rules file:
   ```
   sudo nano /etc/udev/rules.d/99-dxl-arms.rules
   ```
3. Add these two lines, one per arm:
   ```
   SUBSYSTEM=="tty", ATTRS{serial}=="<RIGHT_ARM_SERIAL>", SYMLINK+="dxl_right"
   SUBSYSTEM=="tty", ATTRS{serial}=="<LEFT_ARM_SERIAL>", SYMLINK+="dxl_left"
   ```
4. Reload rules:
   ```
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
5. Confirm both symlinks exist:
   ```
   ls -l /dev/dxl_right /dev/dxl_left
   ```

The node's default parameters point at `/dev/dxl_right` and
`/dev/dxl_left`, so once the symlinks exist no further configuration is
needed. If an arm's USB2Dynamixel is unplugged, its symlink disappears
and that arm is treated as "not detected" (see below).

## Build

From your ROS 2 workspace root:

```
colcon build --packages-select nova_arm_driver
source install/setup.bash
```

## Run

```
ros2 launch nova_arm_driver arm_driver.launch.py
```

Override ports if you haven't set up udev symlinks yet (not
recommended for regular use, only for quick bench testing):

```
ros2 launch nova_arm_driver arm_driver.launch.py \
    right_port:=/dev/ttyUSB0 left_port:=/dev/ttyUSB1
```

The health-related parameters below can be set the same way if your launch
file forwards them, or directly on the node:

```
ros2 run nova_arm_driver arm_driver --ros-args -p fill_absent_joints:=true
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `right_port` | `/dev/dxl_right` | Serial device for the right arm |
| `left_port` | `/dev/dxl_left` | Serial device for the left arm |
| `baudrate` | `1000000` | Must match the servos |
| `status_rate_hz` | `2.0` | Rate of `/arm_servo_status` |
| `reconnect_period_s` | `3.0` | How often a missing arm's port is retried |
| `hold_on_enable` | `true` | Write Goal = Present Position before enabling torque (see below) |
| `fill_absent_joints` | `false` | Keep all 12 joints in `/joint_states`, with never-read joints at 0.0 rad |
| `recovery_max_temp_c` | `50` | Torque-limit restore is refused above this temperature |
| `require_motion_inhibit_for_restore` | `true` | Restore service refuses unless that arm's `motion_allowed` is False |

Float parameters must be given as floats on the command line (e.g. `2.0`, not `2`).

## Topics

| Topic          | Type                     | Direction | Notes                                    |
|-----------------|--------------------------|-----------|-------------------------------------------|
| `/arm_command`  | `sensor_msgs/JointState` | sub       | Goal positions, radians, any joint subset. Goals for an arm that is absent or has motion inhibited are dropped; the other arm is unaffected |
| `/arm_speed`    | `std_msgs/Int32`         | sub       | Speed as a percent (0-100), applies to all joints on both arms. Remembered and re-applied when a servo is re-enabled |
| `/joint_states` | `sensor_msgs/JointState` | pub       | Measured positions, radians, published at 50 Hz. Fixed joint order (right arm first, then left), containing only joints that have been read at least once (see `fill_absent_joints`) |
| `/arm_servo_status` | `diagnostic_msgs/DiagnosticArray` | pub | Read-only health data, `status_rate_hz` (see below) |
| `/arm/right/motion_allowed`, `/arm/left/motion_allowed` | `std_msgs/Bool` | sub | Latched (transient_local). While `False`, `/arm_command` goals for that arm are dropped. Default is `True` |

## Services

| Service | Type | Notes |
|---|---|---|
| `/arm/<side>/servo_<id>/restore_torque_limit` | `std_srvs/Trigger` | One per servo, `<side>` is `right` or `left`, `<id>` is the DYNAMIXEL ID (1-6 right, 11-16 left) |

`restore_torque_limit` repairs the known fault where an alarm shutdown leaves the RAM Torque Limit below Max Torque. In order, it:

1. refuses unless `/arm/<side>/motion_allowed` is `False` (when `require_motion_inhibit_for_restore` is true)
2. pings the servo and reads temperature, torque limit, Max Torque and position
3. refuses if EEPROM Max Torque is 0, or if the servo is hotter than `recovery_max_temp_c`
4. returns success with no change if the torque limit is already at Max Torque
5. writes Goal = Present Position (verified by read-back), so the servo holds where it is
6. only then writes Torque Limit = Max Torque (verified by read-back)

## Health status: `/arm_servo_status`

One `DiagnosticArray` containing, for each arm, one **bus** entry followed by one entry per servo. Every entry has an `arm` key (`right` / `left`) and a `kind` key.

Bus entry: name `arm_<side>_bus`

| Key | Meaning |
|---|---|
| `kind` | `bus` |
| `connected` | `1` if the serial port is open, else `0` |
| `port` | Device path in use |
| `connect_attempts` | Number of times the port has been tried |
| `last_error` | Last open/IO error text (empty when healthy) |
| `responding`, `total` | Servos that answered this cycle / servos on the bus |

Level is `ERROR` when the port is not available or no servo responds, `WARN` when only some respond, `OK` otherwise.

Servo entry: name `arm_<side>_servo_<id>`, `hardware_id` is the DYNAMIXEL ID

| Key | Meaning |
|---|---|
| `kind`, `arm`, `id`, `joint` | `servo`, side, DYNAMIXEL ID, joint name |
| `comm_ok` | `1` if the servo answered, else `0` (then no register keys follow) |
| `err` | Hardware error flags from the status packet (bit 0 voltage, 2 overheat, 5 overload, ...) |
| `torque_enable`, `goal`, `moving_speed`, `torque_limit`, `max_torque` | Raw register values |
| `position`, `load`, `voltage`, `temp` | Raw values (voltage in 0.1 V, temperature in °C) |
| `cw_margin`, `ccw_margin`, `cw_slope`, `ccw_slope` | Compliance registers |

Level is `ERROR` ("no response") when `comm_ok` is `0`, otherwise `OK`. The driver does not interpret the values; judging them is the job of whatever consumes this topic.

## Notes on behavior

- Both arms' sync-write packets are sent back-to-back within the same
  `command_callback` invocation, keeping them as close to
  simultaneous as possible without routing commands through a separate
  coordinator node.
- **Missing arm at startup.** If an arm's port cannot be opened (for
  example its USB2Dynamixel is unplugged, so `/dev/dxl_*` does not
  exist), the node logs it once, leaves that bus disconnected, and
  runs with the other arm. This includes the `SerialException` that
  `dynamixel_sdk` raises for a nonexistent device node, which would
  previously have escaped the `RuntimeError` handler and crashed the node.
- **Arm unplugged at runtime.** A serial error marks that bus down
  instead of propagating. Its goals are dropped with a throttled
  warning, and the other arm is unaffected.
- **Arm comes back.** Every `reconnect_period_s` the node retries each
  disconnected port. On success it re-opens the bus (with fresh
  port/packet handlers) and enables torque.
- **Arm powered off, USB still connected.** The port stays open but no
  servo answers. Polling for that bus drops to a single ping per status
  cycle, so a dead arm's read timeouts cannot slow the other arm's 50 Hz
  feedback. When the servos answer again with Torque Enable 0 (their
  power-on state), the driver re-enables them.
- **Torque enable holds position.** With `hold_on_enable` (default), the
  driver writes Goal = Present Position, re-applies the last `/arm_speed`,
  and only then enables torque, on startup, reconnect and servo return.
  This avoids a jump to a stale goal or a full-speed move. If a servo
  gives no position reading, its torque is left off and retried when it
  answers.
- **Per-joint read failures.** A read failure on one arm during normal
  operation does not block `/joint_states` publishing for the other arm;
  the affected joint(s) fall back to their last known position for that
  cycle. Position reads are accepted whenever the packet itself is valid,
  even if a hardware alarm flag is set in the status byte.
- **`/joint_states` contents.** A joint that has never been read
  successfully is omitted, so an absent arm is not reported at 0 rad.
  Consumers should look joints up by name. Set `fill_absent_joints:=true`
  to restore the earlier fixed 12-joint layout.
- **Motion gating is per arm.** Publishing `False` on
  `/arm/right/motion_allowed` stops goals to the right arm only. With no
  publisher the default is `True`, so the driver works unchanged without a
  health node.
- The driver runs on the default single-threaded executor, so timers,
  subscriptions and services never access a serial port at the same time.
- At shutdown, torque is disabled and ports are closed for any bus that is
  connected; a disconnected bus is skipped.

## Quick checks

```
# Is each arm detected, and do its servos respond?
ros2 topic echo /arm_servo_status --field status --once

# Only the joints that are actually present
ros2 topic echo /joint_states --field name --once

# Inhibit the left arm, then restore a servo's torque limit (ID 13)
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
    /arm/left/motion_allowed std_msgs/msg/Bool "{data: false}"
ros2 service call /arm/left/servo_13/restore_torque_limit std_srvs/srv/Trigger

# Allow motion again
ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable \
    /arm/left/motion_allowed std_msgs/msg/Bool "{data: true}"
```

To test absent-arm handling on the bench, start the node with one
USB2Dynamixel unplugged: it should log `arm not detected` for that side and
keep running, then log `arm detected` within a few seconds of plugging it in.