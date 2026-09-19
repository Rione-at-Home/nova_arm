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

The main architectural change is the separation of **arm-level control from per-arm DYNAMIXEL bus communication**. `ArmDriver` coordinates the two arms, while `ArmBus` encapsulates communication with one physical arm.

### Architecture

```text
                    ROS 2
                      │
          ┌───────────┴───────────┐
          │      ArmDriver        │
          │  /arm_command         │
          │  /arm_speed           │
          │  /joint_states        │
          └───────────┬───────────┘
                      │
             ┌────────┴────────┐
             │                 │
        ArmBus (right)    ArmBus (left)
             │                 │
       USB2Dynamixel       USB2Dynamixel
             │                 │
        Right Arm           Left Arm
```

The existing ROS 2 topic interface remains unchanged, so higher-level nodes do not need to know that communication is now split across two independent buses.

---

## Package layout

...


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
needed.

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

## Topics

| Topic          | Type                     | Direction | Notes                                    |
|-----------------|--------------------------|-----------|-------------------------------------------|
| `/arm_command`  | `sensor_msgs/JointState` | sub       | Goal positions, radians, any joint subset |
| `/arm_speed`    | `std_msgs/Int32`         | sub       | Speed as a percent (0-100), applies to all joints on both arms |
| `/joint_states` | `sensor_msgs/JointState` | pub       | Measured positions, radians, published at 50 Hz, fixed joint order (right arm first, then left) |

## Notes on behavior

- Both arms' sync-write packets are sent back-to-back within the same
  `command_callback` invocation, keeping them as close to
  simultaneous as possible without routing commands through a separate
  coordinator node.
- A connection failure on one arm's port at startup does not prevent
  the other arm from coming up -- it's logged and that bus is left
  disconnected.
- A read failure on one arm during normal operation does not block
  `/joint_states` publishing for the other arm; the affected joint(s)
  fall back to their last known position for that cycle.
