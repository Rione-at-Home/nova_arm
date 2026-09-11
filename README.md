# arm_driver_pkg

ROS 2 driver for a dual-arm AX-12A setup, one independent USB2Dynamixel
bus per arm. See the electrical-setup issue for wiring/power details
(each arm has its own USB2Dynamixel + SMPS2Dynamixel + power supply,
fully isolated from the other arm).

External topic contracts are unchanged from the original single-bus
OpenCR driver -- this package only changes what's underneath them.

## Package layout

```
arm_driver_pkg/
  arm_driver_pkg/
    arm_bus.py          # ArmBus: one independent DYNAMIXEL bus (one arm)
    arm_driver_node.py  # ArmDriver node: owns two ArmBus instances
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
   
   ```
2. Create the file or modify the `/etc/udev/rules.d/99-dxl-arms.rules`:
   ```
   sudo nano /etc/udev/rules.d/99-dynamixel.rules
   ```
3. Add rules to `/etc/udev/rules.d/99-dxl-arms.rules`:
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
colcon build --packages-select arm_driver_pkg
source install/setup.bash
```

## Run

```
ros2 launch arm_driver_pkg arm_driver.launch.py
```

Override ports if you haven't set up udev symlinks yet (not
recommended for regular use, only for quick bench testing):

```
ros2 launch arm_driver_pkg arm_driver.launch.py \
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