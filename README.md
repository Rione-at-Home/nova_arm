# Nova Arm Driver

ROS 2 driver for a dual-arm AX-12A manipulator, redesigned from the original single-bus OpenCR architecture to use **independent DYNAMIXEL communication buses for each arm**. This version is located in the dual-arm-setup branch.

### Project Overview
This iteration of the nova_arm driver is redesigned from the dual-arm AX-12A manipulator version for the Hold Tray task of the Nagoya Expo. Because of the nature of the task, this iteration only utilizes a single arm while providing interfaces and control of other components of the robot. The other components are listed below:
* Kobuki Base
* Sudo Head's pan-tilt motors
* Sudo Cat's pan-tilt motors.

The robot holds a tray, waits for a QR code, then drives forward, scans the crowd with its head, and speaks. A separate **arm health coordinator** watches the AX-12A servos and automatically recovers a known fault (Torque Limit forced to 0 by an alarm shutdown), so the demo can show *detect → diagnose → recover → verify → resume*.

<img width="767" height="1024" alt="image" src="https://github.com/user-attachments/assets/eefbddf7-733e-4038-8655-123eea5a5d78" />

## Contents

1. [Architecture](#architecture)
2. [Build](#build)
3. [Run](#run)
4. [Monitor GUI](#monitor-gui)
5. [Presenter coordinator](#presenter-coordinator)
6. [Arm health coordinator](#arm-health-coordinator)
7. [QR trigger](#qr-trigger)
8. [Topics and services](#topics-and-services)
9. [Testing the arm recovery](#testing-the-arm-recovery)
10. [Manual diagnostic scripts](#manual-diagnostic-scripts)
11. [Troubleshooting](#troubleshooting)

## Architecture

Only `arm_driver` opens the arm's serial port (`/dev/ttyUSB1`, AX-12A, Dynamixel Protocol 1.0, 1 Mbps). Everything else talks to it through ROS 2.

```
qr_trigger ──/presenter_trigger──► Presenter Coordinator ──/arm_command────────┐
 (RealSense)                        (DRIVE → SCAN → SPEAK)                     ▼
                                       ▲  /arm_health_state              ┌────────────┐
                                       │                                 │ arm_driver │──► AX-12A servos
Arm Health Coordinator ─/arm_motion_allowed (gate)────────────────────────►│            │
   (HEALTHY/DIAGNOSTIC/                 ─/arm/servo_N/restore_torque_limit─►│            │
    RECOVERING/VERIFYING/FAULT)  ◄──────/arm_servo_status──────────────────└────────────┘
```

Design rules:
* The two coordinators have separate jobs. The **presenter** runs the expo sequence. The **health coordinator** never publishes arm poses; it only watches, decides, and asks the driver to run the recovery.
* The driver drops `/arm_command` while `/arm_motion_allowed` is `False`, so nothing can fight a recovery.
* Recovery is verified after every write, limited to a few attempts, and ends in a latched `FAULT` that needs a human.

Files (in `nova_arm_driver/`):

| File | Role |
|---|---|
| `ArmDriver.py` | Owns the arm serial port. Executes `/arm_command`, publishes `/arm_servo_status`, hosts the restore services and motion gate |
| `arm_health_coordinator.py` | Health state machine and automatic recovery |
| `presentor_coordinator.py` | Expo state machine (IDLE / DRIVE / SCAN / SPEAK), pauses/aborts on arm health |
| `qr_trigger.py` | Watches the camera for the QR code, starts the countdown, publishes `/presenter_trigger` |
| `HeadNode.py` | Head pan/tilt interface |
| `arm_monitor_gui.py` | Monitoring GUI (run with `python3`, not required for the robot to work) |
| `launch/expo.launch.py` | Starts the whole robot |
| `arm_servo_status.py`, `restore_torque_limit.py` | Manual, stand-alone servo tools (see below) |

## Build

From your ROS 2 workspace root:

```
colcon build --packages-select nova_arm_driver
source install/setup.bash
```

Rebuild after adding a launch file or entry point. `colcon build --symlink-install` avoids rebuilding after Python edits.

Dependencies: ROS 2 (rclpy, `sensor_msgs`, `std_msgs`, `std_srvs`, `diagnostic_msgs`, `geometry_msgs`, `cv_bridge`), `kobuki_node`, `realsense2_camera`, the Python `dynamixel_sdk`, OpenCV, PyYAML. For the GUI: `sudo apt install python3-tk python3-matplotlib`.

## Run

### Everything at once

```
ros2 launch nova_arm_driver expo.launch.py
# options
ros2 launch nova_arm_driver expo.launch.py kobuki_port:=/dev/ttyUSB0 use_realsense:=false
```

This starts the Kobuki node, `arm_driver`, `HeadNode`, the RealSense camera, `qr_trigger`, `arm_health_coordinator` and `presenter_coordinator`. Start-up order does not matter: the presenter stays paused until the arm health is `HEALTHY`. Nodes are deliberately **not** respawned automatically.

### One terminal per node (debugging)

**Note:** Ensure each command has its own terminal
```
ros2 run kobuki_node kobuki_ros_node --ros-args -p device_port:=/dev/ttyUSB0
ros2 run nova_arm_driver arm_driver
ros2 run nova_arm_driver HeadNode
ros2 launch realsense2_camera rs_launch.py
ros2 run nova_arm_driver qr_trigger
ros2 run nova_arm_driver arm_health_coordinator
ros2 run nova_arm_driver presenter_coordinator
```

To run the presenter without the health coordinator: `ros2 run nova_arm_driver presenter_coordinator --ros-args -p require_arm_health:=false`.

Test the sequence without the QR code:
```
ros2 topic pub --once /presenter_trigger std_msgs/msg/String "{data: start}"
ros2 topic pub --once /presenter_abort std_msgs/msg/Empty "{}"
```

## Monitor GUI

```
python3 arm_monitor_gui.py                       # after sourcing the workspace
python3 arm_monitor_gui.py --window 300 --servos 1 2 3 4 5 6
```

Shows the arm health state, the presenter state, the QR trigger (camera OK, QR seen, confirm frames, countdown), an event log, a per-servo table (torque, position, goal, temperature, voltage, load, Torque Limit, Max Torque, error flags) and live charts of temperature, load, Torque Limit and position. It only listens, except for one button that calls `/arm_health/reset` to leave a `FAULT` after a human has fixed the problem.

![alt text](<Screenshot from 2026-09-21 17-02-12.png>)

## Presenter coordinator

Holds the saved arm pose (`tray_hold` from `poses.yaml`) and, on `/presenter_trigger`, runs DRIVE → SCAN → SPEAK. `/presenter_abort` stops the base, centers the head, and returns to IDLE.

Reaction to arm health (`/arm_health_state`):

| Arm health | Presenter behavior |
|---|---|
| `HEALTHY` | Normal operation |
| `DIAGNOSTIC`, `RECOVERING`, `VERIFYING` | **Paused**: base stopped, sequence timers frozen (resumes where it left off), no arm commands, triggers ignored. `/presenter_state` = `ARM_PAUSED` |
| `FAULT` | **Aborted**: base stopped, head centered, sequence cleared, no arm commands. `/presenter_state` = `ARM_FAULT` |
| no message for `health_timeout` (5 s) | Treated as paused (fail-safe) |

`/presenter_state` is re-published once a second (values: `IDLE`, `DRIVE`, `SCAN`, `SPEAK`, `ARM_PAUSED`, `ARM_FAULT`).

## Arm health coordinator

States:

```
DIAGNOSTIC ─► HEALTHY ─(fault persists 1 s)─► DIAGNOSTIC ─► RECOVERING ─► VERIFYING ─► HEALTHY
                                                 │              │            │
                                                 └──────────────┴────────────┴──► FAULT (latched)
```

What it checks per servo (from `/arm_servo_status`): communication, Torque Enable, Torque Limit vs Max Torque, temperature, hardware error flags, load, voltage, present vs goal position.

Different faults are handled differently:

| Condition | Action |
|---|---|
| Torque Limit < Max Torque (alarm-shutdown fault) | Wait until cool (`recover_max_temp_c`, default 50 °C), then recover |
| Overheat / overload / voltage flag with cause still present | Wait for it to clear; `FAULT` after `cooldown_timeout_s` |
| No communication | Wait; `FAULT` after `comm_fault_s` |
| Torque Enable = 0 | `FAULT` (not auto-fixed) |
| Max Torque = 0 (EEPROM) | `FAULT` (fix with Dynamixel Wizard) |

Recovery (per servo, run by the driver's `restore_torque_limit` service, same steps as `restore_torque_limit.py`): ping → read temperature, Torque Limit, Max Torque, Present Position → refuse if too hot → set Goal = Present Position and **verify** → set Torque Limit = Max Torque and **verify**. Then the coordinator re-checks the servo for `verify_duration_s`, and checks the arm did not move away from the hold position. Only then it reports `ARM_RECOVERED`.

Safety limits: `max_recovery_attempts` per servo (default 3, reset after `attempt_reset_s` of continuous health), a delay between attempts, and a latched `FAULT` that requires a human. Leave `FAULT` after fixing the problem:
```
ros2 service call /arm_health/reset std_srvs/srv/Trigger
```

Events on `/arm_health_event` for the TTS / face developer (the coordinator itself never speaks): `ARM_PROBLEM_DETECTED`, `ARM_RECOVERING`, `ARM_RECOVERED`, `ARM_PROBLEM_CLEARED`, `ARM_FAULT`.

Main parameters (float parameters must be given as floats on the command line, e.g. `60.0`):

| Parameter | Default | Meaning |
|---|---|---|
| `recover_max_temp_c` | 50 | Refuse to restore torque above this |
| `warn_temp_c` | 55 | Log a warning above this |
| `max_recovery_attempts` | 3 | Per servo |
| `cooldown_timeout_s` | 180.0 | Wait for an alarm/thermal condition to clear |
| `comm_fault_s` | 15.0 | Communication loss before `FAULT` |
| `verify_duration_s` | 3.0 | How long the servo must stay good |
| `hold_tolerance_ticks` | 30 | Allowed position drift after recovery (~9°) |
| `servo_ids` | 1–6 | Servos to monitor |

## QR trigger

`qr_trigger` watches `/camera/camera/color/image_raw`. After the expected QR code is seen for `confirm_frames` consecutive frames it counts down `delay_sec` (default 6 s) and publishes `/presenter_trigger`, then waits `cooldown_sec` before it can trigger again. `/qr_trigger_status` (JSON, 5 Hz) reports the camera, whether a QR is visible, and the countdown; the GUI shows it.

```
ros2 run nova_arm_driver qr_trigger --ros-args -p expected_payload:=NOVA_START -p show:=true
```

## Topics and services

| Name | Type | From → To |
|---|---|---|
| `/arm_command` | `sensor_msgs/JointState` | presenter → arm_driver (dropped while motion is inhibited) |
| `/arm_speed` | `std_msgs/Int32` | presenter → arm_driver |
| `/arm_servo_status` | `diagnostic_msgs/DiagnosticArray` | arm_driver → health coordinator, GUI |
| `/arm/servo_{1..6}/restore_torque_limit` | `std_srvs/Trigger` | health coordinator → arm_driver |
| `/arm_motion_allowed` | `std_msgs/Bool` (latched) | health coordinator → arm_driver |
| `/arm_health_state` | `std_msgs/String` (latched, 1 Hz) | health coordinator → presenter, GUI |
| `/arm_health_event` | `std_msgs/String` | health coordinator → TTS / face / GUI |
| `/arm_health_detail` | `std_msgs/String` (JSON, latched) | health coordinator → GUI, logging |
| `/arm_health/reset` | `std_srvs/Trigger` | human / GUI → health coordinator |
| `/presenter_trigger` | `std_msgs/String` | qr_trigger → presenter |
| `/presenter_abort` | `std_msgs/Empty` | anyone → presenter |
| `/presenter_state` | `std_msgs/String` | presenter → GUI, UI |
| `/qr_trigger_status` | `std_msgs/String` (JSON) | qr_trigger → GUI |
| `/tts/say` | `std_msgs/String` | presenter → TTS |
| `/head/pan_target`, `/head/tilt_target` | `std_msgs/Float32` | presenter → HeadNode |
| `/commands/velocity` | `geometry_msgs/Twist` | presenter → Kobuki |

## Testing the arm recovery

**Support the arm first.** The servo under test goes limp. Start with the gripper (ID 6) or another servo whose collapse is harmless.

1. **Real recovery.** Stop `arm_driver` (Ctrl+C), then force Torque Limit = 0 on one servo:
   ```
   python3 - <<'EOF'
   from dynamixel_sdk import PortHandler, PacketHandler
   p = PortHandler("/dev/ttyUSB1"); p.openPort(); p.setBaudRate(1000000)
   print(PacketHandler(1.0).write2ByteTxRx(p, 6, 34, 0)); p.closePort()
   EOF
   ```
   Restart the stack. Expected: `DIAGNOSTIC → RECOVERING → VERIFYING → HEALTHY`, events `ARM_RECOVERING` then `ARM_RECOVERED`. To see `ARM_PROBLEM_DETECTED` as well, cause the fault while everything is already running.
2. **Pause mid-sequence.** Trigger the presenter, then cause the fault. The base stops (`ARM_PAUSED`) and the sequence resumes after recovery.
3. **Thermal gate / FAULT path (no fault injection needed).**
   ```
   ros2 run nova_arm_driver arm_health_coordinator --ros-args -p recover_max_temp_c:=10 -p cooldown_timeout_s:=20.0
   ```
   With the Torque Limit fault present it refuses to restore torque and goes to `FAULT` after 20 s.
4. **Driver service by hand** (the driver refuses unless motion is inhibited):
   ```
   ros2 topic pub --once --qos-durability transient_local --qos-reliability reliable /arm_motion_allowed std_msgs/msg/Bool "{data: false}"
   ros2 service call /arm/servo_6/restore_torque_limit std_srvs/srv/Trigger
   ```
   Publish `{data: true}` afterwards.

Useful watch commands:
```
ros2 topic echo /arm_health_state
ros2 topic echo /arm_health_event
ros2 topic echo /presenter_state
ros2 topic echo /arm_servo_status --once
```

## Manual diagnostic scripts

`arm_servo_status.py` (read-only status table with alarm, compliance and punch settings) and `restore_torque_limit.py` (manual recovery) talk to the serial port directly. **Stop `arm_driver` before using them**, and support the arm. Note that stopping `arm_driver` with Ctrl+C disables torque on every servo.

```
python3 arm_servo_status.py --port /dev/ttyUSB1
python3 restore_torque_limit.py --port /dev/ttyUSB1 --ids 1 3
```

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Presenter stays `ARM_PAUSED` | Health is not `HEALTHY` yet, or `arm_health_coordinator` is not running (or use `require_arm_health:=false`) |
| Health `FAULT`, "condition did not clear" | Servo still hot or alarm still active; let it cool, then reset |
| Health `FAULT`, "restore service not available" | `arm_driver` is not the updated version or is not running |
| `/arm_command` ignored, warning about motion not allowed | The health coordinator has inhibited motion during diagnosis/recovery/fault |
| GUI panel shows `NO SIGNAL` | That node is not running, or its topic is not being published (check with `ros2 topic echo`) |
| QR panel shows `Camera: NO IMAGES` | RealSense is not publishing on the configured `image_topic` |
| A servo goes limp again right after recovery | Root cause not fixed (load, heat, voltage); recovery attempts are limited on purpose |