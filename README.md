# Nova Arm Driver

### Project Overview
This iteration of the nova_arm driver is redesigned from the dual-arm AX-12A manipulator version for the Hold Tray task of the Nagoya Expo. Because of the nature of the task, this iteration only utilizes a single arm while providing interfaces and control of other components of the robot. The other components are listed below:
* Kobuki Base
* Sudo Head's pan-tilt motors
* Sudo Cat's pan-tilt motors.

<img width="767" height="1024" alt="image" src="https://github.com/user-attachments/assets/eefbddf7-733e-4038-8655-123eea5a5d78" />

## Build

From your ROS 2 workspace root:

```
colcon build --packages-select nova_arm_driver
source install/setup.bash
```

## Run

**Note:** Ensure each command has its own terminal
```
ros2 run kobuki_node kobuki_ros_node --ros-args -p device_port:=/dev/ttyUSB0
ros2 run nova_arm_driver arm_driver
ros2 run nova_arm_driver HeadDriver
ros2 run nova_arm_driver HeadNode
```
  the other arm from coming up -- it's logged and that bus is left
  disconnected.
- A read failure on one arm during normal operation does not block
  `/joint_states` publishing for the other arm; the affected joint(s)
  fall back to their last known position for that cycle.
