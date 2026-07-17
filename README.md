# nova_arm

A ROS 2 driver package for controlling a custom 6-DOF robotic arm using **Dynamixel AX-12A** servos.

## Features

* ROS 2 Humble compatible
* Supports 6 Dynamixel AX-12A servos
* Position control using `sensor_msgs/JointState`
* Adjustable motor speed
* Automatic torque enable on startup

---

## Hardware

* **Servos:** 6 × Dynamixel AX-12A
* **Protocol:** 1.0
* **Baudrate:** 1000000 bps
* **Communication:** TTL Serial

Current motor IDs:

| Joint   | ID |
| ------- | -: |
| Joint 1 |  1 |
| Joint 2 |  2 |
| Joint 3 |  3 |
| Joint 4 |  4 |
| Joint 5 |  5 |
| Gripper |  6 |

---

## Workspace

```bash
mkdir -p ~/nova_arm_ws/src
cd ~/nova_arm_ws

colcon build
source install/setup.bash
```

---

## Build

```bash
cd ~/nova_arm_ws

colcon build --symlink-install

source install/setup.bash
```

---

## Run the Driver

```bash
ros2 run nova_arm_driver arm_driver
```

---

## Topics

### Subscribe

#### `/arm_command`

Type:

```
sensor_msgs/msg/JointState
```

Example:

```bash
ros2 topic pub --once /arm_command sensor_msgs/msg/JointState "{
name: ['joint1','joint2','joint3','joint4','joint5','gripper'],
position: [0.0,0.5,-0.5,0.2,0.0,0.3]
}"
```

---

#### `/arm_speed`

Type:

```
std_msgs/msg/Int32
```

Example:

```bash
ros2 topic pub --once /arm_speed std_msgs/msg/Int32 "{data: 50}"
```

The value is a percentage from **0–100%**.

---

## Joint Names

The driver recognizes the following joint names:

```
joint1
joint2
joint3
joint4
joint5
gripper
```

---

## Future Development

Planned features include:

* Joint state publisher
* GUI for manual control
* Home position service
* URDF/Xacro model
* RViz visualization
* MoveIt integration
* Inverse kinematics
* Pick-and-place demonstrations

---

## License

This project is released under the MIT License.
