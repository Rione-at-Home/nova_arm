# Nova Arm Tactile

ROS 2 package for FlexiTac tactile sensing and grasp control.

## Package structure

- `core.py`: ROS-independent tactile sensor processing.
- `tactile_node.py`: ROS 2 tactile sensor node.
- `config/`: tactile sensor configuration.
- `launch/`: launch files.
- `udev/`: device permission rules.
- `test/`: package tests.

The tactile message and action definitions are provided by
`nova_arm_tactile_msgs`.
