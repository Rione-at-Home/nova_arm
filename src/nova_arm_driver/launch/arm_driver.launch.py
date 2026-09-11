"""
Launch file for nova_arm_driver.

Exposes right_port / left_port / baudrate as launch arguments so the
udev-mapped serial device paths can be overridden per-machine without
editing code, e.g.:

    ros2 launch nova_arm_driver arm_driver.launch.py \\
        right_port:=/dev/dxl_right left_port:=/dev/dxl_left
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    right_port_arg = DeclareLaunchArgument(
        "right_port",
        default_value="/dev/dxl_right",
        description=(
            "Serial device for the right arm's USB2Dynamixel. Should "
            "be a udev-mapped persistent symlink, not a raw "
            "/dev/ttyUSB* path, so arm identity survives "
            "reconnects/reboots."
        ),
    )

    left_port_arg = DeclareLaunchArgument(
        "left_port",
        default_value="/dev/dxl_left",
        description=(
            "Serial device for the left arm's USB2Dynamixel. Should "
            "be a udev-mapped persistent symlink, not a raw "
            "/dev/ttyUSB* path, so arm identity survives "
            "reconnects/reboots."
        ),
    )

    baudrate_arg = DeclareLaunchArgument(
        "baudrate",
        default_value="1000000",
        description=(
            "Serial baud rate, must match the AX-12A configuration."
        ),
    )

    arm_driver_node = Node(
        package="nova_arm_driver",
        executable="arm_driver",
        name="arm_driver",
        output="screen",
        parameters=[{
            "right_port": LaunchConfiguration("right_port"),
            "left_port": LaunchConfiguration("left_port"),
            "baudrate": LaunchConfiguration("baudrate"),
        }],
    )

    return LaunchDescription([
        right_port_arg,
        left_port_arg,
        baudrate_arg,
        arm_driver_node,
    ])