#!/usr/bin/env python3
"""
expo.launch.py  -  starts everything the expo robot needs.

    ros2 launch nova_arm_driver expo.launch.py
    ros2 launch nova_arm_driver expo.launch.py kobuki_port:=/dev/ttyUSB0 use_realsense:=false

Equivalent to running these by hand:
    ros2 run kobuki_node kobuki_ros_node --ros-args -p device_port:=/dev/ttyUSB0
    ros2 run nova_arm_driver arm_driver
    ros2 run nova_arm_driver HeadNode
    ros2 launch realsense2_camera rs_launch.py
    ros2 run nova_arm_driver qr_trigger
    ros2 run nova_arm_driver arm_health_coordinator
    ros2 run nova_arm_driver presenter_coordinator

Startup order does not matter: the health coordinator waits for the arm driver
(FAULT after comm_fault_s if it never appears) and the presenter stays PAUSED
until the arm health is HEALTHY.

Nothing is respawned automatically on purpose. A restarted arm_driver would
re-enable torque without a hold goal; if a node dies, the health coordinator
reports it and the presenter stays safe.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

PKG = "nova_arm_driver"


def generate_launch_description():
    kobuki_port = LaunchConfiguration("kobuki_port")
    use_realsense = LaunchConfiguration("use_realsense")

    kobuki = Node(
        package="kobuki_node",
        executable="kobuki_ros_node",
        output="screen",
        emulate_tty=True,
        parameters=[{"device_port": kobuki_port}],
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("realsense2_camera"),
                                  "launch", "rs_launch.py"])),
        condition=IfCondition(use_realsense),
    )

    # Names are left to each node's own default where the node sets one, so
    # topic/parameter names stay exactly as when started with ros2 run.
    def plain(executable):
        return Node(package=PKG, executable=executable,
                    output="screen", emulate_tty=True)

    return LaunchDescription([
        DeclareLaunchArgument("kobuki_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("use_realsense", default_value="true"),

        kobuki,
        plain("arm_driver"),
        plain("HeadNode"),
        realsense,
        plain("qr_trigger"),
        plain("arm_health_coordinator"),
        plain("presenter_coordinator"),
    ])