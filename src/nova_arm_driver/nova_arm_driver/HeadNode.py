#  ROS2 controller node for Sudo's pan-tilt head AND the cat head on the back.
#  Every axis (pan, tilt, cat1, cat2) uses the same pipeline:
#  One Euro Filter on the target -> quintic trajectory -> Dynamixel command.
#
#  Topics (all std_msgs/Float32, degrees):
#    /head/pan_target    /head/tilt_target
#    /cat/joint1_target  /cat/joint2_target

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .filters import OneEuroFilter
from .HeadDriver import DynamixelDriver


class Axis:
    """One filtered, quintic-profiled axis. Same logic as the original pan/tilt code."""

    def __init__(self, dt, min_cutoff, beta, d_cutoff):
        self.filter = OneEuroFilter(
            dt=dt,
            min_cutoff=min_cutoff,
            beta=beta,
            d_cutoff=d_cutoff,
        )
        self.raw = 0.0
        self.filtered = 0.0
        self.current = 0.0
        self.goal = 0.0
        self.start = 0.0
        self.elapsed = 0.0

    def step(self, dt, motion_time):
        # Filter raw target, retarget the trajectory if it moved enough
        self.filtered = self.filter.update(self.raw)

        if abs(self.filtered - self.goal) > 0.5:
            self.start = self.current
            self.goal = self.filtered
            self.elapsed = 0.0

        # Quintic polynomial trajectory
        self.elapsed += dt
        s = min(max(self.elapsed / motion_time, 0.0), 1.0)
        blend = 6 * (s ** 5) - 15 * (s ** 4) + 10 * (s ** 3)

        self.current = self.start + blend * (self.goal - self.start)
        return self.current


class HeadNode(Node):

    def __init__(self):
        super().__init__("head_node")

        # Parameters for 1E filter and motion profiling
        self.declare_parameter("min_cutoff", 1.0)
        self.declare_parameter("beta", 0.05)
        self.declare_parameter("d_cutoff", 1.0)
        self.declare_parameter("motion_profile", "quintic")
        self.declare_parameter("motion_time", 2.5)

        # Hardware
        self.declare_parameter("device_name", "/dev/ttyUSB2")
        self.declare_parameter("use_cat", True)
        self.declare_parameter("cat_profile_velocity", 0)
        self.declare_parameter("cat_profile_acceleration", 0)

        self.min_cutoff = self.get_parameter("min_cutoff").get_parameter_value().double_value
        self.beta = self.get_parameter("beta").get_parameter_value().double_value
        self.d_cutoff = self.get_parameter("d_cutoff").get_parameter_value().double_value
        self.motion_profile = self.get_parameter("motion_profile").get_parameter_value().string_value
        self.motion_time = self.get_parameter("motion_time").get_parameter_value().double_value

        device_name = self.get_parameter("device_name").get_parameter_value().string_value
        self.use_cat = self.get_parameter("use_cat").get_parameter_value().bool_value

        self.control_period = 0.05  # 20 Hz

        def make_axis():
            return Axis(
                self.control_period,
                self.min_cutoff,
                self.beta,
                self.d_cutoff,
            )

        self.pan = make_axis()
        self.tilt = make_axis()
        self.cat1 = make_axis()
        self.cat2 = make_axis()

        self.driver = DynamixelDriver(
            device_name=device_name,
            use_cat=self.use_cat,
            cat_profile_velocity=self.get_parameter("cat_profile_velocity").value,
            cat_profile_acceleration=self.get_parameter("cat_profile_acceleration").value,
        )
        self.driver.enable()
        self.driver.calibrate_zero()

        self.create_subscription(Float32, "/head/pan_target", self.pan_cb, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_cb, 10)
        self.create_subscription(Float32, "/cat/joint1_target", self.cat1_cb, 10)
        self.create_subscription(Float32, "/cat/joint2_target", self.cat2_cb, 10)

        self.timer = self.create_timer(self.control_period, self.control_loop)
        self.get_logger().info(
            f"Head node active on {device_name} (cat={'on' if self.use_cat else 'off'}). "
            f"Filter: One Euro (min_cutoff={self.min_cutoff}, beta={self.beta}, d_cutoff={self.d_cutoff})"
        )

    def pan_cb(self, msg: Float32):
        self.pan.raw = msg.data

    def tilt_cb(self, msg: Float32):
        self.tilt.raw = msg.data

    def cat1_cb(self, msg: Float32):
        self.cat1.raw = msg.data

    def cat2_cb(self, msg: Float32):
        self.cat2.raw = msg.data

    def control_loop(self):
        dt = self.control_period
        mt = self.motion_time

        # Hardware Command
        self.driver.set_pan(self.pan.step(dt, mt))
        self.driver.set_tilt(self.tilt.step(dt, mt))

        if self.use_cat:
            self.driver.set_cat(1, self.cat1.step(dt, mt))
            self.driver.set_cat(2, self.cat2.step(dt, mt))

    def destroy_node(self):
        try:
            self.driver.disable()
            self.driver.close()
            if rclpy.ok():
                self.get_logger().info("Dynamixel driver safely disabled and closed.")

        except Exception as e:
            if rclpy.ok():
                self.get_logger().error(f"Error shutting down Dynamixel driver: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HeadNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()