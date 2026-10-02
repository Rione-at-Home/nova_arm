#  ROS2 controller node for Sudo's pan-tilt head.
#
#  Behaviour
#   * Startup: every axis HOLDS its calibrated zero. Filters are bypassed and
#     nothing is commanded for `startup_hold_sec`.
#   * Commanded movement (a target arrives on a topic): that axis switches to
#     One Euro filter -> quintic trajectory, starting from where it is now.
#     After `idle_resume_sec` without a new target the axis goes back to
#     holding its position with filters bypassed.
#   * Idle sway: when the head is not being commanded it gently sways
#     left-right (pan). The sway starts at zero offset and fades in/out;
#     it fades out during commands.
#   * Rate limit: commanded motion is capped at `max_speed` deg/s.
#   * Diagnostics: at startup it logs how far each motor drifted after
#     torque-on and warns if another node already publishes the target topics.
#
#  Topics (all std_msgs/Float32, degrees from the startup position):
#    /head/pan_target    /head/tilt_target

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from .filters import OneEuroFilter
from .HeadDriver import DynamixelDriver


class Axis:
    """One axis. Bypassed (holds `current`) until a target is commanded."""

    def __init__(self, name, dt, min_cutoff, beta, d_cutoff):
        self.name = name
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
        self.active = False
        self.last_cmd = None

    def command(self, value, now):
        if not self.active:
            # Start filter and trajectory exactly where the axis is now
            self.filter.reset(self.current)
            self.filtered = self.current
            self.goal = self.current
            self.start = self.current
            self.elapsed = 0.0
            self.active = True

        self.raw = float(value)
        self.last_cmd = now

    def release_if_quiet(self, now, quiet_sec):
        if (self.active and self.last_cmd is not None
                and now - self.last_cmd > quiet_sec):
            self.active = False   # hold current position, filters bypassed

    def step(self, dt, motion_time):
        if not self.active:
            return self.current

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

        # Startup safety
        self.declare_parameter("startup_hold_sec", 1.5)
        self.declare_parameter("max_speed", 60.0)          # deg/s backstop

        # Idle sway (degrees / seconds). Amplitude 0 disables an axis.
        self.declare_parameter("idle_enable", True)
        self.declare_parameter("idle_resume_sec", 5.0)     # quiet time before sway/hold resumes
        self.declare_parameter("idle_fade_sec", 2.0)       # fade in/out time
        self.declare_parameter("idle_pan_amp", 8.0)
        self.declare_parameter("idle_pan_period", 9.0)
        self.declare_parameter("idle_tilt_amp", 0.0)
        self.declare_parameter("idle_tilt_period", 10.0)

        gp = self.get_parameter
        self.min_cutoff = gp("min_cutoff").get_parameter_value().double_value
        self.beta = gp("beta").get_parameter_value().double_value
        self.d_cutoff = gp("d_cutoff").get_parameter_value().double_value
        self.motion_profile = gp("motion_profile").get_parameter_value().string_value
        self.motion_time = gp("motion_time").get_parameter_value().double_value

        device_name = gp("device_name").get_parameter_value().string_value

        self.startup_hold = float(gp("startup_hold_sec").value)
        self.max_speed = float(gp("max_speed").value)

        self.idle_enable = bool(gp("idle_enable").value)
        self.idle_resume = float(gp("idle_resume_sec").value)
        self.idle_fade = float(gp("idle_fade_sec").value)

        # name -> (amplitude, period)
        self.idle = {
            "pan": (float(gp("idle_pan_amp").value), float(gp("idle_pan_period").value)),
            "tilt": (float(gp("idle_tilt_amp").value), float(gp("idle_tilt_period").value)),
        }

        self.control_period = 0.05  # 20 Hz

        def make_axis(name):
            return Axis(
                name,
                self.control_period,
                self.min_cutoff,
                self.beta,
                self.d_cutoff,
            )

        self.pan = make_axis("pan")
        self.tilt = make_axis("tilt")
        self.axes = [self.pan, self.tilt]

        self.driver = DynamixelDriver(device_name=device_name)
        # Calibrate BEFORE enabling torque so zero = where the head really is
        self.driver.calibrate_zero()
        self.driver.enable()

        # Startup / idle state
        self.t0 = self.now()
        self.started = False
        self.idle_ok_after = self.t0 + self.startup_hold + 2.0
        self.last_cmd = None
        self.idle_scale = 0.0
        self.sway_t0 = self.t0
        self.prev_cmd = {"pan": 0.0, "tilt": 0.0}

        self.create_subscription(Float32, "/head/pan_target", self.pan_cb, 10)
        self.create_subscription(Float32, "/head/tilt_target", self.tilt_cb, 10)

        self.timer = self.create_timer(self.control_period, self.control_loop)
        self.get_logger().info(
            f"Head node active on {device_name}. "
            f"Holding still for {self.startup_hold}s, idle sway "
            f"{'on' if self.idle_enable else 'off'}. "
            f"Filter: One Euro (min_cutoff={self.min_cutoff}, beta={self.beta}, d_cutoff={self.d_cutoff})"
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ------------------------------------------------------------ callbacks

    def on_target(self, axis, value):
        now = self.now()
        if axis.last_cmd is None:
            self.get_logger().info(
                f"First target received for {axis.name}: {value:.1f} deg"
            )
        axis.command(value, now)
        self.last_cmd = now

    def pan_cb(self, msg: Float32):
        self.on_target(self.pan, msg.data)

    def tilt_cb(self, msg: Float32):
        self.on_target(self.tilt, msg.data)

    # ------------------------------------------------------------ helpers

    def report_startup(self):
        """Log what actually happened at torque-on, and who else publishes targets."""

        drift = self.driver.startup_drift()
        text = ", ".join(
            f"{k}={'read failed' if v is None else f'{v:+d} ticks'}"
            for k, v in drift.items()
        )
        self.get_logger().info(f"Startup drift after torque-on: {text}")

        if any(v is not None and abs(v) > 5 for v in drift.values()):
            self.get_logger().warn(
                "A motor moved after torque-on (>5 ticks). "
                "Motion is coming from the hardware side, not from filters."
            )

        for topic in ("/head/pan_target", "/head/tilt_target"):
            n = self.count_publishers(topic)
            if n > 0:
                self.get_logger().warn(
                    f"{n} node(s) already publish {topic}. "
                    f"Check `ros2 topic info {topic} -v`."
                )

    def sway(self, name, ts):
        """Offset in degrees, zero at ts=0 (two mixed sines, less mechanical)."""
        amp, period = self.idle[name]
        if amp <= 0.0 or period <= 0.0:
            return 0.0
        w = 2.0 * math.pi / period
        return amp * (0.7 * math.sin(w * ts) + 0.3 * math.sin(w * ts / 1.7))

    def update_idle_scale(self, now):
        """Fade sway in when the head is quiet, out while targets are commanded."""
        last = self.last_cmd
        quiet = last is None or (now - last) > self.idle_resume
        target = 1.0 if (self.idle_enable and quiet and now >= self.idle_ok_after) else 0.0

        step = self.control_period / max(self.idle_fade, 0.1)
        s = self.idle_scale
        s = min(target, s + step) if target > s else max(target, s - step)
        self.idle_scale = s

        # Restart the sway phase from zero whenever it is fully off
        if s == 0.0:
            self.sway_t0 = now

        return s

    def limit(self, name, value):
        """Backstop: never command faster than max_speed deg/s."""
        max_step = self.max_speed * self.control_period
        prev = self.prev_cmd[name]
        new = prev + max(-max_step, min(max_step, value - prev))
        self.prev_cmd[name] = new
        return new

    # ------------------------------------------------------------ control

    def control_loop(self):
        dt = self.control_period
        mt = self.motion_time
        now = self.now()

        # Startup hold: send nothing, motors keep their start pose.
        if now - self.t0 < self.startup_hold:
            return

        if not self.started:
            self.started = True
            self.report_startup()
            self.get_logger().info("Startup hold finished. Head is live.")

        for ax in self.axes:
            ax.release_if_quiet(now, self.idle_resume)

        head_s = self.update_idle_scale(now)
        th = now - self.sway_t0

        # Hardware Command
        pan = self.pan.step(dt, mt) + head_s * self.sway("pan", th)
        tilt = self.tilt.step(dt, mt) + head_s * self.sway("tilt", th)
        self.driver.set_pan(self.limit("pan", pan))
        self.driver.set_tilt(self.limit("tilt", tilt))

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