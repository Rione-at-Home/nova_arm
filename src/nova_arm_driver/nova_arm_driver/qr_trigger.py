#!/usr/bin/env python3
"""
qr_trigger.py

Watches the RealSense color image for a QR code. When the expected code is
seen (for a few consecutive frames, to avoid false positives) a countdown
starts. After `delay_sec` (default 6 s) a std_msgs/String is published on
/presenter_trigger. A cooldown then prevents re-triggering.

Test only: it does NOT move the robot.

Run:
    python3 qr_trigger.py
    python3 qr_trigger.py --ros-args -p expected_payload:=NOVA_START -p show:=true

Watch the result:
    ros2 topic echo /presenter_trigger
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

IDLE, ARMED, COOLDOWN = "IDLE", "ARMED", "COOLDOWN"


class QRTrigger(Node):

    def __init__(self):
        super().__init__("qr_trigger")

        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("delay_sec", 6.0)
        self.declare_parameter("cooldown_sec", 15.0)
        self.declare_parameter("expected_payload", "")   # "" = accept any QR
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("cancel_if_lost", False)  # cancel if QR removed
        self.declare_parameter("lost_grace_sec", 1.5)
        self.declare_parameter("show", False)            # debug window

        p = self.get_parameter
        self.delay = p("delay_sec").value
        self.cooldown = p("cooldown_sec").value
        self.expected = p("expected_payload").value
        self.confirm_frames = p("confirm_frames").value
        self.cancel_if_lost = p("cancel_if_lost").value
        self.lost_grace = p("lost_grace_sec").value
        self.show = p("show").value
        topic = p("image_topic").value

        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()

        self.state = IDLE
        self.consecutive = 0
        self.armed_at = None
        self.last_seen = None
        self.cooldown_until = None
        self.last_logged_sec = None

        self.create_subscription(Image, topic, self.image_callback, 1)
        self.trigger_pub = self.create_publisher(String, "/presenter_trigger", 10)
        self.create_timer(0.1, self.tick)

        self.get_logger().info(
            f"Listening on {topic} | delay={self.delay}s | "
            f"expected='{self.expected or '<any>'}'"
        )

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ---------------------------------------------------------- image

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        data, points, _ = self.detector.detectAndDecode(frame)
        valid = bool(data) and (not self.expected or data == self.expected)

        if valid:
            self.consecutive += 1
            self.last_seen = self.now()
            if self.state == IDLE and self.consecutive >= self.confirm_frames:
                self.state = ARMED
                self.armed_at = self.now()
                self.last_logged_sec = None
                self.get_logger().info(
                    f"QR '{data}' detected. Trigger in {self.delay:.0f}s..."
                )
        else:
            self.consecutive = 0

        if self.show:
            if points is not None and len(points) > 0:
                pts = points.reshape(-1, 2).astype(int)
                cv2.polylines(frame, [pts], True, (0, 255, 0) if valid else (0, 0, 255), 3)
            cv2.putText(frame, self.state, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            cv2.imshow("qr_trigger", frame)
            cv2.waitKey(1)

    # ---------------------------------------------------------- state

    def tick(self):
        t = self.now()

        if self.state == ARMED:
            if (self.cancel_if_lost and self.last_seen is not None
                    and t - self.last_seen > self.lost_grace):
                self.get_logger().info("QR lost. Countdown cancelled.")
                self.state = IDLE
                self.consecutive = 0
                return

            remaining = self.delay - (t - self.armed_at)
            sec = int(remaining) + 1
            if remaining > 0 and sec != self.last_logged_sec:
                self.last_logged_sec = sec
                self.get_logger().info(f"  {sec}...")

            if remaining <= 0:
                msg = String()
                msg.data = "start"
                self.trigger_pub.publish(msg)
                self.get_logger().info("TRIGGER published on /presenter_trigger")
                self.state = COOLDOWN
                self.cooldown_until = t + self.cooldown

        elif self.state == COOLDOWN:
            if t >= self.cooldown_until:
                self.state = IDLE
                self.consecutive = 0
                self.get_logger().info("Ready for next trigger.")


def main(args=None):
    rclpy.init(args=args)
    node = QRTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()