"""
safety_gate — the only thing publishing /cmd_vel into cmd_vel_to_dbw.

Topic flow:
  Nav2 controller_server
        v
   /cmd_vel_nav  (geometry_msgs/Twist)
        v
  [safety_gate] +-- /run_demo (Bool, latched)
                +-- /estop_pressed (Bool, latched, published by mega_pedals)
                +-- /perception/nearest_obstacle_m (Float32)
                +-- watchdog: no /cmd_vel_nav in N ms => zero Twist
        v
   /cmd_vel  (geometry_msgs/Twist)
        v
  cmd_vel_to_dbw -> ODrive + Mega

Rules (most-restrictive wins):
  - /estop_pressed true                          => publish zero Twist
  - /run_demo false                              => publish zero Twist
  - /perception/nearest_obstacle_m < hard_m      => publish zero Twist
  - hard_m <= obstacle_m < soft_m                => scale linear.x linearly
  - no /cmd_vel_nav for watchdog_s                => publish zero Twist
  - else                                          => pass-through with linear cap
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String


class SafetyGateNode(Node):
    def __init__(self):
        super().__init__("safety_gate")

        self.declare_parameter("hard_distance_m", 4.0)
        self.declare_parameter("soft_distance_m", 9.0)
        self.declare_parameter("watchdog_s", 0.5)
        self.declare_parameter("max_linear_velocity", 2.5)  # m/s
        self.declare_parameter("max_angular_velocity", 0.8) # rad/s

        self.hard_m = float(self.get_parameter("hard_distance_m").value)
        self.soft_m = float(self.get_parameter("soft_distance_m").value)
        self.wdt = float(self.get_parameter("watchdog_s").value)
        self.max_v = float(self.get_parameter("max_linear_velocity").value)
        self.max_w = float(self.get_parameter("max_angular_velocity").value)

        self.run_demo = False
        self.estop = False
        self.nearest_m = float("inf")
        self.last_cmd_t = self.get_clock().now()

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/run_demo", self._on_run_demo, latching)
        self.create_subscription(Bool, "/estop_pressed", self._on_estop, latching)
        self.create_subscription(Float32, "/perception/nearest_obstacle_m",
                                 self._on_nearest, 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav_cmd, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/cart_safety/status", 10)

        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"safety_gate up. hard={self.hard_m}m, soft={self.soft_m}m, "
            f"watchdog={self.wdt}s, max_v={self.max_v} m/s"
        )

    def _on_run_demo(self, msg: Bool):
        if msg.data != self.run_demo:
            self.get_logger().info(f"/run_demo -> {msg.data}")
        self.run_demo = bool(msg.data)

    def _on_estop(self, msg: Bool):
        if msg.data and not self.estop:
            self.get_logger().error("E-STOP latched")
        self.estop = bool(msg.data)

    def _on_nearest(self, msg: Float32):
        self.nearest_m = float(msg.data)

    def _on_nav_cmd(self, msg: Twist):
        self.last_cmd_t = self.get_clock().now()

        out = Twist()
        if not self.run_demo:
            self._publish_stop("/run_demo false")
            return
        if self.estop:
            self._publish_stop("/estop_pressed")
            return
        if self.nearest_m < self.hard_m:
            self._publish_stop(f"obstacle {self.nearest_m:.1f}m < {self.hard_m}m")
            return

        scale = 1.0
        if self.nearest_m < self.soft_m:
            scale = (self.nearest_m - self.hard_m) / (self.soft_m - self.hard_m)
            scale = max(0.0, min(1.0, scale))

        out.linear.x = max(-self.max_v, min(self.max_v, msg.linear.x * scale))
        out.angular.z = max(-self.max_w, min(self.max_w, msg.angular.z))
        self.cmd_pub.publish(out)
        self._status(
            f"OK v={out.linear.x:+.2f} w={out.angular.z:+.2f} "
            f"obs={self.nearest_m:.1f}m scale={scale:.2f}"
        )

    def _tick(self):
        dt = (self.get_clock().now() - self.last_cmd_t).nanoseconds * 1e-9
        if dt > self.wdt:
            self._publish_stop(f"watchdog dt={dt:.2f}s")

    def _publish_stop(self, reason: str):
        self.cmd_pub.publish(Twist())
        self._status(f"STOP: {reason}")

    def _status(self, s: str):
        m = String(); m.data = s
        self.status_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(SafetyGateNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
