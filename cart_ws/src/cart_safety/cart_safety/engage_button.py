"""
engage_button — dead-man switch wired to a controller button.

Subscribes:
  /joy   sensor_msgs/Joy

Publishes (latched):
  /run_demo   std_msgs/Bool

While the configured button is held → /run_demo true. On release → false.
Republishes only on edges (cuts /joy spam down to actual transitions).

This node is the ONLY thing that should publish /run_demo from a human in
the loop. Autonomy demos can override with a one-shot `ros2 topic pub
--once /run_demo std_msgs/Bool data:=true`, but during driving sessions a
held button is the right primitive — let go and the cart stops.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class EngageButtonNode(Node):
    def __init__(self):
        super().__init__("engage_button")
        self.declare_parameter("engage_button_index", 4)   # L1 on DualSense
        self.btn = int(self.get_parameter("engage_button_index").value)

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(Bool, "/run_demo", latching)
        self.create_subscription(Joy, "/joy", self._on_joy, 10)

        # Publish initial false so subscribers latch a known state.
        self._held = False
        m = Bool(); m.data = False
        self.pub.publish(m)
        self.get_logger().info(
            f"engage_button up. dead-man = button[{self.btn}]"
        )

    def _on_joy(self, msg: Joy):
        if self.btn >= len(msg.buttons):
            return
        held = bool(msg.buttons[self.btn])
        if held == self._held:
            return
        self._held = held
        m = Bool(); m.data = held
        self.pub.publish(m)
        self.get_logger().info(f"engage -> {held}")


def main():
    rclpy.init()
    rclpy.spin(EngageButtonNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
