"""
cmd_vel_to_dbw — translate Nav2's geometry_msgs/Twist into our DBW topics.

Nav2's regulated_pure_pursuit emits Twist (linear.x m/s, angular.z rad/s).
The cart is bicycle-model-ish; we convert with:

  steering_column_deg = degrees( atan2(wheelbase * angular_z, max(linear_x, eps)) )
                         * column_per_road_wheel_ratio
  throttle_norm       = pid(linear_x_target, current_speed)         (open-loop OK at v0)
  brake_norm          = positive part of speed_error when decelerating

Subscribes:
  /cmd_vel            geometry_msgs/Twist    from Nav2 (or safety_gate output)
  /odometry/filtered  nav_msgs/Odometry      for current speed (open-loop if absent)

Publishes:
  /cmd_steer          std_msgs/Float64       column degrees
  /cmd_throttle       std_msgs/Float64       0..1
  /cmd_brake          std_msgs/Float64       0..1

Subscribed-to /cmd_vel comes from the SAFETY GATE, not directly from Nav2.
The wiring is:
   Nav2 -> /cmd_vel_nav -> safety_gate -> /cmd_vel -> THIS NODE -> DBW topics
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class CmdVelToDbwNode(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_dbw")

        self.declare_parameter("wheelbase_m", 1.65)            # measure on cart
        self.declare_parameter("max_steer_column_deg", 270.0)  # limits.py STEERING_MAX_DEG
        self.declare_parameter("max_linear_velocity", 2.5)     # m/s ~5.6 mph; cap
        self.declare_parameter("speed_kp", 0.4)                # throttle per (m/s error)
        self.declare_parameter("brake_kp", 0.6)
        self.declare_parameter("min_speed_for_steering", 0.1)  # m/s; below this, hold straight
        self.declare_parameter("steering_smoothing", 0.3)      # EMA on output, 0=raw, 1=frozen

        self.L = float(self.get_parameter("wheelbase_m").value)
        self.max_steer = float(self.get_parameter("max_steer_column_deg").value)
        self.max_v = float(self.get_parameter("max_linear_velocity").value)
        self.kp_v = float(self.get_parameter("speed_kp").value)
        self.kp_b = float(self.get_parameter("brake_kp").value)
        self.min_steer_v = float(self.get_parameter("min_speed_for_steering").value)
        self.alpha = float(self.get_parameter("steering_smoothing").value)

        self.current_speed = 0.0
        self.smoothed_steer = 0.0

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self._on_odom, 10)

        self.steer_pub = self.create_publisher(Float64, "/cmd_steer", 10)
        self.throttle_pub = self.create_publisher(Float64, "/cmd_throttle", 10)
        self.brake_pub = self.create_publisher(Float64, "/cmd_brake", 10)

        self.get_logger().info(
            f"cmd_vel_to_dbw up. wheelbase={self.L}m, max_steer=±{self.max_steer:.0f}°, "
            f"max_v={self.max_v:.2f} m/s"
        )

    def _on_odom(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.current_speed = math.sqrt(vx * vx + vy * vy)

    def _on_cmd_vel(self, msg: Twist):
        v_target = max(-self.max_v, min(self.max_v, msg.linear.x))
        omega = msg.angular.z

        # Bicycle-model steering at the road wheel, then * column-per-roadwheel.
        # Cart has manual steering through the column; assume the operator ratio
        # rod-to-tire is ~1:1 in our column-degrees-to-tire-angle for now (verify
        # mechanically). For safety we cap aggressively.
        if abs(v_target) < self.min_steer_v:
            road_steer_rad = 0.0
        else:
            road_steer_rad = math.atan2(self.L * omega, abs(v_target))
        column_deg = math.degrees(road_steer_rad)
        column_deg = max(-self.max_steer, min(self.max_steer, column_deg))

        # Smooth on the output to soften any controller jitter. 0.3 EMA = quick
        # but not chattery.
        self.smoothed_steer = (
            self.alpha * self.smoothed_steer + (1 - self.alpha) * column_deg
        )

        # Throttle/brake from speed error. Open-loop OK at v0 — cmd_vel_to_dbw
        # is not the place for closed-loop tuning; that's the controller_server.
        speed_err = v_target - self.current_speed
        throttle = max(0.0, self.kp_v * speed_err) if v_target > 0 else 0.0
        brake = max(0.0, -self.kp_b * speed_err) if v_target <= self.current_speed else 0.0

        # Publish.
        m = Float64(); m.data = float(self.smoothed_steer); self.steer_pub.publish(m)
        m = Float64(); m.data = float(min(1.0, throttle)); self.throttle_pub.publish(m)
        m = Float64(); m.data = float(min(1.0, brake)); self.brake_pub.publish(m)


def main():
    rclpy.init()
    rclpy.spin(CmdVelToDbwNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
