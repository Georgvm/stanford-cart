"""
odrive_steering_node — ROS 2 wrapper around the cart's ODrive S1 steering.

Subscribes:
  /cmd_steer         std_msgs/Float64   column angle in DEGREES (left = +)
  /run_demo          std_msgs/Bool      master arm; idles ODrive when false
  /estop_pressed     std_msgs/Bool      latches IDLE while true

Publishes:
  /steering/state    sensor_msgs/JointState  current column angle + velocity

Hardware contract (from the cart repo's docs/steering.md):
  - ODrive S1 over USB-C
  - M8325s motor, HTD 5M belt, 20T motor → 60T column = 3:1 reduction
  - 3 motor turns == 1 wheel turn == 360° at the column
  - POSITION_CONTROL + TRAP_TRAJ planner with vel/accel/decel limits
  - 300 ms axis watchdog: host MUST call axis.watchdog_feed() within window
    or the ODrive disarms itself.
  - Soft column-angle limits enforced in software (params below)

Safety:
  - Watchdog fed at 50 Hz from a dedicated timer, NOT from the command callback
    (commands may arrive slowly; the watchdog must stay fed regardless).
  - On /estop_pressed → IDLE the axis (motor goes limp; cart's hardware brake
    actuator is what actually stops the cart — handled by mega_pedals_node).
  - On node shutdown, drive to 0° column then IDLE.
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64


def _column_deg_to_motor_turns(deg: float, belt_ratio: float) -> float:
    return (deg / 360.0) * belt_ratio


def _motor_turns_to_column_deg(turns: float, belt_ratio: float) -> float:
    return (turns / belt_ratio) * 360.0


class OdriveSteeringNode(Node):
    def __init__(self):
        super().__init__("odrive_steering")

        self.declare_parameter("belt_ratio", 3.0)
        self.declare_parameter("column_min_deg", -270.0)
        self.declare_parameter("column_max_deg", 270.0)
        self.declare_parameter("trap_vel_max", 8.0)        # motor turns/s
        self.declare_parameter("trap_accel_max", 15.0)
        self.declare_parameter("trap_decel_max", 15.0)
        self.declare_parameter("watchdog_timeout_s", 0.30)
        self.declare_parameter("connect_timeout_s", 10.0)
        self.declare_parameter("dry_run", False)

        self.belt = float(self.get_parameter("belt_ratio").value)
        self.col_min = float(self.get_parameter("column_min_deg").value)
        self.col_max = float(self.get_parameter("column_max_deg").value)
        self.trap_vel = float(self.get_parameter("trap_vel_max").value)
        self.trap_accel = float(self.get_parameter("trap_accel_max").value)
        self.trap_decel = float(self.get_parameter("trap_decel_max").value)
        self.wdt_s = float(self.get_parameter("watchdog_timeout_s").value)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.lock = threading.Lock()
        self.armed = False
        self.estop = False
        self.last_cmd_deg = 0.0
        self.start_pos = 0.0
        self.odrv = None
        self.axis = None
        self._AxisState = None
        self._InputMode = None

        self._connect_or_die()

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Float64, "/cmd_steer", self._on_cmd_steer, 10)
        self.create_subscription(Bool, "/run_demo", self._on_run_demo, latching)
        self.create_subscription(Bool, "/estop_pressed", self._on_estop, latching)
        self.state_pub = self.create_publisher(JointState, "/steering/state", 10)

        self.create_timer(0.02, self._watchdog_tick)   # 50 Hz, must be < wdt_s
        self.create_timer(0.05, self._publish_state)   # 20 Hz state
        self.get_logger().info(
            f"odrive_steering up. belt={self.belt}:1, "
            f"limits=±{min(self.col_max, -self.col_min):.0f}°, "
            f"trap=({self.trap_vel}, {self.trap_accel}, {self.trap_decel}), "
            f"dry_run={self.dry_run}"
        )

    # ----- ODrive lifecycle -------------------------------------------------

    def _connect_or_die(self):
        if self.dry_run:
            self.get_logger().warn("dry_run=True — ODrive will NOT be opened.")
            return

        import odrive  # type: ignore
        from odrive.enums import AxisState, ControlMode, InputMode  # type: ignore

        self._AxisState = AxisState
        self._InputMode = InputMode

        timeout = float(self.get_parameter("connect_timeout_s").value)
        self.get_logger().info(f"connecting to ODrive (timeout {timeout}s)...")
        self.odrv = odrive.find_any(timeout=timeout)
        self.axis = self.odrv.axis0
        self.get_logger().info(
            f"connected: serial={self.odrv.serial_number}, "
            f"vbus={self.odrv.vbus_voltage:.1f}V"
        )

        # Quiet any residual watchdog from a previous (uncleanly exited) session.
        cfg = self.axis.config if hasattr(self.axis, "config") else self.axis
        try:
            self.axis.watchdog_feed()
            cfg.enable_watchdog = False
        except Exception as e:
            self.get_logger().warn(f"residual watchdog quiet failed: {e!r}")

        if self.axis.active_errors != 0:
            self.get_logger().warn(f"clearing active errors: {self.axis.active_errors}")
            self.odrv.clear_errors()
            time.sleep(0.3)

        ctrl_cfg = self.axis.controller.config if hasattr(self.axis.controller, "config") else self.axis.controller
        trap_cfg = self.axis.trap_traj.config if hasattr(self.axis.trap_traj, "config") else self.axis.trap_traj
        ctrl_cfg.control_mode = ControlMode.POSITION_CONTROL
        ctrl_cfg.input_mode = InputMode.TRAP_TRAJ
        try:
            ctrl_cfg.vel_limit = self.trap_vel
        except Exception:
            pass
        trap_cfg.vel_limit = self.trap_vel
        trap_cfg.accel_limit = self.trap_accel
        trap_cfg.decel_limit = self.trap_decel

        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        time.sleep(0.35)
        if self.axis.current_state != AxisState.CLOSED_LOOP_CONTROL:
            raise RuntimeError(
                f"ODrive failed CLOSED_LOOP_CONTROL: state={self.axis.current_state}, "
                f"disarm={self.axis.disarm_reason}"
            )

        self.start_pos = float(self.axis.pos_estimate)
        self.get_logger().info(
            f"zero reference = {self.start_pos:.4f} turns "
            f"(operator MUST have wheel centered at startup)"
        )

        # Configure the watchdog window now; arm it on the first /run_demo true.
        self._cfg = cfg
        try:
            if hasattr(cfg, "watchdog_timeout"):
                cfg.watchdog_timeout = self.wdt_s
            self.get_logger().info(
                f"axis watchdog configured ({self.wdt_s * 1000:.0f} ms), "
                "armed on first /run_demo true"
            )
        except Exception as e:
            self.get_logger().error(f"watchdog config failed: {e!r}")

    def _arm(self):
        if self.dry_run or self.axis is None or self.armed:
            return
        try:
            self.axis.requested_state = self._AxisState.CLOSED_LOOP_CONTROL
            self.axis.controller.input_pos = self.start_pos
            self.axis.watchdog_feed()
            self._cfg.enable_watchdog = True
            self.armed = True
            self.get_logger().info("axis ARMED (watchdog enabled)")
        except Exception as e:
            self.get_logger().error(f"arm failed: {e!r}")

    def _idle(self, reason: str):
        if self.dry_run or self.axis is None or not self.armed:
            return
        try:
            self._cfg.enable_watchdog = False
        except Exception:
            pass
        try:
            self.axis.controller.input_pos = self.start_pos
            time.sleep(0.4)  # let trap_traj move toward 0 before we cut power
            self.axis.requested_state = self._AxisState.IDLE
        except Exception as e:
            self.get_logger().error(f"idle failed ({reason}): {e!r}")
        self.armed = False
        self.get_logger().warn(f"axis IDLE — {reason}")

    # ----- subscribers ------------------------------------------------------

    def _on_run_demo(self, msg: Bool):
        with self.lock:
            if msg.data and not self.estop:
                self._arm()
            elif not msg.data:
                self._idle("/run_demo false")

    def _on_estop(self, msg: Bool):
        with self.lock:
            self.estop = bool(msg.data)
            if self.estop:
                self._idle("/estop_pressed true")

    def _on_cmd_steer(self, msg: Float64):
        with self.lock:
            if not self.armed or self.estop or self.axis is None:
                return
            deg = max(self.col_min, min(self.col_max, float(msg.data)))
            turns = _column_deg_to_motor_turns(deg, self.belt)
            try:
                self.axis.controller.input_pos = self.start_pos + turns
                self.last_cmd_deg = deg
            except Exception as e:
                self.get_logger().error(f"input_pos write failed: {e!r}")
                self._idle("input_pos write failed")

    # ----- timers -----------------------------------------------------------

    def _watchdog_tick(self):
        with self.lock:
            if not self.armed or self.dry_run or self.axis is None:
                return
            try:
                self.axis.watchdog_feed()
            except Exception:
                pass

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["steering_column"]
        try:
            if self.axis is not None and not self.dry_run:
                col_deg = _motor_turns_to_column_deg(
                    self.axis.pos_estimate - self.start_pos, self.belt,
                )
                vel = _motor_turns_to_column_deg(self.axis.vel_estimate, self.belt)
                msg.position = [math.radians(col_deg)]
                msg.velocity = [math.radians(vel)]
            else:
                msg.position = [math.radians(self.last_cmd_deg)]
                msg.velocity = [0.0]
        except Exception:
            msg.position = [0.0]
            msg.velocity = [0.0]
        self.state_pub.publish(msg)

    # ----- shutdown ---------------------------------------------------------

    def destroy_node(self):
        self._idle("node shutdown")
        super().destroy_node()


def main():
    rclpy.init()
    node = OdriveSteeringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
