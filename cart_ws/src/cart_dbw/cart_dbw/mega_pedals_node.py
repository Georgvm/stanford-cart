"""
mega_pedals_node — ROS 2 wrapper around the cart's Arduino Mega pedal firmware.

Speaks the protocol from sketches/pedal_control/pedal_control.ino:
  Host -> Mega:
    G <0..GAS_POT_MAX>\n      gas target (normalized pot)
    B <0..BRAKE_POT_MAX>\n    brake target
    S\n                       both targets = MIN (release)
    H\n                       heartbeat-only (no target change)
    D\n                       graceful disarm (parks the watchdog)
  Mega -> Host:
    EVT,ESTOP,1|0             e-stop edge
    STAT,g=,b=,tg=,tb=,hb=,fs=,es=
    INFO,/ERR,                logs

The Mega's own watchdog kills both pedals if the host goes silent for >300 ms,
so we MUST tick at least every 200 ms. We tick at 50 Hz (every 20 ms).

Subscribes:
  /cmd_throttle      std_msgs/Float64   0..1, scaled by FSD_GAS_LIMIT cap
  /cmd_brake         std_msgs/Float64   0..1, scaled by BRAKE_POT_MAX
  /run_demo          std_msgs/Bool      master arm; releases pedals when false
  /estop_pressed     std_msgs/Bool      mirror; we already get EVT,ESTOP from Mega

Publishes:
  /estop_pressed     std_msgs/Bool      sourced from Mega EVT,ESTOP (latched)
  /pedals/state      std_msgs/String    pass-through of Mega STAT line for debug

Authority cap (mirrors limits.py from the cart repo):
  GAS_POT_MAX        = 0.68    (mechanical max)
  GLOBAL_SPEED_LIMIT = 0.45    (project-wide governor)
  FSD_GAS_LIMIT      = 0.25    (autonomy cap — never exceeded)
  effective cap      = min of all three = 0.25 by default

The cap is enforced HERE (not just in the Nav2 controller) so a buggy upstream
node cannot make the cart go faster than the limits hierarchy allows.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float64, String


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class MegaPedalsNode(Node):
    def __init__(self):
        super().__init__("mega_pedals")

        self.declare_parameter("port", "")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("gas_pot_max", 0.68)
        self.declare_parameter("brake_pot_max", 0.45)
        self.declare_parameter("global_speed_limit", 0.45)
        self.declare_parameter("fsd_gas_limit", 0.25)
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("dry_run", False)

        self.gas_max = float(self.get_parameter("gas_pot_max").value)
        self.brake_max = float(self.get_parameter("brake_pot_max").value)
        self.gas_cap = min(
            self.gas_max,
            float(self.get_parameter("global_speed_limit").value),
            float(self.get_parameter("fsd_gas_limit").value),
        )
        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.hz = float(self.get_parameter("control_hz").value)

        self.lock = threading.Lock()
        self.ser = None
        self.target_gas = 0.0
        self.target_brake = 0.0
        self.armed = False
        self.estop = False
        self.faulted = False
        self.last_send_ok_s = time.monotonic()
        self._rx_buf = bytearray()

        self._open_serial_or_die()

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Float64, "/cmd_throttle", self._on_throttle, 10)
        self.create_subscription(Float64, "/cmd_brake", self._on_brake, 10)
        self.create_subscription(Bool, "/run_demo", self._on_run_demo, latching)
        self.estop_pub = self.create_publisher(Bool, "/estop_pressed", latching)
        self.state_pub = self.create_publisher(String, "/pedals/state", 10)

        # Latch a starting estop=false (TRANSIENT_LOCAL → late subscribers see it)
        m = Bool(); m.data = False
        self.estop_pub.publish(m)

        period = 1.0 / self.hz
        self.create_timer(period, self._tick)
        self.create_timer(0.01, self._poll_serial)  # 100 Hz drain

        self.get_logger().info(
            f"mega_pedals up. cap={self.gas_cap:.3f} (autonomy max throttle), "
            f"brake_max={self.brake_max:.3f}, hz={self.hz}, dry_run={self.dry_run}"
        )

    def _open_serial_or_die(self):
        if self.dry_run:
            self.get_logger().warn("dry_run=True — Mega serial NOT opened.")
            return
        import serial  # type: ignore
        port = self.get_parameter("port").value or _autodetect_mega()
        if not port:
            raise RuntimeError(
                "Couldn't auto-detect Mega. Set parameter 'port' "
                "(e.g. /dev/ttyACM0). Use ros2 run cart_safety list_serial_ports."
            )
        baud = int(self.get_parameter("baud").value)
        self.get_logger().info(f"opening Mega @ {port} ({baud} baud)")
        self.ser = serial.Serial(port, baud, timeout=0.5)
        # Mega resets on port open; let bootloader settle.
        time.sleep(2.0)

    # ----- subscribers ------------------------------------------------------

    def _on_throttle(self, msg: Float64):
        with self.lock:
            self.target_gas = _clamp(float(msg.data), 0.0, 1.0) * self.gas_cap

    def _on_brake(self, msg: Float64):
        with self.lock:
            self.target_brake = _clamp(float(msg.data), 0.0, 1.0) * self.brake_max

    def _on_run_demo(self, msg: Bool):
        with self.lock:
            new = bool(msg.data)
            if new == self.armed:
                return
            self.armed = new
            if not self.armed:
                self.target_gas = 0.0
                self.target_brake = 0.0
                self.get_logger().warn("/run_demo false — releasing pedals")

    # ----- timers -----------------------------------------------------------

    def _tick(self):
        """50 Hz: send a G/B pair (or release if not armed/estop)."""
        with self.lock:
            if self.faulted:
                return
            if self.estop or not self.armed:
                gas = 0.0
                brake = self.brake_max if self.estop else 0.0
            else:
                gas = _clamp(self.target_gas, 0.0, self.gas_cap)
                brake = _clamp(self.target_brake, 0.0, self.brake_max)

            if self.dry_run or self.ser is None:
                self.last_send_ok_s = time.monotonic()
                return
            payload = f"G {gas:.3f}\nB {brake:.3f}\n".encode("ascii")
            try:
                self.ser.write(payload)
                self.last_send_ok_s = time.monotonic()
            except Exception as e:
                self.faulted = True
                self.get_logger().error(f"serial write failed: {e!r} — bailing")

    def _poll_serial(self):
        """100 Hz: drain Mega → parse EVT,ESTOP and emit STAT."""
        if self.dry_run or self.ser is None or self.faulted:
            return
        try:
            n = self.ser.in_waiting
            if n > 0:
                self._rx_buf.extend(self.ser.read(n))
        except Exception as e:
            self.faulted = True
            self.get_logger().error(f"serial read failed: {e!r}")
            return

        while b"\n" in self._rx_buf:
            line, _, rest = self._rx_buf.partition(b"\n")
            self._rx_buf = bytearray(rest)
            text = line.decode("ascii", errors="replace").strip()
            if not text:
                continue
            if text.startswith("EVT,ESTOP,"):
                active = text.endswith(",1")
                if active != self.estop:
                    self.estop = active
                    self.get_logger().error(
                        f"E-STOP {'ENGAGED' if active else 'released'} (Mega: {text})"
                    )
                    m = Bool(); m.data = active
                    self.estop_pub.publish(m)
            elif text.startswith("STAT,"):
                m = String(); m.data = text
                self.state_pub.publish(m)
            elif text.startswith(("INFO,", "ERR,")):
                self.get_logger().info(f"[mega] {text}")

    # ----- shutdown ---------------------------------------------------------

    def destroy_node(self):
        if self.ser is not None and not self.dry_run:
            try:
                # Graceful disarm: D parks the watchdog so the brake doesn't slam
                # after we close the port.
                for _ in range(3):
                    self.ser.write(b"D\n")
                    time.sleep(0.02)
            except Exception:
                pass
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def _autodetect_mega() -> str:
    """Best-effort. Prefers Arduino USB VIDs; refuses ODrive VIDs."""
    try:
        import serial.tools.list_ports
    except ImportError:
        return ""
    ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}
    ODRIVE_VIDS = {0x1209, 0x0483}
    ports = [
        p for p in serial.tools.list_ports.comports()
        if any(t in p.device for t in ("usbmodem", "ttyACM", "ttyUSB", "usbserial"))
    ]
    arduino = [p for p in ports if p.vid in ARDUINO_VIDS]
    if arduino:
        return arduino[0].device
    non_odrive = [p for p in ports if p.vid not in ODRIVE_VIDS]
    if non_odrive:
        return min(non_odrive, key=lambda p: len(p.device)).device
    return ""


def main():
    rclpy.init()
    node = MegaPedalsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
