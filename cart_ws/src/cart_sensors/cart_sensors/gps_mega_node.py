"""
gps_mega_node — publish GPS fixes from a USB GPS receiver on the Thor.

(Historical name: when the GPS lived on the Arduino Mega's Serial1 and
was forwarded over USB this was a "Mega" passthrough. The current cart
has the GPS USB receiver wired directly to the Thor, so the autodetect
prefers dedicated GPS chipset VIDs over Arduino-class VIDs. Behavior /
topics unchanged.)

Publishes:
  /fix      sensor_msgs/NavSatFix
  /vel      geometry_msgs/TwistStamped     (course-over-ground from RMC)

Autodetect order (see _autodetect_gps_mega):
  1. u-blox / Garmin / SiRF / MediaTek dedicated GPS VIDs
  2. Arduino-class VIDs (FTDI, CH340, CP210x, ATmega) — last serial wins,
     to skip a co-plugged pedal Mega.

Override with the `port` parameter when autodetect picks the wrong device.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped


class GpsMegaNode(Node):
    def __init__(self):
        super().__init__("gps_mega")

        self.declare_parameter("port", "")            # e.g. /dev/ttyACM1
        self.declare_parameter("baud", 115200)
        self.declare_parameter("frame_id", "gps")

        self.frame_id = str(self.get_parameter("frame_id").value)

        import serial  # type: ignore
        port = self.get_parameter("port").value or _autodetect_gps_mega()
        if not port:
            raise RuntimeError(
                "Couldn't find the GPS Mega. Set 'port' parameter explicitly."
            )
        baud = int(self.get_parameter("baud").value)
        self.get_logger().info(f"opening GPS Mega @ {port} ({baud} baud)")
        self.ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(2.0)  # bootloader settle

        try:
            import pynmea2  # type: ignore
        except ImportError:
            raise RuntimeError("pynmea2 missing — pip install pynmea2")
        self.pynmea2 = pynmea2

        self.fix_pub = self.create_publisher(NavSatFix, "/fix", qos_profile_sensor_data)
        self.vel_pub = self.create_publisher(TwistStamped, "/vel", qos_profile_sensor_data)

        self.create_timer(0.05, self._poll)  # 20 Hz drain
        self.get_logger().info("gps_mega up.")

    def _poll(self):
        try:
            n = self.ser.in_waiting
            if n <= 0:
                return
            data = self.ser.read(n)
        except Exception as e:
            self.get_logger().error(f"serial read failed: {e!r}")
            return

        for raw in data.split(b"\n"):
            line = raw.decode("ascii", errors="replace").strip()
            if not line.startswith("$"):
                continue
            try:
                msg = self.pynmea2.parse(line)
            except Exception:
                continue
            self._handle(msg)

    def _handle(self, msg):
        now = self.get_clock().now().to_msg()
        if isinstance(msg, self.pynmea2.GGA):
            try:
                lat = float(msg.latitude)
                lon = float(msg.longitude)
                alt = float(msg.altitude or 0.0)
                qual = int(msg.gps_qual or 0)
            except (TypeError, ValueError):
                return
            f = NavSatFix()
            f.header.stamp = now
            f.header.frame_id = self.frame_id
            f.latitude, f.longitude, f.altitude = lat, lon, alt
            f.status.status = (
                NavSatStatus.STATUS_NO_FIX if qual == 0 else NavSatStatus.STATUS_FIX
            )
            f.status.service = NavSatStatus.SERVICE_GPS
            # NEO-6M typical ~2-3 m horizontal CEP without SBAS.
            sigma = 3.0 if qual == 1 else (1.5 if qual >= 2 else 99.9)
            f.position_covariance = [
                sigma * sigma, 0.0, 0.0,
                0.0, sigma * sigma, 0.0,
                0.0, 0.0, (sigma * 2) ** 2,
            ]
            f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(f)
        elif isinstance(msg, self.pynmea2.RMC):
            try:
                speed_kn = float(msg.spd_over_grnd or 0.0)  # knots
                course = float(msg.true_course or 0.0)      # deg, true north
            except (TypeError, ValueError):
                return
            v = speed_kn * 0.514444  # knots → m/s
            import math
            heading_rad = math.radians(course)
            t = TwistStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.twist.linear.x = v * math.cos(heading_rad)
            t.twist.linear.y = v * math.sin(heading_rad)
            self.vel_pub.publish(t)


def _autodetect_gps_mega() -> str:
    """Find the GPS USB serial port.

    The cart's GPS is a USB receiver wired directly to the Thor (no Mega
    forwarding). We prefer the dedicated-GPS-chipset VIDs first (u-blox,
    Garmin, SiRF, MediaTek, Globalsat) so a u-blox always wins over a
    pedal Mega even if both are present. If none of those match, fall back
    to Arduino/serial-bridge VIDs and pick the LAST one (Mega-on-pedals
    convention puts GPS on the higher-numbered serial)."""
    try:
        import serial.tools.list_ports
    except ImportError:
        return ""

    DEDICATED_GPS_VIDS = {
        0x1546,  # u-blox AG (NEO-6M USB, NEO-M8U, ZED-F9P, etc.)
        0x067B,  # Prolific (some GPS-USB serial bridges)
        0x091E,  # Garmin
        0x067E,  # MediaTek SiRF / older GPS
        0x1199,  # Sierra Wireless (modems w/ GPS)
        0x1199,
    }
    ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}

    all_ports = [
        p for p in serial.tools.list_ports.comports()
        if any(t in p.device for t in
               ("usbmodem", "ttyACM", "ttyUSB", "usbserial"))
    ]

    # 1) Dedicated GPS chipset wins.
    gps_first = [p for p in all_ports if p.vid in DEDICATED_GPS_VIDS]
    if gps_first:
        gps_first.sort(key=lambda p: p.serial_number or "")
        return gps_first[0].device

    # 2) Otherwise an Arduino-class port; pick the highest serial-number one
    #    in case there are multiple Megas (the pedal Mega is conventionally
    #    plugged in first → lower serial → not us).
    arduino_ports = [p for p in all_ports if p.vid in ARDUINO_VIDS]
    if not arduino_ports:
        return ""
    arduino_ports.sort(key=lambda p: p.serial_number or "")
    return arduino_ports[-1].device


def main():
    rclpy.init()
    rclpy.spin(GpsMegaNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
