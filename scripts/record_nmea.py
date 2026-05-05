#!/usr/bin/env python3
"""
record_nmea — open the GPS Mega serial port and tee NMEA to a file.

No ROS dependency. Designed to run on the Thor in parallel with teleop:

    ./scripts/record_nmea.py --route route_oval

Output goes to ~/recordings/<route>/<timestamp>/gps.log. That file feeds
straight into the Modal pipeline:

    ./scripts/upload_to_modal.sh ~/recordings/route_oval/<ts>/gps.log route_oval

Autodetects the GPS Mega the same way the ROS gps_mega_node does (last
Arduino-class USB serial port). Override with --port if multiple Megas
confuse the picker.
"""

import argparse
import datetime
import os
import pathlib
import sys
import time


DEDICATED_GPS_VIDS = {
    0x1546,  # u-blox AG (NEO-6M USB, ZED-F9P)
    0x067B,  # Prolific (some GPS-USB bridges)
    0x091E,  # Garmin
    0x067E,  # MediaTek SiRF
}
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x0403, 0x10C4}


def autodetect_gps_port() -> str:
    """Same logic as cart_sensors.gps_mega_node._autodetect_gps_mega."""
    import serial.tools.list_ports
    all_ports = [
        p for p in serial.tools.list_ports.comports()
        if any(t in p.device for t in
               ("usbmodem", "ttyACM", "ttyUSB", "usbserial"))
    ]
    gps_first = [p for p in all_ports if p.vid in DEDICATED_GPS_VIDS]
    if gps_first:
        gps_first.sort(key=lambda p: p.serial_number or "")
        return gps_first[0].device
    arduino = [p for p in all_ports if p.vid in ARDUINO_VIDS]
    if not arduino:
        return ""
    arduino.sort(key=lambda p: p.serial_number or "")
    return arduino[-1].device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--route", default="route_default",
                    help="route name; output goes to ~/recordings/<route>/<ts>/")
    ap.add_argument("--port", default="",
                    help="explicit serial port (skip autodetect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out-root", default=os.path.expanduser("~/recordings"))
    ap.add_argument("--max-duration-s", type=float, default=0.0,
                    help="auto-exit after this many seconds (0 = until ctrl-c)")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    port = args.port or autodetect_gps_port()
    if not port:
        print("no GPS Mega detected. Pass --port /dev/ttyACM1 (or similar).",
              file=sys.stderr)
        sys.exit(1)

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = pathlib.Path(args.out_root) / args.route / ts
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "gps.log"

    print(f"recording NMEA from {port} @ {args.baud} → {out_path}", file=sys.stderr)
    print("press ctrl-c to stop. resulting file is ready for Modal upload.",
          file=sys.stderr)

    ser = serial.Serial(port, args.baud, timeout=0.5)
    time.sleep(2.0)  # bootloader settle

    n_lines = 0
    start = time.time()
    with open(out_path, "wb") as f:
        try:
            while True:
                if args.max_duration_s and (time.time() - start) > args.max_duration_s:
                    break
                buf = ser.readline()
                if not buf:
                    continue
                if not buf.startswith(b"$"):
                    continue
                f.write(buf)
                f.flush()
                n_lines += 1
                if n_lines % 100 == 0:
                    print(f"  ... {n_lines} NMEA lines, "
                          f"{time.time()-start:.0f}s", file=sys.stderr)
        except KeyboardInterrupt:
            pass

    print(f"\n{n_lines} NMEA lines saved to {out_path}", file=sys.stderr)
    print(f"\nnext:", file=sys.stderr)
    print(f"  scp {out_path} <mac>:/tmp/", file=sys.stderr)
    print(f"  # then on the mac:", file=sys.stderr)
    print(f"  ./scripts/upload_to_modal.sh /tmp/{out_path.name} {args.route}",
          file=sys.stderr)


if __name__ == "__main__":
    main()
