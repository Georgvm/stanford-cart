"""
Generate a synthetic NMEA log: a 200m loop around Stanford's Oval, ~1 fix/s.
Use to smoke-test the Modal pipeline before real route data exists.

Usage:
  python modal_pipeline/_make_fake_nmea.py > /tmp/fake_oval.nmea
  modal run modal_pipeline/process_route.py::main \
    --nmea-log /tmp/fake_oval.nmea --route-name fake_oval
"""

import math
from datetime import datetime, timedelta


# Approximate Stanford Oval centroid
CENTER_LAT = 37.4317
CENTER_LON = -122.1693
RADIUS_M = 100.0
N_POINTS = 240   # 4 minutes at 1 Hz


def _nmea_checksum(payload: str) -> str:
    cs = 0
    for c in payload:
        cs ^= ord(c)
    return f"{cs:02X}"


def _gga(t: datetime, lat: float, lon: float) -> str:
    lat_d = int(abs(lat))
    lat_m = (abs(lat) - lat_d) * 60.0
    lon_d = int(abs(lon))
    lon_m = (abs(lon) - lon_d) * 60.0
    lat_str = f"{lat_d:02d}{lat_m:07.4f}"
    lon_str = f"{lon_d:03d}{lon_m:07.4f}"
    ns = "N" if lat >= 0 else "S"
    ew = "E" if lon >= 0 else "W"
    hms = t.strftime("%H%M%S.00")
    payload = (
        f"GPGGA,{hms},{lat_str},{ns},{lon_str},{ew},"
        f"1,08,0.9,30.5,M,46.9,M,,"
    )
    return f"${payload}*{_nmea_checksum(payload)}"


def main():
    # Loop around the oval; convert metric offsets to lat/lon (small-angle ok).
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = m_per_deg_lat * math.cos(math.radians(CENTER_LAT))
    t0 = datetime(2026, 5, 4, 14, 0, 0)
    for i in range(N_POINTS):
        theta = 2 * math.pi * i / N_POINTS
        dx = RADIUS_M * math.cos(theta)
        dy = RADIUS_M * math.sin(theta)
        lat = CENTER_LAT + dy / m_per_deg_lat
        lon = CENTER_LON + dx / m_per_deg_lon
        t = t0 + timedelta(seconds=i)
        print(_gga(t, lat, lon))


if __name__ == "__main__":
    main()
