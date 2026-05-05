"""
Modal H200 offline pipeline.

Inputs (one of):
  A) ROS 2 bag containing /fix and /front_*/image_raw streams
  B) NMEA log file + Caddy-Training-Data folder (4× MP4) — what your existing
     scripts/record_cameras.py already produces

Outputs (uploaded to /data/<route_name>/ on the Modal volume):
  waypoints.yaml          Nav2 follow_gps_waypoints input
  route_preview.html      folium polyline for sanity check
  poses.json              (optional, if VGGT/COLMAP pass enabled) per-keyframe poses

Run:
  modal run modal_pipeline/process_route.py::main \
    --nmea-log ~/recordings/route_oval/gps.log \
    --route-name route_oval
"""

import modal

app = modal.App("stanford-cart-route")

image = (
    modal.Image.debian_slim(python_version="3.11")
    .apt_install("libgl1", "libglib2.0-0", "ffmpeg", "git")
    .pip_install(
        "numpy", "scipy", "pyyaml", "folium", "utm",
        "opencv-python-headless", "pynmea2", "pyproj", "boto3",
    )
)

volume = modal.Volume.from_name("stanford-cart-data", create_if_missing=True)


@app.function(image=image, volumes={"/data": volume}, timeout=60 * 60)
def process_nmea_log(nmea_text: str, route_name: str = "route_oval"):
    """Read an NMEA log (one sentence per line, optionally with leading
    timestamps from sensor_test.py-style logs), smooth, and emit waypoints."""
    from pathlib import Path
    import yaml
    out_dir = Path("/data") / route_name
    out_dir.mkdir(parents=True, exist_ok=True)

    fixes = _parse_nmea(nmea_text)
    print(f"[process] {len(fixes)} GPS fixes parsed")
    if len(fixes) < 10:
        raise RuntimeError("not enough GPS fixes — check NMEA input")

    waypoints = _smooth_and_downsample(fixes, spacing_m=3.0)
    print(f"[process] {len(waypoints)} waypoints after smoothing")

    payload = _to_nav2_yaml(waypoints, route_name)
    (out_dir / "waypoints.yaml").write_text(yaml.safe_dump(payload))
    _write_preview(waypoints, out_dir / "route_preview.html")
    (out_dir / "raw_nmea.log").write_text(nmea_text)

    volume.commit()
    print(f"[process] wrote /data/{route_name}/")
    return str(out_dir)


def _parse_nmea(text: str):
    import pynmea2
    fixes = []
    for line in text.splitlines():
        line = line.strip()
        # tolerate leading timestamps like "[GPS/GGA]" from sensor_test.py
        idx = line.find("$")
        if idx < 0:
            continue
        sentence = line[idx:]
        try:
            msg = pynmea2.parse(sentence)
        except Exception:
            continue
        if isinstance(msg, pynmea2.GGA):
            try:
                lat = float(msg.latitude); lon = float(msg.longitude)
                qual = int(msg.gps_qual or 0)
            except (TypeError, ValueError):
                continue
            if qual == 0:
                continue
            fixes.append((lat, lon))
    return fixes


def _smooth_and_downsample(fixes, spacing_m: float = 3.0):
    import numpy as np
    import utm
    from scipy.signal import savgol_filter
    lats = np.array([f[0] for f in fixes])
    lons = np.array([f[1] for f in fixes])
    e0, n0, zn, zl = utm.from_latlon(lats[0], lons[0])
    es, ns = [], []
    for lat, lon in zip(lats, lons):
        e, n, _, _ = utm.from_latlon(lat, lon, force_zone_number=zn)
        es.append(e); ns.append(n)
    es = np.array(es); ns = np.array(ns)
    win = min(31, max(5, len(es) // 2 * 2 + 1))
    if win >= 5 and len(es) > win:
        es = savgol_filter(es, win, 3)
        ns = savgol_filter(ns, win, 3)
    out_e = [es[0]]; out_n = [ns[0]]
    for e, n in zip(es[1:], ns[1:]):
        de = e - out_e[-1]; dn = n - out_n[-1]
        if (de * de + dn * dn) ** 0.5 >= spacing_m:
            out_e.append(e); out_n.append(n)
    waypoints = []
    for e, n in zip(out_e, out_n):
        lat, lon = utm.to_latlon(e, n, zn, zl)
        waypoints.append({"lat": float(lat), "lon": float(lon)})
    return waypoints


def _to_nav2_yaml(waypoints, route_name: str):
    return {
        "route_name": route_name,
        "frame_id": "map",
        "waypoints": [
            {"latitude": w["lat"], "longitude": w["lon"], "yaw": 0.0}
            for w in waypoints
        ],
    }


def _write_preview(waypoints, out_path):
    import folium
    if not waypoints:
        return
    m = folium.Map(
        location=[waypoints[0]["lat"], waypoints[0]["lon"]],
        zoom_start=18, tiles="OpenStreetMap",
    )
    pts = [(w["lat"], w["lon"]) for w in waypoints]
    folium.PolyLine(pts, color="red", weight=4).add_to(m)
    for i, (lat, lon) in enumerate(pts):
        folium.CircleMarker((lat, lon), radius=3, popup=f"wp {i}").add_to(m)
    m.save(str(out_path))


@app.local_entrypoint()
def main(nmea_log: str, route_name: str = "route_oval"):
    from pathlib import Path
    text = Path(nmea_log).expanduser().read_text()
    out = process_nmea_log.remote(nmea_text=text, route_name=route_name)
    print(f"Done. {out}")
    print(f"Pull: modal volume get stanford-cart-data {route_name} ./out/")
