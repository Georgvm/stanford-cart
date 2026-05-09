"""
Post-processor for a stanford-cart Modal run.

Reads a rosbag2 directory written by `ros2 bag record -a` during a
dataset_replay launch, extracts trajectories from the four sources we
care about, and writes:

  trajectories.csv  — one row per sample, columns:
      t_s, source, x_m, y_m, z_m, lat, lon
  trajectory_overhead.png — overhead plot, all four sources overlaid
  topic_summary.txt — counts + duration of every topic in the bag

Sources extracted:
  /visual_slam/tracking/odometry    nav_msgs/Odometry         (cuVSLAM)
  /odom/arkit                       nav_msgs/Odometry         (ARKit playback)
  /odometry/filtered/global         nav_msgs/Odometry         (EKF fused)
  /fix                              sensor_msgs/NavSatFix     (GPS playback)

Run inside the Modal container after the bag is recorded:
  python _extract_trajectories.py <bag_dir> <out_dir>
"""

import csv
import math
import pathlib
import sys


TOPICS_OF_INTEREST = {
    "/visual_slam/tracking/odometry": "cuvslam",
    "/odom/arkit":                    "arkit",
    "/odometry/filtered/global":      "ekf",
    "/fix":                           "gps",
}


def _utm_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    """Cheap equirectangular projection — good enough for trajectory overlay
    over a few hundred meters at golf-cart scale. Replace with utm.from_latlon
    if you need real UTM."""
    R = 6_378_137.0
    x = math.radians(lon - lon0) * R * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R
    return x, y


def extract(bag_dir: str, out_dir: str) -> None:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import NavSatFix

    out = pathlib.Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)

    storage = StorageOptions(uri=str(bag_dir), storage_id="mcap")
    converter = ConverterOptions("", "")
    reader = SequentialReader()
    try:
        reader.open(storage, converter)
    except Exception:
        # Fall back to sqlite3 if the bag isn't mcap.
        storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
        reader = SequentialReader()
        reader.open(storage, converter)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    counts: dict[str, int] = {}
    rows: list[dict] = []
    gps_origin: tuple[float, float] | None = None

    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        source = TOPICS_OF_INTEREST.get(topic)
        if source is None:
            continue
        t_s = t_ns * 1e-9
        if type_map[topic].endswith("nav_msgs/msg/Odometry"):
            msg: Odometry = deserialize_message(raw, Odometry)
            rows.append({
                "t_s": t_s,
                "source": source,
                "x_m": msg.pose.pose.position.x,
                "y_m": msg.pose.pose.position.y,
                "z_m": msg.pose.pose.position.z,
                "lat": "",
                "lon": "",
            })
        elif type_map[topic].endswith("sensor_msgs/msg/NavSatFix"):
            msg2: NavSatFix = deserialize_message(raw, NavSatFix)
            if gps_origin is None:
                gps_origin = (msg2.latitude, msg2.longitude)
            x, y = _utm_xy(msg2.latitude, msg2.longitude, *gps_origin)
            rows.append({
                "t_s": t_s,
                "source": source,
                "x_m": x,
                "y_m": y,
                "z_m": 0.0,
                "lat": msg2.latitude,
                "lon": msg2.longitude,
            })

    with open(out / "trajectories.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else
                           ["t_s", "source", "x_m", "y_m", "z_m", "lat", "lon"])
        w.writeheader()
        w.writerows(rows)

    with open(out / "topic_summary.txt", "w") as f:
        for t, c in sorted(counts.items()):
            f.write(f"{c:>8d}  {t}\n")

    _plot(rows, out / "trajectory_overhead.png")
    print(f"[extract] wrote {len(rows)} rows across {len(set(r['source'] for r in rows))} sources")


def _plot(rows: list[dict], out_path: pathlib.Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 10))
    colors = {"cuvslam": "tab:red", "arkit": "tab:blue",
              "ekf": "tab:green", "gps": "tab:orange"}
    for source, color in colors.items():
        xs = [r["x_m"] for r in rows if r["source"] == source]
        ys = [r["y_m"] for r in rows if r["source"] == source]
        if not xs:
            continue
        ax.plot(xs, ys, "-", color=color, alpha=0.7, label=f"{source} (n={len(xs)})")
        ax.plot(xs[0], ys[0], "o", color=color)   # start marker
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_title("Trajectory overlay — cuVSLAM vs ARKit vs EKF vs GPS")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)


if __name__ == "__main__":
    extract(sys.argv[1], sys.argv[2])
