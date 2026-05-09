"""
dataset_playback_node — replay a recorded driving session as ROS topics.

Lets us exercise the cuVSLAM / nvblox / EKF / Nav2 stack without driving
the cart (no hardware, no sun, no battery). Drop-in replacement for
sensors.launch.py + the live ARKit feed.

Publishes:
  /front_narrow/image_raw    sensor_msgs/Image
  /front_narrow/camera_info  sensor_msgs/CameraInfo
  /front_wide/image_raw      sensor_msgs/Image
  /front_wide/camera_info    sensor_msgs/CameraInfo
  /left/image_raw            sensor_msgs/Image
  /left/camera_info          sensor_msgs/CameraInfo
  /right/image_raw           sensor_msgs/Image
  /right/camera_info         sensor_msgs/CameraInfo
  /fix                       sensor_msgs/NavSatFix          (from gps.json)
  /odom/arkit                nav_msgs/Odometry              (from ego.jsonl, alpamayo block — REP-103 frame)
  /imu/arkit                 sensor_msgs/Imu                (derived from ARKit pose; OPTIONAL — see imu_publish param)

Coordinate frame: uses the dataset's `alpamayo` block which is already in
PhysicalAI-AV frame (x-fwd, y-left, z-up — same as ROS REP-103). No axis
remapping needed.

Time base: dataset rel_t (seconds since recording start) is mapped to
ROS clock starting at the node's startup time. Wall_t is ignored — we want
deterministic replay, not absolute timing.

Loop / rate / start-offset are parameters; defaults play once at 1×.

Camera intrinsics: STUB (same approximation as usb_camera_node). cuVSLAM
will track but absolute pose precision will be off until intrinsics are
properly calibrated. Override per-camera via the `intrinsics_<slug>` YAML
parameters when you have them.
"""

import json
import math
import pathlib
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


CAM_FILES = {
    "front_narrow": "front-narrow.mp4",
    "front_wide":   "front-wide.mp4",
    "left":         "cross-left.mp4",
    "right":        "cross-right.mp4",
}
CAM_FRAMES = {
    "front_narrow": "front_narrow_optical",
    "front_wide":   "front_wide_optical",
    "left":         "left_optical",
    "right":        "right_optical",
}


def yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


@dataclass
class CamState:
    cap: cv2.VideoCapture
    pub_img: object
    pub_ci: object
    frame_id: str
    fps: float
    next_pub_t: float


class DatasetPlaybackNode(Node):
    def __init__(self):
        super().__init__("dataset_playback")

        self.declare_parameter("dataset_dir", "")
        self.declare_parameter("speed", 1.0)            # playback rate multiplier
        self.declare_parameter("loop", False)
        self.declare_parameter("start_rel_t", 0.0)      # skip ahead N seconds
        self.declare_parameter("imu_publish", False)    # synthesize IMU from ARKit pose
        self.declare_parameter("odom_frame", "arkit_odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gps_frame", "gps")

        ds = pathlib.Path(self.get_parameter("dataset_dir").value)
        if not ds.is_dir():
            raise RuntimeError(
                f"dataset_dir not found: {ds!s}. Set the dataset_dir parameter."
            )
        self.ds = ds
        self.speed = float(self.get_parameter("speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.start_rel_t = float(self.get_parameter("start_rel_t").value)
        self.publish_imu = bool(self.get_parameter("imu_publish").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.gps_frame = str(self.get_parameter("gps_frame").value)

        self._init_cameras()
        self._init_ego()
        self._init_gps()
        self._fix_pub = self.create_publisher(NavSatFix, "/fix", qos_profile_sensor_data)
        self._odom_pub = self.create_publisher(Odometry, "/odom/arkit", 50)
        self._imu_pub = self.create_publisher(Imu, "/imu/arkit", 100) if self.publish_imu else None

        # Single playback thread — keeps frame/ego/gps in lockstep against rel_t.
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self.get_logger().info(
            f"dataset_playback up. dir={ds!s} speed={self.speed}× "
            f"start={self.start_rel_t}s loop={self.loop} imu={self.publish_imu}"
        )

    def _init_cameras(self):
        self.cams: dict[str, CamState] = {}
        for slug, fname in CAM_FILES.items():
            path = self.ds / fname
            if not path.exists():
                self.get_logger().warn(f"missing {fname}, skipping {slug}")
                continue
            cap = cv2.VideoCapture(str(path))
            fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            ns = "/" + slug
            self.cams[slug] = CamState(
                cap=cap,
                pub_img=self.create_publisher(Image, f"{ns}/image_raw", qos_profile_sensor_data),
                pub_ci=self.create_publisher(CameraInfo, f"{ns}/camera_info", qos_profile_sensor_data),
                frame_id=CAM_FRAMES[slug],
                fps=fps,
                next_pub_t=0.0,
            )
        if not self.cams:
            raise RuntimeError("no camera videos found in dataset_dir")

    def _init_ego(self):
        self.ego = []
        with open(self.ds / "ego.jsonl") as f:
            for line in f:
                if not line.strip():
                    continue
                obj = json.loads(line)
                if "rel_t" not in obj or "alpamayo" not in obj:
                    continue
                self.ego.append(obj)
        if not self.ego:
            raise RuntimeError("ego.jsonl has no usable records")
        # Numerical-derivative buffers for IMU synthesis.
        self._ego_idx = 0
        self._last_speed = self.ego[0]["speed_mps"]
        self._last_t = self.ego[0]["rel_t"]

    def _init_gps(self):
        self.gps_samples = []
        try:
            data = json.load(open(self.ds / "gps.json"))
        except FileNotFoundError:
            self.get_logger().warn("gps.json not found — skipping /fix publication")
            return
        for s in data.get("samples", []):
            if all(k in s for k in ("rel_t", "lat", "lon")):
                self.gps_samples.append(s)
        self._gps_idx = 0

    def _stub_camera_info(self, w: int, h: int, frame_id: str, stamp) -> CameraInfo:
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = frame_id
        ci.height, ci.width = h, w
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [
            float(w), 0.0, w / 2.0,
            0.0, float(h), h / 2.0,
            0.0, 0.0, 1.0,
        ]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [
            float(w), 0.0, w / 2.0, 0.0,
            0.0, float(h), h / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return ci

    # -- main playback loop ----------------------------------------------------

    def _run(self):
        wall_start = time.monotonic()
        ros_start = self.get_clock().now()

        # Skip ahead in cameras if start_rel_t > 0.
        if self.start_rel_t > 0:
            for slug, c in self.cams.items():
                c.cap.set(cv2.CAP_PROP_POS_MSEC, self.start_rel_t * 1000.0)
            while self._ego_idx < len(self.ego) and \
                  self.ego[self._ego_idx]["rel_t"] < self.start_rel_t:
                self._ego_idx += 1
            if self.gps_samples:
                while self._gps_idx < len(self.gps_samples) and \
                      self.gps_samples[self._gps_idx]["rel_t"] < self.start_rel_t:
                    self._gps_idx += 1

        while not self._stop.is_set() and rclpy.ok():
            # Where in the dataset do we want to be right now?
            elapsed = (time.monotonic() - wall_start) * self.speed
            target_rel_t = self.start_rel_t + elapsed

            # ROS time corresponding to that frame (monotonic from start).
            now = self.get_clock().now()

            self._tick_cameras(target_rel_t, now)
            self._tick_ego(target_rel_t, now)
            self._tick_gps(target_rel_t, now)

            # End-of-dataset?
            cam_done = all(c.cap.get(cv2.CAP_PROP_POS_FRAMES) >=
                           c.cap.get(cv2.CAP_PROP_FRAME_COUNT) - 1
                           for c in self.cams.values())
            ego_done = self._ego_idx >= len(self.ego)
            if cam_done and ego_done:
                if self.loop:
                    self.get_logger().info("dataset end — looping")
                    self._reset()
                    wall_start = time.monotonic()
                else:
                    self.get_logger().info("dataset end — stopping playback")
                    return

            time.sleep(0.005)  # 200 Hz tick

    def _reset(self):
        for c in self.cams.values():
            c.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        self._ego_idx = 0
        self._gps_idx = 0
        self._last_speed = self.ego[0]["speed_mps"]
        self._last_t = self.ego[0]["rel_t"]

    def _tick_cameras(self, target_rel_t: float, now):
        for slug, c in self.cams.items():
            cur_t = c.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            if cur_t > target_rel_t:
                continue
            ok, frame = c.cap.read()
            if not ok:
                continue
            stamp = now.to_msg()
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = c.frame_id
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = frame.tobytes()
            c.pub_img.publish(msg)
            c.pub_ci.publish(self._stub_camera_info(
                msg.width, msg.height, c.frame_id, stamp
            ))

    def _tick_ego(self, target_rel_t: float, now):
        while self._ego_idx < len(self.ego) and \
              self.ego[self._ego_idx]["rel_t"] <= target_rel_t:
            r = self.ego[self._ego_idx]
            self._publish_odom(r, now)
            if self.publish_imu:
                self._publish_imu(r, now)
            self._ego_idx += 1

    def _tick_gps(self, target_rel_t: float, now):
        if not self.gps_samples:
            return
        while self._gps_idx < len(self.gps_samples) and \
              self.gps_samples[self._gps_idx]["rel_t"] <= target_rel_t:
            s = self.gps_samples[self._gps_idx]
            f = NavSatFix()
            f.header.stamp = now.to_msg()
            f.header.frame_id = self.gps_frame
            f.latitude = float(s["lat"])
            f.longitude = float(s["lon"])
            f.altitude = float(s.get("alt", 0.0))
            f.status.status = NavSatStatus.STATUS_FIX
            f.status.service = NavSatStatus.SERVICE_GPS
            sigma = 1.0   # ground-truth annotated; tight covariance
            f.position_covariance = [
                sigma * sigma, 0.0, 0.0,
                0.0, sigma * sigma, 0.0,
                0.0, 0.0, (sigma * 2) ** 2,
            ]
            f.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self._fix_pub.publish(f)
            self._gps_idx += 1

    def _publish_odom(self, r: dict, now):
        # `alpamayo` block is already in REP-103-style frame (x-fwd, y-left, z-up).
        a = r["alpamayo"]
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = float(a["xyz_m"][0])
        msg.pose.pose.position.y = float(a["xyz_m"][1])
        msg.pose.pose.position.z = float(a["xyz_m"][2])
        msg.pose.pose.orientation = yaw_to_quat(float(a["yaw_rad"]))
        # Pose covariance — ARKit is good but not perfect; ~10cm position, ~1° yaw.
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.01    # xx
        pose_cov[7]  = 0.01    # yy
        pose_cov[14] = 0.05    # zz
        pose_cov[21] = 0.01    # roll
        pose_cov[28] = 0.01    # pitch
        pose_cov[35] = (math.radians(1.0)) ** 2  # yaw
        msg.pose.covariance = pose_cov
        # Twist — body-frame linear x = forward speed, angular z = yaw_rate.
        msg.twist.twist.linear.x = float(a["speed_mps"])
        msg.twist.twist.angular.z = float(a["yaw_rate_rad_s"])
        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.05     # vx
        twist_cov[7]  = 0.10     # vy (uncertain — we only know forward speed)
        twist_cov[35] = (math.radians(2.0)) ** 2
        msg.twist.covariance = twist_cov
        self._odom_pub.publish(msg)

    def _publish_imu(self, r: dict, now):
        # SYNTHESIZED IMU — derive accel + gyro from ARKit pose stream.
        # 10Hz is too slow for cuVSLAM's IMU fusion to give great results;
        # this is here for plumbing-correctness testing more than performance.
        a = r["alpamayo"]
        rt = float(r["rel_t"])
        sp = float(a["speed_mps"])
        yr = float(a["yaw_rate_rad_s"])
        dt = max(1e-3, rt - self._last_t)
        ax = (sp - self._last_speed) / dt          # linear accel forward (m/s²)
        ay = sp * yr                               # centripetal (lateral)
        self._last_speed, self._last_t = sp, rt

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.base_frame
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = -9.81          # gravity (assumes flat)
        msg.angular_velocity.z = float(yr)
        # Orientation as identity here — REP-145 IMU msg orientation is
        # optional; cuVSLAM derives it from integrating gyro.
        msg.orientation.w = 1.0
        msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # unknown
        msg.angular_velocity_covariance = [
            (math.radians(5.0))**2, 0, 0,
            0, (math.radians(5.0))**2, 0,
            0, 0, (math.radians(2.0))**2,
        ]
        msg.linear_acceleration_covariance = [
            0.5, 0, 0,
            0, 0.5, 0,
            0, 0, 0.5,
        ]
        self._imu_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        for c in self.cams.values():
            try:
                c.cap.release()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = DatasetPlaybackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
