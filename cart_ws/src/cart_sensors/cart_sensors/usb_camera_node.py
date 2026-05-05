"""
usb_camera_node — publish a single USB UVC camera as ROS 2 sensor_msgs/Image.

Spawned 4× by sensors.launch.py with different namespaces and /dev/video
indices. This is intentionally one-camera-per-process so a wedged camera
(USB bandwidth contention is a known issue on the cart — see
docs/cameras.md from the cart repo) can't take down the others.

Camera mapping observed on the cart's Jetson:
  /dev/video0   front_narrow  (varifocal, 2.8-12mm, used for downstream
                                inference; was the 'front_narrow' slot in
                                record_cameras.py)
  /dev/video6   front_wide    (~170° fisheye; mounted UPSIDE DOWN — flip
                                across x-axis at capture time)
  /dev/video2   left          (~170° fisheye)
  /dev/video10  right         (~170° fisheye)

(The autoware_infer.py SLUGS order disagrees on the meaning of indices 0/2;
verify with `v4l2-ctl --list-devices` or the camera_view.py overlay before
trusting the mapping.)

Format: MJPG @ 640x480 by default. Four 1080p MJPG streams will saturate a
single USB 2.0 controller — keep the resolution low unless cameras are on
separate USB controllers.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


# cv2.flip codes:  0 = vertical flip, 1 = horizontal flip, -1 = 180° rotation
_FLIP_CODES = {
    "none": None,
    "vertical": 0,
    "horizontal": 1,
    "rotate_180": -1,
}


class UsbCameraNode(Node):
    def __init__(self):
        super().__init__("usb_camera")

        self.declare_parameter("device_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("fourcc", "MJPG")
        self.declare_parameter("frame_id", "camera_optical")
        self.declare_parameter("flip", "none")        # one of _FLIP_CODES keys
        self.declare_parameter("publish_hz", 20.0)    # publish rate cap

        idx = int(self.get_parameter("device_index").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        fourcc = str(self.get_parameter("fourcc").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        flip_key = str(self.get_parameter("flip").value)
        self.flip_code = _FLIP_CODES.get(flip_key)
        if flip_key not in _FLIP_CODES:
            self.get_logger().warn(f"unknown flip='{flip_key}' — using none")

        self.cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"failed to open /dev/video{idx}")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Background grabber → always-latest single-slot frame.
        self.lock = threading.Lock()
        self.frame: np.ndarray | None = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._grab_loop, daemon=True)
        self.thread.start()

        self.image_pub = self.create_publisher(
            Image, "image_raw", qos_profile_sensor_data,
        )
        self.cam_info_pub = self.create_publisher(
            CameraInfo, "camera_info", qos_profile_sensor_data,
        )

        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.create_timer(period, self._publish)
        self.get_logger().info(
            f"usb_camera /dev/video{idx} {self.w}x{self.h}@{self.fps} "
            f"({fourcc}, flip={flip_key}, frame_id={self.frame_id})"
        )

    def _grab_loop(self):
        while not self.stop_event.is_set():
            ok, f = self.cap.read()
            if not ok or f is None:
                time.sleep(0.01)
                continue
            if self.flip_code is not None:
                f = cv2.flip(f, self.flip_code)
            with self.lock:
                self.frame = f

    def _publish(self):
        with self.lock:
            f = None if self.frame is None else self.frame
        if f is None:
            return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = f.shape[:2]
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = f.tobytes()
        self.image_pub.publish(msg)

        # Minimal CameraInfo — calibrate later, replace with the YAML loader.
        ci = CameraInfo()
        ci.header = msg.header
        ci.height, ci.width = msg.height, msg.width
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0] * 5
        ci.k = [
            float(msg.width), 0.0, msg.width / 2.0,
            0.0, float(msg.height), msg.height / 2.0,
            0.0, 0.0, 1.0,
        ]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [
            float(msg.width), 0.0, msg.width / 2.0, 0.0,
            0.0, float(msg.height), msg.height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        self.cam_info_pub.publish(ci)

    def destroy_node(self):
        self.stop_event.set()
        try:
            self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
