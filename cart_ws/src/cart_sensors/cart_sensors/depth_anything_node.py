"""
depth_anything_node — monocular depth on /front_wide/image_raw.

Publishes:
  /front_wide/depth   sensor_msgs/Image  (32FC1, meters)
  /front_wide/depth/camera_info   sensor_msgs/CameraInfo  (mirrors source)

This is what nvblox subscribes to for the 3D occupancy grid. The model
(Depth Anything V2 Small via HuggingFace transformers) outputs a relative
inverse-depth map, which we convert to metric meters via a static
calibration:

    z_m = max(MIN_M, min(MAX_M, A / (disparity + EPS) + B))

A and B should be calibrated on day-of with two known distances (a person
standing at 2 m and 10 m gives a clean 2-point fit). Defaults are
reasonable for a fisheye front_wide at golf-cart scale but WILL be wrong
in absolute meters until calibrated.

Performance: Depth-Anything V2 Small ~25 ms/frame on a Jetson Thor with
TensorRT. Without TRT it's 80-150 ms; cap publish_hz to 5 Hz in that case
to leave headroom.
"""

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo


class DepthAnythingNode(Node):
    def __init__(self):
        super().__init__("depth_anything")

        self.declare_parameter("model", "depth-anything/Depth-Anything-V2-Small-hf")
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("a_param", 60.0)        # disparity → meters scale
        self.declare_parameter("b_param", 0.0)         # offset
        self.declare_parameter("min_m", 0.5)
        self.declare_parameter("max_m", 30.0)
        self.declare_parameter("input_topic", "/front_wide/image_raw")
        self.declare_parameter("input_camera_info", "/front_wide/camera_info")
        self.declare_parameter("output_topic", "/front_wide/depth")
        self.declare_parameter("output_camera_info", "/front_wide/depth/camera_info")

        self.A = float(self.get_parameter("a_param").value)
        self.B = float(self.get_parameter("b_param").value)
        self.min_m = float(self.get_parameter("min_m").value)
        self.max_m = float(self.get_parameter("max_m").value)

        self._init_model()
        self.lock = threading.Lock()
        self.latest_img: Image | None = None
        self.latest_ci: CameraInfo | None = None

        self.create_subscription(
            Image, self.get_parameter("input_topic").value,
            self._on_img, qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo, self.get_parameter("input_camera_info").value,
            self._on_ci, qos_profile_sensor_data,
        )
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter("output_topic").value,
            qos_profile_sensor_data,
        )
        self.ci_pub = self.create_publisher(
            CameraInfo, self.get_parameter("output_camera_info").value,
            qos_profile_sensor_data,
        )
        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.create_timer(period, self._infer)
        self.get_logger().info(
            f"depth_anything up. model={self.get_parameter('model').value} "
            f"@ {1.0/period:.1f} Hz. metric A={self.A} B={self.B}"
        )

    def _init_model(self):
        from transformers import pipeline
        import torch
        device = 0 if torch.cuda.is_available() else -1
        self.pipe = pipeline(
            task="depth-estimation",
            model=self.get_parameter("model").value,
            device=device,
        )

    def _on_img(self, msg: Image):
        with self.lock:
            self.latest_img = msg

    def _on_ci(self, msg: CameraInfo):
        with self.lock:
            self.latest_ci = msg

    def _infer(self):
        with self.lock:
            img = self.latest_img
            ci = self.latest_ci
        if img is None:
            return

        try:
            arr = np.frombuffer(img.data, dtype=np.uint8).reshape(
                img.height, img.width, -1,
            )
            if img.encoding == "bgr8":
                arr = arr[:, :, ::-1].copy()    # → RGB for the HF pipeline
        except Exception as e:
            self.get_logger().warn(f"image decode: {e!r}")
            return

        from PIL import Image as PILImage
        out = self.pipe(PILImage.fromarray(arr))
        disp = np.array(out["depth"], dtype=np.float32)
        # Normalize disparity to roughly [0, 1] so A/B mean the same thing
        # across runs (the HF pipeline output range varies by image).
        d_min, d_max = float(np.min(disp)), float(np.max(disp))
        if d_max - d_min < 1e-6:
            return
        d_norm = (disp - d_min) / (d_max - d_min)
        # Meters via inverse model. Foreground (high disparity) → small z.
        z = self.A / (d_norm + 1e-3) + self.B
        z = np.clip(z, self.min_m, self.max_m).astype(np.float32)

        depth_msg = Image()
        depth_msg.header = img.header
        depth_msg.height = z.shape[0]
        depth_msg.width = z.shape[1]
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = 0
        depth_msg.step = z.shape[1] * 4
        depth_msg.data = z.tobytes()
        self.depth_pub.publish(depth_msg)

        if ci is not None:
            ci_out = CameraInfo()
            ci_out.header = depth_msg.header
            ci_out.height, ci_out.width = depth_msg.height, depth_msg.width
            ci_out.distortion_model = ci.distortion_model
            ci_out.d = list(ci.d)
            ci_out.k = list(ci.k)
            ci_out.r = list(ci.r)
            ci_out.p = list(ci.p)
            self.ci_pub.publish(ci_out)


def main():
    rclpy.init()
    rclpy.spin(DepthAnythingNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
