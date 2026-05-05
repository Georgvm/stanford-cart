"""
perception_estop — pretrained YOLOv8 + Depth Anything V2 fallback obstacle gate.

Subscribes:
  /front_wide/image_raw     sensor_msgs/Image     (BGR, 640x480 ish)

Publishes:
  /perception/nearest_obstacle_m   std_msgs/Float32   (inf if no hazard)

This is the BACKUP safety layer. Primary obstacle perception is nvblox
(camera depth → 3D occupancy → Nav2 costmap). This node exists in case
nvblox isn't running yet, or as a redundant emergency-stop gate that
doesn't depend on the costmap pipeline being healthy.

Calibration: _disparity_to_meters() is a heuristic. For real numbers,
place a person at known distances (2 m, 5 m, 10 m) on day-of and fit
a line through the median ROI disparity values.
"""

import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


HAZARD_CLASSES = {0: "person", 1: "bicycle", 2: "car", 3: "motorcycle",
                  5: "bus", 7: "truck"}


class PerceptionEstopNode(Node):
    def __init__(self):
        super().__init__("perception_estop")

        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("depth_model",
                               "depth-anything/Depth-Anything-V2-Small-hf")
        self.declare_parameter("confidence", 0.4)
        self.declare_parameter("publish_hz", 10.0)

        self._init_models()
        self.lock = threading.Lock()
        self.latest: np.ndarray | None = None

        self.create_subscription(Image, "/front_wide/image_raw", self._on_img, 1)
        self.pub = self.create_publisher(
            Float32, "/perception/nearest_obstacle_m", 10,
        )
        period = 1.0 / float(self.get_parameter("publish_hz").value)
        self.create_timer(period, self._infer)
        self.get_logger().info("perception_estop up.")

    def _init_models(self):
        from ultralytics import YOLO
        from transformers import pipeline
        import torch
        self.yolo = YOLO(self.get_parameter("yolo_model").value)
        device = 0 if torch.cuda.is_available() else -1
        self.depth = pipeline(
            task="depth-estimation",
            model=self.get_parameter("depth_model").value,
            device=device,
        )
        self.conf = float(self.get_parameter("confidence").value)

    def _on_img(self, msg: Image):
        try:
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1,
            )
            if msg.encoding == "rgb8":
                arr = arr[:, :, ::-1].copy()
        except Exception as e:
            self.get_logger().warn(f"image decode: {e!r}")
            return
        with self.lock:
            self.latest = arr

    def _infer(self):
        with self.lock:
            img = None if self.latest is None else self.latest.copy()
        if img is None:
            return

        results = self.yolo.predict(img, verbose=False, conf=self.conf)[0]
        hazards = []
        if results.boxes is not None:
            for b in results.boxes:
                if int(b.cls.item()) in HAZARD_CLASSES:
                    hazards.append(b)

        if not hazards:
            self._publish(float("inf"))
            return

        from PIL import Image as PILImage
        depth_out = self.depth(PILImage.fromarray(img))
        depth = np.array(depth_out["depth"])  # disparity-like; larger = closer

        nearest = float("inf")
        for box in hazards:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            roi = depth[y1:y2, x1:x2]
            if roi.size == 0:
                continue
            disp = float(np.median(roi))
            nearest = min(nearest, self._disparity_to_meters(disp))
        self._publish(nearest)

    def _disparity_to_meters(self, disp: float) -> float:
        # Heuristic — RECALIBRATE on day-of with known distances.
        if disp <= 1e-3:
            return 50.0
        return float(np.clip(60.0 / disp, 0.5, 50.0))

    def _publish(self, m: float):
        msg = Float32(); msg.data = float(m)
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(PerceptionEstopNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
