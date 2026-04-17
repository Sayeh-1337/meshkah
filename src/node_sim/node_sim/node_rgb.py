import math
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .sensor_model import SensorModel, enu_to_az_el, latlon_to_enu

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional dependency at runtime
    YOLO = None


def _iou(box_a: Tuple[int, int, int, int], box_b: Tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    if inter_x2 <= inter_x1 or inter_y2 <= inter_y1:
        return 0.0
    inter = (inter_x2 - inter_x1) * (inter_y2 - inter_y1)
    area_a = max((ax2 - ax1), 0) * max((ay2 - ay1), 0)
    area_b = max((bx2 - bx1), 0) * max((by2 - by1), 0)
    union = area_a + area_b - inter
    return float(inter / union) if union > 0 else 0.0


class RGBNode(Node):
    """Hybrid RGB node - virtual phase 1 and real phase 3."""

    def __init__(self) -> None:
        super().__init__("node_rgb")
        self.declare_parameter("node_id", "rgb_1")
        self.declare_parameter("gps_lat", 30.0)
        self.declare_parameter("gps_lon", 31.0)
        self.declare_parameter("gps_alt", 0.0)
        self.declare_parameter("ref_lat", 30.0)
        self.declare_parameter("ref_lon", 31.0)
        self.declare_parameter("ref_alt", 0.0)
        self.declare_parameter("heading_deg", 0.0)
        self.declare_parameter("use_virtual", True)
        self.declare_parameter("video_source", "")
        self.declare_parameter("camera_fx", 900.0)
        self.declare_parameter("camera_fy", 900.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)
        self.declare_parameter("camera_heading_deg", 0.0)
        self.declare_parameter("camera_tilt_deg", 0.0)
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("target_class_id", 0)

        self.node_id = str(self.get_parameter("node_id").value)
        self.gps_lat = float(self.get_parameter("gps_lat").value)
        self.gps_lon = float(self.get_parameter("gps_lon").value)
        self.gps_alt = float(self.get_parameter("gps_alt").value)
        self.ref_lat = float(self.get_parameter("ref_lat").value)
        self.ref_lon = float(self.get_parameter("ref_lon").value)
        self.ref_alt = float(self.get_parameter("ref_alt").value)
        self.heading_deg = float(self.get_parameter("heading_deg").value)
        self.use_virtual = self._as_bool(self.get_parameter("use_virtual").value)
        self.video_source = str(self.get_parameter("video_source").value)
        self.fx = float(self.get_parameter("camera_fx").value)
        self.fy = float(self.get_parameter("camera_fy").value)
        self.cx = float(self.get_parameter("camera_cx").value)
        self.cy = float(self.get_parameter("camera_cy").value)
        self.camera_heading_deg = float(self.get_parameter("camera_heading_deg").value)
        self.camera_tilt_deg = float(self.get_parameter("camera_tilt_deg").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)

        self.sensor_model = SensorModel()
        self.sensor_enu = latlon_to_enu(
            self.gps_lat, self.gps_lon, self.gps_alt, self.ref_lat, self.ref_lon, self.ref_alt
        )

        self.pub = self.create_publisher(DetectionReport, "/detections", 20)
        self.last_truth_enu: Optional[np.ndarray] = None
        self.mog2 = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=20, detectShadows=False)
        self.cap: Optional[cv2.VideoCapture] = None
        self.yolo = None

        if self.use_virtual:
            self.sub = self.create_subscription(PoseStamped, "/gazebo/drone_truth", self._truth_cb, 20)
            self.timer = self.create_timer(1.0 / 8.0, self._virtual_tick)
        else:
            self._setup_real_detector()
            self.timer = self.create_timer(1.0 / 8.0, self._real_tick)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _setup_real_detector(self) -> None:
        if YOLO is not None:
            try:
                self.yolo = YOLO(self.model_path)
                self.get_logger().info(f"Loaded YOLO model from {self.model_path}")
            except Exception as exc:
                self.get_logger().warn(f"YOLO load failed: {exc}. Fallback to motion-only mode.")
                self.yolo = None
        src: object = self.video_source
        if self.video_source.isdigit():
            src = int(self.video_source)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video source: {self.video_source}")

    def _truth_cb(self, msg: PoseStamped) -> None:
        self.last_truth_enu = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float
        )

    def _virtual_tick(self) -> None:
        if self.last_truth_enu is None:
            return
        az, el, rng = enu_to_az_el(self.last_truth_enu, self.sensor_enu)
        sampled = self.sensor_model.sample_detection(az, el, rng, heading_deg=self.heading_deg)
        if sampled is None:
            return
        noisy_az, noisy_el, conf = sampled
        self._publish_detection(noisy_az, noisy_el, conf, "rgb")

    def _real_tick(self) -> None:
        if self.cap is None:
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        yolo_box, yolo_score = self._run_yolo(frame)
        mog_box = self._run_mog2(frame)
        if yolo_box is None or mog_box is None or _iou(yolo_box, mog_box) < 0.3:
            return
        center_x = 0.5 * (yolo_box[0] + yolo_box[2])
        center_y = 0.5 * (yolo_box[1] + yolo_box[3])
        az, el = self._pixel_to_az_el(center_x, center_y)
        self._publish_detection(az, el, max(min(yolo_score, 1.0), 0.05), "rgb")

    def _run_yolo(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int, int, int]], float]:
        if self.yolo is None:
            return None, 0.0
        try:
            results = self.yolo.predict(frame, verbose=False, conf=0.2, device=0)
            if not results:
                return None, 0.0
            boxes = results[0].boxes
            if boxes is None or len(boxes) == 0:
                return None, 0.0
            best_idx = int(np.argmax(boxes.conf.cpu().numpy()))
            xyxy = boxes.xyxy[best_idx].cpu().numpy().astype(int).tolist()
            cls_id = int(boxes.cls[best_idx].item())
            if self.target_class_id >= 0 and cls_id != self.target_class_id:
                return None, 0.0
            score = float(boxes.conf[best_idx].item())
            return (xyxy[0], xyxy[1], xyxy[2], xyxy[3]), score
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference error: {exc}")
            return None, 0.0

    def _run_mog2(self, frame: np.ndarray) -> Optional[Tuple[int, int, int, int]]:
        fg = self.mog2.apply(frame)
        _, mask = cv2.threshold(fg, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 80:
            return None
        x, y, w, h = cv2.boundingRect(cnt)
        return (x, y, x + w, y + h)

    def _pixel_to_az_el(self, x: float, y: float) -> Tuple[float, float]:
        dx = (x - self.cx) / max(self.fx, 1e-6)
        dy = (y - self.cy) / max(self.fy, 1e-6)
        az = math.degrees(math.atan2(dx, 1.0)) + self.camera_heading_deg
        el = -math.degrees(math.atan2(dy, 1.0)) + self.camera_tilt_deg
        az = (az + 360.0) % 360.0
        return az, el

    def _publish_detection(self, azimuth_deg: float, elevation_deg: float, confidence: float, modality: str) -> None:
        msg = DetectionReport()
        msg.node_id = self.node_id
        msg.lat = self.gps_lat
        msg.lon = self.gps_lon
        msg.alt = self.gps_alt
        msg.azimuth_deg = float(azimuth_deg)
        msg.elevation_deg = float(elevation_deg)
        msg.confidence = float(confidence)
        msg.modality = modality
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RGBNode()
    try:
        rclpy.spin(node)
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
