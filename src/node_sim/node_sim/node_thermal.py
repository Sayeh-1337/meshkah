import math
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport
from rclpy.node import Node

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - optional dependency at runtime
    YOLO = None


class ThermalNode(Node):
    """Thermal detector node with YOLO or grayscale placeholder."""

    def __init__(self) -> None:
        super().__init__("node_thermal")
        self.declare_parameter("node_id", "thermal_1")
        self.declare_parameter("gps_lat", 30.0)
        self.declare_parameter("gps_lon", 31.0)
        self.declare_parameter("gps_alt", 0.0)
        self.declare_parameter("video_source", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("camera_fx", 900.0)
        self.declare_parameter("camera_fy", 900.0)
        self.declare_parameter("camera_cx", 640.0)
        self.declare_parameter("camera_cy", 360.0)
        self.declare_parameter("camera_heading_deg", 0.0)
        self.declare_parameter("camera_tilt_deg", 0.0)

        self.node_id = str(self.get_parameter("node_id").value)
        self.gps_lat = float(self.get_parameter("gps_lat").value)
        self.gps_lon = float(self.get_parameter("gps_lon").value)
        self.gps_alt = float(self.get_parameter("gps_alt").value)
        self.video_source = str(self.get_parameter("video_source").value)
        self.model_path = str(self.get_parameter("model_path").value)
        self.fx = float(self.get_parameter("camera_fx").value)
        self.fy = float(self.get_parameter("camera_fy").value)
        self.cx = float(self.get_parameter("camera_cx").value)
        self.cy = float(self.get_parameter("camera_cy").value)
        self.heading_deg = float(self.get_parameter("camera_heading_deg").value)
        self.tilt_deg = float(self.get_parameter("camera_tilt_deg").value)

        self.pub = self.create_publisher(DetectionReport, "/detections", 20)
        self.yolo = None
        if YOLO is not None and self.model_path:
            try:
                self.yolo = YOLO(self.model_path)
            except Exception as exc:
                self.get_logger().warn(f"Thermal YOLO load failed: {exc}; using placeholder.")

        src: object = self.video_source
        if self.video_source.isdigit():
            src = int(self.video_source)
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open thermal source: {self.video_source}")
        self.timer = self.create_timer(1.0 / 5.0, self._tick)

    def _tick(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        box, conf = self._detect(frame)
        if box is None:
            return
        x1, y1, x2, y2 = box
        cx = 0.5 * (x1 + x2)
        cy = 0.5 * (y1 + y2)
        az, el = self._pixel_to_az_el(cx, cy)
        msg = DetectionReport()
        msg.node_id = self.node_id
        msg.lat = self.gps_lat
        msg.lon = self.gps_lon
        msg.alt = self.gps_alt
        msg.azimuth_deg = az
        msg.elevation_deg = el
        msg.confidence = conf
        msg.modality = "thermal"
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

    def _detect(self, frame: np.ndarray) -> Tuple[Optional[Tuple[int, int, int, int]], float]:
        if self.yolo is not None:
            try:
                results = self.yolo.predict(frame, verbose=False, conf=0.2, device=0)
                boxes = results[0].boxes if results else None
                if boxes is None or len(boxes) == 0:
                    return None, 0.0
                idx = int(np.argmax(boxes.conf.cpu().numpy()))
                xyxy = boxes.xyxy[idx].cpu().numpy().astype(int).tolist()
                return (xyxy[0], xyxy[1], xyxy[2], xyxy[3]), float(boxes.conf[idx].item())
            except Exception as exc:
                self.get_logger().warn(f"Thermal inference error: {exc}")
                return None, 0.0

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 0.0
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) < 50:
            return None, 0.0
        x, y, w, h = cv2.boundingRect(cnt)
        return (x, y, x + w, y + h), 0.6

    def _pixel_to_az_el(self, x: float, y: float) -> Tuple[float, float]:
        dx = (x - self.cx) / max(self.fx, 1e-6)
        dy = (y - self.cy) / max(self.fy, 1e-6)
        az = (math.degrees(math.atan2(dx, 1.0)) + self.heading_deg + 360.0) % 360.0
        el = -math.degrees(math.atan2(dy, 1.0)) + self.tilt_deg
        return az, el


def main() -> None:
    rclpy.init()
    node = ThermalNode()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
