from collections import deque
from typing import Deque, List

import rclpy
from cuas_msgs.msg import DetectionReport, GlobalTrack
from rclpy.node import Node

from .ekf import TrackEKF
from .lora_transport import LoRaTransport
from .modal_vote import compute_modal_vote
from .triangulation import triangulate_target


class FusionNode(Node):
    """Fusion node for rolling window triangulation and EKF tracking."""

    def __init__(self) -> None:
        super().__init__("fusion_node")
        self.declare_parameter("ref_lat", 30.0)
        self.declare_parameter("ref_lon", 31.0)
        self.declare_parameter("ref_alt", 0.0)
        self.declare_parameter("fusion_window_sec", 1.0)
        self.declare_parameter("use_lora", False)
        self.declare_parameter("lora_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("lora_baud", 115200)

        self.ref_lat = float(self.get_parameter("ref_lat").value)
        self.ref_lon = float(self.get_parameter("ref_lon").value)
        self.ref_alt = float(self.get_parameter("ref_alt").value)
        self.fusion_window_sec = float(self.get_parameter("fusion_window_sec").value)
        self.use_lora = self._as_bool(self.get_parameter("use_lora").value)
        self.lora_serial_port = str(self.get_parameter("lora_serial_port").value)
        self.lora_baud = int(self.get_parameter("lora_baud").value)

        self.ekf = TrackEKF(dt=0.05)
        self.buffer: Deque[DetectionReport] = deque()
        self.latest_vote = None
        self.track_state = "lost"

        self.pub = self.create_publisher(GlobalTrack, "/global_track", 20)
        self.sub = self.create_subscription(DetectionReport, "/detections", self._detection_cb, 200)
        self.predict_timer = self.create_timer(0.05, self._predict_tick)  # 20 Hz
        self.fuse_timer = self.create_timer(0.05, self._fuse_tick)

        self.transport = LoRaTransport(
            serial_port=self.lora_serial_port,
            baud=self.lora_baud,
            use_sim=not self.use_lora,
        )

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _detection_cb(self, msg: DetectionReport) -> None:
        self.buffer.append(msg)
        self._prune_buffer()

    def _prune_buffer(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        while self.buffer:
            msg = self.buffer[0]
            stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
            if now - stamp <= self.fusion_window_sec:
                break
            self.buffer.popleft()

    def _collect_lora(self) -> None:
        if not self.use_lora:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        for report in self.transport.read_reports():
            if LoRaTransport.is_fresh(report, now_sec=now, max_skew_sec=2.0):
                self.buffer.append(report)

    def _predict_tick(self) -> None:
        self.ekf.predict()

    def _fuse_tick(self) -> None:
        self._collect_lora()
        self._prune_buffer()
        reports = list(self.buffer)
        if not reports:
            return

        vote = compute_modal_vote(reports)
        self.latest_vote = vote
        self.track_state = vote.track_state
        if len({r.node_id for r in vote.bearing_reports}) < 2:
            return
        try:
            pos_enu = triangulate_target(vote.bearing_reports, self.ref_lat, self.ref_lon, self.ref_alt)
        except Exception as exc:
            self.get_logger().warn(f"Triangulation failed: {exc}")
            return

        self.ekf.update(pos_enu)
        if vote.track_state in ("tentative", "confirmed"):
            self._publish_track(vote.combined_confidence, vote.num_sensors, vote.track_state)

    def _publish_track(self, confidence: float, num_sensors: int, state: str) -> None:
        msg = GlobalTrack()
        pos = self.ekf.get_position()
        vel = self.ekf.get_velocity()
        pred = self.ekf.predict_position(1.0)
        msg.position_enu = [float(pos[0]), float(pos[1]), float(pos[2])]
        msg.velocity_ms = [float(vel[0]), float(vel[1]), float(vel[2])]
        msg.predicted_1s = [float(pred[0]), float(pred[1]), float(pred[2])]
        msg.confidence = float(confidence)
        msg.num_sensors = int(num_sensors)
        msg.track_state = state
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = FusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
