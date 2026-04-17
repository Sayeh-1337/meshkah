import random

import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from .sensor_model import latlon_to_enu


class RFNode(Node):
    """Synthetic RF detector publishing non-directional detections at 1 Hz."""

    def __init__(self) -> None:
        super().__init__("node_rf")
        self.declare_parameter("node_id", "rf_1")
        self.declare_parameter("gps_lat", 30.0)
        self.declare_parameter("gps_lon", 31.0)
        self.declare_parameter("gps_alt", 0.0)
        self.declare_parameter("ref_lat", 30.0)
        self.declare_parameter("ref_lon", 31.0)
        self.declare_parameter("ref_alt", 0.0)
        self.declare_parameter("range_threshold_m", 300.0)
        self.declare_parameter("p_detect_in", 0.85)
        self.declare_parameter("p_detect_out", 0.05)

        self.node_id = str(self.get_parameter("node_id").value)
        self.gps_lat = float(self.get_parameter("gps_lat").value)
        self.gps_lon = float(self.get_parameter("gps_lon").value)
        self.gps_alt = float(self.get_parameter("gps_alt").value)
        self.ref_lat = float(self.get_parameter("ref_lat").value)
        self.ref_lon = float(self.get_parameter("ref_lon").value)
        self.ref_alt = float(self.get_parameter("ref_alt").value)
        self.range_threshold_m = float(self.get_parameter("range_threshold_m").value)
        self.p_detect_in = float(self.get_parameter("p_detect_in").value)
        self.p_detect_out = float(self.get_parameter("p_detect_out").value)

        self.sensor_enu = latlon_to_enu(
            self.gps_lat, self.gps_lon, self.gps_alt, self.ref_lat, self.ref_lon, self.ref_alt
        )
        self.truth_enu = None
        self.pub = self.create_publisher(DetectionReport, "/detections", 20)
        self.sub = self.create_subscription(PoseStamped, "/gazebo/drone_truth", self._truth_cb, 20)
        self.timer = self.create_timer(1.0, self._tick)

    def _truth_cb(self, msg: PoseStamped) -> None:
        self.truth_enu = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)

    def _tick(self) -> None:
        p = self.p_detect_out
        if self.truth_enu is not None:
            rng = float(np.linalg.norm(self.truth_enu - self.sensor_enu))
            p = self.p_detect_in if rng <= self.range_threshold_m else self.p_detect_out
        if random.random() > p:
            return

        msg = DetectionReport()
        msg.node_id = self.node_id
        msg.lat = self.gps_lat
        msg.lon = self.gps_lon
        msg.alt = self.gps_alt
        msg.azimuth_deg = 0.0
        msg.elevation_deg = 0.0
        msg.confidence = 0.4
        msg.modality = "rf"
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RFNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
