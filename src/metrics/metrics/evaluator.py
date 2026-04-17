from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport, GlobalTrack
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class Evaluator(Node):
    """Compute RMSE, latency, and false positives every 5 seconds."""

    def __init__(self) -> None:
        super().__init__("evaluator")
        self.track_sub = self.create_subscription(GlobalTrack, "/global_track", self._track_cb, 50)
        self.truth_sub = self.create_subscription(PoseStamped, "/gazebo/drone_truth", self._truth_cb, 50)
        self.det_sub = self.create_subscription(DetectionReport, "/detections", self._det_cb, 200)
        self.timer = self.create_timer(5.0, self._log_metrics)

        self.latest_truth: Optional[np.ndarray] = None
        self.latest_truth_stamp = 0.0
        self.position_errors: Deque[float] = deque(maxlen=2000)
        self.velocity_errors: Deque[float] = deque(maxlen=2000)
        self.latencies_ms: Deque[float] = deque(maxlen=2000)
        self.total_tracks = 0
        self.false_positives = 0
        self.last_detection_stamp = 0.0
        self.prev_truth: Optional[Tuple[float, np.ndarray]] = None

    def _det_cb(self, msg: DetectionReport) -> None:
        self.last_detection_stamp = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9

    def _truth_cb(self, msg: PoseStamped) -> None:
        t = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
        self.latest_truth = pos
        self.latest_truth_stamp = t
        self.prev_truth = (t, pos)

    def _track_cb(self, msg: GlobalTrack) -> None:
        self.total_tracks += 1
        if self.latest_truth is None:
            self.false_positives += 1
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.latest_truth_stamp > 1.0:
            self.false_positives += 1
            return

        est_pos = np.array(msg.position_enu, dtype=float)
        err = float(np.linalg.norm(est_pos - self.latest_truth))
        self.position_errors.append(err)

        if self.prev_truth is not None:
            t_prev, p_prev = self.prev_truth
            dt = max(self.latest_truth_stamp - t_prev, 1e-3)
            truth_vel = (self.latest_truth - p_prev) / dt
            est_vel = np.array(msg.velocity_ms, dtype=float)
            verr = float(np.linalg.norm(est_vel - truth_vel))
            self.velocity_errors.append(verr)

        if self.last_detection_stamp > 0:
            pub_time = self.get_clock().now().nanoseconds * 1e-9
            self.latencies_ms.append((pub_time - self.last_detection_stamp) * 1000.0)

    def _log_metrics(self) -> None:
        pos_rmse = float(np.sqrt(np.mean(np.square(self.position_errors)))) if self.position_errors else 0.0
        vel_rmse = float(np.sqrt(np.mean(np.square(self.velocity_errors)))) if self.velocity_errors else 0.0
        mean_latency = float(np.mean(self.latencies_ms)) if self.latencies_ms else 0.0
        fpr = float(self.false_positives / max(self.total_tracks, 1))
        self.get_logger().info(
            f"RMSE_pos={pos_rmse:.2f}m RMSE_vel={vel_rmse:.2f}m/s "
            f"latency={mean_latency:.1f}ms FPR={fpr:.3f}"
        )


def main() -> None:
    rclpy.init()
    node = Evaluator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
