from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rclpy
from cuas_msgs.msg import DetectionReport, GlobalTrack
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from .sensor_model import latlon_to_enu


@dataclass
class DetectionRay:
    sensor_enu: np.ndarray
    stamp_sec: float


class TrackVizNode(Node):
    """Publish RViz-friendly markers and path from /global_track + /detections."""

    def __init__(self) -> None:
        super().__init__("track_viz_node")

        self.declare_parameter("frame_id", "enu")
        self.declare_parameter("ref_lat", 30.0)
        self.declare_parameter("ref_lon", 31.0)
        self.declare_parameter("ref_alt", 0.0)
        self.declare_parameter("max_path_points", 1500)
        self.declare_parameter("ray_fresh_sec", 1.5)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.ref_lat = float(self.get_parameter("ref_lat").value)
        self.ref_lon = float(self.get_parameter("ref_lon").value)
        self.ref_alt = float(self.get_parameter("ref_alt").value)
        self.max_path_points = int(self.get_parameter("max_path_points").value)
        self.ray_fresh_sec = float(self.get_parameter("ray_fresh_sec").value)

        self.markers_pub = self.create_publisher(MarkerArray, "/track_markers", 20)
        self.path_pub = self.create_publisher(Path, "/track_path", 20)

        self.track_sub = self.create_subscription(GlobalTrack, "/global_track", self._track_cb, 50)
        self.det_sub = self.create_subscription(DetectionReport, "/detections", self._det_cb, 200)

        self.path_points: Deque[np.ndarray] = deque(maxlen=max(10, self.max_path_points))
        self.latest_track: Optional[GlobalTrack] = None
        self.latest_track_stamp = 0.0
        self.latest_rays: Dict[str, DetectionRay] = {}

        self.get_logger().info(
            "track_viz_node: publishing /track_markers and /track_path (frame_id=%s)" % self.frame_id
        )

    def _det_cb(self, msg: DetectionReport) -> None:
        sensor_enu = latlon_to_enu(
            msg.lat, msg.lon, msg.alt, self.ref_lat, self.ref_lon, self.ref_alt
        )
        t = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9
        self.latest_rays[msg.node_id] = DetectionRay(sensor_enu=sensor_enu, stamp_sec=t)

    def _track_cb(self, msg: GlobalTrack) -> None:
        self.latest_track = msg
        self.latest_track_stamp = self.get_clock().now().nanoseconds * 1e-9

        pos = np.array(msg.position_enu, dtype=float)
        self.path_points.append(pos)

        self._publish_path()
        self._publish_markers(msg)

    def _publish_path(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        path = Path()
        path.header = Header(stamp=now_msg, frame_id=self.frame_id)

        for p in self.path_points:
            ps = PoseStamped()
            ps.header = Header(stamp=now_msg, frame_id=self.frame_id)
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.path_pub.publish(path)

    def _publish_markers(self, msg: GlobalTrack) -> None:
        now_msg = self.get_clock().now().to_msg()
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        ma = MarkerArray()

        # Marker 0: current fused target position.
        current = Marker()
        current.header = Header(stamp=now_msg, frame_id=self.frame_id)
        current.ns = "track"
        current.id = 0
        current.type = Marker.SPHERE
        current.action = Marker.ADD
        current.pose.position.x = float(msg.position_enu[0])
        current.pose.position.y = float(msg.position_enu[1])
        current.pose.position.z = float(msg.position_enu[2])
        current.pose.orientation.w = 1.0
        current.scale.x = 2.0
        current.scale.y = 2.0
        current.scale.z = 2.0
        current.color.r = 0.1
        current.color.g = 0.9
        current.color.b = 0.1
        current.color.a = 0.95
        ma.markers.append(current)

        # Marker 1: predicted 1s position.
        pred = Marker()
        pred.header = Header(stamp=now_msg, frame_id=self.frame_id)
        pred.ns = "track"
        pred.id = 1
        pred.type = Marker.SPHERE
        pred.action = Marker.ADD
        pred.pose.position.x = float(msg.predicted_1s[0])
        pred.pose.position.y = float(msg.predicted_1s[1])
        pred.pose.position.z = float(msg.predicted_1s[2])
        pred.pose.orientation.w = 1.0
        pred.scale.x = 1.2
        pred.scale.y = 1.2
        pred.scale.z = 1.2
        pred.color.r = 1.0
        pred.color.g = 0.75
        pred.color.b = 0.1
        pred.color.a = 0.9
        ma.markers.append(pred)

        # Marker 2: line strip for path (mirrors nav_msgs/Path, easier for quick RViz view).
        path_line = Marker()
        path_line.header = Header(stamp=now_msg, frame_id=self.frame_id)
        path_line.ns = "track"
        path_line.id = 2
        path_line.type = Marker.LINE_STRIP
        path_line.action = Marker.ADD
        path_line.pose.orientation.w = 1.0
        path_line.scale.x = 0.25
        path_line.color.r = 0.0
        path_line.color.g = 0.85
        path_line.color.b = 1.0
        path_line.color.a = 0.9
        for p in self.path_points:
            pt = Point()
            pt.x = float(p[0])
            pt.y = float(p[1])
            pt.z = float(p[2])
            path_line.points.append(pt)
        ma.markers.append(path_line)

        # Marker 3: detection rays (sensor position -> fused target position).
        rays = Marker()
        rays.header = Header(stamp=now_msg, frame_id=self.frame_id)
        rays.ns = "track"
        rays.id = 3
        rays.type = Marker.LINE_LIST
        rays.action = Marker.ADD
        rays.pose.orientation.w = 1.0
        rays.scale.x = 0.08
        rays.color.r = 1.0
        rays.color.g = 0.3
        rays.color.b = 0.3
        rays.color.a = 0.8
        target_pt = Point(
            x=float(msg.position_enu[0]),
            y=float(msg.position_enu[1]),
            z=float(msg.position_enu[2]),
        )
        for ray in list(self.latest_rays.values()):
            if now_sec - ray.stamp_sec > self.ray_fresh_sec:
                continue
            s = Point(x=float(ray.sensor_enu[0]), y=float(ray.sensor_enu[1]), z=float(ray.sensor_enu[2]))
            rays.points.append(s)
            rays.points.append(target_pt)
        ma.markers.append(rays)

        self.markers_pub.publish(ma)


def main() -> None:
    rclpy.init()
    node = TrackVizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

