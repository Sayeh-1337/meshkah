import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DroneTruthPublisher(Node):
    """Simple straight line drone truth publisher for phase 1."""

    def __init__(self) -> None:
        super().__init__("drone_truth_publisher")
        self.declare_parameter("speed_ms", 10.0)
        self.declare_parameter("start_east", 0.0)
        self.declare_parameter("start_north", 0.0)
        self.declare_parameter("start_up", 80.0)
        self.declare_parameter("heading_deg", 35.0)
        self.declare_parameter("publish_hz", 20.0)

        self.speed_ms = float(self.get_parameter("speed_ms").value)
        self.start_e = float(self.get_parameter("start_east").value)
        self.start_n = float(self.get_parameter("start_north").value)
        self.start_u = float(self.get_parameter("start_up").value)
        self.heading_deg = float(self.get_parameter("heading_deg").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.publisher = self.create_publisher(PoseStamped, "/gazebo/drone_truth", 10)
        self.t = 0.0
        self.dt = 1.0 / max(publish_hz, 1.0)
        self.timer = self.create_timer(self.dt, self._tick)

    def _tick(self) -> None:
        heading = math.radians(self.heading_deg)
        east = self.start_e + self.speed_ms * math.sin(heading) * self.t
        north = self.start_n + self.speed_ms * math.cos(heading) * self.t
        up = self.start_u

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "enu"
        msg.pose.position.x = east
        msg.pose.position.y = north
        msg.pose.position.z = up
        msg.pose.orientation.w = 1.0
        self.publisher.publish(msg)
        self.t += self.dt


def main() -> None:
    rclpy.init()
    node = DroneTruthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
