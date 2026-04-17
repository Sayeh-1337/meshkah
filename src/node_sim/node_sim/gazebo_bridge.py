import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class GazeboBridge(Node):
    """Bridge Gazebo pose to /gazebo/drone_truth in ENU."""

    def __init__(self) -> None:
        super().__init__("gazebo_bridge")
        self.declare_parameter("input_topic", "/ardupilot/vehicle_pose")
        self.declare_parameter("input_is_ned", True)
        input_topic = str(self.get_parameter("input_topic").value)
        self.input_is_ned = self._as_bool(self.get_parameter("input_is_ned").value)

        self.pub = self.create_publisher(PoseStamped, "/gazebo/drone_truth", 20)
        self.sub = self.create_subscription(PoseStamped, input_topic, self._pose_cb, 20)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _pose_cb(self, msg: PoseStamped) -> None:
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = "enu"
        if self.input_is_ned:
            # NED -> ENU: (x_n, y_e, z_d) -> (e, n, u) = (y, x, -z)
            out.pose.position.x = msg.pose.position.y
            out.pose.position.y = msg.pose.position.x
            out.pose.position.z = -msg.pose.position.z
        else:
            out.pose = msg.pose
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
