"""
gz_pose_relay.py
----------------
Extracts the iris_with_gimbal pose from the Gazebo dynamic_pose TFMessage
and republishes it as PoseStamped on /ardupilot/vehicle_pose.

Data flow:
  ros_gz_bridge (parameter_bridge)
    /world/cuas_field/dynamic_pose/info  (gz.msgs.Pose_V → tf2_msgs/msg/TFMessage)
  gz_pose_relay (this node)
    Finds child_frame_id == model_name in TFMessage
    Converts TransformStamped → PoseStamped
    /ardupilot/vehicle_pose  (geometry_msgs/msg/PoseStamped, ENU world frame)
  gazebo_bridge (input_is_ned:=false)
    /gazebo/drone_truth  (geometry_msgs/msg/PoseStamped, ENU)

Why dynamic_pose/info?
  Gazebo Harmonic's SceneBroadcaster publishes /world/<w>/dynamic_pose/info at
  EVERY physics step (not lazily). This is more reliable than the per-model
  /world/<w>/model/<m>/pose topic which is lazy and may have 0 Hz.

Parameters:
  input_topic          (str)  ROS topic where TFMessage is published.
                              Default: /world/cuas_field/dynamic_pose/info
  output_topic         (str)  Where to publish the PoseStamped.
                              Default: /ardupilot/vehicle_pose
  model_name           (str)  child_frame_id to extract.
                              Default: iris_with_gimbal
  fallback_index       (int)  Fallback transform index when child_frame_id is empty.
                              Default: 0
  frame_id             (str)  frame_id for the output PoseStamped.
                              Default: enu
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class GzPoseRelay(Node):
    """Extract one model's pose from a TFMessage and publish as PoseStamped."""

    def __init__(self) -> None:
        super().__init__("gz_pose_relay")
        self.declare_parameter("input_topic", "/world/cuas_field/dynamic_pose/info")
        self.declare_parameter("output_topic", "/ardupilot/vehicle_pose")
        self.declare_parameter("model_name", "iris_with_gimbal")
        self.declare_parameter("fallback_index", 0)
        self.declare_parameter("frame_id", "enu")

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.model_name = str(self.get_parameter("model_name").value)
        self.fallback_index = int(self.get_parameter("fallback_index").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.pub = self.create_publisher(PoseStamped, output_topic, 20)
        self.sub = self.create_subscription(TFMessage, input_topic, self._cb, 20)
        self._found_once = False
        self._logged_names = False
        self._used_index_fallback = False
        self.get_logger().info(
            f"gz_pose_relay: {input_topic} → filter[{self.model_name}] → {output_topic}"
        )

    def _cb(self, msg: TFMessage) -> None:
        # On the very first message, dump ALL child_frame_ids so we can see what's available.
        if not self._logged_names:
            self._logged_names = True
            names = [tf.child_frame_id for tf in msg.transforms]
            self.get_logger().info(
                f"gz_pose_relay: first TFMessage has {len(names)} transforms.\n"
                f"  child_frame_ids: {names}"
            )

        for tf in msg.transforms:
            # Match exact name OR scoped link names like
            # "iris_with_gimbal::iris_with_standoffs::base_link".
            # We take the first transform that belongs to this model.
            if tf.child_frame_id == self.model_name or tf.child_frame_id.startswith(
                self.model_name + "::"
            ):
                if not self._found_once:
                    self.get_logger().info(
                        f"gz_pose_relay: matched '{tf.child_frame_id}' for model '{self.model_name}'."
                    )
                    self._found_once = True
                out = PoseStamped()
                out.header.stamp = tf.header.stamp
                out.header.frame_id = self.frame_id
                out.pose.position.x = tf.transform.translation.x
                out.pose.position.y = tf.transform.translation.y
                out.pose.position.z = tf.transform.translation.z
                out.pose.orientation = tf.transform.rotation
                self.pub.publish(out)
                return

        # Some ros_gz_bridge builds map Pose_V -> TFMessage with empty child_frame_id.
        # In that case we use a deterministic index fallback (default 0), which
        # corresponds to the first pose in Gazebo's Pose_V list.
        if msg.transforms and all(tf.child_frame_id == "" for tf in msg.transforms):
            idx = max(0, min(self.fallback_index, len(msg.transforms) - 1))
            tf = msg.transforms[idx]
            if not self._used_index_fallback:
                self.get_logger().warn(
                    "gz_pose_relay: TFMessage child_frame_id is empty for all transforms; "
                    f"using fallback_index={idx}.",
                )
                self._used_index_fallback = True
            out = PoseStamped()
            out.header.stamp = tf.header.stamp
            out.header.frame_id = self.frame_id
            out.pose.position.x = tf.transform.translation.x
            out.pose.position.y = tf.transform.translation.y
            out.pose.position.z = tf.transform.translation.z
            out.pose.orientation = tf.transform.rotation
            self.pub.publish(out)
            return
        # Model not found in this message — log once every 5 s so it's not spammy
        self.get_logger().warn(
            f"gz_pose_relay: '{self.model_name}' not found in TFMessage "
            f"({len(msg.transforms)} transforms). "
            "Check model_name parameter or wait for Gazebo to step.",
            throttle_duration_sec=5.0,
        )


def main() -> None:
    rclpy.init()
    node = GzPoseRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
