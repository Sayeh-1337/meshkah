import math
import subprocess
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Return yaw (rad) assuming ENU where yaw=0 points North and +yaw turns East."""
    # Standard yaw-from-quaternion (Z-Y-X / yaw-pitch-roll convention).
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def _rpy_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Roll/pitch/yaw (rad) -> quaternion (x,y,z,w). Uses ZYX (yaw-z, pitch-y, roll-x)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


@dataclass
class FollowOffsets:
    right_m: float
    forward_m: float
    up_m: float


class CinematicCamController(Node):
    """
    Side-cinematic camera follow controller.

    Behavior:
      - Subscribe to `/ardupilot/vehicle_pose` (ENU position, quaternion orientation)
      - Compute camera position = drone_pos + (right*4m) + (forward*0m) + (up*2m)
      - Optionally set camera orientation to "look at" the drone position
      - Set camera rig pose in Gazebo using `gz service .../set_pose`
    """

    def __init__(self) -> None:
        super().__init__("cinematic_cam_controller")

        self.declare_parameter("drone_pose_topic", "/ardupilot/vehicle_pose")
        self.declare_parameter("world_name", "cuas_field")
        self.declare_parameter("camera_entity_name", "cinematic_cam")
        self.declare_parameter("right_offset_m", 4.0)
        self.declare_parameter("forward_offset_m", 0.0)
        self.declare_parameter("up_offset_m", 2.0)
        self.declare_parameter("set_pose_rate_hz", 10.0)
        self.declare_parameter("look_at_drone", True)
        self.declare_parameter("gz_timeout_ms", 300)

        self._drone_pose_topic = str(self.get_parameter("drone_pose_topic").value)
        self._world_name = str(self.get_parameter("world_name").value)
        self._camera_entity_name = str(self.get_parameter("camera_entity_name").value)
        self._offsets = FollowOffsets(
            right_m=float(self.get_parameter("right_offset_m").value),
            forward_m=float(self.get_parameter("forward_offset_m").value),
            up_m=float(self.get_parameter("up_offset_m").value),
        )
        self._set_pose_rate_hz = float(self.get_parameter("set_pose_rate_hz").value)
        self._look_at_drone = bool(self.get_parameter("look_at_drone").value)
        self._gz_timeout_ms = int(self.get_parameter("gz_timeout_ms").value)

        self._latest_drone_pose: Optional[PoseStamped] = None
        self._have_drone_pose = False
        self._sent_first = False
        self._last_sent_pose: Optional[tuple[float, float, float, float, float, float, float]] = None

        self._sub = self.create_subscription(
            PoseStamped, self._drone_pose_topic, self._drone_pose_cb, 20
        )
        self._timer = self.create_timer(1.0 / max(self._set_pose_rate_hz, 1e-6), self._tick)

        self.get_logger().info(
            f"cinematic_cam_controller: follow[{self._drone_pose_topic}] "
            f"-> camera[{self._camera_entity_name}] in world[{self._world_name}] "
            f"(right={self._offsets.right_m}m up={self._offsets.up_m}m forward={self._offsets.forward_m}m)"
        )

    def _drone_pose_cb(self, msg: PoseStamped) -> None:
        self._latest_drone_pose = msg
        self._have_drone_pose = True

    def _tick(self) -> None:
        if not self._have_drone_pose or self._latest_drone_pose is None:
            return

        p = self._latest_drone_pose.pose.position
        q = self._latest_drone_pose.pose.orientation

        drone_e = float(p.x)
        drone_n = float(p.y)
        drone_u = float(p.z)

        yaw = _quat_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))

        # ENU yaw: yaw=0 => facing North, +yaw => East.
        # forward (in ENU) = (sin(yaw), cos(yaw), 0)
        # right   (in ENU) = (cos(yaw), -sin(yaw), 0)
        fwd_e = math.sin(yaw) * self._offsets.forward_m
        fwd_n = math.cos(yaw) * self._offsets.forward_m
        right_e = math.cos(yaw) * self._offsets.right_m
        right_n = -math.sin(yaw) * self._offsets.right_m

        cam_e = drone_e + right_e + fwd_e
        cam_n = drone_n + right_n + fwd_n
        cam_u = drone_u + self._offsets.up_m

        # Orientation: either copy drone yaw or look-at the drone.
        if self._look_at_drone:
            d_e = drone_e - cam_e
            d_n = drone_n - cam_n
            d_u = drone_u - cam_u
            horiz = math.sqrt(d_e * d_e + d_n * d_n)

            # Yaw points the camera toward the drone on the EN-plane.
            yaw_cam = math.atan2(d_e, d_n)

            # Pitch points camera optical axis toward drone.
            # If the camera is above the drone, d_u is negative and this yields
            # a negative pitch (downward), which keeps the drone in frame.
            pitch_cam = math.atan2(d_u, max(horiz, 1e-6))

            roll_cam = 0.0
            qx, qy, qz, qw = _rpy_to_quat(roll_cam, pitch_cam, yaw_cam)
        else:
            # Keep a simple orientation aligned with drone.
            qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)

        req = (
            f'name: "{self._camera_entity_name}", '
            f"position: {{x: {cam_e:.4f}, y: {cam_n:.4f}, z: {cam_u:.4f}}}, "
            f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}"
        )

        # Avoid flooding Gazebo with nearly-identical pose updates.
        pose_tuple = (cam_e, cam_n, cam_u, qx, qy, qz, qw)
        if self._last_sent_pose is not None:
            p = self._last_sent_pose
            pos_delta = math.sqrt((cam_e - p[0]) ** 2 + (cam_n - p[1]) ** 2 + (cam_u - p[2]) ** 2)
            quat_delta = abs(qx - p[3]) + abs(qy - p[4]) + abs(qz - p[5]) + abs(qw - p[6])
            if pos_delta < 0.01 and quat_delta < 0.002:
                return

        # Use blocking service endpoint so requester/response lifecycle is stable.
        service = f"/world/{self._world_name}/set_pose/blocking"
        cmd = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            f"{self._gz_timeout_ms}",
            "--req",
            req,
        ]

        # Avoid blocking ROS callbacks too long.
        try:
            result = subprocess.run(cmd, check=False, capture_output=True, text=True, timeout=1.0)
        except subprocess.TimeoutExpired:
            # If gz is busy, skip this tick.
            return

        if result.returncode != 0 and "Host unreachable" not in result.stderr:
            self.get_logger().warn(
                f"cinematic_cam_controller: set_pose call failed rc={result.returncode}: {result.stderr.strip()}",
                throttle_duration_sec=5.0,
            )
            return

        self._last_sent_pose = pose_tuple

        if not self._sent_first:
            self._sent_first = True
            self.get_logger().info("cinematic_cam_controller: first set_pose sent to Gazebo.")


def main() -> None:
    rclpy.init()
    node = CinematicCamController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

