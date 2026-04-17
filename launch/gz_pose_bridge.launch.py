"""
gz_pose_bridge.launch.py
------------------------
Bridges Gazebo dynamic model poses into ROS 2 so meshkah's gazebo_bridge
node can drive /gazebo/drone_truth from the real Gazebo simulation.

Why dynamic_pose/info?
  Gazebo Harmonic's SceneBroadcaster publishes:
    /world/<w>/dynamic_pose/info  →  gz.msgs.Pose_V  (ALL dynamic entities)
  at EVERY physics step (not lazy).  The per-model topic
    /world/<w>/model/<m>/pose
  is lazy and only publishes when a subscriber is present on the Gazebo side,
  which was unreliable in testing.

  ros_gz_bridge maps gz.msgs.Pose_V → tf2_msgs/msg/TFMessage.  Each entity's
  pose becomes a TransformStamped with child_frame_id = entity name.
  gz_pose_relay then finds child_frame_id == model_name and republishes
  as PoseStamped on /ardupilot/vehicle_pose.

Data flow:
  Gazebo SceneBroadcaster
    gz topic: /world/<world>/dynamic_pose/info  (gz.msgs.Pose_V)
  ros_gz_bridge (parameter_bridge)
    ROS 2: /world/<world>/dynamic_pose/info  (tf2_msgs/msg/TFMessage)
  gz_pose_relay node (node_sim package)
    Filters child_frame_id == model_name
    ROS 2: /ardupilot/vehicle_pose  (geometry_msgs/msg/PoseStamped, ENU)
  gazebo_bridge node  (input_is_ned:=false)
    ROS 2: /gazebo/drone_truth  (geometry_msgs/msg/PoseStamped, ENU)

Arguments:
  world_name   Gazebo world name  (default: cuas_field)
  model_name   Gazebo model name to extract  (default: iris_with_gimbal)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    world_name = LaunchConfiguration("world_name")
    model_name = LaunchConfiguration("model_name")

    # ROS topic produced by the bridge (same as gz topic name by default)
    dynamic_pose_topic = PythonExpression(
        ["'/world/' + '", world_name, "' + '/dynamic_pose/info'"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world_name",
                default_value="cuas_field",
                description="Gazebo world name (must match <world name=> in the SDF).",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="iris_with_gimbal",
                description="Gazebo model name to extract the pose of.",
            ),

            # Step 1: Bridge gz.msgs.Pose_V → tf2_msgs/msg/TFMessage.
            # dynamic_pose/info is published at every physics step (never lazy).
            # Each entity's pose becomes a TF transform with child_frame_id = entity name.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_dynamic_pose_bridge",
                output="screen",
                arguments=[
                    [
                        dynamic_pose_topic,
                        "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    ],
                ],
            ),

            # Step 2: Extract model_name's pose from the TFMessage and publish
            # as PoseStamped on /ardupilot/vehicle_pose (ENU, no NED conversion).
            Node(
                package="node_sim",
                executable="gz_pose_relay",
                name="gz_pose_relay",
                output="screen",
                parameters=[
                    {
                        "input_topic": dynamic_pose_topic,
                        "output_topic": "/ardupilot/vehicle_pose",
                        "model_name": model_name,
                        "frame_id": "enu",
                    }
                ],
            ),
        ]
    )
