from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_real_camera = LaunchConfiguration("use_real_camera")
    use_lora = LaunchConfiguration("use_lora")
    lora_port = LaunchConfiguration("lora_port")
    use_cinematic_camera = LaunchConfiguration("use_cinematic_camera")
    world_name = LaunchConfiguration("world_name")
    # When use_gazebo_truth:=true the gazebo_bridge receives real Gazebo pose on
    # /ardupilot/vehicle_pose (via gz_pose_bridge.launch.py or ros_gz_bridge) and
    # drone_truth_publisher_fallback is suppressed to avoid two sources on
    # /gazebo/drone_truth.  Default false keeps Phase-1/2 virtual behaviour.
    use_gazebo_truth = LaunchConfiguration("use_gazebo_truth")
    ref_lat = 30.0
    ref_lon = 31.0
    ref_alt = 0.0

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_real_camera", default_value="false"),
            DeclareLaunchArgument("use_lora", default_value="false"),
            DeclareLaunchArgument("lora_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument(
                "use_cinematic_camera",
                default_value="false",
                description="Enable Gazebo cinematic camera rig (side offset follow) and bridge its image to ROS.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="cuas_field",
                description="Gazebo world name (must match the SDF <world name=>).",
            ),
            DeclareLaunchArgument(
                "use_gazebo_truth",
                default_value="false",
                description=(
                    "Set true when Gazebo+SITL are running and ros_gz_bridge is publishing "
                    "/ardupilot/vehicle_pose so gazebo_bridge can drive /gazebo/drone_truth. "
                    "Suppresses the synthetic drone_truth_publisher_fallback."
                ),
            ),
            # Phase 2 path - bridge Gazebo vehicle pose to /gazebo/drone_truth.
            # When use_gazebo_truth:=true, Gazebo gives ENU poses directly
            # (via gz_pose_relay → /ardupilot/vehicle_pose), so input_is_ned=false.
            # When use_gazebo_truth:=false the synthetic fallback runs and
            # input_is_ned is irrelevant (no messages on /ardupilot/vehicle_pose).
            Node(
                package="node_sim",
                executable="gazebo_bridge",
                name="gazebo_bridge",
                output="screen",
                condition=UnlessCondition(use_real_camera),
                parameters=[{"input_is_ned": False}],
            ),
            # Synthetic truth: only when NOT using real Gazebo pose as truth source.
            Node(
                package="node_sim",
                executable="drone_truth_publisher",
                name="drone_truth_publisher_fallback",
                output="screen",
                condition=UnlessCondition(use_gazebo_truth),
            ),
            Node(
                package="node_sim",
                executable="node_rgb",
                name="node_rgb_a_virtual",
                output="screen",
                condition=UnlessCondition(use_real_camera),
                parameters=[
                    {
                        "node_id": "A",
                        "gps_lat": ref_lat,
                        "gps_lon": ref_lon,
                        "gps_alt": ref_alt,
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "use_virtual": True,
                    }
                ],
            ),
            Node(
                package="node_sim",
                executable="node_rgb",
                name="node_rgb_a_real",
                output="screen",
                condition=IfCondition(use_real_camera),
                parameters=[
                    {
                        "node_id": "A",
                        "gps_lat": ref_lat,
                        "gps_lon": ref_lon,
                        "gps_alt": ref_alt,
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "use_virtual": False,
                    }
                ],
            ),
            Node(
                package="node_sim",
                executable="node_rgb",
                name="node_rgb_b",
                output="screen",
                condition=UnlessCondition(use_real_camera),
                parameters=[
                    {
                        "node_id": "B",
                        "gps_lat": ref_lat,
                        "gps_lon": ref_lon + 0.0027,
                        "gps_alt": ref_alt,
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "use_virtual": True,
                    }
                ],
            ),
            Node(
                package="node_sim",
                executable="node_rgb",
                name="node_rgb_c",
                output="screen",
                condition=UnlessCondition(use_real_camera),
                parameters=[
                    {
                        "node_id": "C",
                        "gps_lat": ref_lat + 0.0027,
                        "gps_lon": ref_lon + 0.00135,
                        "gps_alt": ref_alt,
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "use_virtual": True,
                    }
                ],
            ),
            Node(
                package="node_sim",
                executable="node_thermal",
                name="node_thermal",
                output="screen",
                condition=IfCondition(use_real_camera),
            ),
            Node(package="node_sim", executable="node_rf", name="node_rf", output="screen"),
            Node(package="node_sim", executable="node_acoustic", name="node_acoustic", output="screen"),
            # Cinematic camera (optional): follows drone with a side offset.
            Node(
                package="node_sim",
                executable="cinematic_cam_controller",
                name="cinematic_cam_controller",
                output="screen",
                condition=IfCondition(use_cinematic_camera),
                parameters=[
                    {
                        "world_name": world_name,
                        "camera_entity_name": "cinematic_cam",
                        # Side-cinematic offset (requested): 4m right, 2m up.
                        "right_offset_m": 4.0,
                        "up_offset_m": 2.0,
                        "forward_offset_m": 0.0,
                        "look_at_drone": True,
                        "set_pose_rate_hz": 10.0,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="cinematic_camera_bridge",
                output="screen",
                condition=IfCondition(use_cinematic_camera),
                arguments=[
                    [
                        PythonExpression(
                            [
                                "'/world/' + '",
                                world_name,
                                "' + '/model/cinematic_cam/link/cam_link/sensor/camera/image'",
                            ]
                        ),
                        "@sensor_msgs/msg/Image[gz.msgs.Image",
                    ],
                ],
            ),
            Node(
                package="fusion",
                executable="fusion_node",
                name="fusion_node",
                output="screen",
                parameters=[
                    {
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "use_lora": use_lora,
                        "lora_serial_port": lora_port,
                    }
                ],
            ),
            Node(
                package="node_sim",
                executable="track_viz_node",
                name="track_viz_node",
                output="screen",
                parameters=[
                    {
                        "frame_id": "enu",
                        "ref_lat": ref_lat,
                        "ref_lon": ref_lon,
                        "ref_alt": ref_alt,
                        "max_path_points": 1500,
                        "ray_fresh_sec": 1.5,
                    }
                ],
            ),
            Node(package="metrics", executable="evaluator", name="evaluator", output="screen"),
        ]
    )
