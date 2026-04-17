from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ref_lat = 30.0
    ref_lon = 31.0
    ref_alt = 0.0

    nodes = [
        Node(
            package="node_sim",
            executable="drone_truth_publisher",
            name="drone_truth_publisher",
            output="screen",
        ),
        Node(
            package="node_sim",
            executable="node_rgb",
            name="node_rgb_a",
            output="screen",
            parameters=[
                {
                    "node_id": "A",
                    "gps_lat": ref_lat,
                    "gps_lon": ref_lon,
                    "gps_alt": ref_alt,
                    "ref_lat": ref_lat,
                    "ref_lon": ref_lon,
                    "ref_alt": ref_alt,
                    "heading_deg": 45.0,
                    "use_virtual": True,
                }
            ],
        ),
        Node(
            package="node_sim",
            executable="node_rgb",
            name="node_rgb_b",
            output="screen",
            parameters=[
                {
                    "node_id": "B",
                    "gps_lat": ref_lat + 0.0,
                    "gps_lon": ref_lon + 0.0027,
                    "gps_alt": ref_alt,
                    "ref_lat": ref_lat,
                    "ref_lon": ref_lon,
                    "ref_alt": ref_alt,
                    "heading_deg": 225.0,
                    "use_virtual": True,
                }
            ],
        ),
        Node(
            package="node_sim",
            executable="node_rgb",
            name="node_rgb_c",
            output="screen",
            parameters=[
                {
                    "node_id": "C",
                    "gps_lat": ref_lat + 0.0027,
                    "gps_lon": ref_lon + 0.00135,
                    "gps_alt": ref_alt,
                    "ref_lat": ref_lat,
                    "ref_lon": ref_lon,
                    "ref_alt": ref_alt,
                    "heading_deg": 180.0,
                    "use_virtual": True,
                }
            ],
        ),
        Node(
            package="fusion",
            executable="fusion_node",
            name="fusion_node",
            output="screen",
            parameters=[{"ref_lat": ref_lat, "ref_lon": ref_lon, "ref_alt": ref_alt}],
        ),
        Node(
            package="metrics",
            executable="evaluator",
            name="evaluator",
            output="screen",
        ),
    ]
    return LaunchDescription(nodes)
