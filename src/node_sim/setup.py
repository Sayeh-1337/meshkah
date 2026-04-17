from setuptools import setup

package_name = "node_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="meshkah",
    maintainer_email="research@example.com",
    description="Detector nodes for CUAS-SIM meshkah prototype.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "node_rgb = node_sim.node_rgb:main",
            "node_thermal = node_sim.node_thermal:main",
            "node_rf = node_sim.node_rf:main",
            "node_acoustic = node_sim.node_acoustic:main",
            "drone_truth_publisher = node_sim.drone_truth_publisher:main",
            "gazebo_bridge = node_sim.gazebo_bridge:main",
            "gz_pose_relay = node_sim.gz_pose_relay:main",
            "cinematic_cam_controller = node_sim.cinematic_cam_controller:main",
            "track_viz_node = node_sim.track_viz_node:main",
        ],
    },
)
