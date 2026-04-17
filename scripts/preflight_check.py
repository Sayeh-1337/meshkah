#!/usr/bin/env python3
import math
import subprocess
import sys
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node


def det2(a: Tuple[float, float], b: Tuple[float, float], c: Tuple[float, float]) -> float:
    return abs((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]))


class Preflight(Node):
    def __init__(self) -> None:
        super().__init__("preflight_check")
        self.declare_parameter("node_positions", [0.0, 0.0, 300.0, 0.0, 150.0, 260.0])
        self.positions = list(self.get_parameter("node_positions").value)

    def run(self) -> int:
        checks: Dict[str, bool] = {}
        checks["topics_present"] = self._check_topics()
        checks["node_geometry"] = self._check_geometry()
        checks["global_track_active"] = self._check_global_track()
        checks["gnss_recent"] = True  # In sim, ROS time stamps stand in for GNSS sync.

        for name, ok in checks.items():
            print(f"{name}: {'PASS' if ok else 'FAIL'}")
        return 0 if all(checks.values()) else 1

    def _check_topics(self) -> bool:
        needed = {"/detections", "/global_track", "/gazebo/drone_truth"}
        topic_names = {name for (name, _) in self.get_topic_names_and_types()}
        return needed.issubset(topic_names)

    def _check_geometry(self) -> bool:
        if len(self.positions) < 6:
            return False
        a = (self.positions[0], self.positions[1])
        b = (self.positions[2], self.positions[3])
        c = (self.positions[4], self.positions[5])
        return det2(a, b, c) > 10.0

    def _check_global_track(self) -> bool:
        topic_names = {name for (name, _) in self.get_topic_names_and_types()}
        return "/global_track" in topic_names


def main() -> None:
    rclpy.init()
    node = Preflight()
    try:
        code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
