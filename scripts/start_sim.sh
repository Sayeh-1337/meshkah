#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS setup not found for distro '${ROS_DISTRO}' at /opt/ros/${ROS_DISTRO}/setup.bash"
  echo "Set ROS_DISTRO to your installed ROS 2 distro, for example: export ROS_DISTRO=jazzy"
  exit 1
fi

# ROS/colcon setup scripts reference optional unset vars; `set -u` breaks sourcing.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS_ROOT/install/setup.bash"
fi
set -u

echo "Launching full meshkah simulation stack on ROS 2 ${ROS_DISTRO}..."
ros2 launch "$WS_ROOT/launch/full_sim.launch.py"
