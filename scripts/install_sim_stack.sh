#!/usr/bin/env bash
# One-shot installer: Gazebo Harmonic, ArduPilot SITL, ArduPilot Gazebo plugin.
# Run inside WSL2 Ubuntu after ROS 2 Jazzy is installed (ROS is separate).
# Run as a normal user (not root). sudo prompts are expected for apt.
#
# Usage:
#   chmod +x scripts/install_sim_stack.sh
#   ./scripts/install_sim_stack.sh
#
# Optional environment variables:
#   INSTALL_ROS_GZ=1     Also apt install ros-jazzy-ros-gz (default 0)
#   ROS_DISTRO=jazzy
#   BUILD_SITL=0         Skip ./waf build after prereqs (default 1)
#   ARDUPILOT_DIR=...    Default ~/ardupilot
#   ARDUPILOT_GAZEBO_DIR=...  Default ~/ardupilot_gazebo
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "############################################"
echo "# meshkah simulation stack installer"
echo "# Step 1/3: Gazebo Harmonic"
echo "############################################"
"${SCRIPT_DIR}/install_gazebo_harmonic.sh"

echo ""
echo "############################################"
echo "# Step 2/3: ArduPilot SITL"
echo "############################################"
"${SCRIPT_DIR}/install_ardupilot_sitl.sh"

echo ""
echo "############################################"
echo "# Step 3/3: ArduPilot Gazebo plugin"
echo "############################################"
"${SCRIPT_DIR}/install_ardupilot_gazebo.sh"

echo ""
echo "All steps finished."
echo "Re-open your shell or source ~/.bashrc after adding PATH and GZ_* exports shown above."
