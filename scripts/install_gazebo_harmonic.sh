#!/usr/bin/env bash
# Install Gazebo Harmonic on Ubuntu 22.04/24.04 (Jammy/Noble) for WSL2 or native Linux.
# Matches ROS 2 Jazzy expectation: Harmonic.
# Reference: https://gazebosim.org/docs/harmonic/install_ubuntu/
set -euo pipefail

INSTALL_ROS_GZ="${INSTALL_ROS_GZ:-0}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

echo "==> Gazebo Harmonic install (codename: $(lsb_release -cs))"

sudo apt-get update
sudo apt-get install -y curl lsb-release gnupg

KEYRING="/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg"
if [[ ! -f "${KEYRING}" ]]; then
  sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o "${KEYRING}"
fi

LIST="/etc/apt/sources.list.d/gazebo-stable.list"
if [[ ! -f "${LIST}" ]]; then
  echo "deb [arch=$(dpkg --print-architecture) signed-by=${KEYRING}] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee "${LIST}" >/dev/null
fi

sudo apt-get update
sudo apt-get install -y gz-harmonic

echo "==> Verifying gz CLI"
command -v gz
gz --version || gz sim --version || true

if [[ "${INSTALL_ROS_GZ}" == "1" ]]; then
  echo "==> Installing ROS 2 bridge packages for ${ROS_DISTRO} (optional)"
  sudo apt-get install -y "ros-${ROS_DISTRO}-ros-gz" || {
    echo "Warning: ros-${ROS_DISTRO}-ros-gz not available. You can skip this if you only use standalone gz."
  }
fi

echo ""
echo "Quick test (optional):"
echo "  export DISPLAY=:0   # WSLg usually sets this; use X server if needed"
echo "  gz sim shapes.sdf -r"
echo ""
echo "Done."
