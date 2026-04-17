#!/usr/bin/env bash
# Build ArduPilot Gazebo plugin for Gazebo Harmonic (gz-sim8).
# Requires Gazebo Harmonic dev packages and a working compiler.
# Reference: https://github.com/ArduPilot/ardupilot_gazebo
set -euo pipefail

export GZ_VERSION="${GZ_VERSION:-harmonic}"

ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
PLUGIN_REMOTE="${PLUGIN_REMOTE:-https://github.com/ArduPilot/ardupilot_gazebo.git}"
PLUGIN_BRANCH="${PLUGIN_BRANCH:-main}"

echo "==> ArduPilot Gazebo plugin (GZ_VERSION=${GZ_VERSION}) -> ${ARDUPILOT_GAZEBO_DIR}"

sudo apt-get update
sudo apt-get install -y \
  libgz-sim8-dev \
  rapidjson-dev \
  libopencv-dev \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-libav \
  gstreamer1.0-gl \
  cmake \
  build-essential \
  git

if [[ ! -d "${ARDUPILOT_GAZEBO_DIR}/.git" ]]; then
  git clone --branch "${PLUGIN_BRANCH}" "${PLUGIN_REMOTE}" "${ARDUPILOT_GAZEBO_DIR}"
else
  git -C "${ARDUPILOT_GAZEBO_DIR}" pull --ff-only || true
fi

BUILD_DIR="${ARDUPILOT_GAZEBO_DIR}/build"
# Fresh build dir; run cmake/make in env -i so a sourced ROS 2 overlay (e.g. Jazzy gz_sim_vendor)
# cannot pollute CMAKE_PREFIX_PATH / includes (fixes GstCameraPlugin: incomplete gz::msgs::Sensor).
# Do not use bash --login here—it may re-source ~/.bashrc and pull ROS back in.
rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
JOBS="$(nproc 2>/dev/null || echo 4)"
env -i \
  HOME="${HOME}" \
  USER="${USER:-}" \
  SHELL=/bin/bash \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  TERM="${TERM:-dumb}" \
  GZ_VERSION="${GZ_VERSION}" \
  LANG="${LANG:-C.UTF-8}" \
  bash -c 'set -euo pipefail
    cd "$1"
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
    cmake --build . -j"$2"
  ' '_' "${BUILD_DIR}" "${JOBS}"

echo ""
echo "Add these lines to ~/.bashrc (adjust paths if you moved the repo):"
echo "  export GZ_SIM_SYSTEM_PLUGIN_PATH=${BUILD_DIR}:\$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "  export GZ_SIM_RESOURCE_PATH=${ARDUPILOT_GAZEBO_DIR}/models:${ARDUPILOT_GAZEBO_DIR}/worlds:\$GZ_SIM_RESOURCE_PATH"
echo ""
echo "Then start Gazebo with a world that loads the Iris (or your vehicle), and in another terminal:"
echo "  cd ~/ardupilot/ArduCopter"
echo "  ../Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"
echo ""
echo "Done."
