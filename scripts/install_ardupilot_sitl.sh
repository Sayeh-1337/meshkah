#!/usr/bin/env bash
# Clone ArduPilot, install Ubuntu prerequisites, and optionally build SITL.
# Default install dir: ~/ardupilot
# Reference: https://ardupilot.org/dev/docs/building-setup-linux.html
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo "Do not run this script as root. The ArduPilot prereqs script refuses root."
  echo "Run as your normal user; it will call sudo when needed."
  exit 1
fi

ARDUPILOT_DIR="${ARDUPILOT_DIR:-${HOME}/ardupilot}"
ARDUPILOT_REMOTE="${ARDUPILOT_REMOTE:-https://github.com/ArduPilot/ardupilot.git}"
ARDUPILOT_BRANCH="${ARDUPILOT_BRANCH:-master}"
BUILD_SITL="${BUILD_SITL:-1}"

echo "==> ArduPilot SITL setup -> ${ARDUPILOT_DIR}"

if [[ ! -d "${ARDUPILOT_DIR}/.git" ]]; then
  git clone --branch "${ARDUPILOT_BRANCH}" --recurse-submodules "${ARDUPILOT_REMOTE}" "${ARDUPILOT_DIR}"
else
  echo "Repo exists, updating submodules..."
  git -C "${ARDUPILOT_DIR}" pull --ff-only || true
  git -C "${ARDUPILOT_DIR}" submodule update --init --recursive
fi

PREREQ="${ARDUPILOT_DIR}/Tools/environment_install/install-prereqs-ubuntu.sh"
if [[ ! -f "${PREREQ}" ]]; then
  echo "Missing prereqs script: ${PREREQ}"
  exit 1
fi

echo "==> Running ArduPilot Ubuntu prerequisites (may take several minutes)..."
pushd "${ARDUPILOT_DIR}" >/dev/null
bash "${PREREQ}" -y
export PATH="${ARDUPILOT_DIR}/Tools/autotest:${PATH}"

if [[ "${BUILD_SITL}" == "1" ]]; then
  echo "==> Building SITL (copter)..."
  ./waf configure --board sitl
  ./waf copter
fi
popd >/dev/null

echo ""
echo "Add to ~/.bashrc (optional):"
echo "  export PATH=\"${ARDUPILOT_DIR}/Tools/autotest:\$PATH\""
echo ""
echo "Smoke test (no Gazebo):"
echo "  cd ${ARDUPILOT_DIR}/ArduCopter"
echo "  ../Tools/autotest/sim_vehicle.py -v ArduCopter --map --console"
echo ""
echo "WSL2 mirrored networking users may need: --no-wsl2-network"
echo "Done."
