#!/usr/bin/env bash
# Build the workspace so installed ROS entry points use .venv's Python.
#
# If `colcon` comes from /usr/bin, setuptools installs packages with /usr/bin/python3
# and console scripts keep that shebang even when .venv is active. Installing colcon
# inside the venv fixes that (see README "Python runtime for ROS nodes").
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
VENV="${WS_ROOT}/.venv"

if [[ ! -f "${VENV}/bin/activate" ]]; then
  echo "Missing ${VENV}. Create it with:"
  echo "  python3 -m venv ${VENV}"
  echo "  source ${VENV}/bin/activate && pip install -r ${WS_ROOT}/requirements.txt"
  exit 1
fi

# shellcheck source=/dev/null
source "${VENV}/bin/activate"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS not found at /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi
# ROS setup.bash references optional unset vars; `set -u` breaks sourcing.
set +u
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

COLCON="$(command -v colcon)"
if [[ "${COLCON}" != "${VENV}/bin/colcon" ]]; then
  echo "Installing colcon into venv (was: ${COLCON:-not found})..."
  pip install -U colcon-common-extensions
fi

echo "Using colcon at: $(command -v colcon)"
echo "Using python at: $(command -v python3)"
cd "${WS_ROOT}"
colcon build "$@"
