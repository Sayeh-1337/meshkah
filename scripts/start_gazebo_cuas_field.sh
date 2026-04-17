#!/usr/bin/env bash
# Launch Gazebo Harmonic with meshkah cuas_field.world (includes Iris from ardupilot_gazebo).
# Requires a built ardupilot_gazebo tree (see scripts/install_ardupilot_gazebo.sh).
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APG="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
WORLD="${WS_ROOT}/src/sim_world/worlds/cuas_field.world"

if [[ ! -d "${APG}/models" ]]; then
  echo "ardupilot_gazebo models not found under: ${APG}"
  echo "Install/build with: ./scripts/install_ardupilot_gazebo.sh"
  echo "Or set ARDUPILOT_GAZEBO_DIR to your clone."
  exit 1
fi
if [[ ! -f "${APG}/build/libArduPilotPlugin.so" ]]; then
  echo "Missing ${APG}/build/libArduPilotPlugin.so — build ardupilot_gazebo first."
  exit 1
fi
if [[ ! -f "${WORLD}" ]]; then
  echo "Missing world file: ${WORLD}"
  exit 1
fi

export GZ_SIM_SYSTEM_PLUGIN_PATH="${APG}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="${APG}/models:${APG}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

echo "GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}"
echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "Starting: gz sim -v4 -r ${WORLD}"
exec gz sim -v4 -r "${WORLD}"
