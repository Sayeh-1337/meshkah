#!/usr/bin/env bash
# ArduPilot SITL for Gazebo / meshkah bridge.
#
# Optional map UI (--map) starts MAVProxy, which downloads SRTM terrain metadata into
# ~/.tilecache/SRTM/ . If you see:
#   Failed to download /SRTM3/filelist_python : 'utf-8' codec can't decode byte 0x80 ...
# the download returned non-text (gzip, proxy page, or bad cache). Fixes: run without map
# (SIM_NO_MAP=1), rm -rf ~/.tilecache/SRTM , upgrade MAVProxy/pymavlink, or fix VPN/firewall.
set -euo pipefail

SITL_DIR="${SITL_DIR:-$HOME/ardupilot}"
VEHICLE="${VEHICLE:-ArduCopter}"
MODEL="${MODEL:-gazebo-iris}"
INSTANCE="${INSTANCE:-0}"
# Set SIM_NO_MAP=1 to skip --map (avoids SRTM / MAVProxy map download issues).
SIM_NO_MAP="${SIM_NO_MAP:-0}"

if [[ ! -d "$SITL_DIR" ]]; then
  echo "SITL_DIR not found: $SITL_DIR"
  exit 1
fi

cd "$SITL_DIR"

# Run text-only (no GUI console/map) to avoid $DISPLAY blocking in WSL.
# MAVLink is still available on udp:127.0.0.1:14550.
EXTRA=()
if [[ "${SIM_NO_MAP}" != "1" ]]; then
  EXTRA=(--map)
fi

echo "Starting ArduPilot SITL for Gazebo bridge..."
# --model JSON  : required for Gazebo Harmonic + ardupilot_gazebo plugin.
# --no-rebuild  : skip waf compilation if the binary already exists (saves ~2 min on repeat runs).
#                 Remove this flag if you have changed ArduPilot source code.
Tools/autotest/sim_vehicle.py \
  -v "$VEHICLE" \
  -f "$MODEL" \
  --model JSON \
  --no-rebuild \
  --instance "$INSTANCE" \
  --out=udp:127.0.0.1:14550 \
  "${EXTRA[@]}"
