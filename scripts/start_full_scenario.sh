#!/usr/bin/env bash
# start_full_scenario.sh
# -----------------------------------------------------------------------
# Integrated meshkah + Gazebo 3D anti-drone scenario with ENV ISOLATION:
#   - Gazebo process: clean env, NO ROS, but DISPLAY/WAYLAND passed through
#   - SITL process:   venv-ardupilot only, NO ROS, headless (no --map/--console)
#   - ROS processes:  ROS + meshkah overlay (+ optional meshkah .venv)
# -----------------------------------------------------------------------
set -euo pipefail

WS_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APG="${ARDUPILOT_GAZEBO_DIR:-${HOME}/ardupilot_gazebo}"
SITL_DIR="${SITL_DIR:-${HOME}/ardupilot}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
VEHICLE="${VEHICLE:-ArduCopter}"
WORLD_NAME="${WORLD_NAME:-cuas_field}"
WORLD_FILE="${WS_ROOT}/src/sim_world/worlds/cuas_field.world"

# SITL always runs headless inside the script (no --map / --console) to avoid
# $DISPLAY errors in the clean env.  Run start_sitl.sh manually in a separate
# terminal if you want the MAVProxy GUI.
SITL_HEADLESS=1

# Optional venvs (kept separate on purpose)
ROS_VENV="${ROS_VENV:-${WS_ROOT}/.venv}"
SITL_VENV="${SITL_VENV:-${HOME}/venv-ardupilot}"

# Cinematic camera + drone auto-fly are optional (default off).
# Enable with:
#   USE_CINEMATIC_CAMERA:=true  AUTO_FLY:=1  AUTO_FLY_LOOPS:=0
USE_CINEMATIC_CAMERA="${USE_CINEMATIC_CAMERA:-false}"
AUTO_FLY="${AUTO_FLY:-0}"
# Orbit loop count for auto-fly:
#   1 = one pass (A->B->C), 2 = two passes, 0 = infinite loop
AUTO_FLY_LOOPS="${AUTO_FLY_LOOPS:-1}"

GZ_PLUGIN_PATH="${APG}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
GZ_RESOURCE_PATH="${APG}/models:${APG}/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# Pass display variables through to the Gazebo env-i block so the GUI shows.
_DISPLAY="${DISPLAY:-}"
_WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}"
_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}"
_DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}"

# ---- sanity checks -------------------------------------------------------
if [[ ! -f "${APG}/build/libArduPilotPlugin.so" ]]; then
  echo "Missing ${APG}/build/libArduPilotPlugin.so"
  echo "Build with: ./scripts/install_ardupilot_gazebo.sh"
  exit 1
fi
if [[ ! -d "${SITL_DIR}" ]]; then
  echo "SITL_DIR not found: ${SITL_DIR}"
  echo "Install with: ./scripts/install_ardupilot_sitl.sh"
  exit 1
fi
if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS not found at /opt/ros/${ROS_DISTRO}/setup.bash"
  exit 1
fi
if [[ ! -f "${WS_ROOT}/install/setup.bash" ]]; then
  echo "Workspace not built. Run: colcon build (or scripts/build_in_venv.sh)"
  exit 1
fi

if [[ ! -f "${ROS_VENV}/bin/activate" ]]; then
  echo "Info: ROS_VENV not found at ${ROS_VENV}. Continuing without activating a ROS venv."
fi
if [[ ! -f "${SITL_VENV}/bin/activate" ]]; then
  echo "Info: SITL_VENV not found at ${SITL_VENV}. Continuing without activating a SITL venv."
fi

# Check ros_gz_bridge by looking directly at the installed share directory.
# Avoids running ros2 CLI which can fail with BrokenPipeError when a venv's
# ros2 binary differs from the system one.
ROS_GZ_SHARE="/opt/ros/${ROS_DISTRO}/share/ros_gz_bridge"
if [[ ! -d "${ROS_GZ_SHARE}" ]]; then
  echo "ros_gz_bridge not found at ${ROS_GZ_SHARE}."
  echo "Install: sudo apt install ros-${ROS_DISTRO}-ros-gz"
  echo "Or run:  INSTALL_ROS_GZ=1 ./scripts/install_sim_stack.sh"
  exit 1
fi

# ---- cleanup helpers ----------------------------------------------------
PIDS=()
cleanup() {
  echo ""
  echo "Shutting down scenario..."
  for pid in "${PIDS[@]}"; do
    kill "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# ---- Step 1: Gazebo (no ROS env, display vars passed through) ----------
echo "==> [1/4] Starting Gazebo in clean env (no ROS): ${WORLD_FILE}"
env -i \
  HOME="${HOME}" \
  USER="${USER:-}" \
  SHELL=/bin/bash \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  TERM="${TERM:-dumb}" \
  DISPLAY="${_DISPLAY}" \
  WAYLAND_DISPLAY="${_WAYLAND_DISPLAY}" \
  XDG_RUNTIME_DIR="${_XDG_RUNTIME_DIR}" \
  DBUS_SESSION_BUS_ADDRESS="${_DBUS_SESSION_BUS_ADDRESS}" \
  GZ_SIM_SYSTEM_PLUGIN_PATH="${GZ_PLUGIN_PATH}" \
  GZ_SIM_RESOURCE_PATH="${GZ_RESOURCE_PATH}" \
  bash -c 'set -euo pipefail; exec gz sim -v4 -r "$1"' _ "${WORLD_FILE}" &
GZ_PID=$!
PIDS+=("${GZ_PID}")
echo "    Gazebo PID ${GZ_PID}. Waiting 12 s for world init..."
sleep 12

# ---- Step 2: SITL headless (venv-ardupilot only, no ROS, no GUI) -------
# Keep SITL text-only in this terminal. Avoid passing unsupported flags that vary
# across ArduPilot versions (for example --no-console may not exist).
# --no-rebuild skips waf compilation if binary already exists (saves ~2 min).
# MAVLink is still available on udp:127.0.0.1:14550.
# To get the MAVProxy GUI, open a second terminal and run: ./scripts/start_sitl.sh
echo "==> [2/4] Starting SITL headless in clean env (${VEHICLE} / gazebo-iris --model JSON)..."
env -i \
  HOME="${HOME}" \
  USER="${USER:-}" \
  SHELL=/bin/bash \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  TERM="${TERM:-dumb}" \
  SITL_DIR="${SITL_DIR}" \
  VEHICLE="${VEHICLE}" \
  SITL_VENV="${SITL_VENV}" \
  bash -c 'set -euo pipefail
    if [[ -f "${SITL_VENV}/bin/activate" ]]; then
      # shellcheck source=/dev/null
      source "${SITL_VENV}/bin/activate"
    fi
    cd "${SITL_DIR}"
    exec Tools/autotest/sim_vehicle.py \
      -v "${VEHICLE}" \
      -f gazebo-iris \
      --model JSON \
      --no-rebuild \
      --out=udp:127.0.0.1:14550
  ' &
SITL_PID=$!
PIDS+=("${SITL_PID}")
# Wait for SITL MAVLink port 5760 to open (poll up to 120 s) so steps 3 & 4
# don't start before SITL is ready.
echo "    SITL PID ${SITL_PID}. Waiting for port 5760 (up to 120 s)..."
for _i in $(seq 1 60); do
  if bash -c "echo > /dev/tcp/127.0.0.1/5760" 2>/dev/null; then
    echo "    SITL port 5760 ready after ~$((_i * 2)) s."
    break
  fi
  sleep 2
done

# Optional: start drone automation (no manual fly) as soon as SITL is ready.
if [[ "${AUTO_FLY}" == "1" ]]; then
  echo "==> [AUTO] Starting automatic drone fly around nodes (no manual)..."
  env -i \
    HOME="${HOME}" \
    USER="${USER:-}" \
    SHELL=/bin/bash \
    PATH="/usr/local/bin:/usr/bin:/bin" \
    TERM="${TERM:-dumb}" \
    WS_ROOT="${WS_ROOT}" \
    SITL_VENV="${SITL_VENV}" \
    AUTO_FLY_LOOPS="${AUTO_FLY_LOOPS}" \
    bash -c 'set -euo pipefail
      if [[ -f "${SITL_VENV}/bin/activate" ]]; then
        # shellcheck source=/dev/null
        source "${SITL_VENV}/bin/activate"
      fi
      exec python3 "${WS_ROOT}/scripts/auto_fly_cuas_field.py" \
        --mavlink_url udp:127.0.0.1:14550 \
        --altitude_m 15 \
        --radius_m 40 \
        --points_per_orbit 6 \
        --orbit_loops "${AUTO_FLY_LOOPS}"
    ' &
  AUTO_FLY_PID=$!
  PIDS+=("${AUTO_FLY_PID}")
fi

# ---- Step 3: ROS gz pose bridge (ROS env) ------------------------------
echo "==> [3/4] Starting ros_gz bridge in ROS env (world=${WORLD_NAME})..."
env -i \
  HOME="${HOME}" \
  USER="${USER:-}" \
  SHELL=/bin/bash \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  TERM="${TERM:-dumb}" \
  ROS_DISTRO="${ROS_DISTRO}" \
  WS_ROOT="${WS_ROOT}" \
  ROS_VENV="${ROS_VENV}" \
  WORLD_NAME="${WORLD_NAME}" \
  bash -c 'set -euo pipefail
    if [[ -f "${ROS_VENV}/bin/activate" ]]; then
      # shellcheck source=/dev/null
      source "${ROS_VENV}/bin/activate"
    fi
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck source=/dev/null
    source "${WS_ROOT}/install/setup.bash"
    set -u
    exec ros2 launch "${WS_ROOT}/launch/gz_pose_bridge.launch.py" \
      world_name:="${WORLD_NAME}" \
      model_name:=iris_with_gimbal
  ' &
BRIDGE_PID=$!
PIDS+=("${BRIDGE_PID}")
echo "    ros_gz bridge PID ${BRIDGE_PID}. Waiting 4 s..."
sleep 4

# ---- Step 4: full_sim ROS stack (ROS env) ------------------------------
echo "==> [4/4] Starting meshkah full_sim in ROS env (use_gazebo_truth:=true)..."
env -i \
  HOME="${HOME}" \
  USER="${USER:-}" \
  SHELL=/bin/bash \
  PATH="/usr/local/bin:/usr/bin:/bin" \
  TERM="${TERM:-dumb}" \
  ROS_DISTRO="${ROS_DISTRO}" \
  WS_ROOT="${WS_ROOT}" \
  ROS_VENV="${ROS_VENV}" \
  USE_CINEMATIC_CAMERA="${USE_CINEMATIC_CAMERA}" \
  bash -c 'set -euo pipefail
    if [[ -f "${ROS_VENV}/bin/activate" ]]; then
      # shellcheck source=/dev/null
      source "${ROS_VENV}/bin/activate"
    fi
    set +u
    # shellcheck source=/dev/null
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    # shellcheck source=/dev/null
    source "${WS_ROOT}/install/setup.bash"
    set -u
    exec ros2 launch "${WS_ROOT}/launch/full_sim.launch.py" \
      use_real_camera:=false \
      use_lora:=false \
      use_gazebo_truth:=true \
      use_cinematic_camera:="${USE_CINEMATIC_CAMERA}"
  ' &
ROS_PID=$!
PIDS+=("${ROS_PID}")
echo "    meshkah ROS PID ${ROS_PID}."

echo ""
echo "=================================================================="
echo "Full scenario running. Ctrl-C to stop all subprocesses."
echo "Env split:"
echo "  Gazebo: clean env, no ROS"
echo "  SITL:   clean env + SITL_VENV (${SITL_VENV})"
echo "  ROS:    ROS setup + overlay + ROS_VENV (${ROS_VENV})"
echo ""
echo "Verify data flow:"
echo "  ros2 topic hz /ardupilot/vehicle_pose  # Gazebo -> bridge"
echo "  ros2 topic hz /gazebo/drone_truth      # bridge -> fusion truth"
echo "  ros2 topic hz /global_track            # fusion output"
echo "Arm and fly in MAVProxy:"
echo "  mode guided ; arm throttle ; takeoff 5"
echo "=================================================================="

# Keep script alive until full_sim exits.
wait "${ROS_PID}" || true
