#!/usr/bin/env python3
"""
Auto-fly ArduCopter around the 3 CUAS nodes in `cuas_field.world`.

This is a "no manual fly" automation for the integrated 3D scenario:
  - connects to MAVLink endpoint (default udp:127.0.0.1:14550)
  - arms + switches to GUIDED
  - climbs/holds at altitude target
  - sends SET_POSITION_TARGET_LOCAL_NED waypoints that orbit nodes A/B/C
  - switches to LAND at the end

Notes:
  - This uses LOCAL_NED (x=north, y=east, z=down) setpoints in GUIDED.
  - The node marker ENU locations come from `src/sim_world/worlds/cuas_field.world`.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Iterable, Tuple

from pymavlink import mavutil


def _set_mode(master: mavutil.mavlink_connection, mode_name: str) -> None:
    mode_id = master.mode_mapping().get(mode_name)
    if mode_id is None:
        raise RuntimeError(f"Unknown mode '{mode_name}'. Available: {list(master.mode_mapping().keys())}")

    base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    master.mav.set_mode_send(
        master.target_system,
        base_mode,
        mode_id,
    )


def _try_disable_arming_check(
    master: mavutil.mavlink_connection,
    timeout_s: float = 5.0,
) -> None:
    """
    Best-effort: disable common pre-arm checks in SITL so automation can arm reliably.

    Sends PARAM_SET for ARMING_CHECK=0. If the param doesn't exist, we just continue.
    """
    try:
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            b"ARMING_CHECK",
            0.0,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
    except Exception:
        # Some pymavlink builds may not accept bytes names; try str fallback.
        try:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                "ARMING_CHECK",
                0.0,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        except Exception:
            return

    # Give ArduPilot a moment; we don't hard-require PARAM_VALUE confirmation.
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        pv = master.recv_match(type="PARAM_VALUE", blocking=False)
        if pv is not None:
            param_id = getattr(pv, "param_id", b"")
            if param_id in (b"ARMING_CHECK", "ARMING_CHECK"):
                return
        time.sleep(0.1)


def _arm(master: mavutil.mavlink_connection, armed: bool = True, timeout_s: float = 15.0) -> None:
    cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        cmd,
        0,
        1.0 if armed else 0.0,  # param1
        0,
        0,
        0,
        0,
        0,
        0,
    )

    # Wait for COMMAND_ACK first so we can surface accept/reject quickly.
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ack = master.recv_match(type="COMMAND_ACK", blocking=False)
        if ack is None:
            time.sleep(0.05)
            continue
        if getattr(ack, "command", None) == cmd:
            # ack.result is MAV_RESULT enum.
            is_accepted = int(getattr(ack, "result", -1)) == int(mavutil.mavlink.MAV_RESULT_ACCEPTED)
            if not armed:
                if is_accepted:
                    return
            else:
                if is_accepted:
                    break  # Proceed to heartbeat confirmation below.
                raise TimeoutError(f"Arming command rejected: COMMAND_ACK result={getattr(ack,'result',None)}")

    # Wait for heartbeat to reflect armed state (safety bit).
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is None:
            continue
        is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if is_armed == armed:
            return
        time.sleep(0.2)

    raise TimeoutError(f"Timeout waiting for armed={armed}")


def _wait_for_local_position(master: mavutil.mavlink_connection, timeout_s: float = 10.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is not None:
            return msg
        time.sleep(0.1)
    raise TimeoutError("No LOCAL_POSITION_NED received")


def _guided_takeoff(master: mavutil.mavlink_connection, alt_m: float, timeout_s: float = 45.0) -> None:
    """
    Send GUIDED takeoff command and wait until altitude is reached.

    Uses MAV_CMD_NAV_TAKEOFF with relative-alt style target (works for ArduCopter SITL).
    We monitor LOCAL_POSITION_NED.z (down-positive), so target is z ~= -alt_m.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0.0,  # min pitch (unused)
        0.0,  # empty
        0.0,  # empty
        float("nan"),  # yaw unchanged
        0.0,  # lat (unused here)
        0.0,  # lon (unused here)
        float(alt_m),  # altitude target (m)
    )

    # Wait for climb to near target altitude.
    target_down = -float(alt_m)
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
        if msg is not None:
            z_down = float(msg.z)
            if z_down <= target_down + 1.0:
                return
        time.sleep(0.2)

    raise TimeoutError(f"GUIDED takeoff timeout (target alt={alt_m}m)")


def _send_local_position_setpoint(
    master: mavutil.mavlink_connection,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_rad: float = 0.0,
) -> None:
    """
    Send position-only setpoint in LOCAL_NED (x=north, y=east, z=down).

    type_mask ignores: velocity, accel, yaw, yaw_rate. Only position is used.
    """
    type_mask = 0
    # Ignore velocities (vx,vy,vz) bits 3..5
    type_mask |= 1 << 3
    type_mask |= 1 << 4
    type_mask |= 1 << 5
    # Ignore accelerations bits 6..8
    type_mask |= 1 << 6
    type_mask |= 1 << 7
    type_mask |= 1 << 8
    # Ignore yaw bits 9 and yaw_rate bit 10
    type_mask |= 1 << 9
    type_mask |= 1 << 10

    # IMPORTANT: use *_send (not *_encode) so the setpoint is actually transmitted.
    # time_boot_ms can be zero for offboard control scripts.
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north_m,
        east_m,
        down_m,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        yaw_rad,
        0.0,
    )


def _send_local_velocity_setpoint(
    master: mavutil.mavlink_connection,
    vx_mps: float,
    vy_mps: float,
    vz_mps: float = 0.0,
) -> None:
    """
    Send velocity-only setpoint in LOCAL_NED frame.

    LOCAL_NED: x=north, y=east, z=down.
    """
    type_mask = 0
    # Ignore positions x,y,z bits 0..2
    type_mask |= 1 << 0
    type_mask |= 1 << 1
    type_mask |= 1 << 2
    # Keep velocities (vx,vy,vz) active => do NOT set bits 3..5
    # Ignore accelerations bits 6..8
    type_mask |= 1 << 6
    type_mask |= 1 << 7
    type_mask |= 1 << 8
    # Ignore yaw and yaw_rate
    type_mask |= 1 << 9
    type_mask |= 1 << 10

    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        float(vx_mps),
        float(vy_mps),
        float(vz_mps),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def _orbit_points(center_n: float, center_e: float, radius_m: float, points: int) -> Iterable[Tuple[float, float]]:
    for i in range(points):
        theta = 2.0 * math.pi * (i / points)
        # ENU: east = E, north = N
        e = center_e + radius_m * math.cos(theta)
        n = center_n + radius_m * math.sin(theta)
        yield n, e


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mavlink_url", default="udp:127.0.0.1:14550")
    ap.add_argument("--altitude_m", type=float, default=15.0)
    ap.add_argument("--radius_m", type=float, default=40.0)
    ap.add_argument("--points_per_orbit", type=int, default=6)
    ap.add_argument("--xy_threshold_m", type=float, default=8.0)
    ap.add_argument("--goto_period_s", type=float, default=0.5)
    ap.add_argument("--goto_timeout_s", type=float, default=40.0)
    ap.add_argument("--land_timeout_s", type=float, default=60.0)
    ap.add_argument(
        "--orbit_loops",
        type=int,
        default=1,
        help=(
            "How many times to repeat A->B->C orbit sequence. "
            "Set 0 for infinite looping."
        ),
    )
    ap.add_argument(
        "--disable_arming_check",
        type=int,
        default=1,
        help="Best-effort: set ARMING_CHECK=0 before arming (useful for SITL automation).",
    )
    args = ap.parse_args()

    master = mavutil.mavlink_connection(args.mavlink_url)
    print(f"[auto_fly] Connecting: {args.mavlink_url}")
    master.wait_heartbeat(timeout=20)
    print("[auto_fly] Heartbeat received.")

    print("[auto_fly] Setting mode GUIDED...")
    _set_mode(master, "GUIDED")
    time.sleep(1.0)

    if int(args.disable_arming_check) == 1:
        print("[auto_fly] Disabling ARMING_CHECK (best-effort) before arming...")
        _try_disable_arming_check(master, timeout_s=5.0)

    print("[auto_fly] Arming...")
    _arm(master, armed=True, timeout_s=20.0)
    print("[auto_fly] Armed.")

    # Node marker ENU centers from `cuas_field.world`:
    # A: (E=0,   N=0,   U=5)
    # B: (E=300, N=0,   U=5)
    # C: (E=150, N=260, U=5)
    #
    # We fly at a constant altitude above the ground: up=altitude_m -> down=-altitude_m.
    down_target = -float(args.altitude_m)

    # GUIDED takeoff first; position setpoints are more reliable after takeoff.
    print(f"[auto_fly] GUIDED takeoff to {args.altitude_m:.1f}m...")
    _guided_takeoff(master, alt_m=float(args.altitude_m), timeout_s=45.0)
    _wait_for_local_position(master, timeout_s=10.0)
    time.sleep(3.0)

    # Orbit A, then B, then C.
    sequence = [
        ("A", 0.0, 0.0),
        ("B", 0.0, 300.0),
        ("C", 260.0, 150.0),
    ]

    loop_idx = 0
    while args.orbit_loops == 0 or loop_idx < args.orbit_loops:
        loop_idx += 1
        if args.orbit_loops == 0:
            print(f"[auto_fly] Starting orbit loop #{loop_idx} (infinite mode)...")
        else:
            print(f"[auto_fly] Starting orbit loop {loop_idx}/{args.orbit_loops}...")

        for label, center_n, center_e in sequence:
            print(f"[auto_fly] Orbit around node {label} (center_n={center_n}, center_e={center_e})...")
            for (n_wp, e_wp) in _orbit_points(
                center_n=center_n,
                center_e=center_e,
                radius_m=args.radius_m,
                points=args.points_per_orbit,
            ):
                t0 = time.time()
                while time.time() - t0 < args.goto_timeout_s:
                    msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=False)
                    if msg is not None:
                        dx = float(msg.x) - float(n_wp)
                        dy = float(msg.y) - float(e_wp)
                        dist_xy = math.sqrt(dx * dx + dy * dy)

                        # Velocity guidance toward waypoint (more robust than pure
                        # position setpoints in this Gazebo+SITL setup).
                        to_n = float(n_wp) - float(msg.x)
                        to_e = float(e_wp) - float(msg.y)
                        norm = max(math.sqrt(to_n * to_n + to_e * to_e), 1e-6)
                        # Cruise speed up to 4 m/s, taper near waypoint.
                        speed = max(0.8, min(4.0, 0.15 * norm))
                        vx = speed * (to_n / norm)
                        vy = speed * (to_e / norm)
                        _send_local_velocity_setpoint(master, vx_mps=vx, vy_mps=vy, vz_mps=0.0)
                    else:
                        dist_xy = float("inf")
                    if dist_xy <= args.xy_threshold_m:
                        # Stop horizontal motion briefly when reaching waypoint.
                        _send_local_velocity_setpoint(master, vx_mps=0.0, vy_mps=0.0, vz_mps=0.0)
                        break
                    time.sleep(args.goto_period_s)

                if time.time() - t0 >= args.goto_timeout_s:
                    print(f"[auto_fly] Warning: timeout reaching waypoint near node {label}. Continuing...")

    print("[auto_fly] Switching to LAND...")
    _set_mode(master, "LAND")

    t0 = time.time()
    while time.time() - t0 < args.land_timeout_s:
        hb = master.recv_match(type="HEARTBEAT", blocking=False)
        if hb is not None:
            is_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if not is_armed:
                print("[auto_fly] Disarmed (landing complete).")
                return
        time.sleep(0.5)

    print("[auto_fly] LAND timeout reached; script exiting (vehicle may still be landing).")


if __name__ == "__main__":
    main()

