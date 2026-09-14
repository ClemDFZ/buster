#!/usr/bin/env python3
"""Pure helpers for sim/real nav + pantilt contracts (no rclpy)."""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

PAN_LIMIT_RAD = math.pi
TILT_LIMIT_RAD = math.pi / 4.0
RAD_OR_DEG_THRESHOLD = math.pi + 0.1


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    """Return (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def joint_value_to_rad(v: float) -> float:
    """JointTrajectory heuristic used by pantilt_bridge / ghost_publisher."""
    if abs(v) <= RAD_OR_DEG_THRESHOLD:
        return v
    return math.radians(v)


def integrate_dps(pos_rad: float, dps: float, dt: float, lo: float, hi: float) -> float:
    return clamp(pos_rad + math.radians(dps) * dt, lo, hi)


def extract_named_positions(
    names: Sequence[str],
    positions: Sequence[float],
    want: Sequence[str],
) -> dict:
    out = {}
    for name, pos in zip(names, positions):
        if name in want:
            out[name] = float(pos)
    return out


def pose_xy_yaw_error(
    nav_xyyaw: Tuple[float, float, float],
    gt_xyyaw: Tuple[float, float, float],
) -> Tuple[float, float, float, float]:
    """Return (dx, dy, dyaw, dist) of GT minus nav (nav-frame error vs GT)."""
    dx = gt_xyyaw[0] - nav_xyyaw[0]
    dy = gt_xyyaw[1] - nav_xyyaw[1]
    dyaw = wrap_pi(gt_xyyaw[2] - nav_xyyaw[2])
    return dx, dy, dyaw, math.hypot(dx, dy)


def imu_from_odometry(
    *,
    qw: float,
    qx: float,
    qy: float,
    qz: float,
    wz: float,
    vx: float,
    vy: float,
    prev_vx: Optional[float],
    prev_vy: Optional[float],
    dt: float,
    gravity: float = 9.80665,
) -> dict:
    """GT-derived IMU (yaw/gyro from odom; accel from body-twist finite diff).

    Linear accel is planar body-frame; z = +gravity (DMP-style, gravity included).
    """
    ax = ay = 0.0
    if prev_vx is not None and prev_vy is not None and dt > 1e-4:
        ax = (vx - prev_vx) / dt
        ay = (vy - prev_vy) / dt
    return {
        "orientation": (qx, qy, qz, qw),
        "angular_velocity": (0.0, 0.0, wz),
        "linear_acceleration": (ax, ay, gravity),
    }


def world_name_from_sdf(text: str, fallback: str) -> str:
    import re

    m = re.search(r"<world\b[^>]*\bname\s*=\s*['\"]([^'\"]+)", text)
    if m:
        return m.group(1)
    return fallback
