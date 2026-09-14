"""Pixel-error PID → pan/tilt deg/s (ported from pantilt_slave PanTiltAsyncSender)."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


def _clamp(v: float, lim: float) -> float:
    if lim <= 0.0:
        return v
    return max(-lim, min(lim, v))


@dataclass
class PixelPid:
    kp_x: float = -0.07
    kp_y: float = 0.02
    kd_x: float = 0.0005
    kd_y: float = 0.0005
    ki_x: float = 0.0
    ki_y: float = 0.0
    integral_limit: float = 300.0
    deadzone_px: float = 8.0
    invert_x: bool = False
    invert_y: bool = False
    motor_direction_sign: float = 1.0
    max_dps: float = 45.0

    def __post_init__(self) -> None:
        self._prev_t: float | None = None
        self._prev_ex = 0.0
        self._prev_ey = 0.0
        self._ix = 0.0
        self._iy = 0.0

    def reset(self) -> None:
        self._prev_t = None
        self._prev_ex = 0.0
        self._prev_ey = 0.0
        self._ix = 0.0
        self._iy = 0.0

    def _dz(self, e: float) -> float:
        if abs(e) < self.deadzone_px:
            return 0.0
        return e

    def step(self, dx_px: float, dy_px: float, now: float) -> Tuple[float, float]:
        """Return (pan_dps, tilt_dps)."""
        dz_x = self._dz(dx_px)
        dz_y = self._dz(dy_px)
        dt = 0.05 if self._prev_t is None else max(1e-3, now - self._prev_t)

        dex = (dz_x - self._prev_ex) / dt if self._prev_t is not None else 0.0
        dey = (dz_y - self._prev_ey) / dt if self._prev_t is not None else 0.0

        self._ix += dz_x * dt
        self._iy += dz_y * dt
        if self.integral_limit > 0.0:
            self._ix = max(-self.integral_limit, min(self.integral_limit, self._ix))
            self._iy = max(-self.integral_limit, min(self.integral_limit, self._iy))

        ux = (self.kp_x * dz_x) + (self.kd_x * dex) + (self.ki_x * self._ix)
        uy = (self.kp_y * dz_y) + (self.kd_y * dey) + (self.ki_y * self._iy)

        pan_delta = (-float(ux)) * self.motor_direction_sign
        tilt_delta = (-float(uy)) * self.motor_direction_sign
        if self.invert_x:
            pan_delta = -pan_delta
        if self.invert_y:
            tilt_delta = -tilt_delta

        self._prev_t = now
        self._prev_ex = dz_x
        self._prev_ey = dz_y

        pan_dps = _clamp(pan_delta / dt, self.max_dps)
        tilt_dps = _clamp(tilt_delta / dt, self.max_dps)
        return pan_dps, tilt_dps
