"""Continuous pan/tilt sweep for VLM explore."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass
class ExploreConfig:
    pan_min_deg: float = -90.0
    pan_max_deg: float = 90.0
    tilt_min_deg: float = -50.0
    tilt_max_deg: float = -15.0
    sweep_pan_dps: float = 27.0
    sweep_tilt_dps: float = 9.0
    sweep_hyst_deg: float = 2.0
    settle_s: float = 0.35
    move_timeout_s: float = 8.0
    angle_tol_deg: float = 2.5


@dataclass
class SweepState:
    pan_dir: float = 1.0
    tilt_dir: float = -1.0


def _flip(pos: float, lo: float, hi: float, direction: float, hyst: float) -> float:
    d = 1.0 if float(direction) >= 0.0 else -1.0
    if hi <= lo:
        return d
    if pos >= hi - hyst:
        return -1.0
    if pos <= lo + hyst:
        return 1.0
    return d


def sweep_vel(
    pan: float, tilt: float, cfg: ExploreConfig, st: SweepState
) -> Tuple[float, float]:
    """Triangle ping-pong: pan ±range, tilt sawtooth-like at a slower dps."""
    hyst = max(0.5, float(cfg.sweep_hyst_deg))
    st.pan_dir = _flip(pan, cfg.pan_min_deg, cfg.pan_max_deg, st.pan_dir, hyst)
    st.tilt_dir = _flip(tilt, cfg.tilt_min_deg, cfg.tilt_max_deg, st.tilt_dir, hyst)
    return st.pan_dir * abs(cfg.sweep_pan_dps), st.tilt_dir * abs(cfg.sweep_tilt_dps)
