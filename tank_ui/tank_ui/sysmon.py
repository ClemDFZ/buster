"""Jetson / Linux host stats: CPU, GPU, RAM, thermal."""
from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple


def _read_text(path: Path) -> Optional[str]:
    try:
        return path.read_text().strip()
    except (OSError, TypeError, ValueError):
        return None


def _read_int(path: Path) -> Optional[int]:
    raw = _read_text(path)
    if raw is None:
        return None
    try:
        return int(raw.split()[0])
    except (ValueError, IndexError):
        return None


def _cpu_times() -> Optional[Tuple[int, int]]:
    try:
        line = Path("/proc/stat").read_text().splitlines()[0]
    except OSError:
        return None
    parts = line.split()
    if parts[0] != "cpu" or len(parts) < 5:
        return None
    vals = [int(x) for x in parts[1:]]
    idle = vals[3] + (vals[4] if len(vals) > 4 else 0)
    return sum(vals), idle


def _ram() -> Tuple[float, float, float]:
    total = avail = 0.0
    try:
        for line in Path("/proc/meminfo").read_text().splitlines():
            if line.startswith("MemTotal:"):
                total = float(line.split()[1]) * 1024.0
            elif line.startswith("MemAvailable:"):
                avail = float(line.split()[1]) * 1024.0
    except OSError:
        return 0.0, 0.0, 0.0
    used = max(0.0, total - avail)
    pct = (100.0 * used / total) if total > 0 else 0.0
    return pct, used / (1024**3), total / (1024**3)


def _gpu_pct() -> Optional[float]:
    cands = [
        Path("/sys/devices/gpu.0/load"),
        Path("/sys/devices/platform/gpu.0/load"),
        Path("/sys/devices/platform/bus@0/17000000.gpu/load"),
    ]
    for p in Path("/sys/devices/platform").glob("**/17000000.gpu/load"):
        cands.append(p)
    for p in cands:
        v = _read_int(p)
        if v is None:
            continue
        if v <= 100:
            return float(v)
        if v <= 1000:
            return float(v) / 10.0
        return min(100.0, float(v) / 10.0)
    return None


def _temps() -> Dict[str, float]:
    out: Dict[str, float] = {}
    root = Path("/sys/class/thermal")
    if not root.is_dir():
        return out
    for zone in sorted(root.glob("thermal_zone*")):
        kind = _read_text(zone / "type")
        raw = _read_int(zone / "temp")
        if not kind or raw is None:
            continue
        c = raw / 1000.0 if raw > 200 else float(raw)
        key = kind.replace("-thermal", "").replace("_thermal", "")
        out[key] = c
    return out


class SysSampler:
    def __init__(self) -> None:
        self._cpu_prev = _cpu_times()

    def snapshot(self) -> dict:
        prev = self._cpu_prev
        now = _cpu_times()
        self._cpu_prev = now
        cpu = None
        if prev and now:
            dt, di = now[0] - prev[0], now[1] - prev[1]
            if dt > 0:
                cpu = max(0.0, min(100.0, 100.0 * (1.0 - di / dt)))
        ram_pct, ram_used, ram_total = _ram()
        temps = _temps()
        gpu = _gpu_pct()
        t_cpu = temps.get("cpu")
        t_gpu = temps.get("gpu")
        t_tj = temps.get("tj")
        t_show = t_tj or t_cpu or t_gpu
        return {
            "cpu_pct": cpu,
            "gpu_pct": gpu,
            "ram_pct": ram_pct,
            "ram_used_gb": ram_used,
            "ram_total_gb": ram_total,
            "temp_c": t_show,
            "temp_cpu_c": t_cpu,
            "temp_gpu_c": t_gpu,
            "temp_tj_c": t_tj,
            "temps": temps,
        }
