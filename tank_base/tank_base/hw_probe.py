"""Probe presence of Mega serial and RealSense USB."""
from __future__ import annotations

import glob
import os
import subprocess
from dataclasses import dataclass
from typing import List, Optional


@dataclass
class ProbeResult:
    name: str
    present: bool
    detail: str


def list_serial_ports() -> List[str]:
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def probe_mega(preferred: str = "/dev/ttyACM0") -> ProbeResult:
    ports = list_serial_ports()
    if preferred and preferred != "auto" and os.path.exists(preferred):
        return ProbeResult("mega", True, f"port {preferred} exists")
    if preferred == "auto" and ports:
        return ProbeResult("mega", True, f"auto → {ports[0]} (candidates: {', '.join(ports)})")
    if ports:
        return ProbeResult(
            "mega",
            False,
            f"{preferred} missing; other ports: {', '.join(ports)} (set serial_port:=...)",
        )
    return ProbeResult("mega", False, f"{preferred} missing; no ttyACM/ttyUSB found")


def resolve_mega_port(preferred: str = "/dev/ttyACM0") -> Optional[str]:
    if preferred and preferred != "auto" and os.path.exists(preferred):
        return preferred
    if preferred == "auto":
        ports = list_serial_ports()
        return ports[0] if ports else None
    return preferred if os.path.exists(preferred) else None


def _lsusb_text() -> str:
    try:
        out = subprocess.run(
            ["lsusb"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        return (out.stdout or "") + (out.stderr or "")
    except (FileNotFoundError, subprocess.SubprocessError):
        return ""


def probe_realsense() -> ProbeResult:
    text = _lsusb_text().lower()
    # Common Intel RealSense USB ids / names
    keys = ("realsense", "intel corp", "8086:0b", "8086:0a", "8086:0ad", "depth camera")
    if any(k in text for k in keys):
        hit = next((ln.strip() for ln in text.splitlines() if "realsense" in ln or "8086:0" in ln), "usb match")
        return ProbeResult("realsense", True, hit)
    # Fallback: pyrealsense2 if installed
    try:
        import pyrealsense2 as rs  # type: ignore

        ctx = rs.context()
        devices = list(ctx.query_devices())
        if devices:
            name = devices[0].get_info(rs.camera_info.name)
            return ProbeResult("realsense", True, f"pyrealsense2: {name}")
    except Exception:
        pass
    return ProbeResult("realsense", False, "no RealSense USB device (lsusb / pyrealsense2)")


def probe_all(serial_port: str = "/dev/ttyACM0") -> List[ProbeResult]:
    return [probe_mega(serial_port), probe_realsense()]


def format_probe_report(results: List[ProbeResult]) -> str:
    lines = ["[tank_base] hardware probe:"]
    for r in results:
        mark = "OK " if r.present else "MISS"
        lines.append(f"  [{mark}] {r.name}: {r.detail}")
    return "\n".join(lines)


def want_enabled(flag: str, present: bool) -> bool:
    """flag: auto|true|false → whether to start component."""
    v = flag.strip().lower()
    if v in ("false", "0", "no", "off"):
        return False
    if v in ("true", "1", "yes", "on"):
        return True
    # auto
    return present
