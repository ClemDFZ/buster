"""Probe presence of ESP serial and CSI (MJPEG or Argus)."""
from __future__ import annotations

import glob
import os
import socket
import subprocess
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import List, Optional
from urllib.parse import urlparse


@dataclass
class ProbeResult:
    name: str
    present: bool
    detail: str


def list_serial_ports() -> List[str]:
    """Prefer ttyUSB* then ttyACM*."""
    return sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))


def probe_esp(preferred: str = "auto") -> ProbeResult:
    ports = list_serial_ports()
    if preferred and preferred != "auto" and os.path.exists(preferred):
        return ProbeResult("esp", True, f"port {preferred} exists")
    if preferred == "auto" and ports:
        return ProbeResult("esp", True, f"auto → {ports[0]} (candidates: {', '.join(ports)})")
    if ports:
        return ProbeResult(
            "esp",
            False,
            f"{preferred} missing; other ports: {', '.join(ports)} (set serial_port:=...)",
        )
    return ProbeResult("esp", False, f"{preferred} missing; no ttyUSB/ttyACM found")


def resolve_esp_port(preferred: str = "auto") -> Optional[str]:
    if preferred and preferred != "auto" and os.path.exists(preferred):
        return preferred
    if preferred == "auto":
        ports = list_serial_ports()
        return ports[0] if ports else None
    return preferred if os.path.exists(preferred) else None


def probe_mjpeg(url: str, timeout_s: float = 1.5) -> ProbeResult:
    try:
        req = urllib.request.Request(url, method="GET")
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            code = getattr(resp, "status", 200)
            _ = resp.read(64)
            return ProbeResult("csi_mjpeg", True, f"HTTP {code} {url}")
    except Exception as exc:
        try:
            u = urlparse(url)
            host = u.hostname or "127.0.0.1"
            port = u.port or 80
            with socket.create_connection((host, port), timeout=timeout_s):
                return ProbeResult("csi_mjpeg", False, f"port {host}:{port} open but GET failed: {exc}")
        except Exception:
            return ProbeResult("csi_mjpeg", False, f"unreachable {url} ({exc})")


def probe_argus() -> ProbeResult:
    videos = sorted(glob.glob("/dev/video*"))
    has_gst = False
    try:
        out = subprocess.run(
            ["gst-inspect-1.0", "nvarguscamerasrc"],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
        has_gst = out.returncode == 0
    except (FileNotFoundError, subprocess.SubprocessError):
        has_gst = False
    if videos and has_gst:
        return ProbeResult("csi_argus", True, f"nvarguscamerasrc + {', '.join(videos[:4])}")
    if has_gst:
        return ProbeResult("csi_argus", False, "nvarguscamerasrc present but no /dev/video*")
    if videos:
        return ProbeResult("csi_argus", False, "/dev/video* present but nvarguscamerasrc missing")
    return ProbeResult("csi_argus", False, "no Argus/CSI video device")


def probe_all(
    serial_port: str = "auto",
    mjpeg_url: str = "http://127.0.0.1:5003/video_feed/color",
    csi_owner: str = "pantilt",
) -> List[ProbeResult]:
    results = [probe_esp(serial_port)]
    if csi_owner == "ros":
        results.append(probe_argus())
    else:
        results.append(probe_mjpeg(mjpeg_url))
    return results


def format_probe_report(results: List[ProbeResult]) -> str:
    lines = ["[tank_pantilt] hardware probe:"]
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
    return present
