"""Spawn / stop ros2 launch stacks with process-group signalling."""
from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional


LOG_DIR = Path("/tmp/tank_ui")


@dataclass
class StackDef:
    name: str
    label: str
    category: str
    cmd: List[str]
    nodes_required: List[str] = field(default_factory=list)
    nodes_optional: List[str] = field(default_factory=list)


@dataclass
class StackRuntime:
    proc: Optional[subprocess.Popen] = None
    log_path: Optional[Path] = None
    started_at: float = 0.0
    last_error: str = ""


def parse_stacks(raw: Any) -> Dict[str, StackDef]:
    """Parse ros parameter dict (or nested yaml) into StackDef map."""
    out: Dict[str, StackDef] = {}
    if not isinstance(raw, dict):
        return out
    for name, cfg in raw.items():
        if not isinstance(cfg, dict):
            continue
        cmd = cfg.get("cmd") or []
        if isinstance(cmd, str):
            cmd = cmd.split()
        out[str(name)] = StackDef(
            name=str(name),
            label=str(cfg.get("label") or name),
            category=str(cfg.get("category") or name),
            cmd=[str(c) for c in cmd],
            nodes_required=[str(n) for n in (cfg.get("nodes_required") or [])],
            nodes_optional=[str(n) for n in (cfg.get("nodes_optional") or [])],
        )
    return out


class LaunchManager:
    def __init__(self, stacks: Dict[str, StackDef]) -> None:
        self._stacks = stacks
        self._rt: Dict[str, StackRuntime] = {k: StackRuntime() for k in stacks}
        self._lock = threading.Lock()
        LOG_DIR.mkdir(parents=True, exist_ok=True)

    def list_defs(self) -> Dict[str, StackDef]:
        return self._stacks

    def start(self, name: str, env: Optional[dict] = None) -> dict:
        with self._lock:
            if name not in self._stacks:
                return {"ok": False, "message": f"unknown stack '{name}'"}
            rt = self._rt[name]
            if rt.proc is not None and rt.proc.poll() is None:
                return {"ok": True, "message": "already running"}
            sd = self._stacks[name]
            if not sd.cmd:
                return {"ok": False, "message": "empty cmd"}
            log_path = LOG_DIR / f"{name}.log"
            try:
                log_f = open(log_path, "ab", buffering=0)
            except OSError as exc:
                return {"ok": False, "message": f"log open failed: {exc}"}
            try:
                proc = subprocess.Popen(
                    sd.cmd,
                    stdout=log_f,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=env or os.environ.copy(),
                )
            except OSError as exc:
                log_f.close()
                rt.last_error = str(exc)
                return {"ok": False, "message": str(exc)}
            rt.proc = proc
            rt.log_path = log_path
            rt.started_at = time.monotonic()
            rt.last_error = ""
            return {"ok": True, "message": f"started pid={proc.pid}", "pid": proc.pid}

    def _node_pids(self, sd: StackDef) -> List[int]:
        """PIDs whose cmdline contains __node:=<required|optional> (orphans after launch dies)."""
        needles = [f"__node:={n.lstrip('/')}" for n in (sd.nodes_required + sd.nodes_optional)]
        if not needles:
            return []
        found: List[int] = []
        try:
            ents = list(Path("/proc").iterdir())
        except OSError:
            return []
        for entry in ents:
            if not entry.name.isdigit():
                continue
            pid = int(entry.name)
            try:
                cmd = (entry / "cmdline").read_bytes().replace(b"\x00", b" ").decode(
                    "utf-8", "replace"
                )
            except OSError:
                continue
            if any(n in cmd for n in needles):
                found.append(pid)
        return found

    def _signal_pids(self, pids: List[int], sig: int) -> None:
        for pid in pids:
            try:
                os.kill(pid, sig)
            except (ProcessLookupError, PermissionError, OSError):
                continue

    def _reap_nodes(self, sd: StackDef, timeout_s: float = 6.0) -> None:
        """SIGINT → SIGTERM → SIGKILL leftover node processes (venv python ignores launch death)."""
        pids = self._node_pids(sd)
        if not pids:
            return
        self._signal_pids(pids, signal.SIGINT)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            pids = self._node_pids(sd)
            if not pids:
                return
            time.sleep(0.15)
        self._signal_pids(pids, signal.SIGTERM)
        time.sleep(0.4)
        pids = self._node_pids(sd)
        if pids:
            self._signal_pids(pids, signal.SIGKILL)

    def stop(self, name: str, timeout_s: float = 8.0) -> dict:
        with self._lock:
            if name not in self._stacks:
                return {"ok": False, "message": f"unknown stack '{name}'"}
            sd = self._stacks[name]
            rt = self._rt[name]
            proc = rt.proc
            if proc is not None and proc.poll() is not None:
                rt.proc = None
                proc = None

        if proc is not None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            except OSError as exc:
                return {"ok": False, "message": str(exc)}

            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    break
                time.sleep(0.1)

            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                except (ProcessLookupError, OSError):
                    pass
                try:
                    proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except (ProcessLookupError, OSError):
                        pass

        self._reap_nodes(sd)

        with self._lock:
            rt.proc = None
        return {"ok": True, "message": "stopped"}

    def process_alive(self, name: str) -> bool:
        with self._lock:
            rt = self._rt.get(name)
            if rt is None or rt.proc is None:
                return False
            if rt.proc.poll() is not None:
                rt.proc = None
                return False
            return True

    def status(
        self,
        name: str,
        live_nodes: set,
    ) -> dict:
        """Return stack status dict with color: red|orange|green."""
        sd = self._stacks[name]
        alive = self.process_alive(name)
        required = [n.lstrip("/") for n in sd.nodes_required]
        optional = [n.lstrip("/") for n in sd.nodes_optional]
        present_req = [n for n in required if n in live_nodes]
        present_opt = [n for n in optional if n in live_nodes]
        missing_req = [n for n in required if n not in live_nodes]
        all_req_up = len(required) == 0 or len(missing_req) == 0

        if all_req_up and (alive or present_req):
            color = "green"
        elif alive or present_req or present_opt:
            color = "orange"
        else:
            color = "red"

        with self._lock:
            rt = self._rt[name]
            pid = rt.proc.pid if rt.proc is not None and rt.proc.poll() is None else None
            err = rt.last_error
            log_path = str(rt.log_path) if rt.log_path else ""

        return {
            "name": name,
            "label": sd.label,
            "category": sd.category,
            "color": color,
            "process_alive": alive,
            "pid": pid,
            "nodes_required": required,
            "nodes_optional": optional,
            "nodes_present": present_req + present_opt,
            "nodes_missing": missing_req,
            "log_path": log_path,
            "last_error": err,
            "cmd": sd.cmd,
        }

    def statuses(self, live_nodes: set) -> List[dict]:
        return [self.status(n, live_nodes) for n in self._stacks]

    def tail_log(self, name: str, n: int = 200) -> dict:
        with self._lock:
            rt = self._rt.get(name)
            path = rt.log_path if rt else None
        if path is None or not path.is_file():
            fallback = LOG_DIR / f"{name}.log"
            path = fallback if fallback.is_file() else None
        if path is None:
            return {"ok": False, "message": "no log", "lines": []}
        try:
            data = path.read_bytes()
        except OSError as exc:
            return {"ok": False, "message": str(exc), "lines": []}
        text = data.decode("utf-8", errors="replace")
        lines = text.splitlines()
        return {"ok": True, "lines": lines[-max(1, n) :], "path": str(path)}
