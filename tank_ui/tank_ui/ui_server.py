#!/usr/bin/env python3
"""Unified tank control panel — HTTP :8766 + ROS I/O."""
from __future__ import annotations

import json
import math
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tank_track_interfaces.action import TrackTarget
from tank_ui.launch_manager import LaunchManager, parse_stacks
from tank_ui.mjpeg import MjpegHub
from tank_ui.sysmon import SysSampler

DEFAULT_PORT = 8766
RATE_HZ = 20.0
PAN_LIM_DEG = 180.0
TILT_MIN_DEG = -75.0
TILT_MAX_DEG = 0.0
VEL_MAX_DPS = 60.0

PID_DEFAULTS = {
    "kp_x": -0.07,
    "kp_y": 0.02,
    "kd_x": 0.0005,
    "kd_y": 0.0005,
    "ki_x": 0.0,
    "ki_y": 0.0,
    "deadzone_px": 8.0,
    "max_dps": 45.0,
}


def _fmt_yaml_num(v: float) -> str:
    """Always emit a YAML float (ROS declare_parameter 0.0 ≠ INTEGER 0)."""
    fv = float(v)
    s = f"{fv:.8f}".rstrip("0").rstrip(".")
    if not s or s == "-":
        return "0.0"
    if "." not in s:
        s += ".0"
    return s


def _track_yaml_paths() -> list:
    cands = [
        Path.home() / "tank_ws" / "src" / "tank_track" / "config" / "track.yaml",
        Path("/home/orinclem/tank_ws/src/tank_track/config/track.yaml"),
    ]
    here = Path(__file__).resolve()
    for p in (here, *here.parents):
        hit = p / "src" / "tank_track" / "config" / "track.yaml"
        cands.append(hit)
        pkg = p / "tank_track" / "config" / "track.yaml"
        cands.append(pkg)
    try:
        cands.append(Path(get_package_share_directory("tank_track")) / "config" / "track.yaml")
    except Exception:
        pass
    seen = set()
    out = []
    for p in cands:
        try:
            if not p.is_file():
                continue
            key = str(p.resolve())
        except OSError:
            continue
        if key in seen:
            continue
        seen.add(key)
        out.append(p)
    return out


def _load_pid_from_yaml() -> dict:
    out = dict(PID_DEFAULTS)
    for path in _track_yaml_paths():
        try:
            import yaml

            data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        except Exception:
            continue
        params = (data.get("track_server") or {}).get("ros__parameters") or {}
        hit = False
        for k in PID_DEFAULTS:
            if k not in params:
                continue
            try:
                out[k] = float(params[k])
                hit = True
            except (TypeError, ValueError):
                continue
        if hit:
            break
    return out


def _patch_yaml_keys(path: Path, updates: dict) -> int:
    """Replace scalar keys in-place; keep comments / order. Returns #keys patched."""
    text = path.read_text(encoding="utf-8")
    n = 0
    for k, v in updates.items():
        pat = re.compile(rf"^(\s*{re.escape(k)}\s*:\s*)([^\s#\n]+)", re.M)
        new, c = pat.subn(
            lambda m, val=v: m.group(1) + _fmt_yaml_num(float(val)),
            text,
            count=1,
        )
        if c:
            text = new
            n += 1
    if n:
        path.write_text(text, encoding="utf-8")
    return n

STREAM_TOPICS = {
    "/csi_cam/image_raw": "raw",
    "/camera/camera/color/image_raw": "raw",
    "/vlm/annotated": "raw",
    "/vlm/track_annotated/compressed": "compressed",
    "/vlm/track_crop/compressed": "compressed",
}


def _resolve_share() -> Path:
    try:
        return Path(get_package_share_directory("tank_ui"))
    except Exception:
        return Path(__file__).resolve().parent.parent


def _resolve_web_root() -> Path:
    share = _resolve_share() / "web"
    if share.is_dir():
        return share
    return Path(__file__).resolve().parent.parent / "web"


def _load_stacks_from_yaml() -> dict:
    """Load stack defs from share/config/stacks.yaml (not a ROS params file)."""
    try:
        import yaml
    except ImportError:
        return {}
    for candidate in (
        _resolve_share() / "config" / "stacks.yaml",
        Path(__file__).resolve().parent.parent / "config" / "stacks.yaml",
        _resolve_share() / "config" / "ui.yaml",
        Path(__file__).resolve().parent.parent / "config" / "ui.yaml",
    ):
        if not candidate.is_file():
            continue
        try:
            data = yaml.safe_load(candidate.read_text()) or {}
        except Exception:
            continue
        if not isinstance(data, dict):
            continue
        if isinstance(data.get("stacks"), dict):
            return data["stacks"]
        block = data.get("ui_server", {})
        if isinstance(block, dict) and isinstance(block.get("stacks"), dict):
            return block["stacks"]
    return {}


class UiServer(Node):
    def __init__(self) -> None:
        super().__init__("ui_server")
        self.declare_parameter("http_port", DEFAULT_PORT)
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")
        self.declare_parameter("status_hz", 2.0)

        self._port = int(self.get_parameter("http_port").value)
        self._host = str(self.get_parameter("host").value)
        self._pan_name = str(self.get_parameter("pan_joint").value)
        self._tilt_name = str(self.get_parameter("tilt_joint").value)
        self._status_hz = float(self.get_parameter("status_hz").value)

        stacks = parse_stacks(_load_stacks_from_yaml())
        if not stacks:
            self.get_logger().error("no stacks loaded from ui.yaml")
        self._lm = LaunchManager(stacks)
        self._sys = SysSampler()

        # --- pantilt state ---
        self._pan_deg = 0.0
        self._tilt_deg = 0.0
        self._mode = "UNKNOWN"
        self._have_js = False
        self._cmd_pan_dps = 0.0
        self._cmd_tilt_dps = 0.0
        self._vel_active = False
        self._cmd_lock = threading.Lock()

        self._vel_pub = self.create_publisher(Twist, "/pantilt/cmd_vel", 10)
        self._traj_pub = self.create_publisher(JointTrajectory, "/pantilt/joint_trajectory", 10)
        self._mode_pub = self.create_publisher(String, "/pantilt/mode", 10)
        self.create_subscription(JointState, "/pantilt/joint_states", self._on_js, 10)
        self.create_subscription(String, "/pantilt/mode_state", self._on_mode, 10)
        self._home_cli = self.create_client(Trigger, "/pantilt/home")
        self._stop_cli = self.create_client(Trigger, "/pantilt/stop")
        self.create_timer(1.0 / RATE_HZ, self._vel_tick)

        # --- tracking ---
        self._motion_enabled = True
        self._sweep_enabled = True
        self._pid = _load_pid_from_yaml()
        self._motion_pub = self.create_publisher(Bool, "/track/motion_enable", 10)
        self._sweep_pub = self.create_publisher(Bool, "/track/sweep_enable", 10)
        self._pid_pub = self.create_publisher(String, "/track/pid", 10)
        self._track_client = ActionClient(self, TrackTarget, "/vlm/track_target")
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._fb: Dict[str, Any] = {
            "phase": "",
            "explore_step": "",
            "score": 0.0,
            "pan_dps": 0.0,
            "tilt_dps": 0.0,
            "dx_px": 0.0,
            "dy_px": 0.0,
            "bbox_xyxy": [0.0, 0.0, 0.0, 0.0],
            "active": False,
            "target": "",
            "status": "idle",
            "message": "",
        }
        self._vlm_status: Dict[str, Any] = {}
        self.create_subscription(String, "/vlm/status", self._on_vlm_status, 10)

        self._odom = None
        self._odom_t = 0.0
        self._odom_wheel = None
        self._odom_wheel_t = 0.0
        self._odom_vo = None
        self._odom_vo_t = 0.0
        self._odom_source = ""
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Odometry, "/odom_wheel", self._on_odom_wheel, 10)
        self.create_subscription(Odometry, "/odom_vo", self._on_odom_vo, 10)
        self.create_subscription(String, "/odom_source", self._on_odom_source, 10)
        self._odom_mode_pub = self.create_publisher(String, "/odom_mux/set_mode", 10)
        self._odom_zero_cli = self.create_client(Trigger, "/odom/zero")

        # --- MJPEG ---
        self._mjpeg = MjpegHub(jpeg_quality=70)
        for topic, kind in STREAM_TOPICS.items():
            if kind == "compressed":
                self.create_subscription(
                    CompressedImage,
                    topic,
                    lambda msg, t=topic: self._mjpeg.on_compressed(t, msg),
                    qos_profile_sensor_data,
                )
            else:
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, t=topic: self._mjpeg.on_image(t, msg),
                    qos_profile_sensor_data,
                )

        # --- node status cache ---
        self._live_nodes: set = set()
        self.create_timer(1.0 / max(self._status_hz, 0.5), self._poll_nodes)

        self._web_root = _resolve_web_root()
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._http_thread = threading.Thread(target=self._serve, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            f"tank_ui → http://{self._host}:{self._port}/  web={self._web_root}"
        )

    # ---------- pantilt ----------
    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name == self._pan_name:
                self._pan_deg = math.degrees(float(pos))
                self._have_js = True
            elif name == self._tilt_name:
                self._tilt_deg = math.degrees(float(pos))
                self._have_js = True

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data or "UNKNOWN"

    def _vel_tick(self) -> None:
        with self._cmd_lock:
            pan = self._cmd_pan_dps
            tilt = self._cmd_tilt_dps
            active = self._vel_active
            if not active and pan == 0.0 and tilt == 0.0:
                return
            if pan == 0.0 and tilt == 0.0:
                self._vel_active = False
        tw = Twist()
        tw.angular.z = float(pan)
        tw.angular.y = float(tilt)
        self._vel_pub.publish(tw)

    def _set_vel(self, pan_dps: float, tilt_dps: float) -> None:
        with self._cmd_lock:
            self._cmd_pan_dps = max(-VEL_MAX_DPS, min(VEL_MAX_DPS, float(pan_dps)))
            self._cmd_tilt_dps = max(-VEL_MAX_DPS, min(VEL_MAX_DPS, float(tilt_dps)))
            if self._cmd_pan_dps != 0.0 or self._cmd_tilt_dps != 0.0:
                self._vel_active = True

    def _set_mode(self, mode: str) -> dict:
        name = str(mode or "").strip().upper()
        if name not in ("IDLE", "HOMING", "RUNNING"):
            return {"ok": False, "message": f"unknown mode '{mode}'"}
        if name == "IDLE":
            self._set_vel(0.0, 0.0)
        msg = String()
        msg.data = name
        self._mode_pub.publish(msg)
        return {"ok": True, "message": f"MODE {name}"}

    def _send_ang(self, pan_deg: float, tilt_deg: float) -> None:
        pan = max(-PAN_LIM_DEG, min(PAN_LIM_DEG, float(pan_deg)))
        tilt = max(TILT_MIN_DEG, min(TILT_MAX_DEG, float(tilt_deg)))
        traj = JointTrajectory()
        traj.joint_names = [self._pan_name, self._tilt_name]
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(pan), math.radians(tilt)]
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _call_trigger(self, cli) -> dict:
        if not cli.wait_for_service(timeout_sec=0.5):
            return {"ok": False, "message": "service unavailable"}
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < 2.0:
            time.sleep(0.02)
        if not fut.done():
            return {"ok": False, "message": "service timeout"}
        try:
            resp = fut.result()
        except Exception as exc:
            return {"ok": False, "message": str(exc)}
        return {"ok": bool(resp.success), "message": resp.message}

    def _pantilt_state(self) -> dict:
        return {
            "pan": self._pan_deg,
            "tilt": self._tilt_deg,
            "mode": self._mode,
            "connected": self._have_js,
        }

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg
        self._odom_t = time.monotonic()

    def _on_odom_wheel(self, msg: Odometry) -> None:
        self._odom_wheel = msg
        self._odom_wheel_t = time.monotonic()

    def _on_odom_vo(self, msg: Odometry) -> None:
        self._odom_vo = msg
        self._odom_vo_t = time.monotonic()

    def _on_odom_source(self, msg: String) -> None:
        self._odom_source = msg.data or ""

    @staticmethod
    def _pack_odom(msg: Optional[Odometry], t0: float) -> dict:
        if msg is None:
            return {"ok": False}
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        p = msg.pose.pose.position
        tw = msg.twist.twist
        return {
            "ok": True,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(p.x),
            "y": float(p.y),
            "yaw_deg": math.degrees(yaw),
            "vx": float(tw.linear.x),
            "vy": float(tw.linear.y),
            "wz": float(tw.angular.z),
            "age_s": time.monotonic() - t0,
        }

    def _odom_state(self) -> dict:
        fused = self._pack_odom(self._odom, self._odom_t)
        out = {
            "ok": bool(fused.get("ok") or self._odom_wheel or self._odom_vo),
            "source": self._odom_source or ("fused" if fused.get("ok") else ""),
            "fused": fused,
            "wheel": self._pack_odom(self._odom_wheel, self._odom_wheel_t),
            "vo": self._pack_odom(self._odom_vo, self._odom_vo_t),
        }
        if fused.get("ok"):
            out.update({k: fused[k] for k in ("x", "y", "yaw_deg", "vx", "vy", "wz", "age_s", "frame_id", "child_frame_id")})
        return out

    def _set_odom_mode(self, mode: str) -> dict:
        name = str(mode or "").strip().lower()
        if name not in ("auto", "wheels", "vo", "fuse"):
            return {"ok": False, "message": f"unknown mode '{mode}'"}
        msg = String()
        msg.data = name
        self._odom_mode_pub.publish(msg)
        return {"ok": True, "message": f"odom mode {name}"}

    # ---------- stacks / nodes ----------
    def _poll_nodes(self) -> None:
        try:
            pairs = self.get_node_names_and_namespaces()
        except Exception:
            return
        names = set()
        for n, ns in pairs:
            bare = n.lstrip("/")
            names.add(bare)
            if ns and ns != "/":
                names.add(f"{ns.rstrip('/')}/{bare}".lstrip("/"))
        self._live_nodes = names

    def _stack_snapshot(self) -> dict:
        statuses = self._lm.statuses(self._live_nodes)
        by_cat: Dict[str, list] = {}
        for s in statuses:
            by_cat.setdefault(s["category"], []).append(s)
        return {"stacks": statuses, "by_category": by_cat, "nodes": sorted(self._live_nodes)}

    # ---------- tracking ----------
    def _on_vlm_status(self, msg: String) -> None:
        try:
            self._vlm_status = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self._vlm_status = {"raw": msg.data}

    def _set_motion(self, enabled: bool) -> dict:
        self._motion_enabled = bool(enabled)
        m = Bool()
        m.data = self._motion_enabled
        self._motion_pub.publish(m)
        if not self._motion_enabled:
            self._set_vel(0.0, 0.0)
        return {"ok": True, "motion_enabled": self._motion_enabled}

    def _set_sweep(self, enabled: bool) -> dict:
        self._sweep_enabled = bool(enabled)
        m = Bool()
        m.data = self._sweep_enabled
        self._sweep_pub.publish(m)
        return {"ok": True, "sweep_enabled": self._sweep_enabled}

    def _set_pid(self, payload: dict) -> dict:
        for k in PID_DEFAULTS:
            if k not in payload or payload[k] is None:
                continue
            try:
                self._pid[k] = float(payload[k])
            except (TypeError, ValueError):
                continue
        msg = String()
        msg.data = json.dumps(self._pid)
        self._pid_pub.publish(msg)
        return {"ok": True, "pid": dict(self._pid)}

    def _save_pid(self, payload: dict) -> dict:
        self._set_pid(payload)
        updates = {k: self._pid[k] for k in PID_DEFAULTS}
        written = []
        missing = []
        paths = _track_yaml_paths()
        if not paths:
            missing.append("no track.yaml found")
        for path in paths:
            try:
                n = _patch_yaml_keys(path, updates)
            except OSError as exc:
                missing.append(f"{path}: {exc}")
                continue
            if n == 0:
                missing.append(f"{path}: no pid keys")
            else:
                written.append(f"{path} ({n})")
        ok = bool(written)
        msg = ("saved " + ", ".join(written)) if ok else ("save failed: " + "; ".join(missing))
        if missing and ok:
            msg += " | " + "; ".join(missing)
        self.get_logger().info(f"pid Set: {msg}")
        return {
            "ok": ok,
            "pid": updates,
            "written": written,
            "message": msg,
        }

    def _track_feedback_cb(self, fb_msg) -> None:
        fb = fb_msg.feedback
        with self._goal_lock:
            self._fb.update(
                {
                    "phase": fb.phase,
                    "explore_step": fb.explore_step,
                    "score": float(fb.score),
                    "pan_dps": float(fb.pan_dps),
                    "tilt_dps": float(fb.tilt_dps),
                    "dx_px": float(getattr(fb, "dx_px", 0.0)),
                    "dy_px": float(getattr(fb, "dy_px", 0.0)),
                    "bbox_xyxy": [float(x) for x in fb.bbox_xyxy],
                    "active": True,
                    "status": "running",
                }
            )

    def _track_start(self, target: str, do_home: bool = False) -> dict:
        target = (target or "").strip()
        if not target:
            return {"ok": False, "message": "empty target"}
        if not self._track_client.wait_for_server(timeout_sec=1.0):
            return {"ok": False, "message": "track_server not available"}
        with self._goal_lock:
            if self._goal_handle is not None:
                return {"ok": False, "message": "already tracking — cancel first"}
        goal = TrackTarget.Goal()
        goal.target = target
        goal.do_home = bool(do_home)

        def _done(fut):
            try:
                gh = fut.result()
            except Exception as exc:
                with self._goal_lock:
                    self._fb["status"] = "error"
                    self._fb["message"] = str(exc)
                    self._fb["active"] = False
                    self._goal_handle = None
                return
            if not gh.accepted:
                with self._goal_lock:
                    self._fb["status"] = "rejected"
                    self._fb["message"] = "goal rejected"
                    self._fb["active"] = False
                    self._goal_handle = None
                return
            with self._goal_lock:
                self._goal_handle = gh
                self._fb["status"] = "running"
                self._fb["active"] = True
                self._fb["target"] = target
                self._fb["message"] = ""

            result_fut = gh.get_result_async()

            def _on_result(rf):
                try:
                    res = rf.result().result
                    msg = res.message
                    ok = bool(res.success)
                except Exception as exc:
                    msg = str(exc)
                    ok = False
                with self._goal_lock:
                    self._fb["active"] = False
                    self._fb["status"] = "done" if ok else "cancelled"
                    self._fb["message"] = msg
                    self._goal_handle = None

            result_fut.add_done_callback(_on_result)

        send_fut = self._track_client.send_goal_async(
            goal, feedback_callback=self._track_feedback_cb
        )
        send_fut.add_done_callback(_done)
        with self._goal_lock:
            self._fb["status"] = "sending"
            self._fb["target"] = target
            self._fb["active"] = True
            self._fb["message"] = ""
        # republish current motion flag so track_server syncs
        self._set_motion(self._motion_enabled)
        self._set_sweep(self._sweep_enabled)
        self._set_pid(self._pid)
        return {"ok": True, "message": f"goal sent: {target}"}

    def _track_cancel(self) -> dict:
        with self._goal_lock:
            gh = self._goal_handle
        if gh is None:
            # also try cancel-all via service
            return self._track_cancel_all()
        fut = gh.cancel_goal_async()
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < 3.0:
            time.sleep(0.02)
        with self._goal_lock:
            self._fb["status"] = "cancelling"
        self._set_vel(0.0, 0.0)
        return {"ok": True, "message": "cancel requested"}

    def _track_cancel_all(self) -> dict:
        from action_msgs.srv import CancelGoal

        cli = self.create_client(CancelGoal, "/vlm/track_target/_action/cancel_goal")
        if not cli.wait_for_service(timeout_sec=0.5):
            return {"ok": False, "message": "cancel service unavailable"}
        req = CancelGoal.Request()
        fut = cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < 2.0:
            time.sleep(0.02)
        self._set_vel(0.0, 0.0)
        with self._goal_lock:
            self._fb["status"] = "cancelling"
            self._fb["active"] = False
            self._goal_handle = None
        return {"ok": True, "message": "cancel-all sent"}

    def _track_state(self) -> dict:
        cam_ok = self._mjpeg.has_fresh("/csi_cam/image_raw", 3.0)
        vlm_up = "vlm_bridge" in self._live_nodes
        track_up = "track_server" in self._live_nodes
        with self._goal_lock:
            fb = dict(self._fb)
        tracking = bool(fb.get("active")) and fb.get("phase") in ("TRACKING", "FOUND")
        ann = "/vlm/track_annotated/compressed"
        csi = "/csi_cam/image_raw"
        stream = ann if tracking and self._mjpeg.has_fresh(ann, 1.5) else csi
        return {
            "feedback": fb,
            "motion_enabled": self._motion_enabled,
            "sweep_enabled": self._sweep_enabled,
            "pid": dict(self._pid),
            "cam_ok": cam_ok,
            "vlm_up": vlm_up,
            "track_up": track_up,
            "vlm_status": self._vlm_status,
            "ready": cam_ok and vlm_up and track_up,
            "stream": stream,
            "stream_topics": list(STREAM_TOPICS.keys()),
        }

    # ---------- HTTP ----------
    def _serve(self) -> None:
        node = self
        web_root = self._web_root

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):  # noqa: N802
                return

            def _json(self, code: int, payload: dict) -> None:
                body = json.dumps(
                    payload,
                    default=lambda o: float(o) if hasattr(o, "__float__") else str(o),
                ).encode("utf-8")
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> dict:
                n = int(self.headers.get("Content-Length", "0"))
                raw = self.rfile.read(n) if n else b"{}"
                try:
                    return json.loads(raw.decode("utf-8") or "{}")
                except json.JSONDecodeError:
                    return {}

            def _serve_file(self, rel: str, ctype: str) -> None:
                # Don't resolve() into the target: with colcon --symlink-install
                # share/web/* → src/tank_ui/web/* which sits outside web_root and
                # used to trip a false 404. Block ".." then follow the symlink.
                if ".." in Path(rel).parts:
                    self.send_error(404)
                    return
                path = web_root / rel
                if not path.is_file():
                    self.send_error(404)
                    return
                data = path.read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self):  # noqa: N802
                parsed = urlparse(self.path)
                path = parsed.path
                qs = parse_qs(parsed.query)

                if path in ("/", "/index.html"):
                    self._serve_file("index.html", "text/html; charset=utf-8")
                    return
                if path == "/app.js":
                    self._serve_file("app.js", "application/javascript; charset=utf-8")
                    return
                if path == "/style.css":
                    self._serve_file("style.css", "text/css; charset=utf-8")
                    return

                if path.startswith("/api/sys"):
                    self._json(200, node._sys.snapshot())
                    return
                if path.startswith("/api/stacks"):
                    self._json(200, node._stack_snapshot())
                    return
                if path.startswith("/api/logs"):
                    stack = (qs.get("stack") or [""])[0]
                    n = int((qs.get("n") or ["200"])[0])
                    self._json(200, node._lm.tail_log(stack, n))
                    return
                if path.startswith("/api/state") or path.startswith("/api/pantilt/state"):
                    self._json(200, node._pantilt_state())
                    return
                if path.startswith("/api/odom"):
                    self._json(200, node._odom_state())
                    return
                if path.startswith("/api/track/state"):
                    self._json(200, node._track_state())
                    return

                if path.startswith("/ui/stream"):
                    topic = (qs.get("topic") or ["/csi_cam/image_raw"])[0]
                    if topic not in STREAM_TOPICS:
                        self.send_error(404, f"unknown topic {topic}")
                        return
                    self.send_response(200)
                    self.send_header(
                        "Content-Type", "multipart/x-mixed-replace; boundary=frame"
                    )
                    self.send_header("Cache-Control", "no-cache")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    try:
                        for chunk in node._mjpeg.iter_multipart(topic, fps=12.0):
                            self.wfile.write(chunk)
                            self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        return
                    return

                self.send_error(404)

            def do_POST(self):  # noqa: N802
                path = urlparse(self.path).path
                d = self._read_json()

                if path.startswith("/api/odom/mode"):
                    self._json(200, node._set_odom_mode(d.get("mode", "")))
                    return
                if path.startswith("/api/odom/zero"):
                    self._json(200, node._call_trigger(node._odom_zero_cli))
                    return

                if path.startswith("/api/stack/start"):
                    self._json(200, node._lm.start(d.get("stack", "")))
                    return
                if path.startswith("/api/stack/stop"):
                    self._json(200, node._lm.stop(d.get("stack", "")))
                    return

                if path.startswith("/api/vel") or path.startswith("/api/pantilt/vel"):
                    node._set_vel(d.get("pan_dps", 0.0), d.get("tilt_dps", 0.0))
                    self._json(200, {"ok": True})
                    return
                if path.startswith("/api/ang") or path.startswith("/api/pantilt/ang"):
                    node._send_ang(d.get("pan_deg", 0.0), d.get("tilt_deg", 0.0))
                    self._json(200, {"ok": True})
                    return
                if path.startswith("/api/mode") or path.startswith("/api/pantilt/mode"):
                    self._json(200, node._set_mode(d.get("mode", "")))
                    return
                if path.startswith("/api/home") or path.startswith("/api/pantilt/home"):
                    self._json(200, node._call_trigger(node._home_cli))
                    return
                if path.startswith("/api/stop") or path.startswith("/api/pantilt/stop"):
                    node._set_vel(0.0, 0.0)
                    self._json(200, node._call_trigger(node._stop_cli))
                    return

                if path.startswith("/api/track/start"):
                    self._json(
                        200,
                        node._track_start(
                            d.get("target", ""), bool(d.get("do_home", False))
                        ),
                    )
                    return
                if path.startswith("/api/track/cancel"):
                    self._json(200, node._track_cancel())
                    return
                if path.startswith("/api/track/motion"):
                    self._json(200, node._set_motion(bool(d.get("enabled", True))))
                    return
                if path.startswith("/api/track/sweep"):
                    self._json(200, node._set_sweep(bool(d.get("enabled", True))))
                    return
                if path.startswith("/api/track/pid/save"):
                    self._json(200, node._save_pid(d))
                    return
                if path.startswith("/api/track/pid"):
                    self._json(200, node._set_pid(d))
                    return

                self.send_error(404)

        try:
            self._httpd = ThreadingHTTPServer((self._host, self._port), Handler)
            self._httpd.serve_forever()
        except OSError as exc:
            self.get_logger().error(f"HTTP bind {self._host}:{self._port} failed: {exc}")

    def destroy_node(self) -> bool:
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:
                pass
        for name in list(self._lm.list_defs()):
            try:
                self._lm.stop(name, timeout_s=3.0)
            except Exception:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UiServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
