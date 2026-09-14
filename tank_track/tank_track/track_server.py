#!/usr/bin/env python3
"""Action server: VLM explore → OSTrack → pantilt cmd_vel."""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from tank_track.explore import ExploreConfig, SweepState, sweep_vel
from tank_track.img_codec import bgr_to_compressed, bgr_to_imgmsg, imgmsg_to_bgr
from tank_track.pid import PixelPid
from tank_track_interfaces.action import TrackTarget
from tank_vlm_interfaces.srv import Detect

# torch / OSTrack imported lazily in _ensure_tracker (avoid CUDA claim during EXPLORE)

def _valid_bbox(bbox, w: int, h: int) -> bool:
    x1, y1, x2, y2 = bbox
    if x2 <= x1 or y2 <= y1:
        return False
    if x1 < 0 or y1 < 0 or x2 > w or y2 > h:
        return False
    bw, bh = x2 - x1, y2 - y1
    if bw < 4 or bh < 4:
        return False
    if bw > w * 0.98 or bh > h * 0.98:
        return False
    return True


def _bbox_center(bbox):
    x1, y1, x2, y2 = bbox
    return (x1 + x2) * 0.5, (y1 + y2) * 0.5


def _crop_xyxy(bgr, bbox, pad: float = 0.1):
    if bbox is None or bgr is None or bgr.size == 0:
        return None
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = (float(v) for v in bbox[:4])
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    x1 = int(max(0, x1 - pad * bw))
    y1 = int(max(0, y1 - pad * bh))
    x2 = int(min(w, x2 + pad * bw))
    y2 = int(min(h, y2 + pad * bh))
    if x2 <= x1 or y2 <= y1:
        return None
    return bgr[y1:y2, x1:x2].copy()


def _dxdy(bbox, w: int, h: int) -> Tuple[float, float]:
    if bbox is None:
        return 0.0, 0.0
    tcx, tcy = _bbox_center(bbox)
    return tcx - float(w) * 0.5, tcy - float(h) * 0.5


PID_KEYS = ("kp_x", "kp_y", "kd_x", "kd_y", "ki_x", "ki_y", "deadzone_px", "max_dps")


def _find_package_root() -> Path:
    try:
        share = Path(get_package_share_directory("tank_track"))
    except Exception:
        share = Path(__file__).resolve().parents[1]
    for cand in (
        share.parent.parent.parent / "src" / "tank_track",
        share.parent.parent.parent.parent / "src" / "tank_track",
        Path(__file__).resolve().parents[1],
        share,
    ):
        if (cand / "models").is_dir() or (cand / "package.xml").is_file():
            return cand
    return share


def _resolve_engine(explicit: str) -> Path:
    if explicit.strip():
        return Path(explicit).expanduser().resolve()
    env = os.environ.get("TANK_OSTRACK_ENGINE", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    return (_find_package_root() / "models" / "ostrack" / "ostrack_vitb256_ce.engine").resolve()


class TrackServer(Node):
    def __init__(self) -> None:
        super().__init__("track_server")
        self._cg = ReentrantCallbackGroup()

        self.declare_parameter("image_topic", "/csi_cam/image_raw")
        self.declare_parameter("ostrack_engine", "")
        self.declare_parameter("ostrack_backend", "auto")
        self.declare_parameter("track_conf_threshold", 0.1)
        self.declare_parameter("lost_frames", 6)
        self.declare_parameter("template_refresh_s", 1.5)
        self.declare_parameter("template_refresh_score", 0.55)
        self.declare_parameter("pan_min_deg", -90.0)
        self.declare_parameter("pan_max_deg", 90.0)
        self.declare_parameter("tilt_min_deg", -50.0)
        self.declare_parameter("tilt_max_deg", -15.0)
        self.declare_parameter("sweep_pan_dps", 27.0)
        self.declare_parameter("sweep_tilt_dps", 9.0)
        self.declare_parameter("sweep_hyst_deg", 2.0)
        self.declare_parameter("settle_s", 0.35)
        self.declare_parameter("move_timeout_s", 8.0)
        self.declare_parameter("detect_timeout_s", 180.0)
        self.declare_parameter("kp_x", -0.07)
        self.declare_parameter("kp_y", 0.02)
        self.declare_parameter("kd_x", 0.0005)
        self.declare_parameter("kd_y", 0.0005)
        self.declare_parameter("ki_x", 0.0)
        self.declare_parameter("ki_y", 0.0)
        self.declare_parameter("deadzone_px", 8.0)
        self.declare_parameter("max_dps", 45.0)
        self.declare_parameter("control_hz", 30.0)
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("annotate_compressed", True)
        self.declare_parameter("annotate_raw", False)
        self.declare_parameter("annotate_topic", "/vlm/track_annotated")
        self.declare_parameter("annotate_jpeg_quality", 70)
        self.declare_parameter("annotate_max_side", 640)
        self.declare_parameter("feedback_every_n", 2)
        self.declare_parameter("motion_enabled", True)
        self.declare_parameter("sweep_enabled", True)

        eng = _resolve_engine(str(self.get_parameter("ostrack_engine").value))
        self._engine_path = eng
        self._ostrack_backend = str(self.get_parameter("ostrack_backend").value)
        self._track_conf = float(self.get_parameter("track_conf_threshold").value)
        # Lazy: do NOT import torch/TRT until FOUND — frees GPU for VLM mtmd during EXPLORE.
        self._tracker = None
        self._device = None
        self.get_logger().info(
            f"OSTrack deferred until FOUND engine={eng} exists={eng.is_file()}"
        )

        self._explore = ExploreConfig(
            pan_min_deg=float(self.get_parameter("pan_min_deg").value),
            pan_max_deg=float(self.get_parameter("pan_max_deg").value),
            tilt_min_deg=float(self.get_parameter("tilt_min_deg").value),
            tilt_max_deg=float(self.get_parameter("tilt_max_deg").value),
            sweep_pan_dps=float(self.get_parameter("sweep_pan_dps").value),
            sweep_tilt_dps=float(self.get_parameter("sweep_tilt_dps").value),
            sweep_hyst_deg=float(self.get_parameter("sweep_hyst_deg").value),
            settle_s=float(self.get_parameter("settle_s").value),
            move_timeout_s=float(self.get_parameter("move_timeout_s").value),
        )
        # hardware: tilt 0° = endstop, -75° travel
        self._explore.tilt_max_deg = min(0.0, self._explore.tilt_max_deg)
        self._explore.tilt_min_deg = max(-75.0, min(self._explore.tilt_min_deg, self._explore.tilt_max_deg))
        self._pid = PixelPid(
            kp_x=float(self.get_parameter("kp_x").value),
            kp_y=float(self.get_parameter("kp_y").value),
            kd_x=float(self.get_parameter("kd_x").value),
            kd_y=float(self.get_parameter("kd_y").value),
            ki_x=float(self.get_parameter("ki_x").value),
            ki_y=float(self.get_parameter("ki_y").value),
            deadzone_px=float(self.get_parameter("deadzone_px").value),
            max_dps=float(self.get_parameter("max_dps").value),
        )
        self._lost_max = int(self.get_parameter("lost_frames").value)
        self._tmpl_refresh_s = float(self.get_parameter("template_refresh_s").value)
        self._tmpl_refresh_score = float(self.get_parameter("template_refresh_score").value)
        self._detect_timeout = float(self.get_parameter("detect_timeout_s").value)
        self._pan_name = str(self.get_parameter("pan_joint").value)
        self._tilt_name = str(self.get_parameter("tilt_joint").value)
        self._ctrl_period = 1.0 / max(1.0, float(self.get_parameter("control_hz").value))
        self._ann_jpeg_q = int(self.get_parameter("annotate_jpeg_quality").value)
        self._ann_max_side = int(self.get_parameter("annotate_max_side").value)
        self._fb_every = max(1, int(self.get_parameter("feedback_every_n").value))
        self._fb_i = 0
        self._motion_enabled = bool(self.get_parameter("motion_enabled").value)
        self._motion_lock = threading.Lock()
        self._sweep_enabled = bool(self.get_parameter("sweep_enabled").value)
        self._sweep_lock = threading.Lock()
        self._sweep_st = SweepState()
        self._explore_sweeping = False
        self._sweep_cmd = (0.0, 0.0)
        self._vlm_crop = None
        self.add_on_set_parameters_callback(self._on_set_params)

        self._frame_lock = threading.Lock()
        self._latest: Optional[Tuple[np.ndarray, object]] = None
        self._js_lock = threading.Lock()
        self._pan_deg = 0.0
        self._tilt_deg = 0.0
        self._js_ok = False

        self._goal_lock = threading.Lock()
        self._active = False
        self._cancel_req = False

        image_topic = str(self.get_parameter("image_topic").value)
        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data, callback_group=self._cg
        )
        self.create_subscription(
            JointState, "/pantilt/joint_states", self._on_js, 10, callback_group=self._cg
        )
        self.create_subscription(
            Bool, "/track/motion_enable", self._on_motion, 10, callback_group=self._cg
        )
        self.create_subscription(
            Bool, "/track/sweep_enable", self._on_sweep, 10, callback_group=self._cg
        )
        self.create_subscription(
            String, "/track/pid", self._on_pid, 10, callback_group=self._cg
        )
        self._cmd_pub = self.create_publisher(Twist, "/pantilt/cmd_vel", 10)
        self._traj_pub = self.create_publisher(JointTrajectory, "/pantilt/joint_trajectory", 10)
        self._ann_pub = None
        self._ann_raw_pub = None
        if bool(self.get_parameter("publish_annotated").value):
            base = str(self.get_parameter("annotate_topic").value)
            if bool(self.get_parameter("annotate_compressed").value):
                self._ann_pub = self.create_publisher(
                    CompressedImage, f"{base}/compressed", qos_profile_sensor_data
                )
            if bool(self.get_parameter("annotate_raw").value):
                self._ann_raw_pub = self.create_publisher(
                    Image, base, qos_profile_sensor_data
                )
        self._crop_pub = self.create_publisher(
            CompressedImage, "/vlm/track_crop/compressed", qos_profile_sensor_data
        )

        self._detect_cli = self.create_client(Detect, "/vlm/detect_label", callback_group=self._cg)
        self._home_cli = self.create_client(Trigger, "/pantilt/home", callback_group=self._cg)
        self.create_timer(0.05, self._on_sweep_timer, callback_group=self._cg)

        self._server = ActionServer(
            self,
            TrackTarget,
            "/vlm/track_target",
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._cg,
        )
        self.get_logger().info("Action /vlm/track_target ready")

    def _ensure_tracker(self):
        if self._tracker is None:
            import torch
            from tank_track.ostrack_tracker import OSTrackTracker

            self._device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
            self.get_logger().info(
                f"Loading OSTrack TRT ({self._engine_path.name}) on {self._device} …"
            )
            self._tracker = OSTrackTracker(
                device=self._device,
                ostrack_engine=str(self._engine_path),
                backend=self._ostrack_backend,
                track_conf_threshold=self._track_conf,
            )
            self.get_logger().info(f"OSTrack backend={self._tracker.backend}")
        return self._tracker

    def _release_tracker(self) -> None:
        """Free TRT/CUDA so VLM mmproj can init during EXPLORE."""
        if self._tracker is None:
            return
        try:
            self._tracker.reset()
        except Exception:
            pass
        self._tracker = None
        try:
            import torch

            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        except Exception:
            pass
        self.get_logger().info("OSTrack released (GPU free for VLM)")

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image decode: {exc}")
            return
        with self._frame_lock:
            self._latest = (bgr, msg.header)

    def _on_js(self, msg: JointState) -> None:
        names = list(msg.name)
        pos = list(msg.position)
        with self._js_lock:
            for i, n in enumerate(names):
                if i >= len(pos):
                    break
                deg = math.degrees(float(pos[i]))
                if n == self._pan_name:
                    self._pan_deg = deg
                    self._js_ok = True
                elif n == self._tilt_name:
                    self._tilt_deg = deg

    def _on_motion(self, msg: Bool) -> None:
        with self._motion_lock:
            was = self._motion_enabled
            self._motion_enabled = bool(msg.data)
        if was and not self._motion_enabled:
            self._stop_vel()
            self.get_logger().info("motion_enabled=false — pantilt cmds gated")
        elif (not was) and self._motion_enabled:
            self.get_logger().info("motion_enabled=true")

    def _on_sweep(self, msg: Bool) -> None:
        with self._sweep_lock:
            self._sweep_enabled = bool(msg.data)
            if not self._sweep_enabled:
                self._explore_sweeping = False
        if not msg.data:
            self._stop_vel()
        self.get_logger().info(f"sweep_enabled={bool(msg.data)}")

    def _set_sweeping(self, on: bool, *, stop: bool = True) -> None:
        with self._sweep_lock:
            want = bool(on) and self._sweep_enabled
            was = self._explore_sweeping
            self._explore_sweeping = want
            if not want:
                self._sweep_cmd = (0.0, 0.0)
        if stop and was and not want:
            self._stop_vel()

    def _on_sweep_timer(self) -> None:
        with self._sweep_lock:
            if not (self._explore_sweeping and self._sweep_enabled):
                return
            pan, tilt = self._angles()
            pan_dps, tilt_dps = sweep_vel(pan, tilt, self._explore, self._sweep_st)
            self._sweep_cmd = (pan_dps, tilt_dps)
            self._publish_vel(pan_dps, tilt_dps)

    def _apply_pid(self, d: dict) -> None:
        for k in PID_KEYS:
            if k not in d or d[k] is None:
                continue
            try:
                setattr(self._pid, k, float(d[k]))
            except (TypeError, ValueError):
                continue

    def _on_pid(self, msg: String) -> None:
        try:
            d = json.loads(msg.data or "{}")
        except json.JSONDecodeError:
            self.get_logger().warn("bad /track/pid json")
            return
        if not isinstance(d, dict):
            return
        self._apply_pid(d)
        self.get_logger().info(
            "pid "
            + " ".join(f"{k}={getattr(self._pid, k)}" for k in PID_KEYS)
        )

    def _on_set_params(self, params):
        for p in params:
            n = p.name
            v = p.value
            if n in PID_KEYS:
                try:
                    setattr(self._pid, n, float(v))
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f"bad {n}")
            elif n == "sweep_enabled":
                with self._sweep_lock:
                    self._sweep_enabled = bool(v)
                    if not self._sweep_enabled:
                        self._explore_sweeping = False
                if not v:
                    self._stop_vel()
            elif n in (
                "pan_min_deg",
                "pan_max_deg",
                "tilt_min_deg",
                "tilt_max_deg",
                "sweep_pan_dps",
                "sweep_tilt_dps",
                "sweep_hyst_deg",
                "settle_s",
                "move_timeout_s",
            ):
                try:
                    setattr(self._explore, n, float(v))
                except (TypeError, ValueError):
                    return SetParametersResult(successful=False, reason=f"bad {n}")
                if n == "tilt_max_deg":
                    self._explore.tilt_max_deg = min(0.0, self._explore.tilt_max_deg)
                elif n == "tilt_min_deg":
                    self._explore.tilt_min_deg = max(-75.0, min(self._explore.tilt_min_deg, self._explore.tilt_max_deg))
            elif n == "motion_enabled":
                with self._motion_lock:
                    self._motion_enabled = bool(v)
                if not self._motion_enabled:
                    self._stop_vel()
        return SetParametersResult(successful=True)

    def _motion_ok(self) -> bool:
        with self._motion_lock:
            return self._motion_enabled

    def _grab(self) -> Optional[Tuple[np.ndarray, object]]:
        with self._frame_lock:
            if self._latest is None:
                return None
            bgr, header = self._latest
            return bgr.copy(), header

    def _angles(self) -> Tuple[float, float]:
        with self._js_lock:
            return self._pan_deg, self._tilt_deg

    def _stop_vel(self) -> None:
        tw = Twist()
        self._cmd_pub.publish(tw)

    def _publish_vel(self, pan_dps: float, tilt_dps: float) -> None:
        if not self._motion_ok():
            return
        tw = Twist()
        tw.angular.z = float(pan_dps)
        tw.angular.y = float(tilt_dps)
        self._cmd_pub.publish(tw)

    def _goto_angles(self, pan_deg: float, tilt_deg: float) -> None:
        if not self._motion_ok():
            return
        traj = JointTrajectory()
        traj.joint_names = [self._pan_name, self._tilt_name]
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(pan_deg), math.radians(tilt_deg)]
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _wait_angles(
        self,
        *,
        pan_goal: Optional[float],
        tilt_goal: Optional[float],
        cancel_check,
    ) -> bool:
        if not self._motion_ok():
            return True  # skip wait when motion gated
        deadline = time.monotonic() + self._explore.move_timeout_s
        tol = self._explore.angle_tol_deg
        while time.monotonic() < deadline:
            if cancel_check():
                return False
            if not self._motion_ok():
                return True
            pan, tilt = self._angles()
            pan_ok = pan_goal is None or abs(pan - pan_goal) < tol
            tilt_ok = tilt_goal is None or abs(tilt - tilt_goal) < tol
            if pan_ok and tilt_ok:
                time.sleep(self._explore.settle_s)
                return True
            time.sleep(0.05)
        return True  # proceed anyway after timeout

    def _goal_cb(self, goal_request):
        with self._goal_lock:
            if self._active:
                self.get_logger().warn("reject goal — already tracking")
                return GoalResponse.REJECT
        if not (goal_request.target or "").strip():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        self._cancel_req = True
        return CancelResponse.ACCEPT

    def _fb(
        self,
        goal_handle,
        phase: str,
        step: str,
        bbox,
        score: float,
        pan_dps: float,
        tilt_dps: float,
        dx_px: float = 0.0,
        dy_px: float = 0.0,
    ):
        fb = TrackTarget.Feedback()
        fb.phase = phase
        fb.explore_step = step
        if bbox is not None and len(bbox) >= 4:
            fb.bbox_xyxy = [float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])]
        else:
            fb.bbox_xyxy = [0.0, 0.0, 0.0, 0.0]
        fb.score = float(score)
        fb.pan_dps = float(pan_dps)
        fb.tilt_dps = float(tilt_dps)
        fb.dx_px = float(dx_px)
        fb.dy_px = float(dy_px)
        goal_handle.publish_feedback(fb)

    def _call_detect(self, label: str, frame, header) -> Optional[dict]:
        if not self._detect_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/vlm/detect_label not available")
            return None
        req = Detect.Request()
        req.label = label
        req.image = bgr_to_imgmsg(frame, header)
        fut = self._detect_cli.call_async(req)
        t0 = time.monotonic()
        while not fut.done():
            if self._cancel_req:
                return None
            if time.monotonic() - t0 > self._detect_timeout:
                self.get_logger().error("detect timeout")
                return None
            time.sleep(0.05)
        resp = fut.result()
        if resp is None or not resp.success:
            msg = getattr(resp, "message", "no response") if resp else "no response"
            self.get_logger().warn(f"detect failed: {msg}")
            return {"boxes": [], "error": msg}
        try:
            return json.loads(resp.detections_json) if resp.detections_json else {"boxes": []}
        except json.JSONDecodeError:
            return {"boxes": []}

    def _pick_bbox(self, payload: dict, w: int, h: int):
        boxes = payload.get("boxes") or []
        best = None
        best_area = 0.0
        for b in boxes:
            xy = b.get("xyxy_px") or []
            if len(xy) < 4:
                continue
            bbox = (float(xy[0]), float(xy[1]), float(xy[2]), float(xy[3]))
            if not _valid_bbox(bbox, w, h):
                continue
            area = (bbox[2] - bbox[0]) * (bbox[3] - bbox[1])
            if area > best_area:
                best_area = area
                best = bbox
        return best

    def _maybe_home(self, do_home: bool, cancel_check) -> bool:
        if not do_home:
            return True
        if not self._home_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("/pantilt/home unavailable — skip")
            return True
        fut = self._home_cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done():
            if cancel_check():
                return False
            if time.monotonic() - t0 > 30.0:
                break
            time.sleep(0.05)
        # wait near 0,0
        return self._wait_angles(pan_goal=0.0, tilt_goal=0.0, cancel_check=cancel_check)

    def _pace(self, t0: float) -> None:
        """Sleep only leftover time to hit control_hz (no fixed 50ms tax)."""
        rem = self._ctrl_period - (time.monotonic() - t0)
        if rem > 0.0005:
            time.sleep(rem)

    def _draw_ann(self, bgr, header, bbox, label: str, phase: str, score: float):
        if self._ann_pub is None and self._ann_raw_pub is None:
            return
        import cv2

        oh, ow = bgr.shape[:2]
        dx, dy = _dxdy(bbox, ow, oh)

        out = bgr
        scale = 1.0
        m = self._ann_max_side
        if m > 0:
            h, w = bgr.shape[:2]
            side = max(h, w)
            if side > m:
                scale = float(m) / float(side)
                out = cv2.resize(
                    bgr,
                    (int(round(w * scale)), int(round(h * scale))),
                    interpolation=cv2.INTER_AREA,
                )
            else:
                out = bgr.copy()
        else:
            out = bgr.copy()

        ah, aw = out.shape[:2]
        cx, cy = aw // 2, ah // 2
        cv2.drawMarker(out, (cx, cy), (160, 160, 160), cv2.MARKER_CROSS, 18, 1)

        if bbox is not None:
            x1, y1, x2, y2 = (int(round(v * scale)) for v in bbox[:4])
            color = (0, 220, 255) if phase == "TRACKING" else (80, 255, 80)
            cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
            tcx = int(round((bbox[0] + bbox[2]) * 0.5 * scale))
            tcy = int(round((bbox[1] + bbox[3]) * 0.5 * scale))
            cv2.line(out, (cx, cy), (tcx, tcy), (0, 180, 255), 1)
            cv2.putText(
                out,
                f"{phase} {score:.2f} {label}",
                (x1, max(20, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.putText(
            out,
            f"dX={dx:+.0f} dY={dy:+.0f}",
            (8, ah - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 180, 255),
            1,
            cv2.LINE_AA,
        )
        crop = self._vlm_crop
        if crop is not None and crop.size:
            ch, cw = crop.shape[:2]
            max_side = max(72, min(140, aw // 4))
            s = float(max_side) / float(max(ch, cw, 1))
            tw, th = max(1, int(round(cw * s))), max(1, int(round(ch * s)))
            small = cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)
            x0, y0 = aw - tw - 8, 8
            if x0 >= 0 and y0 + th <= ah:
                out[y0 : y0 + th, x0 : x0 + tw] = small
                cv2.rectangle(out, (x0, y0), (x0 + tw, y0 + th), (80, 255, 80), 1)
                cv2.putText(
                    out,
                    "VLM",
                    (x0, max(12, y0 - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (80, 255, 80),
                    1,
                    cv2.LINE_AA,
                )
        try:
            if self._ann_pub is not None:
                self._ann_pub.publish(
                    bgr_to_compressed(out, header, quality=self._ann_jpeg_q)
                )
            if self._ann_raw_pub is not None:
                self._ann_raw_pub.publish(bgr_to_imgmsg(out, header))
            if crop is not None and crop.size:
                self._crop_pub.publish(
                    bgr_to_compressed(crop, header, quality=self._ann_jpeg_q)
                )
        except Exception:
            pass

    def _maybe_fb(self, goal_handle, phase, step, bbox, score, pan_dps, tilt_dps, dx_px=0.0, dy_px=0.0):
        self._fb_i += 1
        if self._fb_i % self._fb_every == 0:
            self._fb(goal_handle, phase, step, bbox, score, pan_dps, tilt_dps, dx_px, dy_px)
    def _execute(self, goal_handle):
        target = (goal_handle.request.target or "").strip()
        do_home = bool(goal_handle.request.do_home)
        with self._goal_lock:
            self._active = True
            self._cancel_req = False
        self._release_tracker()  # free GPU before VLM explore
        self._pid.reset()
        self._sweep_st = SweepState()
        self._set_sweeping(False)
        self._stop_vel()
        self._vlm_crop = None

        result = TrackTarget.Result()
        cancel = lambda: self._cancel_req or goal_handle.is_cancel_requested

        try:
            if not self._maybe_home(do_home, cancel):
                goal_handle.canceled()
                result.success = False
                result.message = "cancelled"
                return result

            phase = "EXPLORE"
            step = "READY"
            lost_count = 0
            last_tmpl_t = 0.0
            bbox = None
            score = 0.0
            pan_dps = tilt_dps = 0.0
            vlm_err_streak = 0
            max_vlm_errs = 5

            self.get_logger().info(f"TrackTarget start target={target!r} do_home={do_home}")

            while rclpy.ok():
                if cancel():
                    self._set_sweeping(False)
                    self._stop_vel()
                    self._release_tracker()
                    goal_handle.canceled()
                    result.success = False
                    result.message = "cancelled"
                    return result

                if phase == "EXPLORE":
                    self._set_sweeping(True)
                    pan_dps, tilt_dps = self._sweep_cmd
                    step = "WAIT_FRAME"
                    self._fb(goal_handle, phase, step, None, 0.0, pan_dps, tilt_dps)
                    grabbed = self._grab()
                    if grabbed is None:
                        time.sleep(0.1)
                        continue
                    frame, header = grabbed
                    h, w = frame.shape[:2]
                    shot_pan, shot_tilt = self._angles()

                    step = "QUERYING"
                    self._fb(goal_handle, phase, step, None, 0.0, pan_dps, tilt_dps)
                    payload = self._call_detect(target, frame, header)
                    if cancel():
                        continue
                    if payload is None:
                        goal_handle.abort()
                        result.success = False
                        result.message = "vlm unavailable"
                        return result

                    # VLM hard error (mtmd/CUDA/…) ≠ empty detection — do not pan-spam
                    if payload.get("error"):
                        vlm_err_streak += 1
                        step = f"VLM_ERR {vlm_err_streak}/{max_vlm_errs}"
                        self._fb(goal_handle, phase, step, None, 0.0, 0.0, 0.0)
                        self.get_logger().warn(
                            f"VLM error (not a miss): {payload['error']} "
                            f"— retry {vlm_err_streak}/{max_vlm_errs}"
                        )
                        if vlm_err_streak >= max_vlm_errs:
                            goal_handle.abort()
                            result.success = False
                            result.message = f"vlm error: {payload['error']}"
                            return result
                        time.sleep(2.0)
                        continue
                    vlm_err_streak = 0

                    hit = self._pick_bbox(payload, w, h)
                    if hit is not None:
                        bbox = hit
                        self._vlm_crop = _crop_xyxy(frame, bbox)
                        # Return immediately — do not stop_vel (kills motion) and do
                        # not wait on OSTrack load before the trajectory is sent.
                        self._set_sweeping(False, stop=False)
                        step = "RETURN"
                        self._fb(goal_handle, phase, step, None, 0.0, 0.0, 0.0)
                        self.get_logger().info(
                            f"detect hit → return pan={shot_pan:.1f} tilt={shot_tilt:.1f}"
                        )
                        self._goto_angles(shot_pan, shot_tilt)

                        load_err: list = []

                        def _load_ostrack() -> None:
                            try:
                                self._ensure_tracker()
                            except Exception as exc:
                                load_err.append(exc)

                        loader = threading.Thread(target=_load_ostrack, daemon=True)
                        loader.start()
                        if not self._wait_angles(
                            pan_goal=shot_pan, tilt_goal=shot_tilt, cancel_check=cancel
                        ):
                            loader.join(timeout=1.0)
                            continue
                        loader.join()
                        if load_err:
                            self.get_logger().error(f"OSTrack load failed: {load_err[0]}")
                            goal_handle.abort()
                            result.success = False
                            result.message = f"ostrack load: {load_err[0]}"
                            return result

                        step = "FOUND"
                        phase = "FOUND"
                        dx, dy = _dxdy(bbox, w, h)
                        self._fb(goal_handle, phase, step, bbox, 1.0, 0.0, 0.0, dx, dy)
                        self._draw_ann(frame, header, bbox, target, phase, 1.0)
                        tr = self._tracker
                        if tr is None:
                            goal_handle.abort()
                            result.success = False
                            result.message = "ostrack missing"
                            return result
                        if tr.init(frame, bbox):
                            phase = "TRACKING"
                            step = "OSTRACK"
                            lost_count = 0
                            last_tmpl_t = time.monotonic()
                            self._pid.reset()
                            self.get_logger().info(
                                f"TRACKING bbox={tuple(round(v,1) for v in bbox)}"
                            )
                        else:
                            self.get_logger().warn("OSTrack.init failed — hold FOUND")
                        continue

                    # miss → keep continuous sweep (or hold)
                    with self._sweep_lock:
                        sweep = self._sweep_enabled
                    if sweep:
                        pan, tilt = self._angles()
                        pan_dps, tilt_dps = self._sweep_cmd
                        step = "SWEEP"
                        self._fb(goal_handle, phase, step, None, 0.0, pan_dps, tilt_dps)
                        self.get_logger().info(
                            f"explore miss → sweep pan={pan:.1f} tilt={tilt:.1f} "
                            f"v=({pan_dps:.1f},{tilt_dps:.1f})"
                        )
                    else:
                        self._set_sweeping(False)
                        step = "HOLD"
                        self._fb(goal_handle, phase, step, None, 0.0, 0.0, 0.0)
                        self.get_logger().info("explore miss → hold (sweep off)")
                        time.sleep(0.4)
                    step = "READY"
                    continue

                if phase in ("FOUND", "TRACKING"):
                    grabbed = self._grab()
                    if grabbed is None:
                        time.sleep(0.02)
                        continue
                    frame, header = grabbed
                    h, w = frame.shape[:2]
                    now = time.monotonic()

                    if phase == "FOUND":
                        self._stop_vel()
                        dx, dy = _dxdy(bbox, w, h)
                        self._maybe_fb(goal_handle, phase, step, bbox, score, 0.0, 0.0, dx, dy)
                        self._draw_ann(frame, header, bbox, target, phase, score)
                        self._pace(now)
                        continue

                    tr = self._tracker
                    if tr is None:
                        phase = "EXPLORE"
                        step = "REACQUIRE"
                        continue

                    ok, new_bbox, score = tr.update(frame)
                    thr = self._track_conf
                    if ok and new_bbox is not None and score >= thr and _valid_bbox(new_bbox, w, h):
                        lost_count = 0
                        bbox = new_bbox
                        dx, dy = _dxdy(bbox, w, h)
                        pan_dps, tilt_dps = self._pid.step(dx, dy, now)
                        self._publish_vel(pan_dps, tilt_dps)
                        if (
                            self._tmpl_refresh_s > 0
                            and score >= self._tmpl_refresh_score
                            and (now - last_tmpl_t) >= self._tmpl_refresh_s
                            and tr.backend == "trt"
                        ):
                            if tr.init(frame, bbox):
                                last_tmpl_t = now
                        step = "OSTRACK"
                    else:
                        lost_count += 1
                        pan_dps = tilt_dps = 0.0
                        dx = dy = 0.0
                        self._stop_vel()
                        step = f"LOST {lost_count}/{self._lost_max}"
                        if lost_count >= self._lost_max:
                            self._release_tracker()  # free GPU before VLM re-detect
                            self._pid.reset()
                            phase = "EXPLORE"
                            step = "REACQUIRE"
                            bbox = None
                            score = 0.0
                            self._vlm_crop = None
                            self.get_logger().warn("OSTrack lost — re-explore (VLM)")
                            self._fb(goal_handle, "LOST", step, None, 0.0, 0.0, 0.0)
                            continue

                    self._maybe_fb(goal_handle, phase, step, bbox, score, pan_dps, tilt_dps, dx, dy)
                    self._draw_ann(frame, header, bbox, target, phase, score)
                    self._pace(now)
                    continue

                time.sleep(0.05)

            goal_handle.abort()
            result.success = False
            result.message = "shutdown"
            return result
        finally:
            self._set_sweeping(False)
            self._stop_vel()
            self._release_tracker()
            with self._goal_lock:
                self._active = False
                self._cancel_req = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_vel()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
