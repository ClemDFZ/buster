#!/usr/bin/env python3
"""Inline VLM node: CSI Image → local Qwen-VL (llama-cpp) → detections.

Standalone: models + venv live under the tank_vlm package tree.
"""
from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any, Optional, Tuple

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from tank_vlm_interfaces.srv import Detect
from tank_vlm.vlm_core.inference import (
    DEFAULT_CHAT_FORMAT,
    DEFAULT_CLIP_FILE,
    DEFAULT_MODEL_FILE,
    DEFAULT_MODEL_ID,
    VLMEngine,
)


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
    """Decode sensor_msgs/Image without cv_bridge (venv NumPy ≠ ROS ABI)."""
    enc = (msg.encoding or "").lower()
    if enc in ("bgr8", "rgb8", "8uc3"):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.step == msg.width * 3:
            img = arr.reshape(msg.height, msg.width, 3)
        else:
            row = msg.width * 3
            img = np.empty((msg.height, msg.width, 3), dtype=np.uint8)
            for y in range(msg.height):
                img[y] = arr[y * msg.step : y * msg.step + row].reshape(msg.width, 3)
        if enc == "rgb8":
            return img[:, :, ::-1].copy()
        return img.copy() if not img.flags["OWNDATA"] else img
    if enc in ("mono8", "8uc1"):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[
            :, : msg.width
        ]
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported encoding {msg.encoding!r}")


def bgr_to_imgmsg(bgr: np.ndarray, header) -> Image:
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
    return msg


def _find_package_root() -> Path:
    """Prefer source-tree package root (models/ + .venv/), else share/."""
    try:
        share = Path(get_package_share_directory("tank_vlm"))
    except Exception:
        share = Path(__file__).resolve().parents[1]
    # colcon: install/tank_vlm/share/tank_vlm → workspace/src/tank_vlm
    for cand in (
        share.parent.parent.parent / "src" / "tank_vlm",
        share.parent.parent.parent.parent / "src" / "tank_vlm",
        Path(__file__).resolve().parents[1],  # .../tank_vlm/tank_vlm/vlm_bridge.py → tank_vlm/
        share,
    ):
        if (cand / "models").is_dir() or (cand / "package.xml").is_file():
            return cand
    return share


def _resolve_models_dir(explicit: str) -> Path:
    if explicit.strip():
        return Path(explicit).expanduser().resolve()
    env = os.environ.get("TANK_VLM_MODELS", "").strip()
    if env:
        return Path(env).expanduser().resolve()
    root = _find_package_root()
    return (root / "models").resolve()


def _resize_max_side(bgr: np.ndarray, max_side: int) -> Tuple[np.ndarray, float]:
    if max_side <= 0:
        return bgr, 1.0
    h, w = bgr.shape[:2]
    m = max(h, w)
    if m <= max_side:
        return bgr, 1.0
    scale = float(max_side) / float(m)
    out = cv2.resize(
        bgr,
        (int(round(w * scale)), int(round(h * scale))),
        interpolation=cv2.INTER_AREA,
    )
    return out, scale


def _draw_boxes(bgr: np.ndarray, boxes: list, label: str) -> np.ndarray:
    out = bgr.copy()
    for b in boxes:
        xy = b.get("xyxy_px") or []
        if len(xy) < 4:
            continue
        x1, y1, x2, y2 = [int(round(float(v))) for v in xy[:4]]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 220, 80), 2)
        tag = str(b.get("label") or label)
        cv2.putText(
            out,
            tag,
            (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 220, 80),
            1,
            cv2.LINE_AA,
        )
    return out


class VlmBridge(Node):
    def __init__(self) -> None:
        super().__init__("vlm_bridge")
        self.declare_parameter("models_dir", "")
        self.declare_parameter("model_file", DEFAULT_MODEL_FILE)
        self.declare_parameter("clip_file", DEFAULT_CLIP_FILE)
        self.declare_parameter("model_id", DEFAULT_MODEL_ID)
        self.declare_parameter("chat_format", DEFAULT_CHAT_FORMAT)
        self.declare_parameter("n_ctx", 4096)
        self.declare_parameter("n_gpu_layers", -1)
        self.declare_parameter("n_batch", 512)
        self.declare_parameter("n_ubatch", 512)
        self.declare_parameter("image_min_tokens", 256)
        self.declare_parameter("image_max_tokens", 1024)
        self.declare_parameter("vision_use_gpu", True)
        self.declare_parameter("autoload", True)

        self.declare_parameter("image_topic", "/csi_cam/image_raw")
        self.declare_parameter("max_side", 640)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("auto_hz", 0.0)
        self.declare_parameter("default_label", "")
        self.declare_parameter("max_boxes", 4)
        self.declare_parameter("letterbox", True)
        self.declare_parameter("publish_annotated", True)
        self.declare_parameter("annotate_topic", "/vlm/annotated")
        self.declare_parameter("detections_topic", "/vlm/detections")
        self.declare_parameter("status_topic", "/vlm/status")

        self._models_dir = _resolve_models_dir(
            str(self.get_parameter("models_dir").value)
        )
        self._model_path = self._models_dir / str(self.get_parameter("model_file").value)
        self._clip_path = self._models_dir / str(self.get_parameter("clip_file").value)
        self._max_side = int(self.get_parameter("max_side").value)
        self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self._max_boxes = int(self.get_parameter("max_boxes").value)
        self._letterbox = bool(self.get_parameter("letterbox").value)
        self._pub_ann = bool(self.get_parameter("publish_annotated").value)
        auto_hz = float(self.get_parameter("auto_hz").value)

        self._label = str(self.get_parameter("default_label").value).strip()
        self._frame_lock = threading.Lock()
        self._latest: Optional[Tuple[np.ndarray, Any]] = None
        self._busy = threading.Lock()
        self._engine = VLMEngine()

        image_topic = str(self.get_parameter("image_topic").value)
        det_topic = str(self.get_parameter("detections_topic").value)
        st_topic = str(self.get_parameter("status_topic").value)
        ann_topic = str(self.get_parameter("annotate_topic").value)

        self.create_subscription(Image, image_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(String, "/vlm/target", self._on_target, 10)
        self.create_subscription(String, "/vlm/query", self._on_query, 10)

        self._det_pub = self.create_publisher(String, det_topic, 10)
        self._status_pub = self.create_publisher(String, st_topic, 10)
        self._answer_pub = self.create_publisher(String, "/vlm/answer", 10)
        self._ann_pub = (
            self.create_publisher(Image, ann_topic, qos_profile_sensor_data)
            if self._pub_ann
            else None
        )

        self.create_service(Trigger, "/vlm/detect", self._srv_detect)
        self.create_service(Detect, "/vlm/detect_label", self._srv_detect_label)
        self.create_service(Trigger, "/vlm/health", self._srv_health)

        self.create_timer(5.0, self._status_tick)
        if auto_hz > 0.0:
            self.create_timer(1.0 / auto_hz, self._auto_tick)

        self.get_logger().info(
            f"models_dir={self._models_dir} model={self._model_path.name} "
            f"image={image_topic} label={self._label!r}"
        )

        if bool(self.get_parameter("autoload").value):
            threading.Thread(target=self._autoload, daemon=True).start()

    def _autoload(self) -> None:
        try:
            self.get_logger().info(f"loading {self._model_path.name} …")
            t0 = time.perf_counter()
            self._engine.load(
                self._model_path,
                self._clip_path,
                model_id=str(self.get_parameter("model_id").value),
                chat_format=str(self.get_parameter("chat_format").value),
                n_ctx=int(self.get_parameter("n_ctx").value),
                n_gpu_layers=int(self.get_parameter("n_gpu_layers").value),
                n_batch=int(self.get_parameter("n_batch").value),
                n_ubatch=int(self.get_parameter("n_ubatch").value),
                image_min_tokens=int(self.get_parameter("image_min_tokens").value),
                image_max_tokens=int(self.get_parameter("image_max_tokens").value),
                vision_use_gpu=bool(self.get_parameter("vision_use_gpu").value),
            )
            self.get_logger().info(f"model loaded in {time.perf_counter() - t0:.1f}s")
        except Exception as exc:
            self.get_logger().error(f"autoload failed: {exc}")

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as exc:
            self.get_logger().warn(f"image decode: {exc}")
            return
        with self._frame_lock:
            self._latest = (bgr, msg.header)

    def _on_target(self, msg: String) -> None:
        label = msg.data.strip()
        self._label = label
        self.get_logger().info(f"target → {label!r}")
        if label:
            threading.Thread(target=self._run_detect, args=(label,), daemon=True).start()

    def _on_query(self, msg: String) -> None:
        q = msg.data.strip()
        if q:
            threading.Thread(target=self._run_query, args=(q,), daemon=True).start()

    def _srv_detect(self, _req, resp):
        if not self._label:
            resp.success = False
            resp.message = "no label — set /vlm/target or default_label"
            return resp
        if not self._busy.acquire(blocking=False):
            resp.success = False
            resp.message = "detect already in flight"
            return resp
        try:
            ok, info, _ = self._run_detect(self._label, hold_lock=False)
        finally:
            self._busy.release()
        resp.success = ok
        resp.message = info
        return resp

    def _srv_detect_label(self, req: Detect.Request, resp: Detect.Response):
        label = (req.label or "").strip()
        if not label:
            resp.success = False
            resp.message = "empty label"
            resp.detections_json = ""
            return resp
        self._label = label
        frozen = None
        hdr = None
        if req.image.height > 0 and len(req.image.data) > 0:
            try:
                frozen = imgmsg_to_bgr(req.image)
                hdr = req.image.header
            except Exception as exc:
                resp.success = False
                resp.message = f"bad request image: {exc}"
                resp.detections_json = ""
                return resp
        if not self._busy.acquire(blocking=False):
            resp.success = False
            resp.message = "detect already in flight"
            resp.detections_json = ""
            return resp
        try:
            ok, info, payload = self._run_detect(
                label, hold_lock=False, image_bgr=frozen, header=hdr
            )
        finally:
            self._busy.release()
        resp.success = ok
        resp.message = info
        resp.detections_json = json.dumps(payload) if payload else ""
        return resp

    def _srv_health(self, _req, resp):
        st = self._engine.status()
        resp.success = bool(st.get("loaded"))
        resp.message = json.dumps(st)
        return resp

    def _status_tick(self) -> None:
        st = self._engine.status()
        st["label"] = self._label
        st["has_frame"] = self._latest is not None
        st["models_dir"] = str(self._models_dir)
        s = String()
        s.data = json.dumps(st)
        self._status_pub.publish(s)

    def _auto_tick(self) -> None:
        if not self._label or not self._engine.loaded:
            return
        if not self._busy.acquire(blocking=False):
            return
        try:
            self._run_detect(self._label, hold_lock=False)
        finally:
            self._busy.release()

    def _grab(self) -> Optional[Tuple[np.ndarray, Any]]:
        with self._frame_lock:
            if self._latest is None:
                return None
            bgr, header = self._latest
            return bgr.copy(), header

    def _run_detect(
        self,
        label: str,
        *,
        hold_lock: bool = True,
        image_bgr: Optional[np.ndarray] = None,
        header: Any = None,
    ) -> Tuple[bool, str, Optional[dict]]:
        if hold_lock and not self._busy.acquire(blocking=False):
            return False, "detect already in flight", None
        try:
            if not self._engine.loaded:
                return False, "model not loaded", None
            if image_bgr is not None:
                bgr = image_bgr
            else:
                grabbed = self._grab()
                if grabbed is None:
                    return False, "no CSI frame yet", None
                bgr, header = grabbed
            if header is None:
                from std_msgs.msg import Header as _Hdr

                header = _Hdr()
            h0, w0 = bgr.shape[:2]
            small, scale = _resize_max_side(bgr, self._max_side)
            t0 = time.perf_counter()
            try:
                js = self._engine.detect(
                    small,
                    label,
                    letterbox=self._letterbox,
                    encode="jpeg",
                    jpeg_quality=self._jpeg_q,
                    max_boxes=self._max_boxes,
                )
            except Exception as exc:
                return False, f"detect error: {exc}", None
            lat = time.perf_counter() - t0
            boxes = js.get("boxes") or []
            if scale != 1.0 and boxes:
                inv = 1.0 / scale
                for b in boxes:
                    xy = b.get("xyxy_px") or []
                    if len(xy) >= 4:
                        b["xyxy_px"] = [float(v) * inv for v in xy[:4]]
                        b["xyxy_norm"] = [
                            b["xyxy_px"][0] / w0,
                            b["xyxy_px"][1] / h0,
                            b["xyxy_px"][2] / w0,
                            b["xyxy_px"][3] / h0,
                        ]
            payload = {
                "label": label,
                "boxes": boxes,
                "raw": js.get("raw", ""),
                "latency_s": lat,
                "image_size": [w0, h0],
                "model_id": js.get("model_id"),
                "stamp": {
                    "sec": int(header.stamp.sec),
                    "nanosec": int(header.stamp.nanosec),
                },
                "frame_id": header.frame_id,
            }
            out = String()
            out.data = json.dumps(payload)
            self._det_pub.publish(out)
            if self._ann_pub is not None:
                ann = _draw_boxes(bgr, boxes, label)
                try:
                    self._ann_pub.publish(bgr_to_imgmsg(ann, header))
                except Exception as exc:
                    self.get_logger().warn(f"annotate: {exc}")
            self.get_logger().info(f"detect '{label}' n={len(boxes)} lat={lat:.1f}s")
            return True, f"n={len(boxes)} lat={lat:.1f}s", payload
        finally:
            if hold_lock:
                self._busy.release()

    def _run_query(self, question: str) -> None:
        if not self._busy.acquire(blocking=False):
            self.get_logger().warn("query skipped — busy")
            return
        try:
            if not self._engine.loaded:
                self.get_logger().warn("query: model not loaded")
                return
            grabbed = self._grab()
            if grabbed is None:
                self.get_logger().warn("query: no frame")
                return
            bgr, header = grabbed
            small, _ = _resize_max_side(bgr, self._max_side)
            t0 = time.perf_counter()
            try:
                answer = self._engine.query(small, question, max_tokens=128)
            except Exception as exc:
                self.get_logger().warn(f"query failed: {exc}")
                return
            lat = time.perf_counter() - t0
            ans = String()
            ans.data = json.dumps(
                {
                    "question": question,
                    "answer": answer,
                    "latency_s": lat,
                    "frame_id": header.frame_id,
                }
            )
            self._answer_pub.publish(ans)
            self.get_logger().info(f"query ok lat={lat:.1f}s")
        finally:
            self._busy.release()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VlmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
