"""Visual tracker: OSTrack TRT when available, CSRT fallback for dev."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import tensorrt as trt
import torch

from tank_track.logutil import phase

_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 3, 1, 1)
_STD = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 3, 1, 1)


def _valid_bbox(bbox: Tuple[float, float, float, float], w: int, h: int) -> bool:
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


def _bbox_center(bbox: Tuple[float, float, float, float]) -> Tuple[float, float]:
    x1, y1, x2, y2 = bbox
    return (x1 + x2) * 0.5, (y1 + y2) * 0.5


def _xyxy_to_xywh(bbox: Tuple[float, float, float, float]) -> List[float]:
    x1, y1, x2, y2 = bbox
    return [float(x1), float(y1), float(x2 - x1), float(y2 - y1)]


def _xywh_to_xyxy(box: List[float]) -> Tuple[float, float, float, float]:
    x, y, w, h = box
    return (float(x), float(y), float(x + w), float(y + h))


def _clip_xywh(box: List[float], H: int, W: int, margin: int = 0) -> List[float]:
    x, y, w, h = box
    x1 = min(max(0, x), W - margin)
    y1 = min(max(0, y), H - margin)
    x2 = min(max(margin, x + w), W)
    y2 = min(max(margin, y + h), H)
    return [x1, y1, max(1.0, x2 - x1), max(1.0, y2 - y1)]


def _sample_target(
    im: np.ndarray,
    target_bb: List[float],
    search_area_factor: float,
    output_sz: int,
) -> Tuple[np.ndarray, float]:
    """Square crop around target_bb [x,y,w,h]; returns RGB patch + resize_factor."""
    x, y, w, h = target_bb
    crop_sz = math.ceil(math.sqrt(max(1.0, w * h)) * search_area_factor)
    if crop_sz < 1:
        crop_sz = 1
    x1 = round(x + 0.5 * w - crop_sz * 0.5)
    y1 = round(y + 0.5 * h - crop_sz * 0.5)
    x2 = x1 + crop_sz
    y2 = y1 + crop_sz
    x1_pad = max(0, -x1)
    x2_pad = max(x2 - im.shape[1] + 1, 0)
    y1_pad = max(0, -y1)
    y2_pad = max(y2 - im.shape[0] + 1, 0)
    im_crop = im[y1 + y1_pad : y2 - y2_pad, x1 + x1_pad : x2 - x2_pad]
    im_crop_padded = cv2.copyMakeBorder(
        im_crop, y1_pad, y2_pad, x1_pad, x2_pad, cv2.BORDER_CONSTANT
    )
    resize_factor = output_sz / float(crop_sz)
    patch = cv2.resize(im_crop_padded, (output_sz, output_sz), interpolation=cv2.INTER_LINEAR)
    return patch, resize_factor


def _preprocess_rgb(patch_bgr: np.ndarray) -> np.ndarray:
    rgb = cv2.cvtColor(patch_bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
    chw = rgb.transpose(2, 0, 1)[None, ...]  # 1,3,H,W
    return ((chw / 255.0) - _MEAN) / _STD


def _hann2d(sz: int) -> np.ndarray:
    t = np.arange(1, sz + 1, dtype=np.float32)
    w = 0.5 * (1.0 - np.cos((2.0 * math.pi / (sz + 1)) * t))
    return (w.reshape(1, 1, -1, 1) * w.reshape(1, 1, 1, -1)).astype(np.float32)


def _cal_bbox(
    score_map: np.ndarray, size_map: np.ndarray, offset_map: np.ndarray, feat_sz: int
) -> Tuple[np.ndarray, float]:
    """score/size/offset → cx,cy,w,h in [0,1] + max score."""
    sm = score_map.reshape(-1)
    idx = int(np.argmax(sm))
    max_score = float(sm[idx])
    idx_y = idx // feat_sz
    idx_x = idx % feat_sz
    size = size_map.reshape(2, -1)[:, idx]
    offset = offset_map.reshape(2, -1)[:, idx]
    cx = (idx_x + float(offset[0])) / feat_sz
    cy = (idx_y + float(offset[1])) / feat_sz
    return np.array([cx, cy, float(size[0]), float(size[1])], dtype=np.float32), max_score


class MultiIOTrtEngine:
    """TensorRT engine with named multi-input / multi-output tensors (TRT 10)."""

    LOGGER = trt.Logger(trt.Logger.WARNING)

    def __init__(self, engine_path: Path, device: torch.device):
        self.device = device
        runtime = trt.Runtime(self.LOGGER)
        self.engine = runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self.engine is None:
            raise RuntimeError(f"Failed to load engine: {engine_path}")
        self.context = self.engine.create_execution_context()
        if self.context is None:
            raise RuntimeError(f"Failed to create context: {engine_path}")
        if not hasattr(self.engine, "num_io_tensors"):
            raise RuntimeError("OSTrack TRT requires TensorRT 10 tensor API")

        self.input_names: List[str] = []
        self.output_names: List[str] = []
        for i in range(int(self.engine.num_io_tensors)):
            name = self.engine.get_tensor_name(i)
            mode = self.engine.get_tensor_mode(name)
            if mode == trt.TensorIOMode.INPUT:
                self.input_names.append(name)
            else:
                self.output_names.append(name)
        if len(self.input_names) < 2 or len(self.output_names) < 3:
            raise RuntimeError(
                f"Unexpected OSTrack IO: in={self.input_names} out={self.output_names}"
            )

    def __call__(self, inputs: Dict[str, torch.Tensor]) -> Dict[str, torch.Tensor]:
        for name, tensor in inputs.items():
            if tuple(self.context.get_tensor_shape(name)) != tuple(tensor.shape):
                self.context.set_input_shape(name, tuple(tensor.shape))
            self.context.set_tensor_address(name, int(tensor.data_ptr()))

        outputs: Dict[str, torch.Tensor] = {}
        for name in self.output_names:
            shape = tuple(self.context.get_tensor_shape(name))
            dtype_np = trt.nptype(self.engine.get_tensor_dtype(name))
            if dtype_np == np.float16:
                dtype_t = torch.float16
            elif dtype_np == np.int32:
                dtype_t = torch.int32
            else:
                dtype_t = torch.float32
            out_t = torch.empty(size=shape, dtype=dtype_t, device=self.device)
            self.context.set_tensor_address(name, int(out_t.data_ptr()))
            outputs[name] = out_t

        if not self.context.execute_async_v3(0):
            raise RuntimeError("OSTrack TRT execute failed")
        torch.cuda.synchronize()
        return outputs


class OSTrackTracker:
    """init(frame, bbox) then update(frame) -> (ok, bbox, score)."""

    TEMPLATE_SIZE = 128
    SEARCH_SIZE = 256
    TEMPLATE_FACTOR = 2.0
    SEARCH_FACTOR = 4.0
    STRIDE = 16

    def __init__(
        self,
        *,
        device: torch.device,
        ostrack_engine: str,
        backend: str,
        track_conf_threshold: float,
    ):
        self.device = device
        self.track_conf_threshold = track_conf_threshold
        self._backend = "none"
        self._cv_tracker = None
        self._trt: Optional[MultiIOTrtEngine] = None
        self._template_tensor: Optional[torch.Tensor] = None
        self._state_xywh: Optional[List[float]] = None
        self._last_bbox: Optional[Tuple[float, float, float, float]] = None
        self._score = 0.0
        self._feat_sz = self.SEARCH_SIZE // self.STRIDE
        self._hann = torch.from_numpy(_hann2d(self._feat_sz)).to(device)

        engine_p = Path(ostrack_engine).expanduser().resolve() if ostrack_engine else None
        want_trt = backend in ("trt", "auto") and engine_p is not None and engine_p.is_file()

        if want_trt:
            phase("OSTRACK", "START", f"Loading OSTrack TRT: {engine_p}")
            try:
                self._trt = MultiIOTrtEngine(engine_p, device=device)
                self._backend = "trt"
                phase("OSTRACK", "OK", f"IO in={self._trt.input_names} out={self._trt.output_names}")
            except Exception as exc:
                phase("OSTRACK", "WARN", f"TRT load failed ({exc}) — CSRT fallback")
                self._trt = None
                self._backend = "csrt"
        else:
            phase("OSTRACK", "WARN", "No TRT engine — using OpenCV CSRT fallback")
            self._backend = "csrt"

    @property
    def backend(self) -> str:
        return self._backend

    def reset(self) -> None:
        self._cv_tracker = None
        self._template_tensor = None
        self._state_xywh = None
        self._last_bbox = None
        self._score = 0.0

    def init(self, frame_bgr: np.ndarray, bbox: Tuple[float, float, float, float]) -> bool:
        h, w = frame_bgr.shape[:2]
        if not _valid_bbox(bbox, w, h):
            return False
        self.reset()
        if self._backend == "trt" and self._trt is not None:
            xywh = _xyxy_to_xywh(bbox)
            z_patch, _ = _sample_target(
                frame_bgr, xywh, self.TEMPLATE_FACTOR, self.TEMPLATE_SIZE
            )
            z = torch.from_numpy(_preprocess_rgb(z_patch)).to(
                device=self.device, dtype=torch.float32
            )
            self._template_tensor = z.contiguous()
            self._state_xywh = xywh
            self._last_bbox = bbox
            self._score = 1.0
            return True
        try:
            tr = cv2.legacy.TrackerCSRT_create()
        except AttributeError:
            tr = cv2.TrackerCSRT_create()
        x1, y1, x2, y2 = bbox
        ok = tr.init(frame_bgr, (x1, y1, x2 - x1, y2 - y1))
        if not ok:
            return False
        self._cv_tracker = tr
        self._last_bbox = bbox
        self._score = 1.0
        return True

    def update(self, frame_bgr: np.ndarray) -> Tuple[bool, Optional[Tuple[float, float, float, float]], float]:
        h, w = frame_bgr.shape[:2]
        if self._backend == "trt" and self._trt is not None and self._template_tensor is not None:
            return self._update_trt(frame_bgr, w, h)
        if self._cv_tracker is not None:
            ok, box = self._cv_tracker.update(frame_bgr)
            if not ok:
                self._score = 0.0
                return False, None, 0.0
            x, y, bw, bh = box
            bbox = (float(x), float(y), float(x + bw), float(y + bh))
            if not _valid_bbox(bbox, w, h):
                self._score = 0.0
                return False, None, 0.0
            self._last_bbox = bbox
            self._score = 1.0
            return True, bbox, self._score
        return False, None, 0.0

    def _resolve_io_names(self) -> Tuple[str, str, str, str, str]:
        assert self._trt is not None
        # Prefer exported names; fall back to engine order (z, x) / (score, size, offset).
        in_map = {n.lower(): n for n in self._trt.input_names}
        out_map = {n.lower(): n for n in self._trt.output_names}
        z_name = in_map.get("z", self._trt.input_names[0])
        x_name = in_map.get("x", self._trt.input_names[1])
        score_name = out_map.get("score_map", self._trt.output_names[0])
        size_name = out_map.get("size_map", self._trt.output_names[1])
        offset_name = out_map.get("offset_map", self._trt.output_names[2])
        return z_name, x_name, score_name, size_name, offset_name

    def _update_trt(
        self, frame_bgr: np.ndarray, w: int, h: int
    ) -> Tuple[bool, Optional[Tuple[float, float, float, float]], float]:
        assert self._trt is not None and self._state_xywh is not None and self._template_tensor is not None
        try:
            x_patch, resize_factor = _sample_target(
                frame_bgr, self._state_xywh, self.SEARCH_FACTOR, self.SEARCH_SIZE
            )
            x = torch.from_numpy(_preprocess_rgb(x_patch)).to(
                device=self.device, dtype=torch.float32
            ).contiguous()
            z_name, x_name, score_name, size_name, offset_name = self._resolve_io_names()
            outs = self._trt({z_name: self._template_tensor, x_name: x})
            score_map = outs[score_name].float()
            size_map = outs[size_name].float()
            offset_map = outs[offset_name].float()
            response = (score_map * self._hann).detach().cpu().numpy()
            size_np = size_map.detach().cpu().numpy()
            offset_np = offset_map.detach().cpu().numpy()
            pred, max_score = _cal_bbox(response, size_np, offset_np, self._feat_sz)
            # pred in [0,1] → crop pixels then image coords (OSTrack map_box_back)
            cx, cy, bw, bh = (pred * (self.SEARCH_SIZE / resize_factor)).tolist()
            cx_prev = self._state_xywh[0] + 0.5 * self._state_xywh[2]
            cy_prev = self._state_xywh[1] + 0.5 * self._state_xywh[3]
            half_side = 0.5 * self.SEARCH_SIZE / resize_factor
            cx_real = cx + (cx_prev - half_side)
            cy_real = cy + (cy_prev - half_side)
            xywh = _clip_xywh(
                [cx_real - 0.5 * bw, cy_real - 0.5 * bh, bw, bh], h, w, margin=10
            )
            bbox = _xywh_to_xyxy(xywh)
            if not _valid_bbox(bbox, w, h) or max_score < self.track_conf_threshold:
                self._score = max_score
                return False, None, max_score
            self._state_xywh = xywh
            self._last_bbox = bbox
            self._score = max_score
            return True, bbox, max_score
        except Exception as exc:
            phase("OSTRACK", "ERR", str(exc))
            self._score = 0.0
            return False, None, 0.0

    def get_reference_crop(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        if self._last_bbox is None:
            return None
        x1, y1, x2, y2 = (int(v) for v in self._last_bbox)
        h, w = frame_bgr.shape[:2]
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(w, x2), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        return frame_bgr[y1:y2, x1:x2].copy()
