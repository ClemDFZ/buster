"""Grounding helpers: letterbox, GBNF, parse bbox/box_2d 0-1000."""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

# Qwen: bbox_2d [x1,y1,x2,y2] 0–1000.
GROUNDING_PROMPT = (
    'Locate every instance of ["{label}"] in the image. '
    "Return a JSON array only. Each item must be "
    '{{"bbox_2d": [x1, y1, x2, y2], "label": "{label}"}} '
    "where coordinates are integers normalized to 0-1000 "
    "(x1,y1=top-left, x2,y2=bottom-right). "
    "If none are present, return []."
)

# Gemma 4: native box_2d [y1,x1,y2,x2] 0–1000.
GROUNDING_PROMPT_GEMMA = (
    'Detect every instance of "{label}" in the image. '
    "Return a JSON array only. Each item must be "
    '{{"box_2d": [y1, x1, y2, x2], "label": "{label}"}} '
    "where coordinates are integers normalized to 0-1000 "
    "(y1,x1=top-left, y2,x2=bottom-right). "
    "If none are present, return []."
)

# Strict JSON array of {bbox_2d:[i,i,i,i], label:str}. Empty array allowed.
DETECT_GBNF = r"""
root   ::= "[" ws items? ws "]"
items  ::= obj ( "," ws obj )*
obj    ::= "{" ws "\"bbox_2d\"" ws ":" ws box ws "," ws "\"label\"" ws ":" ws string ws "}"
box    ::= "[" ws int ws "," ws int ws "," ws int ws "," ws int ws "]"
int    ::= "0" | [1-9] [0-9]{0,3}
string ::= "\"" chars "\""
chars  ::= char*
char   ::= [^"\\] | "\\" ["\\/bfnrt]
ws     ::= [ \t\n]*
"""

DETECT_GBNF_GEMMA = r"""
root   ::= "[" ws items? ws "]"
items  ::= obj ( "," ws obj )*
obj    ::= "{" ws "\"box_2d\"" ws ":" ws box ws "," ws "\"label\"" ws ":" ws string ws "}"
box    ::= "[" ws int ws "," ws int ws "," ws int ws "," ws int ws "]"
int    ::= "0" | [1-9] [0-9]{0,3}
string ::= "\"" chars "\""
chars  ::= char*
char   ::= [^"\\] | "\\" ["\\/bfnrt]
ws     ::= [ \t\n]*
"""

_JSON_ARRAY_RE = re.compile(r"\[[\s\S]*\]")


@dataclass
class LetterboxMeta:
    out_w: int
    out_h: int
    scale: float
    pad_x: int  # padding on left of square canvas (pre-resize pixels)
    pad_y: int  # padding on top of square canvas (pre-resize pixels)
    src_w: int
    src_h: int
    side: int  # square side before optional resize


def letterbox_square(
    img_bgr: np.ndarray,
    *,
    pad_value: int = 114,
    size: int | None = None,
) -> tuple[np.ndarray, LetterboxMeta]:
    """Pad to square (gray) preserving aspect; optional resize to `size`x`size`."""
    h, w = img_bgr.shape[:2]
    side = max(h, w)
    canvas = np.full((side, side, 3), pad_value, dtype=img_bgr.dtype)
    pad_x = (side - w) // 2
    pad_y = (side - h) // 2
    canvas[pad_y : pad_y + h, pad_x : pad_x + w] = img_bgr
    scale = 1.0
    out = canvas
    out_side = side
    if size is not None and size > 0 and size != side:
        scale = float(size) / float(side)
        out = cv2.resize(canvas, (size, size), interpolation=cv2.INTER_LINEAR)
        out_side = size
    return out, LetterboxMeta(
        out_w=out_side,
        out_h=out_side,
        scale=scale,
        pad_x=pad_x,
        pad_y=pad_y,
        src_w=w,
        src_h=h,
        side=side,
    )


def unletterbox_xyxy(xyxy: list[float], meta: LetterboxMeta) -> list[float]:
    """Map pixel coords from letterboxed (possibly resized) image → original."""
    # out pixels → square-canvas pixels
    if meta.scale and abs(meta.scale - 1.0) > 1e-9:
        x1c = xyxy[0] / meta.scale
        y1c = xyxy[1] / meta.scale
        x2c = xyxy[2] / meta.scale
        y2c = xyxy[3] / meta.scale
    else:
        x1c, y1c, x2c, y2c = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
    x1 = x1c - meta.pad_x
    y1 = y1c - meta.pad_y
    x2 = x2c - meta.pad_x
    y2 = y2c - meta.pad_y
    x1 = max(0.0, min(float(meta.src_w), x1))
    y1 = max(0.0, min(float(meta.src_h), y1))
    x2 = max(0.0, min(float(meta.src_w), x2))
    y2 = max(0.0, min(float(meta.src_h), y2))
    return [x1, y1, x2, y2]


def norm1000_to_px(bbox_2d: list[float], width: int, height: int) -> list[float]:
    x1, y1, x2, y2 = (float(v) for v in bbox_2d[:4])
    return [
        x1 / 1000.0 * width,
        y1 / 1000.0 * height,
        x2 / 1000.0 * width,
        y2 / 1000.0 * height,
    ]


def px_to_norm(xyxy: list[float], width: int, height: int) -> list[float]:
    return [
        xyxy[0] / max(1, width),
        xyxy[1] / max(1, height),
        xyxy[2] / max(1, width),
        xyxy[3] / max(1, height),
    ]


def parse_bbox_response(raw: str, *, coord_order: str = "xyxy") -> list[dict[str, Any]]:
    """Parse model JSON into list of {bbox_2d:[x1,y1,x2,y2], label}.

    ``coord_order``:
      - ``xyxy``: Qwen-style bbox_2d [x1,y1,x2,y2]
      - ``yxyx``: Gemma-style box_2d [y1,x1,y2,x2] → converted to xyxy
    """
    text = (raw or "").strip()
    if not text:
        return []
    # Strip optional markdown fences
    if text.startswith("```"):
        text = re.sub(r"^```(?:json)?\s*", "", text)
        text = re.sub(r"\s*```$", "", text)
    candidates = [text]
    m = _JSON_ARRAY_RE.search(text)
    if m:
        candidates.insert(0, m.group(0))
    order = (coord_order or "xyxy").lower()
    for cand in candidates:
        try:
            data = json.loads(cand)
        except Exception:
            continue
        if not isinstance(data, list):
            continue
        out: list[dict[str, Any]] = []
        for item in data:
            if not isinstance(item, dict):
                continue
            box = (
                item.get("bbox_2d")
                or item.get("box_2d")
                or item.get("bbox")
                or item.get("box")
            )
            if not isinstance(box, (list, tuple)) or len(box) < 4:
                continue
            try:
                a, b, c, d = (float(box[0]), float(box[1]), float(box[2]), float(box[3]))
            except Exception:
                continue
            # Prefer key hint: box_2d alone → Gemma yxyx even if caller said xyxy
            item_order = order
            if "box_2d" in item and "bbox_2d" not in item:
                item_order = "yxyx"
            if item_order == "yxyx":
                nums = [b, a, d, c]  # y1,x1,y2,x2 → x1,y1,x2,y2
            else:
                nums = [a, b, c, d]
            label = str(item.get("label", "") or "")
            out.append({"bbox_2d": nums, "label": label})
        return out
    return []


def encode_image(
    img_bgr: np.ndarray,
    *,
    encode: str = "png",
    jpeg_quality: int = 95,
) -> tuple[bytes, str]:
    enc = (encode or "png").lower()
    if enc in ("jpg", "jpeg"):
        q = max(1, min(100, int(jpeg_quality)))
        ok, buf = cv2.imencode(".jpg", img_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), q])
        if not ok:
            raise RuntimeError("jpeg encode failed")
        return buf.tobytes(), "image/jpeg"
    ok, buf = cv2.imencode(".png", img_bgr)
    if not ok:
        raise RuntimeError("png encode failed")
    return buf.tobytes(), "image/png"


def crop_with_padding(
    img_bgr: np.ndarray,
    xyxy: list[float],
    pad_ratio: float = 0.25,
) -> tuple[np.ndarray, tuple[int, int, int, int]]:
    h, w = img_bgr.shape[:2]
    x1, y1, x2, y2 = xyxy
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    px, py = bw * pad_ratio, bh * pad_ratio
    ax1 = max(0, int(math.floor(x1 - px)))
    ay1 = max(0, int(math.floor(y1 - py)))
    ax2 = min(w, int(math.ceil(x2 + px)))
    ay2 = min(h, int(math.ceil(y2 + py)))
    if ax2 - ax1 < 32:
        cx = (ax1 + ax2) // 2
        ax1, ax2 = max(0, cx - 16), min(w, cx + 16)
    if ay2 - ay1 < 32:
        cy = (ay1 + ay2) // 2
        ay1, ay2 = max(0, cy - 16), min(h, cy + 16)
    return img_bgr[ay1:ay2, ax1:ax2].copy(), (ax1, ay1, ax2, ay2)
