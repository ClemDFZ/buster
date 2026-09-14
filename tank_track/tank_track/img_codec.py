"""sensor_msgs/Image ↔ BGR without cv_bridge (NumPy ABI safety)."""
from __future__ import annotations

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image


def imgmsg_to_bgr(msg: Image) -> np.ndarray:
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


def bgr_to_compressed(bgr: np.ndarray, header, *, quality: int = 70) -> CompressedImage:
    ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("jpeg encode failed")
    msg = CompressedImage()
    msg.header = header
    msg.format = "jpeg"
    msg.data = buf.tobytes()
    return msg
