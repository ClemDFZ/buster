"""Subscribe to Image / CompressedImage and serve MJPEG multipart streams."""
from __future__ import annotations

import threading
import time
from typing import Dict, Optional, Tuple

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image


class FrameBuffer:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._jpeg: Optional[bytes] = None
        self._stamp: float = 0.0

    def set_jpeg(self, data: bytes) -> None:
        with self._lock:
            self._jpeg = data
            self._stamp = time.monotonic()

    def get(self) -> Tuple[Optional[bytes], float]:
        with self._lock:
            return self._jpeg, self._stamp

    @property
    def age_s(self) -> float:
        with self._lock:
            if self._stamp <= 0.0:
                return 1e9
            return time.monotonic() - self._stamp


class MjpegHub:
    """Keep latest JPEG per topic; encode raw Image on the fly."""

    def __init__(self, jpeg_quality: int = 70) -> None:
        self._bufs: Dict[str, FrameBuffer] = {}
        self._bridge = CvBridge()
        self._q = int(jpeg_quality)
        self._lock = threading.Lock()

    def ensure(self, topic: str) -> FrameBuffer:
        with self._lock:
            if topic not in self._bufs:
                self._bufs[topic] = FrameBuffer()
            return self._bufs[topic]

    def on_compressed(self, topic: str, msg: CompressedImage) -> None:
        self.ensure(topic).set_jpeg(bytes(msg.data))

    def on_image(self, topic: str, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return
        ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._q])
        if ok:
            self.ensure(topic).set_jpeg(enc.tobytes())

    def has_fresh(self, topic: str, max_age_s: float = 2.0) -> bool:
        buf = self.ensure(topic)
        return buf.get()[0] is not None and buf.age_s <= max_age_s

    def iter_multipart(self, topic: str, fps: float = 15.0):
        """Generator yielding multipart MJPEG chunks."""
        buf = self.ensure(topic)
        period = 1.0 / max(fps, 1.0)
        boundary = b"--frame"
        last = b""
        while True:
            jpeg, _ = buf.get()
            if jpeg and jpeg is not last:
                last = jpeg
                yield (
                    boundary
                    + b"\r\nContent-Type: image/jpeg\r\nContent-Length: "
                    + str(len(jpeg)).encode()
                    + b"\r\n\r\n"
                    + jpeg
                    + b"\r\n"
                )
            time.sleep(period)
