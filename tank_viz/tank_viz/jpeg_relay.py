#!/usr/bin/env python3
"""Raw Image → CompressedImage (Foxglove). Latest-wins per topic, rate-capped."""
from __future__ import annotations

import time
from typing import Dict, List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def _img_to_bgr(msg: Image) -> np.ndarray:
    enc = (msg.encoding or "").lower()
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ("bgr8", "rgb8", "8uc3"):
        if msg.step == msg.width * 3:
            img = arr.reshape(msg.height, msg.width, 3)
        else:
            row = msg.width * 3
            img = np.empty((msg.height, msg.width, 3), dtype=np.uint8)
            for y in range(msg.height):
                img[y] = arr[y * msg.step : y * msg.step + row].reshape(msg.width, 3)
        return img[:, :, ::-1] if enc == "rgb8" else img
    if enc in ("mono8", "8uc1"):
        gray = arr.reshape(msg.height, msg.step)[:, : msg.width]
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f"unsupported encoding {msg.encoding!r}")


class JpegRelay(Node):
    def __init__(self) -> None:
        super().__init__("jpeg_relay")
        self.declare_parameter("image_topic", "/csi_cam/image_raw")
        self.declare_parameter("compressed_topic", "/csi_cam/image_raw/compressed")
        self.declare_parameter(
            "image_topics",
            [
                "/csi_cam/image_raw",
                "/camera/camera/color/image_raw",
            ],
        )
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("max_hz", 12.0)

        topics: List[str] = list(
            self.get_parameter("image_topics").get_parameter_value().string_array_value
        )
        if not topics:
            topics = [str(self.get_parameter("image_topic").value)]

        self._q = int(self.get_parameter("jpeg_quality").value)
        max_hz = float(self.get_parameter("max_hz").value)
        self._min_dt = 1.0 / max(max_hz, 1.0)
        self._pending: Dict[str, Optional[Image]] = {}
        self._last_pub: Dict[str, float] = {}
        self._pubs: Dict[str, object] = {}

        for src in topics:
            dst = src.rstrip("/") + "/compressed"
            self._pending[src] = None
            self._last_pub[src] = 0.0
            self.create_subscription(
                Image,
                src,
                lambda msg, t=src: self._on_img(t, msg),
                qos_profile_sensor_data,
            )
            self._pubs[src] = self.create_publisher(
                CompressedImage, dst, qos_profile_sensor_data
            )
            self.get_logger().info(f"{src} → {dst} q={self._q} max_hz={max_hz}")

        self.create_timer(self._min_dt, self._flush)

    def _on_img(self, topic: str, msg: Image) -> None:
        self._pending[topic] = msg

    def _flush(self) -> None:
        now = time.monotonic()
        for src, msg in list(self._pending.items()):
            if msg is None:
                continue
            if now - self._last_pub[src] < self._min_dt * 0.5:
                continue
            self._pending[src] = None
            self._last_pub[src] = now
            try:
                bgr = _img_to_bgr(msg)
                ok, buf = cv2.imencode(
                    ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self._q]
                )
                if not ok:
                    continue
                out = CompressedImage()
                out.header = msg.header
                out.format = "jpeg"
                out.data = buf.tobytes()
                self._pubs[src].publish(out)
            except ValueError:
                continue


def main() -> None:
    rclpy.init()
    node = JpegRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
