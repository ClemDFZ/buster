#!/usr/bin/env python3
"""Republish pantilt (or csi-cam1-web) MJPEG stream as /csi_cam/image_raw.

Does not open Argus. Stays up and retries if MJPEG is temporarily down.
"""
from __future__ import annotations

import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CsiRepublish(Node):
    def __init__(self) -> None:
        super().__init__("csi_republish")
        self.declare_parameter("mjpeg_url", "http://127.0.0.1:5003/video_feed/color")
        self.declare_parameter("frame_id", "csi_optical_frame")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("hfov_deg", 62.2)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("reconnect_period_s", 2.0)

        self._url = str(self.get_parameter("mjpeg_url").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        hfov = math.radians(float(self.get_parameter("hfov_deg").value))
        rate = float(self.get_parameter("rate_hz").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period_s").value)

        self._bridge = CvBridge()
        self._img_pub = self.create_publisher(Image, "/csi_cam/image_raw", qos_profile_sensor_data)
        self._info_pub = self.create_publisher(CameraInfo, "/csi_cam/camera_info", qos_profile_sensor_data)

        fx = (self._w * 0.5) / math.tan(hfov * 0.5)
        self._info = CameraInfo()
        self._info.width = self._w
        self._info.height = self._h
        self._info.distortion_model = "plumb_bob"
        self._info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._info.k = [fx, 0.0, self._w * 0.5, 0.0, fx, self._h * 0.5, 0.0, 0.0, 1.0]
        self._info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._info.p = [fx, 0.0, self._w * 0.5, 0.0, 0.0, fx, self._h * 0.5, 0.0, 0.0, 0.0, 1.0, 0.0]

        self._cap = None
        self._ok = False
        self._last_miss_log = 0.0
        self._open_cap(log=True)

        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _open_cap(self, log: bool = False) -> bool:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        cap = cv2.VideoCapture(self._url)
        if not cap.isOpened():
            self._ok = False
            if log:
                self.get_logger().warn(
                    f"[csi] MISSING MJPEG {self._url} — is pantilt/csi-cam1-web running? retrying"
                )
            return False
        self._cap = cap
        self._ok = True
        self.get_logger().info(f"[csi] OK republish from {self._url}")
        return True

    def _tick(self) -> None:
        if self._cap is None or not self._cap.isOpened():
            now = time.monotonic()
            if now - self._last_miss_log >= self._reconnect_period:
                self._last_miss_log = now
                self._open_cap(log=True)
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            if self._ok:
                self.get_logger().warn("[csi] frame read failed — reconnecting")
            self._ok = False
            self._open_cap(log=False)
            return
        if not self._ok:
            self._ok = True
            self.get_logger().info("[csi] stream recovered")
        if frame.shape[1] != self._w or frame.shape[0] != self._h:
            frame = cv2.resize(frame, (self._w, self._h), interpolation=cv2.INTER_AREA)
        stamp = self.get_clock().now().to_msg()
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        self._info.header = msg.header
        self._img_pub.publish(msg)
        self._info_pub.publish(self._info)

    def destroy_node(self) -> bool:
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CsiRepublish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
