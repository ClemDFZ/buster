#!/usr/bin/env python3
"""Own CSI via nvarguscamerasrc and publish /csi_cam/image_raw.

Stop pantilt / csi-cam1-web first. Does not exit if Argus fails — retries.
"""
from __future__ import annotations

import math
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def gstreamer_pipeline(width: int, height: int, fps: int, sensor_id: int) -> str:
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw,format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=1 max-buffers=1"
    )


def open_csi(width: int, height: int, fps: int):
    env_sensor = os.getenv("CSI_SENSOR_ID", "").strip()
    sensor_ids = [0, 1]
    if env_sensor:
        try:
            sid = int(env_sensor)
            sensor_ids = [sid] + [x for x in sensor_ids if x != sid]
        except ValueError:
            pass
    for sensor_id in sensor_ids:
        pipe = gstreamer_pipeline(width, height, fps, sensor_id)
        cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        time.sleep(1.5)
        ok, _ = cap.read()
        if ok:
            return cap, sensor_id
        cap.release()
    return None, None


class CsiArgus(Node):
    def __init__(self) -> None:
        super().__init__("csi_argus")
        self.declare_parameter("frame_id", "csi_optical_frame")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("hfov_deg", 62.2)
        self.declare_parameter("reconnect_period_s", 5.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._fps = int(self.get_parameter("fps").value)
        hfov = math.radians(float(self.get_parameter("hfov_deg").value))
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
        self._last_try = 0.0
        self.get_logger().warn("[csi] Argus mode — ensure pantilt/csi-cam1-web are stopped")
        self._try_open(log=True)
        self.create_timer(1.0 / max(self._fps, 1), self._tick)

    def _try_open(self, log: bool = False) -> bool:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        cap, sid = open_csi(self._w, self._h, self._fps)
        if cap is None:
            if log:
                self.get_logger().warn(
                    "[csi] MISSING Argus open failed — will retry "
                    "(busy sensor or no IMX219?)"
                )
            return False
        self._cap = cap
        self.get_logger().info(f"[csi] OK Argus sensor-id={sid} {self._w}x{self._h}@{self._fps}")
        return True

    def _tick(self) -> None:
        if self._cap is None:
            now = time.monotonic()
            if now - self._last_try >= self._reconnect_period:
                self._last_try = now
                self._try_open(log=True)
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self.get_logger().warn("[csi] Argus read failed — reconnecting")
            self._try_open(log=False)
            return
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
    node = CsiArgus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
