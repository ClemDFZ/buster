#!/usr/bin/env python3
"""Rewrite sensor_msgs header.frame_id (CameraInfo / Image / PointCloud2)."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class RewriteFrameId(Node):
    def __init__(self) -> None:
        super().__init__("rewrite_frame_id")
        self.frame_id = (
            self.declare_parameter("frame_id", "realsense_color_optical_frame")
            .get_parameter_value()
            .string_value
        )
        msg_type = (
            self.declare_parameter("msg_type", "camera_info")
            .get_parameter_value()
            .string_value
        )
        type_map = {
            "camera_info": CameraInfo,
            "image": Image,
            "pointcloud2": PointCloud2,
        }
        if msg_type not in type_map:
            raise SystemExit(f"msg_type must be one of {list(type_map)}")
        cls = type_map[msg_type]
        # Match ros_gz_bridge / RViz sensor QoS (BEST_EFFORT).
        qos = qos_profile_sensor_data
        self.pub = self.create_publisher(cls, "out", qos)
        self.create_subscription(cls, "in", self._cb, qos)

    def _cb(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RewriteFrameId()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
