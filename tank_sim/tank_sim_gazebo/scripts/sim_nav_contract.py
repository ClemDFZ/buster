#!/usr/bin/env python3
"""Sim-side nav contract: /odom, /odom_wheel, /imu/data_raw, odom_gt error.

GZ publishes /odom_gt (no TF). Real stack uses:
  /odom_wheel (no TF) → odom_mux → /odom + TF odom→base_footprint
  /imu/data_raw

This node:
  - always republishes /odom_gt → /odom_wheel with frames odom / base_footprint
  - always publishes /imu/data_raw derived from GT (mock; same topic/frame as Mega)
  - when publish_nav_odom:=true (rtabmap off): /odom + TF from GT
  - when rtabmap owns /odom: still publishes /odom_gt_error vs that /odom
"""
from __future__ import annotations

import math
import os
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

from parity_lib import imu_from_odometry, pose_xy_yaw_error, yaw_from_quat


class SimNavContract(Node):
    def __init__(self) -> None:
        super().__init__("sim_nav_contract")
        self.declare_parameter("gt_topic", "/odom_gt")
        self.declare_parameter("nav_odom_topic", "/odom")
        self.declare_parameter("wheel_topic", "/odom_wheel")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "mpu_link")
        self.declare_parameter("publish_nav_odom", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("path_max", 1500)
        self.declare_parameter("path_min_dist", 0.02)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._publish_nav = bool(self.get_parameter("publish_nav_odom").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._publish_imu = bool(self.get_parameter("publish_imu").value)
        self._path_max = int(self.get_parameter("path_max").value)
        self._path_min_dist = float(self.get_parameter("path_min_dist").value)

        self._gt: Optional[Odometry] = None
        self._nav: Optional[Odometry] = None
        self._prev_vx: Optional[float] = None
        self._prev_vy: Optional[float] = None
        self._prev_t = 0.0
        self._path_gt = []

        gt_topic = str(self.get_parameter("gt_topic").value)
        nav_topic = str(self.get_parameter("nav_odom_topic").value)
        wheel_topic = str(self.get_parameter("wheel_topic").value)
        imu_topic = str(self.get_parameter("imu_topic").value)

        self._wheel_pub = self.create_publisher(Odometry, wheel_topic, 20)
        self._odom_pub = (
            self.create_publisher(Odometry, nav_topic, 20) if self._publish_nav else None
        )
        self._imu_pub = (
            self.create_publisher(Imu, imu_topic, qos_profile_sensor_data)
            if self._publish_imu
            else None
        )
        self._err_pub = self.create_publisher(Float32MultiArray, "/odom_gt_error", 10)
        self._err_pose_pub = self.create_publisher(PoseStamped, "/odom_gt_error_pose", 10)
        self._path_pub = self.create_publisher(Path, "/path/gt", 10)
        self._tf = TransformBroadcaster(self) if (self._publish_nav and self._publish_tf) else None

        self.create_subscription(Odometry, gt_topic, self._on_gt, 20)
        # Always listen to /odom so VO vs GT error works when rtabmap owns nav odom.
        if not self._publish_nav:
            self.create_subscription(Odometry, nav_topic, self._on_nav, 20)

        self.get_logger().info(
            f"sim_nav_contract gt={gt_topic} → {wheel_topic}"
            f" nav_odom={self._publish_nav} imu={self._publish_imu}"
        )

    def _rewrite(self, src: Odometry) -> Odometry:
        out = Odometry()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = src.pose
        out.twist = src.twist
        return out

    def _xyyaw(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return (float(p.x), float(p.y), yaw_from_quat(q.w, q.x, q.y, q.z))

    def _on_nav(self, msg: Odometry) -> None:
        self._nav = msg
        self._publish_error()

    def _on_gt(self, msg: Odometry) -> None:
        self._gt = msg
        wheeled = self._rewrite(msg)
        self._wheel_pub.publish(wheeled)
        if self._odom_pub is not None:
            self._odom_pub.publish(wheeled)
            self._nav = wheeled
            if self._tf is not None:
                t = TransformStamped()
                t.header.stamp = wheeled.header.stamp
                t.header.frame_id = self._odom_frame
                t.child_frame_id = self._base_frame
                t.transform.translation.x = wheeled.pose.pose.position.x
                t.transform.translation.y = wheeled.pose.pose.position.y
                t.transform.translation.z = wheeled.pose.pose.position.z
                t.transform.rotation = wheeled.pose.pose.orientation
                self._tf.sendTransform(t)

        if self._imu_pub is not None:
            now = time.monotonic()
            dt = max(1e-4, min(0.1, now - self._prev_t)) if self._prev_t else 0.02
            self._prev_t = now
            tw = msg.twist.twist
            q = msg.pose.pose.orientation
            packed = imu_from_odometry(
                qw=q.w,
                qx=q.x,
                qy=q.y,
                qz=q.z,
                wz=float(tw.angular.z),
                vx=float(tw.linear.x),
                vy=float(tw.linear.y),
                prev_vx=self._prev_vx,
                prev_vy=self._prev_vy,
                dt=dt,
            )
            self._prev_vx = float(tw.linear.x)
            self._prev_vy = float(tw.linear.y)
            imu = Imu()
            imu.header.stamp = msg.header.stamp
            imu.header.frame_id = self._imu_frame
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = packed[
                "orientation"
            ]
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = packed[
                "angular_velocity"
            ]
            (
                imu.linear_acceleration.x,
                imu.linear_acceleration.y,
                imu.linear_acceleration.z,
            ) = packed["linear_acceleration"]
            imu.orientation_covariance[0] = -1.0
            self._imu_pub.publish(imu)

        self._append_gt_path(wheeled)
        self._publish_error()

    def _append_gt_path(self, odom: Odometry) -> None:
        p = odom.pose.pose.position
        if self._path_gt:
            last = self._path_gt[-1].pose.position
            dx, dy = p.x - last.x, p.y - last.y
            if dx * dx + dy * dy < self._path_min_dist * self._path_min_dist:
                return
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self._path_gt.append(ps)
        if len(self._path_gt) > self._path_max:
            self._path_gt = self._path_gt[-self._path_max :]
        path = Path()
        path.header = odom.header
        path.poses = list(self._path_gt)
        self._path_pub.publish(path)

    def _publish_error(self) -> None:
        if self._gt is None or self._nav is None:
            return
        dx, dy, dyaw, dist = pose_xy_yaw_error(self._xyyaw(self._nav), self._xyyaw(self._gt))
        arr = Float32MultiArray()
        arr.data = [float(dx), float(dy), float(dyaw), float(dist)]
        self._err_pub.publish(arr)
        pose = PoseStamped()
        pose.header.stamp = self._gt.header.stamp
        pose.header.frame_id = self._odom_frame
        pose.pose.position.x = dx
        pose.pose.position.y = dy
        pose.pose.orientation.z = math.sin(dyaw * 0.5)
        pose.pose.orientation.w = math.cos(dyaw * 0.5)
        self._err_pose_pub.publish(pose)


def main() -> None:
    rclpy.init()
    node = SimNavContract()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
