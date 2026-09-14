#!/usr/bin/env python3
"""Mux wheel odom + visual odom onto /odom (sole TF odom→base).

Modes: auto | wheels | vo | fuse
  auto  — fuse if both fresh, else pass through the live source
  fuse  — complementary body vx/vy (xy_fusion), yaw from Mega IMU on /odom_wheel
Sources stay on /odom_wheel and /odom_vo for comparison (plus /path/*).
Optional gt_topic (sim bags / /odom_gt) → /path/gt + /odom_gt_error.
"""
from __future__ import annotations

import math
import time
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


MODES = ("auto", "wheels", "vo", "fuse")


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class OdomMux(Node):
    def __init__(self) -> None:
        super().__init__("odom_mux")
        self.declare_parameter("mode", "auto")
        self.declare_parameter("wheels_topic", "/odom_wheel")
        self.declare_parameter("vo_topic", "/odom_vo")
        self.declare_parameter("out_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("alpha_vo_fresh", 0.7)
        self.declare_parameter("vo_fresh_dt", 0.25)
        self.declare_parameter("source_fresh_dt", 0.35)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("path_hz", 5.0)
        self.declare_parameter("path_max", 1500)
        self.declare_parameter("path_min_dist", 0.02)
        self.declare_parameter("gt_topic", "")

        self._mode = str(self.get_parameter("mode").value).strip().lower()
        if self._mode not in MODES:
            self._mode = "auto"
        self._alpha = float(self.get_parameter("alpha_vo_fresh").value)
        self._vo_fresh_dt = float(self.get_parameter("vo_fresh_dt").value)
        self._src_dt = float(self.get_parameter("source_fresh_dt").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._path_max = int(self.get_parameter("path_max").value)
        self._path_min_dist = float(self.get_parameter("path_min_dist").value)

        self._wheel: Optional[Odometry] = None
        self._vo: Optional[Odometry] = None
        self._gt: Optional[Odometry] = None
        self._wheel_t = 0.0
        self._vo_t = 0.0
        self._last_mono = time.monotonic()
        self._fx = 0.0
        self._fy = 0.0
        self._seeded = False
        self._effective = self._mode
        self._last_out: Optional[Odometry] = None
        self._last_fused: Optional[Odometry] = None

        self._paths = {
            "wheel": deque(maxlen=self._path_max),
            "vo": deque(maxlen=self._path_max),
            "fused": deque(maxlen=self._path_max),
            "gt": deque(maxlen=self._path_max),
        }

        wheels_topic = str(self.get_parameter("wheels_topic").value)
        vo_topic = str(self.get_parameter("vo_topic").value)
        out_topic = str(self.get_parameter("out_topic").value)

        self._out_pub = self.create_publisher(Odometry, out_topic, 20)
        self._src_pub = self.create_publisher(String, "/odom_source", 10)
        self._path_pubs = {
            "wheel": self.create_publisher(Path, "/path/wheel", 10),
            "vo": self.create_publisher(Path, "/path/vo", 10),
            "fused": self.create_publisher(Path, "/path/fused", 10),
        }
        gt_topic = str(self.get_parameter("gt_topic").value).strip()
        if gt_topic:
            self._path_pubs["gt"] = self.create_publisher(Path, "/path/gt", 10)
            self._err_pub = self.create_publisher(Float32MultiArray, "/odom_gt_error", 10)
            self.create_subscription(Odometry, gt_topic, self._on_gt, 20)
        else:
            self._err_pub = None
        self._tf = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Odometry, wheels_topic, self._on_wheel, 20)
        self.create_subscription(Odometry, vo_topic, self._on_vo, 20)
        self.create_subscription(String, "/odom_mux/set_mode", self._on_set_mode, 10)
        self.create_service(Trigger, "/odom/zero", self._on_zero)
        self.add_on_set_parameters_callback(self._on_params)

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        path_hz = max(0.5, float(self.get_parameter("path_hz").value))
        self.create_timer(1.0 / rate, self._on_tick)
        self.create_timer(1.0 / path_hz, self._on_path)
        self.get_logger().info(
            f"odom_mux mode={self._mode} wheels={wheels_topic} vo={vo_topic} "
            f"gt={gt_topic or '-'} → {out_topic}"
        )

    def _on_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "mode":
                m = str(p.value).strip().lower()
                if m not in MODES:
                    return SetParametersResult(successful=False, reason=f"mode not in {MODES}")
                self._mode = m
                self.get_logger().info(f"mode → {m}")
        return SetParametersResult(successful=True)

    def _on_set_mode(self, msg: String) -> None:
        m = (msg.data or "").strip().lower()
        if m not in MODES:
            self.get_logger().warn(f"ignored mode '{msg.data}'")
            return
        self._mode = m
        self.get_logger().info(f"mode → {m}")

    def _on_zero(self, _req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        self._fx = self._fy = 0.0
        self._seeded = False
        for q in self._paths.values():
            q.clear()
        resp.success = True
        resp.message = "fused integrator + paths reset"
        return resp

    def _on_wheel(self, msg: Odometry) -> None:
        self._wheel = msg
        self._wheel_t = time.monotonic()

    def _on_vo(self, msg: Odometry) -> None:
        self._vo = msg
        self._vo_t = time.monotonic()

    def _on_gt(self, msg: Odometry) -> None:
        self._gt = msg

    def _fresh(self, t: float, now: float, dt: float) -> bool:
        return t > 0.0 and (now - t) < dt

    def _resolve(self, now: float) -> str:
        w = self._fresh(self._wheel_t, now, self._src_dt)
        v = self._fresh(self._vo_t, now, self._vo_fresh_dt)
        if self._mode == "auto":
            if w and v:
                return "fuse"
            if v:
                return "vo"
            if w:
                return "wheels"
            return "none"
        if self._mode == "wheels":
            return "wheels" if w else "none"
        if self._mode == "vo":
            return "vo" if v else "none"
        if w or v:
            return "fuse"
        return "none"

    def _seed_from_wheel(self) -> None:
        if self._wheel is None:
            return
        p = self._wheel.pose.pose.position
        self._fx = float(p.x)
        self._fy = float(p.y)
        self._seeded = True

    def _fuse_msg(self, now_msg, dt: float) -> Optional[Odometry]:
        if self._wheel is None and self._vo is None:
            return None
        if not self._seeded:
            if self._wheel is not None:
                self._seed_from_wheel()
            elif self._vo is not None:
                p = self._vo.pose.pose.position
                self._fx = float(p.x)
                self._fy = float(p.y)
                self._seeded = True
            else:
                return None

        now = time.monotonic()
        vo_fresh = self._fresh(self._vo_t, now, self._vo_fresh_dt)
        a = self._alpha if vo_fresh else 0.0
        vx_w = float(self._wheel.twist.twist.linear.x) if self._wheel else 0.0
        vy_w = float(self._wheel.twist.twist.linear.y) if self._wheel else 0.0
        vx_v = float(self._vo.twist.twist.linear.x) if self._vo else 0.0
        vy_v = float(self._vo.twist.twist.linear.y) if self._vo else 0.0
        vx_b = a * vx_v + (1.0 - a) * vx_w
        vy_b = a * vy_v + (1.0 - a) * vy_w

        if self._wheel is not None:
            q = self._wheel.pose.pose.orientation
            yaw = yaw_from_quat(q.w, q.x, q.y, q.z)
            wz = float(self._wheel.twist.twist.angular.z)
            orient = q
        elif self._vo is not None:
            q = self._vo.pose.pose.orientation
            yaw = yaw_from_quat(q.w, q.x, q.y, q.z)
            wz = float(self._vo.twist.twist.angular.z)
            orient = q
        else:
            return None

        c, s = math.cos(yaw), math.sin(yaw)
        self._fx += (c * vx_b - s * vy_b) * dt
        self._fy += (s * vx_b + c * vy_b) * dt

        msg = Odometry()
        msg.header.stamp = now_msg
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._fx
        msg.pose.pose.position.y = self._fy
        msg.pose.pose.orientation = orient
        msg.twist.twist.linear.x = vx_b
        msg.twist.twist.linear.y = vy_b
        msg.twist.twist.angular.z = wz
        return msg

    def _passthrough(self, src: Odometry, now_msg) -> Odometry:
        out = Odometry()
        out.header.stamp = now_msg
        out.header.frame_id = self._odom_frame
        out.child_frame_id = self._base_frame
        out.pose = src.pose
        out.twist = src.twist
        return out

    def _xyyaw(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return (float(p.x), float(p.y), yaw_from_quat(q.w, q.x, q.y, q.z))

    def _publish_gt_error(self, nav: Odometry) -> None:
        if self._err_pub is None or self._gt is None:
            return
        nx, ny, nyaw = self._xyyaw(nav)
        gx, gy, gyaw = self._xyyaw(self._gt)
        dx, dy = gx - nx, gy - ny
        dyaw = math.atan2(math.sin(gyaw - nyaw), math.cos(gyaw - nyaw))
        arr = Float32MultiArray()
        arr.data = [float(dx), float(dy), float(dyaw), float(math.hypot(dx, dy))]
        self._err_pub.publish(arr)

    def _publish_out(self, msg: Odometry, source: str) -> None:
        self._last_out = msg
        self._out_pub.publish(msg)
        s = String()
        s.data = source
        self._src_pub.publish(s)
        self._publish_gt_error(msg)
        if self._tf is None:
            return
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf.sendTransform(t)

    def _on_tick(self) -> None:
        now_msg = self.get_clock().now().to_msg()
        now = time.monotonic()
        dt = max(1e-4, min(0.1, now - self._last_mono))
        self._last_mono = now

        fused = self._fuse_msg(now_msg, dt)
        if fused is not None:
            self._last_fused = fused

        effective = self._resolve(now)
        self._effective = effective
        if effective == "none":
            return
        if effective == "wheels" and self._wheel is not None:
            self._publish_out(self._passthrough(self._wheel, now_msg), "wheels")
            return
        if effective == "vo" and self._vo is not None:
            self._publish_out(self._passthrough(self._vo, now_msg), "vo")
            return
        if fused is not None:
            self._publish_out(fused, "fuse")

    def _append_path(self, key: str, odom: Optional[Odometry]) -> None:
        if odom is None:
            return
        q = self._paths[key]
        p = odom.pose.pose.position
        if q:
            last = q[-1].pose.position
            dx, dy = p.x - last.x, p.y - last.y
            if dx * dx + dy * dy < self._path_min_dist * self._path_min_dist:
                return
        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        q.append(ps)

    def _on_path(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self._append_path("wheel", self._wheel)
        self._append_path("vo", self._vo)
        self._append_path("fused", self._last_fused)
        if "gt" in self._path_pubs:
            self._append_path("gt", self._gt)
        for key, pub in self._path_pubs.items():
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = self._odom_frame
            path.poses = list(self._paths[key])
            pub.publish(path)


def main() -> None:
    rclpy.init()
    node = OdomMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
