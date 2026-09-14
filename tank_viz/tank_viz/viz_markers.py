#!/usr/bin/env python3
"""RViz MarkerArray: cmd_vel arrows, pan/tilt cues, FOV frustums, status text."""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


def _pt(x: float, y: float, z: float = 0.0) -> Point:
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _set_scale(m: Marker, x: float, y: float = 0.0, z: float = 0.0) -> None:
    m.scale.x = float(x)
    m.scale.y = float(y)
    m.scale.z = float(z)


class VizMarkers(Node):
    def __init__(self) -> None:
        super().__init__("viz_markers")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pantilt_cmd_vel_topic", "/pantilt/cmd_vel")
        self.declare_parameter("pantilt_js_topic", "/pantilt/joint_states")
        self.declare_parameter("pantilt_mode_topic", "/pantilt/mode_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("csi_frame", "CSI_link")
        self.declare_parameter("realsense_frame", "realsense_link")
        self.declare_parameter("csi_hfov_deg", 62.2)
        self.declare_parameter("realsense_hfov_deg", 69.0)
        self.declare_parameter("frustum_depth_m", 4.0)
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")

        rate = float(self.get_parameter("rate_hz").value)
        self._base = self.get_parameter("base_frame").get_parameter_value().string_value
        self._csi = self.get_parameter("csi_frame").get_parameter_value().string_value
        self._rs = self.get_parameter("realsense_frame").get_parameter_value().string_value
        self._csi_hfov = math.radians(float(self.get_parameter("csi_hfov_deg").value))
        self._rs_hfov = math.radians(float(self.get_parameter("realsense_hfov_deg").value))
        self._frustum_d = float(self.get_parameter("frustum_depth_m").value)
        self._pan_name = self.get_parameter("pan_joint").get_parameter_value().string_value
        self._tilt_name = self.get_parameter("tilt_joint").get_parameter_value().string_value

        self._cmd_vel = Twist()
        self._pt_dps = (0.0, 0.0)
        self._meas: Dict[str, float] = {}
        self._mode = ""
        self._odom: Optional[Odometry] = None

        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self._on_cmd,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("pantilt_cmd_vel_topic").get_parameter_value().string_value,
            self._on_pt_cmd,
            10,
        )
        self.create_subscription(
            JointState,
            self.get_parameter("pantilt_js_topic").get_parameter_value().string_value,
            self._on_js,
            10,
        )
        self.create_subscription(
            String,
            self.get_parameter("pantilt_mode_topic").get_parameter_value().string_value,
            self._on_mode,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self._on_odom,
            10,
        )

        self._pub = self.create_publisher(MarkerArray, "/viz/markers", 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

    def _on_cmd(self, msg: Twist) -> None:
        self._cmd_vel = msg

    def _on_pt_cmd(self, msg: Twist) -> None:
        self._pt_dps = (float(msg.angular.z), float(msg.angular.y))

    def _on_js(self, msg: JointState) -> None:
        for n, p in zip(msg.name, msg.position):
            self._meas[n] = float(p)

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _mk(self, ns: str, mid: int, mtype: int, frame: str) -> Marker:
        m = Marker()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = frame
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200_000_000
        return m

    def _cmd_vel_markers(self) -> List[Marker]:
        out: List[Marker] = []
        vx = float(self._cmd_vel.linear.x)
        vy = float(self._cmd_vel.linear.y)
        wz = float(self._cmd_vel.angular.z)
        scale = 1.5  # m per (m/s)

        arrow = self._mk("cmd_vel", 0, Marker.ARROW, self._base)
        arrow.points = [_pt(0.0, 0.0, 0.05), _pt(vx * scale, vy * scale, 0.05)]
        _set_scale(arrow, 0.03, 0.06)
        arrow.color = _color(1.0, 0.6, 0.0, 0.9)
        out.append(arrow)

        # yaw arc as LINE_STRIP
        arc = self._mk("cmd_vel", 1, Marker.LINE_STRIP, self._base)
        r = 0.25
        n = 16
        span = _clamp(wz * 0.5, -math.pi / 2, math.pi / 2)
        for i in range(n + 1):
            a = span * i / n
            arc.points.append(_pt(r * math.cos(a), r * math.sin(a), 0.08))
        _set_scale(arc, 0.02)
        arc.color = _color(1.0, 0.2, 0.2, 0.8)
        out.append(arc)
        return out

    def _pantilt_markers(self) -> List[Marker]:
        out: List[Marker] = []
        pan = self._meas.get(self._pan_name, 0.0)
        tilt = self._meas.get(self._tilt_name, 0.0)
        # measured direction (green) — approx in CSI frame: +X forward optical-ish
        meas = self._mk("pantilt", 0, Marker.ARROW, self._csi)
        meas.points = [_pt(0.0, 0.0, 0.0), _pt(0.4, 0.0, 0.0)]
        _set_scale(meas, 0.02, 0.04)
        meas.color = _color(0.1, 0.9, 0.2, 0.9)
        out.append(meas)

        # commanded preview offset in pan (cyan) — body yaw of CSI approx
        cmd_pan = pan + math.radians(self._pt_dps[0]) * 1.0
        cmd_tilt = tilt + math.radians(self._pt_dps[1]) * 1.0
        dx = 0.4 * math.cos(cmd_pan - pan)
        dy = 0.4 * math.sin(cmd_pan - pan)
        dz = 0.15 * math.sin(cmd_tilt - tilt)
        cmd = self._mk("pantilt", 1, Marker.ARROW, self._csi)
        cmd.points = [_pt(0.0, 0.0, 0.0), _pt(dx, dy, dz)]
        _set_scale(cmd, 0.02, 0.04)
        cmd.color = _color(0.1, 0.8, 1.0, 0.9)
        out.append(cmd)
        return out

    def _frustum(self, ns: str, mid: int, frame: str, hfov: float) -> Marker:
        # Optical frame: Z forward, X right, Y down — but CSI_link / realsense_link
        # are SW optical-ish. Draw in link frame with +X forward for readability.
        d = self._frustum_d
        half = hfov * 0.5
        # aspect ~ 16:9 → vfov
        vfov = 2.0 * math.atan(math.tan(half) * 9.0 / 16.0)
        vhalf = vfov * 0.5
        corners = [
            _pt(d, d * math.tan(half), d * math.tan(vhalf)),
            _pt(d, -d * math.tan(half), d * math.tan(vhalf)),
            _pt(d, -d * math.tan(half), -d * math.tan(vhalf)),
            _pt(d, d * math.tan(half), -d * math.tan(vhalf)),
        ]
        m = self._mk(ns, mid, Marker.LINE_LIST, frame)
        origin = _pt(0.0, 0.0, 0.0)
        for c in corners:
            m.points.extend([origin, c])
        for i in range(4):
            m.points.extend([corners[i], corners[(i + 1) % 4]])
        _set_scale(m, 0.01)
        m.color = _color(0.7, 0.7, 1.0, 0.5)
        return m

    def _text_marker(self) -> Marker:
        m = self._mk("status", 0, Marker.TEXT_VIEW_FACING, self._base)
        m.pose.position.z = 0.45
        pan = math.degrees(self._meas.get(self._pan_name, 0.0))
        tilt = math.degrees(self._meas.get(self._tilt_name, 0.0))
        vx = self._cmd_vel.linear.x
        vy = self._cmd_vel.linear.y
        wz = self._cmd_vel.angular.z
        lines = [
            f"mode={self._mode or '-'}",
            f"pan={pan:+.1f}° tilt={tilt:+.1f}°",
            f"pt_cmd={self._pt_dps[0]:+.0f}/{self._pt_dps[1]:+.0f} dps",
            f"v=({vx:+.2f},{vy:+.2f}) ω={wz:+.2f}",
        ]
        m.text = "\n".join(lines)
        m.scale.z = 0.06
        m.color = _color(1.0, 1.0, 1.0, 0.95)
        return m

    def _tick(self) -> None:
        arr = MarkerArray()
        arr.markers.extend(self._cmd_vel_markers())
        arr.markers.extend(self._pantilt_markers())
        arr.markers.append(self._frustum("fov", 0, self._csi, self._csi_hfov))
        arr.markers.append(self._frustum("fov", 1, self._rs, self._rs_hfov))
        arr.markers.append(self._text_marker())
        self._pub.publish(arr)


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def main() -> None:
    rclpy.init()
    node = VizMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
