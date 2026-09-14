#!/usr/bin/env python3
"""Publish a command-preview ghost robot (joint states + TF)."""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from trajectory_msgs.msg import JointTrajectory


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def _quat_from_yaw(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class GhostPublisher(Node):
    def __init__(self) -> None:
        super().__init__("ghost_publisher")
        self.declare_parameter("prefix", "cmd_")
        self.declare_parameter("preview_s", 1.0)
        self.declare_parameter("hold_s", 0.5)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("pantilt_cmd_vel_topic", "/pantilt/cmd_vel")
        self.declare_parameter("pantilt_traj_topic", "/pantilt/joint_trajectory")
        self.declare_parameter("pantilt_js_topic", "/pantilt/joint_states")
        self.declare_parameter(
            "joint_names",
            [
                "FL_joint",
                "FR_wheel_joint",
                "RL_joint",
                "RR_joint",
                "pan_joint",
                "tilt_joint",
            ],
        )
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")
        self.declare_parameter("pan_limit_rad", math.pi)
        self.declare_parameter("tilt_min_rad", math.radians(-75.0))
        self.declare_parameter("tilt_max_rad", 0.0)
        self.declare_parameter("pan_offset_rad", 0.0)
        self.declare_parameter("tilt_offset_rad", math.pi / 4.0)
        self.declare_parameter("pan_scale", -1.0)
        self.declare_parameter("tilt_scale", 1.0)

        self._prefix = self.get_parameter("prefix").get_parameter_value().string_value
        self._preview_s = float(self.get_parameter("preview_s").value)
        self._hold_s = float(self.get_parameter("hold_s").value)
        rate = float(self.get_parameter("rate_hz").value)
        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._pan_name = self.get_parameter("pan_joint").get_parameter_value().string_value
        self._tilt_name = self.get_parameter("tilt_joint").get_parameter_value().string_value
        self._pan_lim = float(self.get_parameter("pan_limit_rad").value)
        self._tilt_min = float(self.get_parameter("tilt_min_rad").value)
        self._tilt_max = float(self.get_parameter("tilt_max_rad").value)
        self._pan_off = float(self.get_parameter("pan_offset_rad").value)
        self._tilt_off = float(self.get_parameter("tilt_offset_rad").value)
        self._pan_scale = float(self.get_parameter("pan_scale").value)
        self._tilt_scale = float(self.get_parameter("tilt_scale").value)

        self._meas: Dict[str, float] = {n: 0.0 for n in self._joint_names}
        # Before first /pantilt/joint_states: HW park (−45°) = CAD rest.
        self._meas[self._tilt_name] = -self._tilt_off
        self._cmd_abs_pan: Optional[float] = None
        self._cmd_abs_tilt: Optional[float] = None
        self._pt_dps = (0.0, 0.0)  # pan, tilt deg/s
        self._cmd_vel = Twist()
        self._last_pt_cmd_t: Optional[rclpy.time.Time] = None
        self._last_base_cmd_t: Optional[rclpy.time.Time] = None
        self._odom: Optional[Odometry] = None

        self.create_subscription(
            JointState,
            self.get_parameter("pantilt_js_topic").get_parameter_value().string_value,
            self._on_js,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("pantilt_cmd_vel_topic").get_parameter_value().string_value,
            self._on_pt_cmd,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            self.get_parameter("pantilt_traj_topic").get_parameter_value().string_value,
            self._on_traj,
            10,
        )
        self.create_subscription(
            Twist,
            self.get_parameter("cmd_vel_topic").get_parameter_value().string_value,
            self._on_cmd_vel,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").get_parameter_value().string_value,
            self._on_odom,
            10,
        )

        self._js_pub = self.create_publisher(JointState, "/cmd/joint_states", 10)
        self._tf_br = TransformBroadcaster(self)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"ghost prefix={self._prefix!r} preview={self._preview_s}s → /cmd/joint_states"
        )

    def _now(self):
        return self.get_clock().now()

    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._meas:
                self._meas[name] = float(pos)

    def _on_pt_cmd(self, msg: Twist) -> None:
        # angular.z = pan_dps, angular.y = tilt_dps (tank_pantilt convention)
        self._pt_dps = (float(msg.angular.z), float(msg.angular.y))
        self._cmd_abs_pan = None
        self._cmd_abs_tilt = None
        self._last_pt_cmd_t = self._now()

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        pt = msg.points[0]
        for i, name in enumerate(msg.joint_names):
            if i >= len(pt.positions):
                break
            v = float(pt.positions[i])
            if abs(v) <= math.pi + 0.1:
                pass  # already rad
            else:
                v = math.radians(v)
            if name == self._pan_name:
                self._cmd_abs_pan = v
            elif name == self._tilt_name:
                self._cmd_abs_tilt = v
        self._pt_dps = (0.0, 0.0)
        self._last_pt_cmd_t = self._now()

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._cmd_vel = msg
        self._last_base_cmd_t = self._now()

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _pt_fresh(self) -> bool:
        if self._last_pt_cmd_t is None:
            return False
        age = (self._now() - self._last_pt_cmd_t).nanoseconds * 1e-9
        return age <= self._hold_s

    def _base_fresh(self) -> bool:
        if self._last_base_cmd_t is None:
            return False
        age = (self._now() - self._last_base_cmd_t).nanoseconds * 1e-9
        return age <= self._hold_s

    def _ghost_pan_tilt(self) -> tuple[float, float]:
        pan = self._meas.get(self._pan_name, 0.0)
        tilt = self._meas.get(self._tilt_name, 0.0)
        if not self._pt_fresh():
            return pan, tilt
        if self._cmd_abs_pan is not None:
            pan = self._cmd_abs_pan
        else:
            pan = pan + math.radians(self._pt_dps[0]) * self._preview_s
        if self._cmd_abs_tilt is not None:
            tilt = self._cmd_abs_tilt
        else:
            tilt = tilt + math.radians(self._pt_dps[1]) * self._preview_s
        pan = _clamp(pan, -self._pan_lim, self._pan_lim)
        tilt = _clamp(tilt, self._tilt_min, self._tilt_max)
        return pan, tilt

    def _tick(self) -> None:
        now = self._now()
        stamp = now.to_msg()
        pan, tilt = self._ghost_pan_tilt()

        positions = []
        for n in self._joint_names:
            if n == self._pan_name:
                positions.append(self._pan_scale * pan + self._pan_off)
            elif n == self._tilt_name:
                positions.append(self._tilt_scale * tilt + self._tilt_off)
            else:
                positions.append(self._meas.get(n, 0.0))

        js = JointState()
        js.header.stamp = stamp
        js.name = [self._prefix + n for n in self._joint_names]
        js.position = positions
        self._js_pub.publish(js)

        t = TransformStamped()
        t.header.stamp = stamp
        child = self._prefix + "base_footprint"

        if self._odom is not None and self._base_fresh():
            ox = self._odom.pose.pose.position.x
            oy = self._odom.pose.pose.position.y
            oq = self._odom.pose.pose.orientation
            yaw = _yaw_from_quat(oq.x, oq.y, oq.z, oq.w)
            vx = float(self._cmd_vel.linear.x)
            vy = float(self._cmd_vel.linear.y)
            wz = float(self._cmd_vel.angular.z)
            dt = self._preview_s
            # body-frame cmd → world
            dx = (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
            dy = (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
            t.header.frame_id = self._odom.header.frame_id or "odom"
            t.child_frame_id = child
            t.transform.translation.x = ox + dx
            t.transform.translation.y = oy + dy
            t.transform.translation.z = 0.0
            t.transform.rotation = _quat_from_yaw(yaw + wz * dt)
        elif self._odom is not None:
            t.header.frame_id = self._odom.header.frame_id or "odom"
            t.child_frame_id = child
            t.transform.translation.x = self._odom.pose.pose.position.x
            t.transform.translation.y = self._odom.pose.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = self._odom.pose.pose.orientation
        else:
            # pantilt-only: park ghost on measured base_footprint
            t.header.frame_id = "base_footprint"
            t.child_frame_id = child
            t.transform.rotation.w = 1.0

        self._tf_br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = GhostPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
