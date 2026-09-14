#!/usr/bin/env python3
"""Merge partial joint sources into a full /joint_states for robot_state_publisher."""
from __future__ import annotations

import math
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class JointStateAggregator(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_aggregator")
        self.declare_parameter("sources", ["/pantilt/joint_states"])
        self.declare_parameter("wheel_omega_topic", "/mega/wheel_omega")
        self.declare_parameter(
            "wheel_joints",
            ["FL_joint", "FR_wheel_joint", "RL_joint", "RR_joint"],
        )
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
        # CAD rest (all URDF joints 0) = HW pantilt (0, -45°).
        # pan_urdf = -pan_hw ; tilt_urdf = tilt_hw + 45°.
        self.declare_parameter(
            "joint_offsets_rad",
            [0.0, 0.0, 0.0, 0.0, 0.0, math.pi / 4.0],
        )
        self.declare_parameter(
            "joint_scales",
            [1.0, 1.0, 1.0, 1.0, -1.0, 1.0],
        )
        self.declare_parameter("rate_hz", 30.0)

        self._joint_names: List[str] = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        self._wheel_joints: List[str] = list(
            self.get_parameter("wheel_joints").get_parameter_value().string_array_value
        )
        offs = list(
            self.get_parameter("joint_offsets_rad").get_parameter_value().double_array_value
        )
        scales = list(
            self.get_parameter("joint_scales").get_parameter_value().double_array_value
        )
        self._offset: Dict[str, float] = {
            n: float(offs[i]) if i < len(offs) else 0.0
            for i, n in enumerate(self._joint_names)
        }
        self._scale: Dict[str, float] = {
            n: float(scales[i]) if i < len(scales) else 1.0
            for i, n in enumerate(self._joint_names)
        }
        sources = list(self.get_parameter("sources").get_parameter_value().string_array_value)
        omega_topic = (
            self.get_parameter("wheel_omega_topic").get_parameter_value().string_value
        )
        rate = float(self.get_parameter("rate_hz").value)

        self._pos: Dict[str, float] = {n: 0.0 for n in self._joint_names}
        self._last_omega: Optional[List[float]] = None
        self._last_t = self.get_clock().now()

        for topic in sources:
            self.create_subscription(JointState, topic, self._on_js, 10)
        self.create_subscription(Float32MultiArray, omega_topic, self._on_omega, 10)
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f"aggregating sources={sources} omega={omega_topic} "
            f"scales={self._scale} offsets={self._offset} → /joint_states"
        )

    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in self._pos:
                self._pos[name] = (
                    float(pos) * self._scale.get(name, 1.0) + self._offset.get(name, 0.0)
                )
        self._publish(self.get_clock().now())

    def _on_omega(self, msg: Float32MultiArray) -> None:
        self._last_omega = [float(v) for v in msg.data]

    def _integrate_wheels(self, dt: float) -> None:
        if not self._last_omega or dt <= 0.0:
            return
        for i, jname in enumerate(self._wheel_joints):
            if i >= len(self._last_omega) or jname not in self._pos:
                continue
            self._pos[jname] = math.fmod(
                self._pos[jname] + self._last_omega[i] * dt, 2.0 * math.pi
            )

    def _publish(self, now) -> None:
        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = list(self._joint_names)
        js.position = [self._pos[n] for n in self._joint_names]
        self._pub.publish(js)

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self._last_t).nanoseconds * 1e-9
        self._last_t = now
        self._integrate_wheels(dt)
        self._publish(now)


def main() -> None:
    rclpy.init()
    node = JointStateAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
