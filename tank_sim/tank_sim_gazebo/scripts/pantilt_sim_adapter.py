#!/usr/bin/env python3
"""Present the real pantilt contract on top of Gazebo JointTrajectoryController.

Real (ESP):  /pantilt/cmd_vel, /pantilt/joint_trajectory, /pantilt/joint_states,
             /pantilt/home, /pantilt/stop, /pantilt/mode_state
Sim (GZ):    /set_joint_trajectory  →  /model/<name>/joint_trajectory
             pan/tilt in /joint_states

A nav / track node must keep using /pantilt/*.
"""
from __future__ import annotations

import os
import sys
import time

# Same install dir as this script (parity_lib.py).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from parity_lib import (
    PAN_LIMIT_RAD,
    TILT_LIMIT_RAD,
    clamp,
    extract_named_positions,
    integrate_dps,
    joint_value_to_rad,
)


class PantiltSimAdapter(Node):
    def __init__(self) -> None:
        super().__init__("pantilt_sim_adapter")
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")
        self.declare_parameter("pan_limit_rad", PAN_LIMIT_RAD)
        self.declare_parameter("tilt_limit_rad", TILT_LIMIT_RAD)
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_s", 0.25)
        self.declare_parameter("gz_traj_topic", "/set_joint_trajectory")
        self.declare_parameter("js_in_topic", "/joint_states")

        self._pan_name = str(self.get_parameter("pan_joint").value)
        self._tilt_name = str(self.get_parameter("tilt_joint").value)
        self._pan_lim = float(self.get_parameter("pan_limit_rad").value)
        self._tilt_lim = float(self.get_parameter("tilt_limit_rad").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_s").value)

        self._pan = 0.0
        self._tilt = 0.0
        self._have_js = False
        self._pan_dps = 0.0
        self._tilt_dps = 0.0
        self._last_cmd_mono = 0.0
        self._last_tick = time.monotonic()
        self._mode = "RUNNING"

        self._js_pub = self.create_publisher(JointState, "/pantilt/joint_states", 10)
        self._mode_pub = self.create_publisher(String, "/pantilt/mode_state", 10)
        self._traj_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("gz_traj_topic").value), 10
        )

        self.create_subscription(
            JointState, str(self.get_parameter("js_in_topic").value), self._on_js, 20
        )
        self.create_subscription(Twist, "/pantilt/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            JointTrajectory, "/pantilt/joint_trajectory", self._on_traj, 10
        )
        self.create_subscription(String, "/pantilt/mode", self._on_mode, 10)
        self.create_service(Trigger, "/pantilt/home", self._srv_home)
        self.create_service(Trigger, "/pantilt/stop", self._srv_stop)

        rate = max(5.0, float(self.get_parameter("rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            "pantilt_sim_adapter: /pantilt/* → /set_joint_trajectory "
            f"(pan±{self._pan_lim:.2f} tilt±{self._tilt_lim:.2f})"
        )

    def _on_js(self, msg: JointState) -> None:
        got = extract_named_positions(msg.name, msg.position, (self._pan_name, self._tilt_name))
        if self._pan_name in got:
            self._pan = got[self._pan_name]
        if self._tilt_name in got:
            self._tilt = got[self._tilt_name]
        if got:
            self._have_js = True
        stamp = msg.header.stamp
        js = JointState()
        js.header.stamp = stamp
        js.name = [self._pan_name, self._tilt_name]
        js.position = [self._pan, self._tilt]
        self._js_pub.publish(js)

    def _on_cmd_vel(self, msg: Twist) -> None:
        # convention: angular.z = pan_dps, angular.y = tilt_dps
        self._pan_dps = float(msg.angular.z)
        self._tilt_dps = float(msg.angular.y)
        self._last_cmd_mono = time.monotonic()

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        names = list(msg.joint_names) if msg.joint_names else [self._pan_name, self._tilt_name]
        pos = list(msg.points[0].positions)
        pan, tilt = self._pan, self._tilt
        for i, name in enumerate(names):
            if i >= len(pos):
                break
            v = joint_value_to_rad(float(pos[i]))
            if name == self._pan_name:
                pan = v
            elif name == self._tilt_name:
                tilt = v
        self._pan_dps = 0.0
        self._tilt_dps = 0.0
        self._last_cmd_mono = 0.0
        self._send_abs(pan, tilt)

    def _on_mode(self, msg: String) -> None:
        mode = (msg.data or "").strip().upper()
        if not mode:
            return
        self._mode = mode
        if mode in ("IDLE", "STOP"):
            self._pan_dps = 0.0
            self._tilt_dps = 0.0
            self._send_abs(self._pan, self._tilt)

    def _srv_home(self, _req, resp):
        self._pan_dps = 0.0
        self._tilt_dps = 0.0
        self._last_cmd_mono = 0.0
        self._send_abs(0.0, 0.0)
        self._mode = "RUNNING"
        resp.success = True
        resp.message = "sim home → pan=0 tilt=0"
        return resp

    def _srv_stop(self, _req, resp):
        self._pan_dps = 0.0
        self._tilt_dps = 0.0
        self._last_cmd_mono = 0.0
        self._send_abs(self._pan, self._tilt)
        self._mode = "IDLE"
        resp.success = True
        resp.message = "sim stop — hold current"
        return resp

    def _send_abs(self, pan: float, tilt: float) -> None:
        pan = clamp(pan, -self._pan_lim, self._pan_lim)
        tilt = clamp(tilt, -self._tilt_lim, self._tilt_lim)
        self._pan, self._tilt = pan, tilt
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = [self._pan_name, self._tilt_name]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points = [pt]
        self._traj_pub.publish(traj)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, min(0.1, now - self._last_tick))
        self._last_tick = now
        if self._last_cmd_mono > 0.0 and (now - self._last_cmd_mono) <= self._cmd_timeout:
            pan = integrate_dps(self._pan, self._pan_dps, dt, -self._pan_lim, self._pan_lim)
            tilt = integrate_dps(self._tilt, self._tilt_dps, dt, -self._tilt_lim, self._tilt_lim)
            self._send_abs(pan, tilt)
        s = String()
        s.data = self._mode
        self._mode_pub.publish(s)


def main() -> None:
    rclpy.init()
    node = PantiltSimAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
