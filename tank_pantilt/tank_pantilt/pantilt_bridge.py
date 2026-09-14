#!/usr/bin/env python3
"""ESP32 pantilt bridge: joint_states + cmd interfaces. Reconnects if serial missing.

Does NOT own CSI or VLM — those stay in pantilt_slave backend or csi_* nodes.
WARNING: do not run pantilt_slave ESP control AND this bridge on the same port.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial required") from exc

from tank_pantilt import esp_protocol as pt
from tank_pantilt.hw_probe import list_serial_ports, resolve_esp_port


class PantiltBridge(Node):
    def __init__(self) -> None:
        super().__init__("pantilt_bridge")
        self.declare_parameter("serial_port", "auto")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("pan_joint", "pan_joint")
        self.declare_parameter("tilt_joint", "tilt_joint")
        self.declare_parameter("status_hz", 10.0)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("auto_running", True)

        self._port_pref = str(self.get_parameter("serial_port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._pan_name = str(self.get_parameter("pan_joint").value)
        self._tilt_name = str(self.get_parameter("tilt_joint").value)
        self._status_hz = float(self.get_parameter("status_hz").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period_s").value)
        self._auto_running = bool(self.get_parameter("auto_running").value)

        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()
        self._rx_buf = bytearray()
        self._stop = threading.Event()
        self._seq = 0
        self._last_miss_log = 0.0

        self._pan_deg = 0.0
        self._tilt_deg = 0.0
        self._mode = "UNKNOWN"
        self._have_status = False

        self._js_pub = self.create_publisher(JointState, "/pantilt/joint_states", 10)
        self._mode_pub = self.create_publisher(String, "/pantilt/mode_state", 10)

        self.create_subscription(Twist, "/pantilt/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(JointTrajectory, "/pantilt/joint_trajectory", self._on_traj, 10)
        self.create_subscription(String, "/pantilt/mode", self._on_mode, 10)

        self.create_service(Trigger, "/pantilt/home", self._srv_home)
        self.create_service(Trigger, "/pantilt/stop", self._srv_stop)

        if not self._try_connect(log_miss=True):
            self.get_logger().warn(
                f"[esp] MISSING — waiting ({self._port_pref}); "
                f"candidates={list_serial_ports() or 'none'}"
            )

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        self.create_timer(1.0 / max(self._status_hz, 1.0), self._status_tick)
        self.create_timer(max(self._reconnect_period, 0.5), self._reconnect_tick)

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return self._seq

    def _try_connect(self, log_miss: bool = False) -> bool:
        port = resolve_esp_port(self._port_pref)
        if port is None:
            if log_miss:
                self.get_logger().warn(
                    f"[esp] port not found (pref={self._port_pref}, "
                    f"seen={list_serial_ports() or 'none'})"
                )
            return False
        try:
            ser = serial.Serial(port, self._baud, timeout=0.02)
        except serial.SerialException as exc:
            if log_miss:
                self.get_logger().warn(f"[esp] open failed {port}: {exc}")
            return False
        time.sleep(0.2)
        try:
            ser.reset_input_buffer()
            if self._auto_running:
                ser.write(pt.cmd_mode(self._next_seq(), "RUNNING"))
            ser.write(pt.cmd_status_req(self._next_seq()))
        except serial.SerialException as exc:
            self.get_logger().warn(f"[esp] configure failed: {exc}")
            try:
                ser.close()
            except Exception:
                pass
            return False
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = ser
            self._rx_buf.clear()
        self.get_logger().info(f"[esp] OK connected {port}@{self._baud}")
        return True

    def _close_serial(self, reason: str) -> None:
        with self._ser_lock:
            if self._ser is None:
                return
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
            self._rx_buf.clear()
        self.get_logger().warn(f"[esp] disconnected ({reason}); will retry")

    def _reconnect_tick(self) -> None:
        with self._ser_lock:
            ok = self._ser is not None and self._ser.is_open
        if ok:
            return
        now = time.monotonic()
        log_miss = (now - self._last_miss_log) > 10.0
        if log_miss:
            self._last_miss_log = now
        self._try_connect(log_miss=log_miss)

    def _write(self, payload: bytes) -> None:
        with self._ser_lock:
            ser = self._ser
        if ser is None or not ser.is_open:
            return
        try:
            ser.write(payload)
        except serial.SerialException as exc:
            self._close_serial(f"write: {exc}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        # convention: angular.z = pan_dps, angular.y = tilt_dps
        self._write(pt.cmd_vel(self._next_seq(), float(msg.angular.z), float(msg.angular.y)))

    def _on_traj(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        names = list(msg.joint_names) if msg.joint_names else [self._pan_name, self._tilt_name]
        pos = list(msg.points[0].positions)
        pan = self._pan_deg
        tilt = self._tilt_deg
        for i, name in enumerate(names):
            if i >= len(pos):
                break
            # JointTrajectory often in radians — convert if |value| looks like rad
            v = float(pos[i])
            if abs(v) <= math.pi + 0.1:
                v = math.degrees(v)
            if name == self._pan_name:
                pan = v
            elif name == self._tilt_name:
                tilt = v
        self._write(pt.cmd_ang(self._next_seq(), pan, tilt))

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip().upper()
        if mode not in pt.MODE_BY_NAME:
            self.get_logger().warn(f"unknown mode '{msg.data}'")
            return
        self._write(pt.cmd_mode(self._next_seq(), mode))

    def _esp_connected(self) -> bool:
        with self._ser_lock:
            return self._ser is not None and self._ser.is_open

    def _srv_home(self, _req, resp):
        if not self._esp_connected():
            resp.success = False
            resp.message = "ESP not connected"
            return resp
        self._write(pt.cmd_home(self._next_seq()))
        resp.success = True
        resp.message = "HOME sent (homing sequence)"
        self.get_logger().info("[esp] HOME requested")
        return resp

    def _srv_stop(self, _req, resp):
        if not self._esp_connected():
            resp.success = False
            resp.message = "ESP not connected"
            return resp
        self._write(pt.cmd_stop(self._next_seq()))
        resp.success = True
        resp.message = "STOP sent"
        return resp

    def _status_tick(self) -> None:
        self._write(pt.cmd_status_req(self._next_seq()))
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        js.name = [self._pan_name, self._tilt_name]
        js.position = [math.radians(self._pan_deg), math.radians(self._tilt_deg)]
        self._js_pub.publish(js)
        m = String()
        m.data = self._mode
        self._mode_pub.publish(m)

    def _rx_loop(self) -> None:
        while not self._stop.is_set() and rclpy.ok():
            with self._ser_lock:
                ser = self._ser
            if ser is None or not ser.is_open:
                time.sleep(0.1)
                continue
            try:
                chunk = ser.read(256)
            except serial.SerialException as exc:
                self._close_serial(f"read: {exc}")
                continue
            if not chunk:
                continue
            for raw in pt.feed_cobs(self._rx_buf, chunk):
                st = pt.unpack_status(raw)
                if st is None:
                    continue
                self._pan_deg = float(st.pan_deg)
                self._tilt_deg = float(st.tilt_deg)
                self._mode = st.mode_name
                self._have_status = True

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self._write(pt.cmd_stop(self._next_seq()))
        except Exception:
            pass
        with self._ser_lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PantiltBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
