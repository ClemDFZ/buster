#!/usr/bin/env python3
"""Serial bridge to omniwheel_slave Mega: /cmd_vel ↔ VEL:, DriveFrame → odom/imu/TF.

Stays up if Mega is missing; reconnects when the port appears.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("pyserial required: sudo apt install python3-serial") from exc

from tank_base.hw_probe import list_serial_ports, resolve_mega_port
from tank_base.protocol import cobs_decode, parse_drive_frame


def yaw_from_quat(w: float, x: float, y: float, z: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class MegaBridge(Node):
    def __init__(self) -> None:
        super().__init__("mega_bridge")
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("bin_period_ms", 20)
        self.declare_parameter("cmd_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_ms", 250)
        self.declare_parameter("vmax_m_s", 0.40)
        self.declare_parameter("yaw_rate_max_rad_s", 1.5)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", False)
        self.declare_parameter("odom_topic", "/odom_wheel")
        self.declare_parameter("gyro_scale", 0.001064225153655079)
        self.declare_parameter("accel_scale", 0.0005987625122070312)
        self.declare_parameter("reconnect_period_s", 2.0)

        self._port_pref = str(self.get_parameter("serial_port").value)
        self._baud = int(self.get_parameter("baud").value)
        self._bin_ms = int(self.get_parameter("bin_period_ms").value)
        self._cmd_rate = float(self.get_parameter("cmd_rate_hz").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_ms").value) * 1e-3
        self._vmax = float(self.get_parameter("vmax_m_s").value)
        self._wmax = float(self.get_parameter("yaw_rate_max_rad_s").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._imu_frame = str(self.get_parameter("imu_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._gyro_scale = float(self.get_parameter("gyro_scale").value)
        self._accel_scale = float(self.get_parameter("accel_scale").value)
        self._reconnect_period = float(self.get_parameter("reconnect_period_s").value)

        self._cmd = Twist()
        self._cmd_lock = threading.Lock()
        self._ser_lock = threading.Lock()
        self._last_cmd_t = 0.0
        self._ser: Optional[serial.Serial] = None
        self._active_port: Optional[str] = None
        self._rx_buf = bytearray()
        self._stop = threading.Event()
        self._connected = False
        self._last_miss_log = 0.0

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._yaw_initialized = False
        self._last_int_t: Optional[float] = None

        self._wheel_odom_pub = self.create_publisher(Odometry, "/wheel/odom", 10)
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, "/imu/data_raw", qos_profile_sensor_data)
        self._omega_pub = self.create_publisher(Float32MultiArray, "/mega/wheel_omega", 10)
        self._tf_br = TransformBroadcaster(self) if self._publish_tf else None

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)

        if not self._try_connect(log_miss=True):
            self.get_logger().warn(
                f"[mega] MISSING — waiting for serial ({self._port_pref}); "
                f"candidates={list_serial_ports() or 'none'}; will retry"
            )

        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        period = 1.0 / max(self._cmd_rate, 1.0)
        self.create_timer(period, self._cmd_tick)
        self.create_timer(max(self._reconnect_period, 0.5), self._reconnect_tick)

    def _try_connect(self, log_miss: bool = False) -> bool:
        port = resolve_mega_port(self._port_pref)
        if port is None:
            if log_miss:
                self.get_logger().warn(
                    f"[mega] port not found (pref={self._port_pref}, "
                    f"seen={list_serial_ports() or 'none'})"
                )
            return False
        try:
            ser = serial.Serial(port, self._baud, timeout=0.02)
        except serial.SerialException as exc:
            if log_miss:
                self.get_logger().warn(f"[mega] open failed {port}: {exc}")
            return False
        time.sleep(0.25)
        try:
            ser.reset_input_buffer()
            ser.write(f"BIN:{self._bin_ms}\n".encode("ascii"))
            ser.write(b"TEL:0\n")
            ser.write(f"LIMITS:{self._vmax:.4f},{self._wmax:.4f}\n".encode("ascii"))
        except serial.SerialException as exc:
            self.get_logger().warn(f"[mega] configure failed: {exc}")
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
            self._active_port = port
            self._rx_buf.clear()
            self._connected = True
        self.get_logger().info(
            f"[mega] OK connected {port}@{self._baud} BIN:{self._bin_ms}ms "
            f"vmax={self._vmax} wmax={self._wmax}"
        )
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
            self._active_port = None
            self._connected = False
            self._rx_buf.clear()
        self.get_logger().warn(f"[mega] disconnected ({reason}); will retry")

    def _reconnect_tick(self) -> None:
        if self._stop.is_set():
            return
        with self._ser_lock:
            ok = self._ser is not None and self._ser.is_open
        if ok:
            return
        now = time.monotonic()
        log_miss = (now - self._last_miss_log) > 10.0
        if log_miss:
            self._last_miss_log = now
        self._try_connect(log_miss=log_miss)

    def _write_line(self, line: str) -> None:
        with self._ser_lock:
            ser = self._ser
        if ser is None or not ser.is_open:
            return
        try:
            ser.write((line + "\n").encode("ascii", errors="ignore"))
        except serial.SerialException as exc:
            self._close_serial(f"write: {exc}")

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._cmd_lock:
            self._cmd = msg
            self._last_cmd_t = time.monotonic()

    def _cmd_tick(self) -> None:
        with self._ser_lock:
            if self._ser is None:
                return
        now = time.monotonic()
        with self._cmd_lock:
            age = now - self._last_cmd_t
            cmd = self._cmd
        if self._last_cmd_t == 0.0 or age > self._cmd_timeout:
            self._write_line("STOP")
            return
        vx = float(cmd.linear.x)
        vy = float(cmd.linear.y)
        wz = float(cmd.angular.z)
        nx = max(-1.0, min(1.0, vx / self._vmax if self._vmax > 1e-6 else 0.0))
        ny = max(-1.0, min(1.0, vy / self._vmax if self._vmax > 1e-6 else 0.0))
        nz = max(-1.0, min(1.0, wz / self._wmax if self._wmax > 1e-6 else 0.0))
        self._write_line(f"VEL:{nx:.4f},{ny:.4f},{nz:.4f}")

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
            self._rx_buf.extend(chunk)
            self._drain_rx()

    def _drain_rx(self) -> None:
        while True:
            try:
                i = self._rx_buf.index(0)
            except ValueError:
                if len(self._rx_buf) > 4096:
                    self._rx_buf.clear()
                return
            block = bytes(self._rx_buf[:i])
            del self._rx_buf[: i + 1]
            if not block:
                continue
            try:
                raw = cobs_decode(block)
            except ValueError:
                continue
            fr = parse_drive_frame(raw)
            if fr is None:
                continue
            self._handle_frame(fr)

    def _handle_frame(self, fr) -> None:
        stamp = self.get_clock().now().to_msg()
        qw, qx, qy, qz = fr.quat
        n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
        if n > 1e-6:
            qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
        yaw_imu = yaw_from_quat(qw, qx, qy, qz)

        gz = float(fr.gyro[2]) * self._gyro_scale
        imu_yaw_rate = -gz

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self._imu_frame
        imu.orientation.w = qw
        imu.orientation.x = qx
        imu.orientation.y = qy
        imu.orientation.z = qz
        imu.angular_velocity.x = float(fr.gyro[0]) * self._gyro_scale
        imu.angular_velocity.y = float(fr.gyro[1]) * self._gyro_scale
        imu.angular_velocity.z = imu_yaw_rate
        imu.linear_acceleration.x = float(fr.accel[0]) * self._accel_scale
        imu.linear_acceleration.y = float(fr.accel[1]) * self._accel_scale
        imu.linear_acceleration.z = float(fr.accel[2]) * self._accel_scale
        imu.orientation_covariance[0] = 0.02
        imu.orientation_covariance[4] = 0.02
        imu.orientation_covariance[8] = 0.04
        self._imu_pub.publish(imu)

        wheel = Odometry()
        wheel.header.stamp = stamp
        wheel.header.frame_id = self._odom_frame
        wheel.child_frame_id = self._base_frame
        wheel.twist.twist.linear.x = float(fr.v_x_enc)
        wheel.twist.twist.linear.y = float(fr.v_y_enc)
        wheel.twist.twist.angular.z = float(fr.omega_z_enc)
        self._wheel_odom_pub.publish(wheel)

        om = Float32MultiArray()
        om.data = [float(w) for w in fr.wheel_omega]
        self._omega_pub.publish(om)

        now = time.monotonic()
        if not self._yaw_initialized:
            self._yaw = yaw_imu
            self._yaw_initialized = True
            self._last_int_t = now
        else:
            assert self._last_int_t is not None
            dt = now - self._last_int_t
            self._last_int_t = now
            if dt > 0.0 and dt < 0.5:
                self._yaw = yaw_imu
                c = math.cos(self._yaw)
                s = math.sin(self._yaw)
                vx_b = float(fr.v_x_enc)
                vy_b = float(fr.v_y_enc)
                self._x += (c * vx_b - s * vy_b) * dt
                self._y += (s * vx_b + c * vy_b) * dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(self._yaw)
        odom.twist.twist.linear.x = float(fr.v_x_enc)
        odom.twist.twist.linear.y = float(fr.v_y_enc)
        odom.twist.twist.angular.z = imu_yaw_rate
        self._odom_pub.publish(odom)

        if self._tf_br is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self._tf_br.sendTransform(t)

    def destroy_node(self) -> bool:
        self._stop.set()
        try:
            self._write_line("STOP")
            self._write_line("BIN:0")
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
    node = MegaBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
