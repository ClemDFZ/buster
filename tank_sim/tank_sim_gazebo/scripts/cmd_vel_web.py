#!/usr/bin/env python3
"""Web teleop for tank_sim: Vx/Vy/Yaw sliders + pan/tilt, port 8768.

Publishes:
  /cmd_vel                  geometry_msgs/Twist
  /set_joint_trajectory     trajectory_msgs/JointTrajectory

Displays mecanum IK wheel speeds (mega_drive constants).
"""
from __future__ import annotations

import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# mega_drive / URDF geometry
R = 0.0485
L = 0.21
W = 0.21
OMEGA_WHEEL_MAX = 30.0  # rad/s
V_MAX = R * OMEGA_WHEEL_MAX  # ~1.455 m/s
YAW_RATE_MAX = R * OMEGA_WHEEL_MAX / (L + W)  # ~3.46 rad/s pure yaw
PAN_LIMIT = math.pi
TILT_LIMIT = math.pi / 4
PORT = 8768
RATE_HZ = 20.0

HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>tank_sim teleop</title>
<style>
  :root { color-scheme: dark; font-family: system-ui, sans-serif; }
  body { margin: 1.5rem; background: #1a1a1e; color: #e8e8ec; }
  h1 { font-size: 1.2rem; font-weight: 600; }
  .row { display: flex; gap: 1rem; flex-wrap: wrap; margin-bottom: 1rem; }
  .card { background: #25252b; border-radius: 8px; padding: 1rem 1.2rem; min-width: 260px; flex: 1; }
  label { display: flex; justify-content: space-between; font-size: 0.9rem; margin: 0.6rem 0 0.2rem; }
  input[type=range] { width: 100%; }
  button { background: #3d5afe; color: #fff; border: 0; border-radius: 6px; padding: 0.5rem 1rem; cursor: pointer; margin-right: 0.5rem; }
  button.secondary { background: #444; }
  pre { background: #111; padding: 0.75rem; border-radius: 6px; font-size: 0.8rem; overflow-x: auto; }
  .hint { opacity: 0.65; font-size: 0.8rem; }
</style>
</head>
<body>
  <h1>tank_sim teleop <span class="hint">:8768</span></h1>
  <div class="row">
    <div class="card">
      <strong>Base (holonomic)</strong>
      <label>vx <span id="vxv">0.00</span></label>
      <input id="vx" type="range" min="-1" max="1" step="0.01" value="0"/>
      <label>vy <span id="vyv">0.00</span></label>
      <input id="vy" type="range" min="-1" max="1" step="0.01" value="0"/>
      <label>yaw <span id="yawv">0.00</span></label>
      <input id="yaw" type="range" min="-1" max="1" step="0.01" value="0"/>
      <p style="margin-top:1rem">
        <button onclick="stopBase()">Stop base</button>
      </p>
    </div>
    <div class="card">
      <strong>Turret</strong>
      <label>pan (rad) <span id="panv">0.00</span></label>
      <input id="pan" type="range" min="-3.14" max="3.14" step="0.01" value="0"/>
      <label>tilt (rad) <span id="tiltv">0.00</span></label>
      <input id="tilt" type="range" min="-0.785" max="0.785" step="0.01" value="0"/>
      <p style="margin-top:1rem">
        <button class="secondary" onclick="zeroTurret()">Zero turret</button>
      </p>
    </div>
    <div class="card">
      <strong>Mecanum IK ω (rad/s)</strong>
      <pre id="wheels">FL=0 FR=0\nRL=0 RR=0</pre>
      <p class="hint">R=0.0485 L=W=0.21 · ω_max=30 rad/s → V≈1.45 m/s · yaw≈3.46 rad/s</p>
    </div>
  </div>
<script>
const ids = ['vx','vy','yaw','pan','tilt'];
ids.forEach(id => {
  const el = document.getElementById(id);
  el.addEventListener('input', () => {
    document.getElementById(id+'v').textContent = Number(el.value).toFixed(2);
    send();
  });
});
function payload() {
  return {
    vx: +document.getElementById('vx').value,
    vy: +document.getElementById('vy').value,
    yaw: +document.getElementById('yaw').value,
    pan: +document.getElementById('pan').value,
    tilt: +document.getElementById('tilt').value,
  };
}
function send() {
  const p = payload();
  fetch('/api/command', {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(p)})
    .then(r => r.json()).then(d => {
      if (d.wheels) {
        document.getElementById('wheels').textContent =
          `FL=${d.wheels.FL.toFixed(2)} FR=${d.wheels.FR.toFixed(2)}\\n` +
          `RL=${d.wheels.RL.toFixed(2)} RR=${d.wheels.RR.toFixed(2)}`;
      }
    }).catch(()=>{});
}
function stopBase() {
  ['vx','vy','yaw'].forEach(id => { document.getElementById(id).value = 0; document.getElementById(id+'v').textContent='0.00'; });
  send();
}
function zeroTurret() {
  ['pan','tilt'].forEach(id => { document.getElementById(id).value = 0; document.getElementById(id+'v').textContent='0.00'; });
  send();
}
setInterval(send, 50);
</script>
</body>
</html>
"""


class CommandState:
    def __init__(self):
        self.lock = threading.Lock()
        self.vx = 0.0
        self.vy = 0.0
        self.yaw = 0.0
        self.pan = 0.0
        self.tilt = 0.0

    def update(self, data: dict) -> None:
        with self.lock:
            self.vx = float(data.get("vx", 0.0))
            self.vy = float(data.get("vy", 0.0))
            self.yaw = float(data.get("yaw", 0.0))
            self.pan = max(-PAN_LIMIT, min(PAN_LIMIT, float(data.get("pan", 0.0))))
            self.tilt = max(-TILT_LIMIT, min(TILT_LIMIT, float(data.get("tilt", 0.0))))

    def snapshot(self):
        with self.lock:
            return self.vx, self.vy, self.yaw, self.pan, self.tilt


def mecanum_ik(vx: float, vy: float, wz: float) -> dict:
    """Wheel angular velocities [rad/s], mega_drive convention."""
    # ω = (1/R) * [vx ∓ vy ∓ (L+W)*wz]  — standard mecanum
    lw = L + W
    fl = (1.0 / R) * (vx - vy - lw * wz)
    fr = (1.0 / R) * (vx + vy + lw * wz)
    rl = (1.0 / R) * (vx + vy - lw * wz)
    rr = (1.0 / R) * (vx - vy + lw * wz)
    return {"FL": fl, "FR": fr, "RL": rl, "RR": rr}


STATE = CommandState()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):  # quiet
        pass

    def _json(self, code: int, obj: dict) -> None:
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        self.send_error(404)

    def do_POST(self):
        if self.path != "/api/command":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            self._json(400, {"ok": False, "error": "bad json"})
            return
        STATE.update(data)
        vx_n, vy_n, yaw_n, _, _ = STATE.snapshot()
        vx, vy, wz = vx_n * V_MAX, vy_n * V_MAX, yaw_n * YAW_RATE_MAX
        self._json(200, {"ok": True, "wheels": mecanum_ik(vx, vy, wz)})


class CmdVelWeb(Node):
    def __init__(self):
        super().__init__("cmd_vel_web")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.traj_pub = self.create_publisher(JointTrajectory, "/set_joint_trajectory", 10)
        self.timer = self.create_timer(1.0 / RATE_HZ, self._tick)
        self.get_logger().info(f"cmd_vel_web listening on http://0.0.0.0:{PORT}")

    def _tick(self):
        vx_n, vy_n, yaw_n, pan, tilt = STATE.snapshot()
        twist = Twist()
        twist.linear.x = vx_n * V_MAX
        twist.linear.y = vy_n * V_MAX
        twist.angular.z = yaw_n * YAW_RATE_MAX
        self.cmd_pub.publish(twist)

        traj = JointTrajectory()
        traj.joint_names = ["pan_joint", "tilt_joint"]
        pt = JointTrajectoryPoint()
        pt.positions = [pan, tilt]
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points = [pt]
        self.traj_pub.publish(traj)


def main():
    rclpy.init()
    node = CmdVelWeb()
    server = ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
