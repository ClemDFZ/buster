# tank_viz

Unified RViz digital twin for real (Orin) and sim (PC).

Preferred: `ros2 launch tank_bringup bringup.launch.py mode:=real|sim` (starts viz for you).

```bash
# Real (pantilt + CSI, no base yet)
ros2 launch tank_viz viz.launch.py mode:=real static_base:=true

# Real with base
ros2 launch tank_base tank_base.launch.py
ros2 launch tank_pantilt pantilt.launch.py
ros2 launch tank_viz viz.launch.py mode:=real static_base:=false

# Sim (PC) — sim.launch already runs RSP
ros2 launch tank_sim_gazebo sim.launch.py rviz:=false
ros2 launch tank_viz viz.launch.py mode:=sim rsp:=false
```

| Node | Role |
|------|------|
| `joint_state_aggregator` | real only: `/pantilt/joint_states` (HW) + `/mega/wheel_omega` → `/joint_states` (URDF). CAD rest = HW (0, −45°) : `pan_urdf = -pan_hw`, `tilt_urdf = tilt_hw + π/4` |
| `ghost_publisher` | `/cmd/joint_states` + TF `cmd_base_footprint` from cmds |
| `viz_markers` | MarkerArray on `/viz/markers` |
| `robot_state_publisher` ×2 | measured + ghost |
| `foxglove.launch.py` | `foxglove_bridge` + JPEG CSI — no raw `image_raw` on the websocket (TF lag) |

Foxglove 3D Image topic: `/csi_cam/image_raw/compressed` (`ws://<orin>:8765`).
