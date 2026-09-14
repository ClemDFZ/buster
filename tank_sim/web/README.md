# tank_sim web URDF viewer

Static three.js viewer for `urdf_export`. No build step (node on this Orin is v12).

## Run

```bash
cd /home/orinclem/tank_sim/web
./serve.sh          # port 8000
# or: ./serve.sh 8080
```

Open: http://localhost:8000/web/

Root served = `tank_sim/`, so `package://urdf_export/meshes/X.STL` → `/urdf_export/meshes/X.STL`.

## Joints (URDF corrected after SolidWorks re-export)

Backup of last raw export: `urdf_export/urdf/urdf_export.pre_limits.urdf`

| Joint | Type | Limits | Source |
|-------|------|--------|--------|
| `pan_joint` | revolute | ±π (±180°) | firmware `PAN_MIN/MAX_DEG` |
| `tilt_joint` | revolute | ±π/4 (±45°) | mécanique réelle (firmware encore −45…0) |
| `FL_joint` / `FR_wheel_joint` / `RL_joint` / `RR_joint` | continuous | wrap ±π in UI | no mechanical stop |
| `realsense_joint` / `mpu_joint` / `CSI_joint` | fixed | — | — |

Firmware ref: `pantilt_slave/ESP/pantilt_state_machine/pantilt_state_machine.ino` L38–49.

- `velocity` pan/tilt derived from `MAX_SPEED_STEPS_S=15000` and steps/deg.
- `effort` (Nm) = NEMA17 estimates (pan direct, tilt 2.5:1 gearbox). **Not measured.**
- Viewer fallback: revolute with `lower==upper` → treated as continuous (SolidWorks lock).

## UI

- Sliders for all non-`fixed` joints (deg + rad)
- Reset → all 0; Home → tilt −45° (endstop), others 0
- Toggles: Frames (AxesHelper + label sur chaque link), wireframe, grid, collision
- Meshes load small-first; `base_link.STL` (~5 MB / 100k tris) last
- Fixed links (`realsense`, `mpu`, `CSI`) load as meshes, no slider

## Meshes

Visual STLs are quadric-decimated from SolidWorks exports. Raw originals live in
`urdf_export/meshes/raw/` (gitignored).

| Asset | Role | Approx. |
|-------|------|---------|
| `base_link.STL` | visual | ~100 k tris |
| `base_link` collision | AABB `<box>` | 1 prim |
| `*_wheel_link.STL` | visual | ~8 k tris |
| `*_wheel_link_collision.STL` | convex hull | ~1 k tris |
| other links | visual (=collision) | 4–8 k tris |

## Stack

- `vendor/three/` = three@0.185.1 (import map, OrbitControls, STLLoader)
- `main.js` = custom URDF parser (DOMParser), Z-up, rpy order `ZYX`
