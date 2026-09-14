# tank_sim_gazebo

ROS 2 Jazzy + Gazebo Harmonic (gz-sim8) simulation of the mecanum tank.

Port of the original Humble / Gazebo Classic 11 plan — Classic is unavailable on Ubuntu 24.04.

## Features

- Holonomic base via `gz::sim::systems::VelocityControl` (`drive:=velocity`, default)
- Ground-truth odom `/odom_gt` without TF (VO owns `odom → base_footprint`)
- Simulated RealSense D415 (`rgbd_camera`) + CSI IMX219 on the turret
- Pan/tilt via `JointTrajectoryController`
- Web teleop on port **8768** (Vx/Vy/Yaw + pan/tilt)
- Optional rtabmap visual odometry + SLAM (`rtabmap:=true`)
- AWS RoboMaker small-house world (fork `sethgi/...` branch `ros2_jazzy`)

## One-time setup

```bash
# 1) apt deps (needs sudo password unless NOPASSWD)
bash ~/tank_ws/scripts/00_apt_install.sh
# If rtabmap dies with diagnostic_updater symbol lookup error:
bash ~/tank_ws/scripts/00b_fix_rtabmap_abi.sh

# 2) fetch world + build
bash ~/tank_ws/scripts/01_fetch_world.sh
bash ~/tank_ws/scripts/02_build.sh
source ~/tank_ws/install/setup.bash
```

## Launch

```bash
source /opt/ros/jazzy/setup.bash
source ~/tank_ws/install/setup.bash

# Headless gz (recommended under xpra) + RViz + rtabmap
ros2 launch tank_sim_gazebo sim.launch.py

# Options
ros2 launch tank_sim_gazebo sim.launch.py gui:=false rviz:=true rtabmap:=true
ros2 launch tank_sim_gazebo sim.launch.py gui:=true   # local gz GUI
```

Teleop UI: http://localhost:8768

## Topics

| ROS topic | Role |
|-----------|------|
| `/cmd_vel` | Twist → VelocityControl |
| `/odom_gt` | GT odometry (no TF) |
| `/odom` | VO odometry + TF |
| `/joint_states` | wheels + pan/tilt |
| `/set_joint_trajectory` | pan/tilt command |
| `/camera/camera/color/image_raw` | D415 color |
| `/camera/camera/depth/image_rect_raw` | D415 depth |
| `/camera/camera/depth/color/points` | D415 cloud |
| `/csi_cam/image_raw` | IMX219 on tilt |

## Mecanum constants

Display IK in the web UI uses **mega_drive** geometry (matches URDF wheel origins ≈ ±0.105 / ±0.102):

- `R = 0.0485 m`
- `L = W = 0.21 m`
- `v_max = R·ω_max ≈ 1.455 m/s`, `yaw_rate_max ≈ 3.46 rad/s` (ω_wheel=30 rad/s)

Note: `omniwheel_slave` used `R=0.04`, `Lb=0.20` — do not mix.

Experimental `drive:=mecanum` switches to `MecanumDrive` (needs regenerating URDF with `--drive mecanum` and installing `urdf/tank_sim_mecanum.urdf`). Wheel axes from SolidWorks may need validation.

## Rendering / xpra

Default `gui:=false` → `gz sim -s` (server only). View everything in RViz2.

If depth/camera rendering fails headless:

```bash
export GZ_SIM_RENDER_ENGINE=ogre2
export DISPLAY=:0
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

GPU: NVIDIA RTX recommended (ogre2 depth cameras need GL). Under headless/EGL, image rates may sit around 5–15 Hz instead of 30 Hz; local `DISPLAY=:0` helps.

## Validation status (this machine)

| Check | Result |
|-------|--------|
| `check_urdf` / `gz sdf -p` | OK |
| `small_house.sdf` load | OK (no missing models) |
| Spawn `tank_sim` | OK; meshes resolve via `GZ_SIM_RESOURCE_PATH=.../share` |
| Holonomic Vx / Vy / Yaw (web :8768) | OK |
| `/odom_gt`, `/joint_states`, cameras | OK (camera Hz GPU-bound) |
| rtabmap / `rgbd_odometry` | Needs `bash scripts/00b_fix_rtabmap_abi.sh` (diagnostic_updater 4.2.7) |

## Regenerate URDF

Meshes live in `tank_description` (shared real+sim). Clone/build that package in the same workspace.

```bash
# Gazebo URDF (plugins + sensors)
python3 ~/tank_ws/src/tank_sim/tools/urdf_postprocess.py \
  --input ~/tank_ws/src/tank_sim/urdf_export/urdf/urdf_export.urdf \
  --output ~/tank_ws/src/tank_sim/tank_sim_gazebo/urdf/tank_sim.urdf \
  --drive velocity --mesh-pkg tank_description --profile gz

# Clean viz URDF (RViz / robot_state_publisher)
python3 ~/tank_ws/src/tank_sim/tools/urdf_postprocess.py \
  --input ~/tank_ws/src/tank_sim/urdf_export/urdf/urdf_export.urdf \
  --output ~/tank_ws/src/tank_description/urdf/tank.urdf \
  --mesh-pkg tank_description --profile viz
```

`urdf_export/` stays the untouched SolidWorks export.

Unified visualizer (ghost + markers) alongside sim:

```bash
ros2 launch tank_sim_gazebo sim.launch.py rviz:=false
ros2 launch tank_viz viz.launch.py mode:=sim rsp:=false
```

## Classic → Harmonic map

| Classic | Harmonic |
|---------|----------|
| `gazebo_ros_planar_move` | `VelocityControl` |
| `gazebo_ros_joint_state_publisher` | `JointStatePublisher` |
| `gazebo_ros_joint_pose_trajectory` | `JointTrajectoryController` |
| `gazebo_ros_camera` | `<sensor type="rgbd_camera\|camera">` + `ros_gz_bridge` / `ros_gz_image` |
| `GAZEBO_MODEL_PATH` | `GZ_SIM_RESOURCE_PATH` |

## Layout

```
tank_ws/
├── scripts/{00_apt_install,01_fetch_world,02_build}.sh
├── third_party/aws-robomaker-small-house-world/   # gitignored
└── src/
    ├── tank_description/     # shared URDF + meshes
    ├── tank_viz/             # RViz digital twin (real+sim)
    └── tank_sim/
        ├── urdf_export/          # raw SW export
        ├── tools/urdf_postprocess.py
        └── tank_sim_gazebo/      # this package
```
