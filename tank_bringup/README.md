# tank_bringup

Unified entry: `mode:=real|sim`. Launch-only package — **do not** add
`exec_depend` on both `tank_base` and `tank_sim_gazebo` (Humble Orin vs Jazzy PC).

## Run

### Real (Orin, Humble)

```bash
source /opt/ros/humble/setup.bash
source ~/tank_ws/install/setup.bash
export ROS_DOMAIN_ID=7          # never reuse this ID on a Jazzy PC
ros2 launch tank_bringup bringup.launch.py mode:=real
```

Equivalent pieces: `tank_base` + `tank_pantilt` + `tank_viz mode:=real`.

### Sim (PC, Jazzy + Gazebo Harmonic)

```bash
source /opt/ros/jazzy/setup.bash
source ~/tank_ws/install/setup.bash
export ROS_DOMAIN_ID=42         # ≠ Orin Humble domain
ros2 launch tank_bringup bringup.launch.py mode:=sim
```

Equivalent: `tank_sim_gazebo sim.launch.py rviz:=false` + `tank_viz mode:=sim rsp:=false`.

Direct sim (no viz twin): `ros2 launch tank_sim_gazebo sim.launch.py`

## Nav contract (same names both modes)

| Interface | Topic / TF |
|-----------|------------|
| base cmd | `/cmd_vel` |
| nav odom + TF | `/odom`, TF `odom → base_footprint` |
| IMU | `/imu/data_raw` (`mpu_link`; sim = GT-derived mock) |
| RealSense | `/camera/camera/color/image_raw`, `.../depth/image_rect_raw` |
| CSI | `/csi_cam/image_raw` |
| pantilt | `/pantilt/cmd_vel`, `/pantilt/joint_trajectory`, `/pantilt/joint_states`, `/pantilt/home`, `/pantilt/stop` |

See `config/contracts.yaml`.

A nav node must **not** check `mode`. Use `/odom_gt` and `/odom_gt_error` only for sim regression (gate before path following).

## Isolation

Humble ↮ Jazzy on the same `ROS_DOMAIN_ID` → Fast DDS `sequence size exceeds remaining buffer`.

| Machine | Distro | Suggested `ROS_DOMAIN_ID` |
|---------|--------|---------------------------|
| Orin | Humble | 7 |
| PC sim | Jazzy | 42 |

## What still differs

- Sim IMU is derived from `/odom_gt`, not a Mega MPU / Gazebo IMU plugin
- Sim `/odom_wheel` is rewritten GT (perfect holonomic), not Mega FK
- Camera resolution: real D415 640×480@15 vs sim 848×480@30
- `rtabmap:=true` (sim default): VO owns `/odom`+TF; GT stays on `/odom_gt`
- `rtabmap:=false`: GT fills `/odom`+TF so nav still has a pose
- Hardware (Mega/ESP/RealSense/CSI) is **not** required to validate this package

## Args

| Arg | Default | Modes |
|-----|---------|-------|
| `mode` | `real` | both |
| `viz` / `rviz` | `true` | both |
| `ros_domain_id` | inherit | both |
| `mega` `realsense` `vo` `odom` | tank_base defaults | real |
| `gui` `rtabmap` `world` `drive` | sim.launch defaults | sim |
