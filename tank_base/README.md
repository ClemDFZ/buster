# tank_base

ROS 2 Humble mobile-base bringup (Mega + RealSense + optional visual odom):

- Mega (`omniwheel_slave`) → `/cmd_vel`, `/odom_wheel`, `/wheel/odom`, `/imu/data_raw`
- RealSense D415
- `odom_mux` → `/odom` + TF `odom → base_footprint` (sole TF publisher)
- optional `rgbd_odometry` → `/odom_vo`

## Launch

```bash
ros2 launch tank_base tank_base.launch.py
```

| Arg | Default | Values |
|-----|---------|--------|
| `mega` | `auto` | `auto\|true\|false` |
| `realsense` | `auto` | `auto\|true\|false` |
| `vo` | `false` | `true\|false` — `rgbd_odometry` |
| `odom` | `auto` | `auto\|wheels\|vo\|fuse` |
| `serial_port` | `auto` | path or `auto` |
| `publish_tf` | `true` | mux TF on/off |

```bash
# wheels only (default UI stack Base)
ros2 launch tank_base tank_base.launch.py vo:=false odom:=wheels

# VO only (no Mega)
ros2 launch tank_base tank_base.launch.py mega:=false vo:=true odom:=vo

# both, mux auto (fuse if VO fresh else wheels)
ros2 launch tank_base tank_base.launch.py vo:=true odom:=auto
```

UI: stack **Base** then stack **VO** (`vo.launch.py`) — same as `vo:=true` without restarting Mega/RS. Do not start both `vo:=true` and the VO stack (double `rgbd_odometry`).

VO needs **Viz** (`robot_state_publisher`) so `base_footprint` reaches the camera, plus static TF `realsense_link → camera_link` (started by `vo.launch.py`).

Switch live:

```bash
ros2 topic pub --once /odom_mux/set_mode std_msgs/String "data: fuse"
# or
ros2 param set /odom_mux mode wheels
```

## Topics

| Topic | Role |
|-------|------|
| `/cmd_vel` | Twist SI → Mega `VEL:` |
| `/odom_wheel` | Mega pose (Vx/Vy wheels + yaw IMU), **no TF** |
| `/wheel/odom` | FK twist from Mega (no pose) |
| `/odom_vo` | `rgbd_odometry`, **no TF** |
| `/odom` | mux output — **only TF** `odom → base_footprint` |
| `/odom_source` | `wheels` / `vo` / `fuse` |
| `/path/wheel` `/path/vo` `/path/fused` | Foxglove 3D traces |
| `/imu/data_raw` | Mega MPU6050 DMP |
| `/camera/camera/color/image_raw` | RealSense |
| `/camera/camera/depth/image_rect_raw` | aligned depth |
| `/camera/camera/depth/color/points` | PointCloud2 (Neon, Orin) |

## Fuse

Same idea as `odom_fusion` `xy_fusion_node`: blend body `vx/vy` (`alpha_vo_fresh=0.7` when VO fresh), yaw from Mega IMU on `/odom_wheel`. No ESP32 ESKF.

## Odom model (wheels)

- Body `Vx`/`Vy` from wheel FK on Mega
- Yaw from IMU quaternion; `twist.angular.z` from gyro Z
- Integration on Orin → `/odom_wheel`
