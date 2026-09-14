# tank_ui

Unified web control panel for the tank (Humble).

```bash
source /opt/ros/humble/setup.bash
source ~/tank_ws/install/setup.bash
ros2 launch tank_ui ui.launch.py
# → http://<orin-ip>:8766/
```

## Tabs

| Tab | Role |
|-----|------|
| **ROS** | Start/Stop stacks (`pantilt`, `base`, `viz`, `foxglove`, `vlm`, `track`) + node status (red/orange/green) + log tail |
| **Pantilt** | Vel / angles / IDLE·HOMING·RUNNING / Go(0,0) / Stop |
| **Tracking** | MJPEG stream + TrackTarget goal + motion_enable toggle |
| **Odom** | RealSense color + `/odom` (Mega si présent) |

## Stacks

Defined in `config/stacks.yaml`. Start = `Popen(ros2 launch …, start_new_session=True)`.  
Stop = `SIGINT` to process group. Logs → `/tmp/tank_ui/<stack>.log`.  
Foxglove : `tank_viz/foxglove.launch.py` → `ws://<orin-ip>:8765`.  
3D : frame `odom`, Robot (URDF), topic `/odom` (Pose/Path). Image : `/csi_cam/image_raw/compressed`.  
Mouvement = stack **Base** (`mega_bridge` → `/odom` + TF) + **Viz** `static_base:=false`. RealSense n’est pas dans Viz (optionnel dans Base, pas besoin pour l’odom roues).

## Tracking

- Action `/vlm/track_target` (Start / Cancel)
- Toggle « envoi → ESP » → `/track/motion_enable` (`std_msgs/Bool`)
- Toggle sweep → `/track/sweep_enable` — cmd_vel continu pan ±90° / tilt -15…-50° si miss VLM
- PID live → `/track/pid` (JSON) ; **Set** → `tank_track/config/track.yaml`
- Stream auto : CSI si idle/explore, `/vlm/track_annotated/compressed` dès FOUND/TRACKING
- Feedback : `dx_px` / `dy_px` (bbox vs centre) même si ESP gated
