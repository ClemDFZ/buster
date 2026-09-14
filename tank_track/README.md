# tank_track

Action ROS **TrackTarget** : prompt → explore pantilt + VLM detect → OSTrack TRT → PID `/pantilt/cmd_vel`.

Standalone : engine TRT dans `models/ostrack/` (pas de dépendance runtime à `pantilt_slave`).

## One-time setup

```bash
cd ~/tank_ws/src/tank_track
bash scripts/fetch_ostrack.sh   # copie ostrack_vitb256_ce.engine (~179 Mo)
bash scripts/setup_venv.sh      # copie pantilt_slave/.venv (torch+TRT)
```

## Build

```bash
cd ~/tank_ws
colcon build --packages-select tank_vlm tank_track
source install/setup.bash
```

## Run

Prérequis : `pantilt.launch.py` (CSI + bridge) + `vlm.launch.py`.

```bash
ros2 launch tank_track track.launch.py

# Goal
ros2 action send_goal /vlm/track_target tank_track_interfaces/action/TrackTarget \
  "{target: 'bottle', do_home: false}" --feedback
```

Cancel → `cmd_vel=0` + reset tracker.

Motion gate (tank_ui toggle) : topic `/track/motion_enable` (`std_msgs/Bool`, default true).  
Quand `false` : pas de `/pantilt/cmd_vel` ni `joint_trajectory` (explore/PID skip wait). PID + `dx_px`/`dy_px` continuent.

Sweep (tank_ui toggle) : `/track/sweep_enable` (default true). Miss VLM → **cmd_vel continu** :
pan ping-pong ±90°, tilt triangle **-15°…-50°**. Off → hold + re-query.

PID live : `/track/pid` (`std_msgs/String` JSON) + `ros2 param set /track_server kp_x …`.

Interfaces packages: `tank_track_interfaces` (action), `tank_vlm_interfaces` (`Detect.srv`).

## Pipeline

1. **EXPLORE** — sweep continu cmd_vel (pan ±90°, tilt -15…-50°) pendant les queries VLM
2. Call sync `/vlm/detect_label` (tank_vlm)
3. Hit → `OSTrack.init` → **TRACKING**
4. Per-frame update → PID → `/pantilt/cmd_vel` (`angular.z`=pan_dps, `angular.y`=tilt_dps)
5. Lost `lost_frames` → re-EXPLORE

## Interfaces

| I/O | Role |
|-----|------|
| Action `/vlm/track_target` | Goal `target` + `do_home` |
| Client `/vlm/detect_label` | Detect.srv sync |
| Sub `/csi_cam/image_raw` | frames OSTrack |
| Sub `/pantilt/joint_states` | explore settle |
| Pub `/pantilt/cmd_vel` | PID |
| Pub `/pantilt/joint_trajectory` | explore steps |
| Pub `/vlm/track_annotated/compressed` | JPEG overlay (Foxglove) |
| Pub `/vlm/track_annotated` | raw overlay (opt `annotate_raw`) |
| Client `/pantilt/home` | si `do_home` |

## Layout

```
tank_track/
├── models/ostrack/     # *.engine (gitignored)
├── action/TrackTarget.action
├── tank_track/
│   ├── track_server.py
│   ├── ostrack_tracker.py
│   ├── explore.py / pid.py
└── scripts/fetch_ostrack.sh
```
