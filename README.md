# tank_ws/src — contexte rapide

Workspace ROS 2 du tank mecanum. **Orin = Humble** ; sim Gazebo = **Jazzy** (PC Ubuntu 24).

| Package | Machine | Rôle |
|---------|---------|------|
| [`tank_base`](tank_base/) | Orin | Base mobile : Mega + odom roues/IMU + RealSense |
| [`tank_pantilt`](tank_pantilt/) | Orin | Tourelle : ESP pan/tilt + CSI |
| [`tank_description`](tank_description/) | Orin + PC | URDF + meshes + `rviz/tank.rviz` (source unique) |
| [`tank_viz`](tank_viz/) | Orin + PC | Jumeau numérique RViz (`mode:=real\|sim`) |
| [`tank_vlm`](tank_vlm/) | Orin | CSI → Qwen3-VL-2B in-package → `/vlm/*` |
| [`tank_track`](tank_track/) | Orin | Action TrackTarget : explore + VLM + OSTrack → pantilt |
| [`tank_ui`](tank_ui/) | Orin | Panneau web `:8766` (ROS / pantilt / tracking) |
| [`tank_sim`](tank_sim/) | PC | Simu Gazebo Harmonic + bridge |

Packages **standalone** : base ⟂ pantilt ⟂ vlm (pas de dépendance croisée).  
`tank_viz` → `tank_description` ; `tank_sim_gazebo` → `tank_description`.

---

## Architecture

```
┌──────────────────────────── Orin (Humble) ────────────────────────────┐
│  tank_base                          tank_pantilt                       │
│  mega_bridge ↔ Mega USB             pantilt_bridge ↔ ESP USB           │
│  /cmd_vel /odom /imu                /pantilt/*                         │
│  realsense → /camera/...            csi_* → /csi_cam/image_raw         │
│                                                                        │
│  tank_viz : aggregator → /joint_states → RSP + ghost + markers + RViz │
│  tank_vlm : /csi_cam → Qwen3-VL-2B (in-pkg models+venv) → /vlm/*         │
│  tank_track : Action /vlm/track_target → explore+OSTrack → cmd_vel      │
│  tank_ui : :8766 panneau ROS / pantilt / tracking                      │
│  tank_description : urdf/tank.urdf + meshes                            │
└────────────────────────────────────────────────────────────────────────┘
         hors ROS (backends) : omniwheel_slave
         (VLM+track = packages ROS ; pantilt_slave = référence legacy)

┌──────────────────────────── PC (Jazzy) ───────────────────────────────┐
│  tank_sim / tank_sim_gazebo : gz-sim + ros_gz                          │
│  tank_viz mode:=sim rsp:=false (+ même tank_description)               │
└────────────────────────────────────────────────────────────────────────┘
```

**Humble ↮ Jazzy** sur le même `ROS_DOMAIN_ID` → spam `sequence size exceeds remaining buffer`.  
Viz remote sans xpra : **Foxglove** sur Orin, ou RViz **Humble** sur le PC.

---

## tank_base

- Firmware Mega : `~/omniwheel_slave` (`VEL:` + DriveFrame COBS `BIN:`)
- Launch : `ros2 launch tank_base tank_base.launch.py` (`mega`/`realsense` = `auto`)
- Node seul : `ros2 run tank_base mega_bridge`
- Topics : `/cmd_vel`, `/odom`, `/wheel/odom`, `/imu/data_raw`, TF `odom→base_footprint`

## tank_pantilt

- Firmware ESP : `tank_pantilt/firmware/pantilt_state_machine/` + `./flash_esp.sh` (ESP32 classique, **pas** S3)
- Tracking/VLM : packages `tank_vlm` + `tank_track` (ne pas double-ouvrir le port ESP avec pantilt_slave)
- Launch : `ros2 launch tank_pantilt pantilt.launch.py`
- Nodes : `pantilt_bridge`, `csi_argus`, `csi_republish`
- Topics : `/pantilt/joint_states`, `/pantilt/cmd_vel` (`angular.z`=pan_dps, `y`=tilt_dps), `/csi_cam/image_raw`
- Services : `/pantilt/home`, `/pantilt/stop`
- UI web : voir **`tank_ui`** (`:8766`)

## tank_ui

- Launch : `ros2 launch tank_ui ui.launch.py` → **http://\<orin-ip\>:8766/**
- Onglets : ROS (start/stop stacks + status nodes), Pantilt, Tracking
- Tracking : ActionClient `/vlm/track_target` + `/track/motion_enable` + `/track/sweep_enable` + `/track/pid`

## tank_description / tank_viz

- URDF partagé : `tank_description/urdf/tank.urdf` (généré via `tank_sim/tools/urdf_postprocess.py --profile viz`)
- Viz réel (pantilt seul) :

```bash
ros2 launch tank_pantilt pantilt.launch.py
ros2 launch tank_viz viz.launch.py mode:=real static_base:=true
```

- Viz sim (PC) : `ros2 launch tank_viz viz.launch.py mode:=sim rsp:=false`

## tank_vlm

- Standalone : Qwen3-VL-2B **dans** `tank_vlm/models/` + `.venv` (pas `~/vlm-server`)
- Setup : `bash scripts/fetch_models.sh && bash scripts/setup_venv.sh`
- Launch : `ros2 launch tank_vlm vlm.launch.py`
- Sub `/csi_cam/image_raw` ; pub `/vlm/detections` / `/vlm/annotated`
- Trigger : `ros2 topic pub --once /vlm/target std_msgs/msg/String "{data: 'bottle'}"`
- Sync : `ros2 service call /vlm/detect_label tank_vlm_interfaces/srv/Detect "{label: 'bottle'}"`

## tank_track

- Setup : `bash scripts/fetch_ostrack.sh && bash scripts/setup_venv.sh`
- Launch : `ros2 launch tank_track track.launch.py` (après pantilt + vlm)
- Action : `/vlm/track_target` — explore pan + detect VLM → OSTrack → `/pantilt/cmd_vel`

```bash
ros2 action send_goal /vlm/track_target tank_track_interfaces/action/TrackTarget \
  "{target: 'bottle', do_home: false}" --feedback
```

## tank_sim

- Branche `feat/simulation` : package `tank_sim_gazebo` (Jazzy + Gazebo Harmonic)
- Meshes : `package://tank_description/meshes/` (dépendance `tank_description`)
- `urdf_export/` : export SolidWorks brut
- `tools/urdf_postprocess.py` → URDF sim (`--profile gz`) + viz (`--profile viz`)
- `web/` : viewer Three.js joints (sans ROS)
- Launch PC : `ros2 launch tank_sim_gazebo sim.launch.py`

---

## Build / source (Orin)

```bash
source /opt/ros/humble/setup.bash
cd ~/tank_ws && colcon build --packages-select tank_description tank_viz tank_base tank_pantilt tank_vlm_interfaces tank_vlm tank_track_interfaces tank_track tank_ui
source install/setup.bash
export ROS_DOMAIN_ID=7   # aligné PC si besoin LAN
# Fast DDS sans SHM (Jetson) :
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/fastdds_no_shm.xml
# Panel unique :
ros2 launch tank_ui ui.launch.py
```

## Repos hors workspace (liés)

| Path | Lien |
|------|------|
| `~/omniwheel_slave` | Drive Mega + teleop Python legacy |
| `~/odom_fusion` | Fusion IMU ESP32 + VO/rtabmap (phase 2) |
| `~/pantilt_slave` | Tracking CSI + VLM + UI |
| `~/vlm-server` | Qwen detect :8200 |
| `~/slam-bench` | RTAB-Map / RealSense launches |

## Hors scope actuel

- Bringup unique `mode:=sim|real`
- Fusion VO + wheel odom sur Orin
- Gazebo sur Orin (8 Go trop juste ; sim reste PC)
- Static TF RealSense `realsense_link → camera_link` (capteur pas sous la main)
