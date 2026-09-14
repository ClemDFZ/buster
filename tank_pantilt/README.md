# tank_pantilt

Standalone ROS 2 Humble package for the **turret** (ESP pan/tilt + CSI).

Independent from `tank_base` (Mega mecanum + RealSense).

## Firmware ESP (unchanged protocol)

ROS bridge uses the **existing** COBS firmware from `pantilt_slave`  
(`pantilt_state_machine`) — **not modified** for ROS; only vendored here.

```bash
# ESP32-WROOM (not S3)
cd ~/tank_ws/src/tank_pantilt
bash flash_esp.sh
# PORT=/dev/ttyUSB0 bash flash_esp.sh
```

Sketch: `firmware/pantilt_state_machine/`  
Needs: `arduino-cli`, core `esp32:esp32`, lib `AccelStepper`.

## What stays in pantilt_slave (not this package)

- Vision tracking (face / VLM / OSTrack)
- Flask UI `:5003`
- VLM server `:8200`

Run tracking separately if needed:

```bash
bash ~/pantilt_slave/start.sh
```

Then ROS can republish CSI via MJPEG (`csi_owner:=pantilt`).

## Launch

```bash
source /opt/ros/humble/setup.bash
source ~/tank_ws/install/setup.bash
ros2 launch tank_pantilt pantilt.launch.py
```

Defaults `auto`: probe ESP + CSI, log OK|MISS, start only what is present.

| Arg | Default |
|-----|---------|
| `bridge` | `auto` |
| `csi` | `auto` |
| `csi_owner` | `auto` (MJPEG then Argus) |
| `serial_port` | `auto` |

```bash
# CSI only via Argus (stop pantilt_slave first)
ros2 launch tank_pantilt pantilt.launch.py bridge:=false csi:=true csi_owner:=ros

# ESP bridge only
ros2 launch tank_pantilt pantilt.launch.py csi:=false bridge:=true serial_port:=/dev/ttyUSB0
```

**Do not** open the same ESP port from pantilt_slave tracking AND `pantilt_bridge`.

## Topics / services

| Interface | Role |
|-----------|------|
| `/pantilt/joint_states` | pan/tilt rad |
| `/pantilt/mode_state` | IDLE/HOMING/RUNNING |
| `/pantilt/cmd_vel` | Twist: `angular.z`=pan_dps, `angular.y`=tilt_dps |
| `/pantilt/joint_trajectory` | absolute angles (rad or deg) |
| `/pantilt/mode` | String IDLE/HOMING/RUNNING |
| `/pantilt/home`, `/pantilt/stop` | Trigger (homing / emergency stop) |
| `/csi_cam/image_raw` | CSI |

## Teleop web UI

Moved to **`tank_ui`** (`:8766`):

```bash
ros2 launch tank_ui ui.launch.py
# → http://<orin-ip>:8766/  onglet Pantilt
```

CLI home:

```bash
ros2 service call /pantilt/home std_srvs/srv/Trigger
```

## Graph

```
pantilt_bridge  ↔ ESP32 @921600 COBS
csi_argus|csi_republish → /csi_cam/image_raw
tank_ui → :8766 (cmd_vel / joint_trajectory / mode / home / stop)
```
