# tank_vlm

Standalone ROS 2 Humble package: **CSI → Qwen3-VL-2B (llama.cpp) → detections**.

No dependency on `~/vlm-server`. Weights + Python venv live **inside this package**.

## One-time setup

```bash
cd ~/tank_ws/src/tank_vlm
bash scripts/fetch_models.sh    # copies Qwen3-VL-2B GGUF into models/
bash scripts/setup_venv.sh      # copies llama-cpp CUDA venv into .venv/ (~6 Go)
```

## Build / run

```bash
cd ~/tank_ws && colcon build --packages-select tank_vlm
source install/setup.bash
# CSI already publishing
ros2 launch tank_vlm vlm.launch.py
ros2 topic pub --once /vlm/target std_msgs/msg/String "{data: 'bottle'}"
```

## Layout

```
tank_vlm/
├── models/          # *.gguf (gitignored) — Qwen3-VL-2B
├── .venv/           # llama-cpp CUDA (gitignored)
├── tank_vlm/
│   ├── vlm_bridge.py
│   └── vlm_core/    # vendored inference + grounding
└── scripts/
    ├── vlm_bridge       # bash launcher → .venv python
    ├── fetch_models.sh
    └── setup_venv.sh
```

## Interfaces

| I/O | Role |
|-----|------|
| sub `/csi_cam/image_raw` | frames |
| sub `/vlm/target` | label + trigger detect |
| sub `/vlm/query` | VQA |
| pub `/vlm/detections` | JSON boxes |
| pub `/vlm/annotated` | image + bbox |
| pub `/vlm/answer` | VQA JSON |
| pub `/vlm/status` | loaded / paths |
| srv `/vlm/detect`, `/vlm/health` | Trigger |
| srv `/vlm/detect_label` | `Detect.srv` sync (`label` + optional frozen `image` → JSON) |

Detect latency ≈ 20–40 s on Orin CPU/GPU vision — do not spam.
