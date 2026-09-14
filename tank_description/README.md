# tank_description

Shared robot description for the mecanum tank (Humble Orin + Jazzy PC).

| Path | Role |
|------|------|
| `urdf/tank.urdf` | Clean URDF (no Gazebo plugins), meshes via `package://tank_description/meshes/` |
| `meshes/*.STL` | Visual / collision meshes |
| `rviz/tank.rviz` | Unified RViz config (real + sim) |

Regenerate URDF from SolidWorks export (in `tank_sim` repo):

```bash
python3 tools/urdf_postprocess.py \
  --input urdf_export/urdf/urdf_export.urdf \
  --output ../tank_description/urdf/tank.urdf \
  --profile viz --mesh-pkg tank_description
```

Launch visualizer: `ros2 launch tank_viz viz.launch.py`
