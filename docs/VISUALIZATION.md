# Visualization and debug

Two ways to inspect map, perception, costs, and localization output:

1. **Offline PNG** (`viz_frame`) — no extra dependencies
2. **ROS 2 RViz** (`cam_loc_ros`) — interactive playback

Both use `LocalizationEngine::setDebugCapture(true)` to snapshot DT images, cost grids, and map chunks for the active frame.

## Offline PNG (`viz_frame`)

### Smoke example

```bash
./scripts/run_viz_smoke.sh
# Opens: data/viz_smoke/frame_000020_panel.png
```

### Manual run

```bash
./build/apps/viz_frame/viz_frame \
  --kitti-root data/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --frame 20 \
  --trajectory \
  --output-dir data/viz
```

### Output files (per frame)

| File | Content |
|------|---------|
| `frame_NNNNNN_camera.png` | Camera (or gray canvas) + perception + projected map |
| `frame_NNNNNN_image_dt.png` | Image distance transform heatmap |
| `frame_NNNNNN_bev.png` | BEV map + perception overlay |
| `frame_NNNNNN_bev_dt.png` | BEV distance transform |
| `frame_NNNNNN_cost_xy.png` | Aggregated cost slice at best yaw; magenta = argmin |
| `frame_NNNNNN_topdown.png` | Top-down X–Z: map, GT (white), estimate (red) |
| `frame_NNNNNN_panel.png` | Composite of the above |
| `frame_NNNNNN_meta.json` | Costs, match flags, file list |
| `trajectory_gt_est.png` | Full sequence GT vs estimate (with `--trajectory`) |

KITTI `image_0/` images are loaded automatically when present.

## ROS 2 RViz playback

### Prerequisites

- ROS 2 Humble or Jazzy
- Built core library + ROS package:

```bash
cmake -B build && cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"
./scripts/build_ros.sh
source ros/ws/install/setup.bash
```

### Run

```bash
./scripts/prepare_smoke_kitti.sh
./scripts/run_ros_viz.sh
```

Or:

```bash
ros2 launch cam_loc_ros cam_loc_viz.launch.py \
  kitti_root:=data/smoke_kitti \
  perception_mode:=oracle \
  use_gt_plane:=true
```

### Keyboard controls

Focus the **terminal running the node** (not RViz):

| Key | Action |
|-----|--------|
| **Space** | Step one frame (pauses continuous mode) |
| **R** | Toggle continuous playback / pause |
| **Q** | Quit |

### Topics (`/cam_loc/...`)

| Topic | Type | Description |
|-------|------|-------------|
| `map_markers` | `visualization_msgs/MarkerArray` | Local map polylines |
| `perception_markers` | `visualization_msgs/MarkerArray` | Perception on ground plane |
| `cost_markers` | `visualization_msgs/MarkerArray` | Best pose sample |
| `gt_path` / `estimate_path` | `nav_msgs/Path` | Trajectories |
| `gt_pose` / `estimate_pose` | `geometry_msgs/PoseStamped` | Current poses |
| `camera/image` | `sensor_msgs/Image` | KITTI cam0 grayscale |
| `status` | `std_msgs/String` | Frame index and match status |

Fixed frame: **`map`** (KITTI world). RViz config: `ros/cam_loc_ros/rviz/cam_loc.rviz`.

### ROS parameters

`kitti_root`, `sequence`, `perception_mode`, `perception_root`, `use_gt_plane`, `use_cuda`, `playback_hz`, `map_path`, `start_frame`, `max_frames`, and map georef flags (same as CLI apps).

## Eval CSV (non-visual debug)

```bash
./build/apps/eval_sequence/eval_sequence \
  --kitti-root data/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --output-csv data/eval.csv
```

Columns: `frame`, `translation_m`, `yaw_deg`, `min_cost`, `cost_spread`, `match`, `flat`, `synth`, `offset_m`.
