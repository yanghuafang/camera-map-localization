# Visualization and debug

Two ways to inspect map, perception, costs, and localization output:

1. **Offline PNG** (`viz_frame`) — no extra dependencies
2. **ROS 2 RViz** (`cam_loc_ros`) — interactive playback

Paths used below, both siblings of the repository:

```bash
B=../camera-map-localization-build   # ./scripts/ci.sh writes here
D=../camera-map-localization-data    # datasets and evaluation output
```

Both use `LocalizationEngine::set_debug_capture(true)` to snapshot DT images, cost grids, and map chunks for the active frame.

## Offline PNG (`viz_frame`)

### Smoke example

```bash
./scripts/run_viz_smoke.sh
# Opens: "$D"/viz_smoke/frame_000020_panel.png
```

### Manual run

```bash
"$B"/apps/viz_frame/viz_frame \
  --kitti-root "$D"/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --frame 20 \
  --trajectory \
  --output-dir "$D"/viz
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

ROS 2 is not installed by `install_deps_ubuntu.sh` — `ros-*-desktop` is around
2 GB, and only this section needs it. Everything else in the project builds and
runs without it, and `scripts/run_all.sh` skips the two ROS steps when it is
absent rather than failing.

`build_ros.sh` installs it for you — it asks first, since it needs sudo and
adds an apt source:

```bash
./scripts/build_ros.sh                # prompts before installing
./scripts/build_ros.sh --install-ros  # no prompt
./scripts/run_all.sh --install-ros    # same, from the run-everything script
```

By hand, if you would rather. Do not pick the distribution by name: the apt
repository carries exactly one per Ubuntu release, so ask it which — 24.04
answers `jazzy`, 26.04 answers `lyrical`:

```bash
sudo apt install -y curl gnupg
sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

ROS_DISTRO_PKG=$(apt-cache search --names-only '^ros-[a-z]+-desktop$' | awk '{print $1}' | head -n1)
sudo apt install -y "$ROS_DISTRO_PKG" python3-colcon-common-extensions
```

Then build the core library and the ROS package against it:

```bash
./scripts/ci.sh --no-style
./scripts/build_ros.sh
source ../camera-map-localization-build-ros/ws/install/setup.bash
```

`build_ros.sh` finds the distribution on its own — the newest under `/opt/ros`
on Ubuntu, or the micromamba prefix beside the repository on macOS — so nothing
above needs repeating for it.

### Run

```bash
./scripts/prepare_smoke_kitti.sh
./scripts/run_ros_viz.sh
```

Or:

```bash
ros2 launch cam_loc_ros cam_loc_viz.launch.py \
  kitti_root:="$D"/smoke_kitti \
  perception_mode:=oracle \
  use_gt_plane:=true
```

### Keyboard controls

Only when the node has a terminal on stdin, which means running it directly:

```bash
ros2 run cam_loc_ros cam_loc_viz_node --ros-args -p kitti_root:=... -p autoplay:=false
```

`run_ros_viz.sh` goes through `ros2 launch`, which gives the node a pipe rather
than a terminal, so the keys below do nothing there and the node says so on
startup. Use the `autoplay` and `playback_hz` parameters instead. Focus the
terminal running the node, not RViz:

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
"$B"/apps/eval_sequence/eval_sequence \
  --kitti-root "$D"/smoke_kitti \
  --perception-mode oracle \
  --use-gt-plane \
  --output-csv "$D"/eval.csv
```

Columns: `frame`, `translation_m`, `lateral_m`, `longitudinal_m`, `vertical_m`, `yaw_deg`, `min_cost`, `cost_spread`, `match`, `flat`, `synth`, `offset_m`.

`translation_m` is the unsigned 3-D norm; the three that follow are the same
error resolved onto the ground-truth vehicle axes and are **signed** — positive
is left of / ahead of / above truth. They reconstruct the norm exactly, so
`lateral_m² + longitudinal_m² + vertical_m² == translation_m²` is a valid check
on a row. Splitting them apart is what makes an along-track bias visible: see
*What each feature class constrains* in [ARCHITECTURE.md](ARCHITECTURE.md).
