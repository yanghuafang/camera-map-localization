# ROS 2 RViz visualization

Interactive playback of map-matching localization with keyboard controls in the **node terminal**.

Full documentation: [docs/VISUALIZATION.md](../docs/VISUALIZATION.md).

## Prerequisites

- ROS 2 (Humble or newer) with `rclcpp`, `rviz2`, `visualization_msgs`, `nav_msgs`
- Built `cam_loc_core` in the main repo: `cmake -B build && cmake --build build`

## Build

```bash
# From repository root
./scripts/build_ros.sh
source ros/ws/install/setup.bash
```

## Run (smoke data)

```bash
./scripts/prepare_smoke_kitti.sh
./scripts/run_ros_viz.sh
```

Focus the **terminal running the node** (not RViz) for keyboard input:

| Key | Action |
|-----|--------|
| **Space** | Advance one frame (pauses continuous mode) |
| **R** | Toggle continuous playback / pause |
| **Q** | Quit |

## Topics (namespace `/cam_loc`)

| Topic | Type | Description |
|-------|------|-------------|
| `map_markers` | `visualization_msgs/MarkerArray` | Local map polylines |
| `perception_markers` | `visualization_msgs/MarkerArray` | Perception on ground plane |
| `cost_markers` | `visualization_msgs/MarkerArray` | Best pose sample (arrow) |
| `gt_path` | `nav_msgs/Path` | Full GT trajectory |
| `estimate_path` | `nav_msgs/Path` | Localization estimate so far |
| `gt_pose` / `estimate_pose` | `geometry_msgs/PoseStamped` | Current frame poses |
| `camera/image` | `sensor_msgs/Image` | KITTI cam0 (if available) |
| `status` | `std_msgs/String` | Frame / match status |

Fixed frame: **`map`** (KITTI world).

## Parameters

Pass via launch or `--ros-args -p name:=value`:

- `kitti_root`, `sequence`, `perception_mode`, `perception_root`
- `use_gt_plane`, `use_cuda`, `playback_hz` (continuous mode rate)
- `map_path`, `georef_path`, `start_frame`, `max_frames`

Example:

```bash
ros2 launch cam_loc_ros cam_loc_viz.launch.py \
  kitti_root:=/path/to/kitti_odometry \
  perception_mode:=auto \
  use_gt_plane:=false
```

## Manual launch

```bash
ros2 run cam_loc_ros cam_loc_viz_node --ros-args \
  -p kitti_root:=data/smoke_kitti \
  -p perception_mode:=oracle \
  -p use_gt_plane:=true
```

In another terminal: `rviz2 -d $(ros2 pkg prefix cam_loc_ros)/share/cam_loc_ros/rviz/cam_loc.rviz`
