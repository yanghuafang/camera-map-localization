# Scripts

All scripts assume repository root as working directory (they resolve paths relative to `scripts/..`).

## Build and test

| Script | Purpose |
|--------|---------|
| `ci.sh` | Local mirror of GitHub Actions (CPU build + ctest + smoke benchmark) |
| `run_smoke.sh` | Prepare smoke data + run `run_sequence` (CPU and CUDA if GPU present) |
| `run_benchmark.sh` | Smoke regression + micro-benchmarks; optional kitti00 if poses downloaded |
| `build_ros.sh` | Build optional `cam_loc_ros` package (requires ROS 2) |

## Data preparation

| Script | Purpose |
|--------|---------|
| `prepare_smoke_kitti.sh [length_m]` | Generate `data/smoke_kitti/` (poses along +Z, calib) |
| `download_kitti_odometry.sh <dest>` | Fetch poses + calib zips into `data/kitti_odometry/` |
| `download_semantic_kitti_labels.sh <kitti_root>` | Fetch Semantic KITTI label archives |

## Evaluation pipelines

| Script | Purpose |
|--------|---------|
| `run_real_kitti.sh` | `eval_sequence` on seq 00 if odometry data present |
| `run_perception_eval.sh` | Preprocess (if velodyne) + eval with file/auto perception |
| `run_perception_tuning.sh` | `eval_perception_compare` oracle vs noisy |

## Visualization

| Script | Purpose |
|--------|---------|
| `run_viz_smoke.sh` | Offline PNG panel for smoke frame 20 |
| `run_ros_viz.sh` | Launch ROS 2 RViz node on smoke data |

See [docs/VISUALIZATION.md](../docs/VISUALIZATION.md) for manual `viz_frame` / `ros2 launch` usage.
