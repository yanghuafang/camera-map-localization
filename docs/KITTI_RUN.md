# Running on KITTI

End-to-end guides for smoke tests (no download), real KITTI Odometry, perception preprocessing, and evaluation.

Related: [KITTI_DATA.md](KITTI_DATA.md) (formats), [BENCHMARK.md](BENCHMARK.md), [VISUALIZATION.md](VISUALIZATION.md).

## Smoke test (no download)

Synthetic trajectory with poses + calibration only; perception is synthesized from the corridor map.

```bash
cmake -B build -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"

./scripts/run_smoke.sh
```

This creates `data/smoke_kitti/` and runs `run_sequence` for 50 frames.

Individual steps:

```bash
./scripts/prepare_smoke_kitti.sh 120   # 120 m straight drive along +Z
./build/apps/run_sequence/run_sequence \
  --kitti-root data/smoke_kitti \
  --sequence 00 \
  --max-frames 50 \
  --use-gt-plane
```

Use `--use-gt-plane` for oracle validation (sampling plane aligned with GT). Without it, the grid is centered on the KF estimate (realistic mode).

## Real KITTI Odometry

Register and download from [KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php):

| Archive | Contents |
|---------|----------|
| `data_odometry_poses.zip` | `poses/XX.txt` |
| `data_odometry_calib.zip` | `dataset/sequences/XX/calib.txt` |
| `data_odometry_gray.zip` (optional) | `dataset/sequences/XX/image_0/` |

Expected tree:

```
data/kitti_odometry/
  poses/00.txt
  dataset/sequences/00/calib.txt
  dataset/sequences/00/image_0/   # optional
```

### Download script

```bash
./scripts/download_kitti_odometry.sh data/kitti_odometry
```

### Quick eval

```bash
./scripts/run_real_kitti.sh
```

Or manually:

```bash
./build/apps/eval_sequence/eval_sequence \
  --kitti-root data/kitti_odometry \
  --sequence 00 \
  --max-frames 200 \
  --skip-frames 10 \
  --use-cuda \
  --output-csv data/eval_seq00.csv
```

### `run_sequence` on real data

```bash
# Synthesized perception from GT corridor map
./build/apps/run_sequence/run_sequence \
  --kitti-root data/kitti_odometry \
  --sequence 00 \
  --max-frames 200

# GPU pose-grid + DT
./build/apps/run_sequence/run_sequence \
  --kitti-root data/kitti_odometry \
  --sequence 00 \
  --max-frames 200 \
  --use-cuda
```

## Semantic KITTI perception (optional)

### 1. Download labels

```bash
./scripts/download_semantic_kitti_labels.sh data/kitti_odometry
```

### 2. Velodyne scans

For LiDAR-based preprocessing, add velodyne binaries from the KITTI Odometry velodyne archive (~80 GB full set):

`data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin`

### 3. Preprocess to JSON

```bash
# LiDAR labels projected to image (needs velodyne)
./build/apps/preprocess_kitti/preprocess_kitti \
  --mode lidar \
  --kitti-root data/kitti_odometry \
  --output-root data/perception \
  --sequence 00 \
  --start 10 --end 200

# Or label mode from a prepared 16-bit label directory (no velodyne).
# png mode reads <labels-root>/<seq>/labels/NNNNNN.label (not --kitti-root).
./build/apps/preprocess_kitti/preprocess_kitti \
  --mode png \
  --labels-root data/semantic_labels \
  --output-root data/perception \
  --sequence 00
```

### 4. Evaluate with file perception

```bash
./build/apps/eval_sequence/eval_sequence \
  --kitti-root data/kitti_odometry \
  --perception-root data/perception \
  --sequence 00 \
  --perception-mode file \
  --max-frames 200 \
  --skip-frames 10 \
  --use-cuda
```

Full pipeline (falls back to synthesized perception if velodyne is missing):

```bash
./scripts/run_perception_eval.sh
```

## Perception tuning (oracle vs noisy)

```bash
./scripts/run_perception_tuning.sh
```

Or:

```bash
./build/apps/eval_perception_compare/eval_perception_compare \
  --kitti-root data/kitti_odometry \
  --perception-root data/perception \
  --sequence 00 \
  --skip-frames 10 \
  --max-frames 200 \
  --noise-px 4 \
  --use-gt-plane \
  --use-cuda \
  --output-csv data/eval_perception_compare_seq00.csv
```

`--perception-mode` (`auto | file | oracle | noisy`) is accepted by `eval_sequence` and `viz_frame`; `eval_perception_compare` always runs oracle, file, and noisy for comparison.

`eval_sequence` tuning flags: `--cost-flat-threshold`, `--cost-softmax-scale`, `--aggregation-window`, `--noise-px`.

## External map

```bash
./build/apps/run_sequence/run_sequence \
  --kitti-root data/kitti_odometry \
  --map-path data/map/00/corridor.map.json \
  --sequence 00
```

OSM with georef: see [KITTI_DATA.md](KITTI_DATA.md#native-osm--georef).

## Interpreting output

`run_sequence` prints the mean translation error (m) and mean yaw error (deg) vs GT.

`eval_sequence` (and `benchmark`) additionally report:

| Metric | Meaning |
|--------|---------|
| Translation RMSE (m) | root-mean-square of ‖t_est − t_gt‖ over frames (mean and max also printed) |
| Yaw RMSE (deg) | RMS absolute yaw difference vs GT (mean also printed) |
| Match rate | fraction of frames with an applied (non-flat) map-matching update |
| Flat cost rate | fraction of frames where the map update was skipped (flat cost surface) |

`--use-gt` injects near-perfect global measurements (KF path check). Default mode uses map matching + KF only.
