# Running on KITTI

End-to-end guides for smoke tests (no download) and real KITTI Odometry.

Related: [KITTI_DATA.md](KITTI_DATA.md) (formats).

Paths used throughout, both siblings of the repository:

```bash
B=../camera-map-localization-build   # ./scripts/ci.sh writes here
D=../camera-map-localization-data    # datasets and evaluation output
```

## Smoke test (no download)

Synthetic trajectory with poses + calibration only; perception is synthesized from the corridor map.

```bash
./scripts/ci.sh --no-style   # configure, build, test
./scripts/run_smoke.sh
```

This creates `<repo>-data/smoke_kitti/` and runs `run_sequence` for 50 frames.

Individual steps:

```bash
./scripts/prepare_smoke_kitti.sh 120   # 120 m straight drive along +Z
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/smoke_kitti \
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
camera-map-localization-data/kitti_odometry/
  poses/00.txt
  dataset/sequences/00/calib.txt
  dataset/sequences/00/image_0/   # optional
```

### Download script

```bash
./scripts/download_kitti_odometry.sh "$D"/kitti_odometry
```

### Quick eval

```bash
./scripts/run_real_kitti.sh
```

Or manually:

```bash
"$B"/apps/eval_sequence/eval_sequence \
  --kitti-root "$D"/kitti_odometry \
  --sequence 00 \
  --max-frames 200 \
  --skip-frames 10 \
  --output-csv "$D"/eval_seq00.csv
```

`eval_sequence` tuning flags: `--cost-flat-threshold`, `--cost-softmax-scale`,
`--aggregation-window`, `--noise-px`, and `--bev` to also score the bird's-eye
branch (off by default).

`--max-frames` is a **count** from `--skip-frames`, so
`--skip-frames 10 --max-frames 80` runs frames 10 through 89.

### `run_sequence` on real data

```bash
# Synthesized perception from the GT corridor map
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/kitti_odometry \
  --sequence 00 \
  --max-frames 200
```

## Perception tuning (oracle vs noisy)

```bash
"$B"/apps/eval_perception_compare/eval_perception_compare \
  --kitti-root "$D"/kitti_odometry \
  --perception-root "$D"/perception \
  --sequence 00 \
  --skip-frames 10 \
  --max-frames 200 \
  --noise-px 4 \
  --use-gt-plane \
  --output-csv "$D"/eval_perception_compare_seq00.csv
```

`--perception-mode` (`auto | file | oracle | noisy`) is accepted by
`eval_sequence`; `eval_perception_compare` always runs oracle, file and noisy
for comparison, so the three share one frame range and one map.

## External map

```bash
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/kitti_odometry \
  --map-path "$D"/map/00/corridor.map.json \
  --sequence 00
```

OSM with georef: see [KITTI_DATA.md](KITTI_DATA.md#native-osm--georef).

## Interpreting output

`run_sequence` prints the mean translation error (m) and mean yaw error (deg) vs GT.

`eval_sequence` additionally reports:

| Metric | Meaning |
|--------|---------|
| Translation RMSE (m) | root-mean-square of ‖t_est − t_gt‖ over frames (mean and max also printed) |
| Yaw RMSE (deg) | RMS absolute yaw difference vs GT (mean also printed) |
| Match rate | fraction of frames with an applied (non-flat) map-matching update |
| Flat cost rate | fraction of frames where the map update was skipped (flat cost surface) |

`--use-gt` injects near-perfect global measurements (KF path check). Default mode uses map matching + KF only.
