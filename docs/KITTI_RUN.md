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

### `run_sequence` on real data

```bash
# Synthesized perception from the GT corridor map
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/kitti_odometry \
  --sequence 00 \
  --max-frames 200
```

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

`--use-gt` injects near-perfect global measurements (KF path check). Default mode uses map matching + KF only.
