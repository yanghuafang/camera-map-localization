# Running on KITTI

End-to-end guides for smoke tests (no download), real KITTI Odometry, perception preprocessing, and evaluation.

Related: [KITTI_DATA.md](KITTI_DATA.md) (formats), [BENCHMARK.md](BENCHMARK.md), [VISUALIZATION.md](VISUALIZATION.md).

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
./scripts/prepare_smoke_kitti.sh       # 400 m straight drive along +Z
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
  --use-cuda \
  --output-csv "$D"/eval_seq00.csv
```

### `run_sequence` on real data

```bash
# Synthesized perception from GT corridor map
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/kitti_odometry \
  --sequence 00 \
  --max-frames 200

# GPU pose-grid + DT
"$B"/apps/run_sequence/run_sequence \
  --kitti-root "$D"/kitti_odometry \
  --sequence 00 \
  --max-frames 200 \
  --use-cuda
```

## Semantic KITTI perception (optional)

### 1. Download labels

```bash
./scripts/download_semantic_kitti_labels.sh "$D"/kitti_odometry
```

### 2. Velodyne scans

For LiDAR-based preprocessing, add velodyne binaries from the KITTI Odometry velodyne archive (~80 GB full set):

`<repo>-data/kitti_odometry/dataset/sequences/00/velodyne/000000.bin`

### 3. Preprocess to JSON

```bash
# LiDAR labels projected to image (needs velodyne). Extracts lane markings,
# road boundaries, poles and traffic signs; SemanticKITTI has no traffic-light
# or crosswalk class.
"$B"/apps/preprocess_kitti/preprocess_kitti \
  --mode lidar \
  --kitti-root "$D"/kitti_odometry \
  --output-root "$D"/perception \
  --sequence 00 \
  --start 10 --end 200

# Or label mode from a prepared 16-bit label directory (no velodyne).
# png mode reads <labels-root>/<seq>/labels/NNNNNN.label (not --kitti-root).
"$B"/apps/preprocess_kitti/preprocess_kitti \
  --mode png \
  --labels-root "$D"/semantic_labels \
  --output-root "$D"/perception \
  --sequence 00
```

### 4. Evaluate with file perception

```bash
"$B"/apps/eval_sequence/eval_sequence \
  --kitti-root "$D"/kitti_odometry \
  --perception-root "$D"/perception \
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
"$B"/apps/eval_perception_compare/eval_perception_compare \
  --kitti-root "$D"/kitti_odometry \
  --perception-root "$D"/perception \
  --sequence 00 \
  --skip-frames 10 \
  --max-frames 200 \
  --noise-px 4 \
  --use-gt-plane \
  --use-cuda \
  --output-csv "$D"/eval_perception_compare_seq00.csv
```

`--perception-mode` (`auto | file | oracle | noisy`) is accepted by `eval_sequence` and `viz_frame`; `eval_perception_compare` always runs oracle, file, and noisy for comparison.

`eval_sequence` tuning flags: `--cost-threads`, `--adaptive-extent`,
`--cost-softmax-scale`,
`--aggregation-window`, `--noise-px`, and `--bev` to also score the bird's-eye
branch (off by default — see [ARCHITECTURE.md](ARCHITECTURE.md)).

### Odometry error

`--noise-px` and friends corrupt what the camera sees. These corrupt what the
wheels report, which is the other half of the system and the one every sequence
here otherwise gets for free:

| Flag | Meaning |
|------|---------|
| `--ego-scale-error F` | systematic scale error on travelled distance, as a fraction (`0.01` = an odometer reading 1% long) |
| `--ego-scale-sigma F` | standard deviation of the random part, per step |
| `--ego-heading-bias F` | systematic heading drift, degrees per metre travelled |
| `--ego-heading-sigma F` | standard deviation of the random part, degrees per metre |
| `--ego-noise-seed N` | seed; the same seed and frame always give the same perturbation |

The units are per *metre*, not per frame, because that is how odometry error
behaves — a vehicle standing still accumulates none of it however long it
stands. The systematic terms matter more than the random ones: white noise
averages out over a window, while a scale error or a heading bias integrates
into the steady drift that dead reckoning is actually limited by, and that a map
match has to keep pulling back.

Only the relative motion is corrupted. The global pose stays truthful, because
it is what the filter initializes from on frame 0 — a localizer that cannot be
told where it starts is a different problem — and it is not fed back as a
measurement unless `--use-global-ego` asks for it.

Measured on the smoke sequence with oracle perception, 120 frames:

| Odometry | Translation RMSE | Yaw RMSE | ANEES | Within 95% |
|----------|-----------------|----------|-------|------------|
| ground truth | 0.308 m | 0.104° | 0.30 | 100% |
| random only, matched to the default `Q` | 0.426 m | 0.225° | 0.53 | 100% |
| 1% scale error | 0.930 m | 0.104° | 1.66 | 67% |
| 1% scale, 0.05°/m bias, both random terms | 1.271 m | 1.097° | 5.22 | 28% |

ANEES crosses 1 between the second and third rows, so the metric now discriminates
rather than sitting pinned near zero — but read which way it crosses. **The filter
is over-confident under odometry drift**: at a 1% scale error only 67% of frames
sit inside a bound that should hold 95% of them, and past what `OdometryNoiseParams`
models it is 28%. Over 400 m a 1% scale error is 4 m of accumulated drift and lane
geometry gives the matcher almost nothing to pull it back with, so the error grows
faster than the covariance does. The 60 m route this replaces was too short to show
it at all.

It reads that way only because **both** halves carry error. Ask for a perfect map
with `--map-error-m 0` and the first row collapses to 0.020 m at ANEES 0.002 — the
filter is not suddenly better, the thing it is being scored against has stopped
being a measurement.

Inject more drift than `OdometryNoiseParams` models — the last row runs twice
the heading bias and twice the random heading drift — and the filter is
genuinely over-confident: ANEES above 1, frames falling outside the bound.
Inject what it does model and it stays conservative.

`--max-frames` is a **count** from `--skip-frames`, so
`--skip-frames 10 --max-frames 80` runs frames 10 through 89.

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

`eval_sequence` (and `benchmark`) additionally report:

| Metric | Meaning |
|--------|---------|
| Translation RMSE (m) | root-mean-square of ‖t_est − t_gt‖ over frames (mean and max also printed) |
| Yaw RMSE (deg) | RMS absolute yaw difference vs GT (mean also printed) |
| Match rate | fraction of frames with an applied (non-flat) map-matching update |
| Flat cost rate | fraction of frames where the map update was skipped (flat cost surface) |
| Mean mode margin | mean cost gap from the winner to the next separated minimum, DT pixels; small means a rival pose fits nearly as well |
| Multimodal rate | fraction of frames where a second minimum was found at all |
| Mean cells searched | hypotheses scored per frame; equals the grid size unless `--adaptive-extent` narrowed it |
| NEES mean | mean of `eᵀP⁻¹e` over frames the filter published; 6 when the covariance is honest |
| ANEES | NEES mean over its 6 degrees of freedom, so 1.0 is the target |
| Within 95% bound | fraction of frames whose NEES stays under the 6-DOF chi-square 95% point (12.59); 95% is the target |

### Reading the consistency numbers

RMSE says how far the estimate is from truth. It cannot say whether the filter
*knew* it would be that far, and the covariance it publishes is what every
consumer downstream plans against. ANEES answers that: 1.0 is a filter whose
uncertainty matches its error, above 1.0 is over-confident, below 1.0 is
needlessly conservative.

The exceedance rate is printed beside it because the mean alone is ambiguous. A
filter that is uniformly a little too sure of itself and one that is fine except
for three divergent frames can report the same ANEES, and only the fraction of
frames inside the bound tells them apart.

> **KITTI ships no odometry stream**, so by default the prediction step is fed
> relative motion derived from the ground-truth poses
> ([KITTI_DATA.md](KITTI_DATA.md)) — perfect input, against which the filter
> still accumulates process noise every frame. That inflates the covariance
> against an error that never grows, and it is why the default ANEES on the
> smoke sequence would be 0.0002 rather than anything near 1 if the map were
> also perfect. Use the odometry-error flags below together with the map's own
> survey error: with both, ANEES reads 0.96 at a 1% scale error and crosses 1
> past the drift the filter models.
>
> The update step needs the same treatment and gets it from the map rather than
> from a flag: the corridor is surveyed wrong, so the map measurement carries
> real error for its covariance to be calibrated against. `--map-error-m 0`
> turns that off and is the quickest way to see what a one-sided evaluation
> does to the metric. A meaningful ANEES needs error on both halves.

`--use-gt` injects near-perfect global measurements (KF path check). Default mode uses map matching + KF only.
