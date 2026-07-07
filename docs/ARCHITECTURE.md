# Architecture

## Per-frame algorithm

`LocalizationEngine::processFrame` runs the following pipeline for each KITTI frame:

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. Predict (frame > 0)                                          │
│    T_curr_prev from VO/GT ego → propagate SE(3) + process noise │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Map matching (camera localization core)                      │
│    a. Load perception (file / oracle / noisy)                   │
│    b. Rasterize BEV + image polylines → distance transforms     │
│    c. Pose grid: sample DT costs for each (x, y, yaw) hypothesis│
│    d. Temporal aggregation over ring buffer of past cost volumes│
│    e. Argmin → best (x, y, yaw) + cost surface stats            │
│    f. If cost surface not flat → EKF update (map observation)   │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Optional updates                                             │
│    GT prior (--use-gt) or global odometry fallback              │
│    Flat-ground plane prior (z, roll, pitch) when enabled        │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. writeResult() → LocalizationResult + optional debug snapshot │
└─────────────────────────────────────────────────────────────────┘
```

### Pose grid

- DOF: **x, y, yaw** in a local sampling plane anchored at the KF estimate (or GT plane with `--use-gt-plane`)
- Default grid: 21 × 31 × 13 = 8463 hypotheses (see `LocalizationParams`)
- Map polylines transformed per hypothesis; cost = weighted distance-transform lookup

### Cost fusion (camera)

| Branch | Input | Raster | DT | Notes |
|--------|-------|--------|-----|-------|
| **BEV** | Lane lines + road boundaries | BEV canvas | Felzenszwalb EDT | Primary path for KITTI |
| **Image** | Perception in image plane | Per-camera canvas | Felzenszwalb EDT | Used when image polylines present |

Both branches contribute to the pose-grid cost; BEV dominates for corridor-map validation.

### Temporal aggregation

- Ring buffer of past per-frame 3D cost volumes (`CostAggregator`)
- For each current hypothesis, back-project into past volumes using relative pose
- Weights decay with traveled distance (`aggregation_window` frames)

### Kalman filter (`LocalizationKF`)

- **State:** SE(3) pose in KITTI world frame (error-state formulation)
- **Predict:** `predict(T_curr_prev, Q)` — relative transform from egomotion
- **Updates (in order when applicable):**
  1. **Map observation** — best (x, y, yaw) from aggregated cost grid + covariance from cost surface spread
  2. **Global measurement** — full pose (e.g. GT prior with noise, or odometry anchor)
  3. **Plane measurement** — optional flat-ground constraint on z, roll, pitch

Flat cost surfaces skip the map update (`match=false`, `flat=true`).

## Device placement

| Stage | CPU | CUDA (when `use_cuda`) |
|-------|-----|------------------------|
| KF predict / updates | ✓ | — |
| DT raster + EDT | ✓ fallback | ✓ Felzenszwalb GPU |
| Pose-grid cost sampling | ✓ | ✓ image + BEV kernels |
| Temporal aggregation | ✓ fallback | ✓ `aggregateCostsGpu` |
| Argmin + variance | ✓ | ✓ GPU reduce |

## Module boundaries

```
apps/ (CLI)
  run_sequence, eval_sequence, eval_perception_compare,
  benchmark, viz_frame, preprocess_kitti
        │
        ▼
cam_loc::LocalizationEngine
  ├── map::createMapLoader → corridor / JSON / OSM
  ├── perception::PerceptionAdapter (+ noise, resolve)
  ├── core::LocalizationKF
  ├── core::PoseSampler, CostAggregator, DistanceTransform
  └── cuda::* (optional, linked via cam_loc_cuda)
```

## Map sources (KITTI)

| Source | When | Implementation |
|--------|------|----------------|
| Trajectory corridor | Default (no `--map-path`) | `TrajectoryCorridorMap` from GT poses ± lane width |
| JSON polylines | `--map-path *.json` | World-frame polylines |
| OSM XML | `--map-path *.osm` + georef | `OsmMapLoader`, `MapGeoref` |

See [KITTI_DATA.md](KITTI_DATA.md) for layout and georef JSON.

## Key source files

| Area | Path |
|------|------|
| Engine orchestration | `src/core/localization_engine.cpp` |
| EKF | `src/core/localization_kf.cpp` |
| Pose grid + aggregation | `src/core/pose_sampler.cpp`, `cost_aggregator.cpp` |
| CUDA | `src/cuda/distance_transform.cu`, `pose_sampler_*.cu` |
| Params | `include/cam_loc/types/params.hpp` |
| Debug capture | `include/cam_loc/core/localization_debug.hpp` |
