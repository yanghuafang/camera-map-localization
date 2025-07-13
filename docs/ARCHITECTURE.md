# Architecture

## Per-frame algorithm

`LocalizationEngine::ProcessFrame` runs the following pipeline for each KITTI frame:

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. Predict (frame > 0)                                          │
│    T_curr_prev from VO/GT ego → propagate SE(3) + process noise │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. Map matching (camera localization core)                      │
│    a. Rasterize perception polylines → distance transform       │
│    b. Pose grid: per-class DT cost for each (fwd, left, yaw)    │
│    c. Temporal aggregation over recent cost volumes             │
│    d. Argmin + parabolic sub-cell refinement                    │
│    e. Gate on ambiguity and on fit; else EKF update             │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. Optional updates                                             │
│    GT prior (--use-gt), else global odometry fallback           │
│      when map matching failed (--use-global-ego)                │
└────────────────────────────┬────────────────────────────────────┘
                             ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. WriteResult() → LocalizationResult + optional debug snapshot │
└─────────────────────────────────────────────────────────────────┘
```

### Frames

Two, and keeping them apart is most of what makes the rest work.

**cam0** — KITTI's rectified left camera: X right, Y down, Z forward. Every
pose, map point and world coordinate is in this frame, because that is what
KITTI odometry poses are.

**vehicle** — X forward, Y left, Z up. Nothing is stored in it. It exists so the
quantities a localizer reasons about mean what they say: a pose-grid offset of
(1, 0, 0) is one metre *forward*, and yaw is *heading*.

`core::Frames` holds the fixed rotation between them and is the only place the
conversion appears. A pose-grid offset is built as an SE(2) move in the vehicle
ground plane and rewritten in cam0, so composing it onto a pose moves the
hypothesis the way a vehicle moves.

### Pose grid

- DOF: **forward, left, yaw** in a sampling plane anchored at the KF estimate
  (or at ground truth with `--use-gt-plane`, for oracle experiments)
- Default grid: 21 × 31 × 13 = 8463 hypotheses, ±5 m × ±7.5 m × ±3°
- Map polylines are transformed per hypothesis, projected, and scored against
  the perception distance transform
- The argmin is refined below the cell pitch by fitting a parabola through it
  and its neighbours on each axis — otherwise the answer is quantized to the
  half-metre step, which shows up as a sawtooth in the trajectory error

### Scoring: per class, not per point

Each hypothesis is scored **per landmark class**, and the class scores are
averaged.

This is not a detail. A lane boundary is sampled every couple of metres along
the road, so it contributes an order of magnitude more points than the handful
belonging to a pole — and lane geometry runs parallel to travel, so its cost
barely changes as the hypothesis slides forward. Averaging over points lets that
flat majority outvote the sparse landmarks that actually pin along-track
position, and the argmin wanders off down the road. Per class, a pole counts as
much as a lane.

Within a class, a hypothesis that projects fewer than `min_support_points` gets
the shortfall scored as maximally wrong, so a pose seeing almost none of the map
cannot win by aligning the few points it does see. Every cell therefore lands in
`[0, max_cost]`, which is what makes the spread across the grid meaningful.

### What each feature class constrains

The grid searches three DOF, but map features are not interchangeable — each
constrains a different combination, so a map built from one kind alone leaves the
others unobservable no matter how good the perception is.

| Feature | Lateral | Longitudinal | Heading | Why |
|---------|---------|--------------|---------|-----|
| Lane line (solid or dashed) | strong | ~none | strong | Runs parallel to travel, so the cost surface aliases freely along the road. A dashed line's stripe *ends* would constrain longitudinally, but only if the map stores stripe endpoints rather than one continuous polyline — this one does not. |
| Road/curb boundary | strong | ~none | strong | Same geometry, same aliasing. |
| Pole | strong | **strong** | moderate | Best constraint per unit of effort: a point in the map, easy to detect, and dense on the suburban streets KITTI was recorded on. |
| Traffic sign | strong | **strong** | moderate | Elevated, so it is only scorable through the image branch — never through IPM/BEV, which assumes the ground plane. |
| Traffic light | strong | **strong** | moderate | Elevated, sparse, and far away in a forward camera; the lowest value of the six. |
| Crosswalk / stop line | weak | **strong** | strong | Perpendicular to travel, so it pins along-track position — but only at intersections. |

**What this project supports, and why.** The first four rows: lane markings,
road boundaries, poles and traffic signs. That is not a shortlist — it is
everything the datasets have. SemanticKITTI carries no traffic-light class and no
crosswalk class, so the last two rows cannot be extracted from it at any effort.

Lane geometry alone leaves along-track position unobservable, which is why the
poles and signs matter out of proportion to their number, and why the cost is
averaged per class rather than per point. `PoseSamplerTest.RecoversAlongTrackOffset`
is the regression test for exactly this: a 3 m along-track offset that lane
geometry cannot see and the upright landmarks can.

The eval measures this asymmetry rather than assuming it. `kitti::PoseError`
reports the translation error resolved onto the vehicle axes — see *Error
metrics* below — so the two columns of this table that differ most are the two
the numbers separate. On the `smoke_oracle_cpu` benchmark case — 50 frames of
oracle perception with the sampling plane at ground truth — the translation RMSE
of 3.0 mm is 2.9 mm along-track against 0.7 mm lateral, and 2.7 mm of that
2.9 mm is signed bias: nearly all of it is a steady lag, not noise.

### Cost fusion (camera)

| Branch | Input | Constrains | Default |
|--------|-------|-----------|---------|
| **Image** | All classes, in the image plane | forward, left, yaw | on |
| **BEV** | Ground-plane classes, inverse-projected onto the road | left, yaw | **off** |

Costs are averaged over the branches that ran, so enabling BEV changes the
balance of evidence rather than the scale.

> **BEV is off by default, on measurement.** On the smoke sequence it takes
> translation RMSE from 0.006 m to 0.022 m and yaw RMSE from 0.005° to 0.059°.
> The reason is structural: only ground-plane classes can go through inverse
> perspective, and a top-down view of lane geometry is invariant along the road —
> sliding a hypothesis forward costs nothing. The branch has no opinion about
> along-track position, and averaging it in dilutes the branch that does. It
> still constrains lateral offset and heading, which is why it is kept and can be
> switched on with `--bev`.
>
> An elevated pole or sign cannot go through this branch at all: inverse
> perspective assumes the point is on the road, so it would be placed at whatever
> range that assumption implies rather than where it is.

### Temporal aggregation

What accumulates across frames is evidence about the **pose error**, not about a
pose. Every sampling plane is the same drifting estimate seen at a different
time, so a grid cell meaning "the estimate is a metre long" means that in every
frame; the only thing that changes between two planes is the axes it is
expressed in. Each history offset is conjugated by the relative plane motion,
`M · T_offset · M⁻¹`.

Asking the old grid what it thought of where the vehicle is *now* reads the old
observation at a pose it never scored as good, and drags the estimate a frame's
travel backwards every frame.

- Ring buffer `window_size` deep (default 70, `--aggregation-window`)
- Weights decay linearly with distance travelled **since** each history frame
  (`distance_decay`, default 0.01 → zero at 100 m)
- History whose plane has moved further than the grid is wide is dropped: its
  warped offset falls off the grid, where sampling can only return a clamped
  border value
- The weighted average is fused 50/50 with the current frame; if no history
  carries weight, the current frame is left alone

### Kalman filter (`LocalizationKF`)

- **State:** SE(3) pose in KITTI world frame (error-state formulation)
- **Predict:** `Predict(T_curr_prev, Q)` — relative transform from egomotion
- **Updates (in order when applicable):**
  1. **Map observation** — best (x, y, yaw) from aggregated cost grid + covariance from cost surface spread
  2. **Global measurement** — full pose (GT prior with `--use-gt`, or an odometry anchor with `--use-global-ego` when map matching failed)

Two gates skip the map update, for opposite reasons. A **flat** cost surface
cannot tell hypotheses apart, so the argmin is noise. A uniformly **high** one
says none of them fit — which is what the last stretch of a finite map looks
like, and what a frame of bad perception looks like. Either way the argmin is
not evidence, and the filter coasts on the motion model instead.

The measurement covariance comes from the softmax-weighted spread of the cost
surface about the argmin, built on **vehicle axes** and rotated into the world
frame the filter's error state uses.

There is no ground-plane measurement. `--use-gt-plane` sets the *sampling-plane
anchor* — it builds the pose grid at the GT pose instead of the filter estimate,
for oracle experiments — and does not constrain z, roll or pitch.

## Inputs and outputs

`LocalizationEngine` takes a calibration and a map loader once, then one
`Egomotion` and one `FramePerception` per frame. Unless a row says otherwise,
everything is in the **KITTI rectified cam0 frame** — X right, Y down, Z forward
— and "world" means the cam0 world the odometry poses are expressed in.

> **This system never reads a camera image.** The only camera input is a set of
> image-space polylines, and today those come from projecting the map (oracle) or
> from SemanticKITTI's LiDAR labels — not from a camera perception model.
> `image_0/` is loaded by `viz_frame` and the ROS node for a background only.
>
> Perception is an *input*, never manufactured by the engine. Projecting the map
> at the sampling plane — which is the filter's own estimate — would make the
> observation move with the estimate, so the match would report success however
> far it had drifted. The oracle exists, but it projects at the **ground-truth**
> pose and is labelled as synthesized.

### Per-frame inputs

| Input | Shape | 2D/3D | Frame · units | Origin |
|-------|-------|-------|---------------|--------|
| `Egomotion::global.T_world_cam0` | 4×4 SE(3) | 3D | world ← cam0 · m | KITTI Odometry `poses/XX.txt`, downloaded |
| `Egomotion::global.timestamp_ns` | scalar | — | ns | Synthesized at 10 Hz (`frame · 1e8`); the odometry poses carry none |
| `Egomotion::T_curr_prev` | 4×4 SE(3) | 3D | body · m | Derived in C++ by `BuildEgomotion` from consecutive poses |
| `Egomotion::cov_global` | 6×6 | — | `[x, y, z, ωx, ωy, ωz]` · m², rad² | Fixed in `BuildEgomotion`: 0.25 m², 3e-4 rad² |
| `Egomotion::cov_relative` | 6×6 | — | same order | Set by `BuildEgomotion` and **never read** — the predict step uses `LocalizationKF::DefaultProcessCov()` instead |
| `FramePerception::features` | polylines of `Vec2`, each with a class | **2D** | rectified cam0 **image pixels** (u, v), via `P0` | See *Perception sources* below |
| `MapChunk::polylines` | polylines of `Vec3` | **3D** | world · m | Corridor from GT poses, JSON, or OSM + georef |

Set once, not per frame:

| Input | Shape | Frame · units | Origin and use |
|-------|-------|---------------|----------------|
| `Calibration::P0` | 3×4 | px | `dataset/sequences/XX/calib.txt`, downloaded. `fx, fy, cx, cy` for every projection — the only calibration the pose grid touches |
| `Calibration::P1` | 3×4 | px | Same file. **Parsed and never read**: the right camera is unused, as stereo is unimplemented |
| `Calibration::R0_rect` | 3×3 | — | Same file, identity when absent. Read only inside `T_cam0_velo()`, so only on the LiDAR preprocessing path |
| `Calibration::Tr_velo_to_cam0` | 3×4 | cam0 ← velodyne · m | Same file. Also only the LiDAR path |

### Perception sources

| Source | `--perception-mode` | Produced by | Status |
|--------|--------------------|-------------|--------|
| Oracle | `oracle` | C++ `SynthesizeFromMap`, projecting the local map at the **ground-truth** pose | Works. An upper bound on the backend, not a perception result |
| SemanticKITTI file | `file` | C++ `preprocess_kitti` → `<repo>-data/perception/<seq>/<frame:06d>.lanes.json` | Works. Lane markings, road boundaries, poles, traffic signs |
| Either, plus noise | `noisy` | C++ `AddPerceptionNoise` (seeded jitter, dropout, bias) | Works |
| File if present, else oracle | `auto` | The two above | Works (the default) |
| **Camera perception model** | — | — | **TODO.** No model is run in this repo. This is the input a real deployment would supply, and the reason only two of the six classes in *What each feature class constrains* are reachable today |

`preprocess_kitti` has two input paths of its own:

| `--mode` | Reads | Origin of that data |
|----------|-------|---------------------|
| `lidar` | `velodyne/NNNNNN.bin` (float32 x, y, z, intensity) + `labels/NNNNNN.label` (uint32, low 16 bits = class) | Velodyne archive (~80 GB, manual) + `scripts/download_semantic_kitti_labels.sh` |
| `png` | `<labels-root>/<seq>/labels/NNNNNN.label` — despite the extension, a **16-bit grayscale PNG** label raster | Prepared externally |

Both project or scan to image-space polylines and write the same JSON, so the
engine sees one 2-D contract regardless.

### Outputs

`LocalizationEngine::result()`, one per processed frame:

| Output | Shape | Frame · units |
|--------|-------|---------------|
| `T_world_rig` | 4×4 SE(3) | world ← cam0 · m — the estimate |
| `covariance` | 6×6 | error state `[x, y, z, ωx, ωy, ωz]` |
| `best_sample_xyyaw` | `Vec3` | **not a point** — the packed offset `(x_m, y_m, yaw_rad)` of the winning cell, in the sampling plane |
| `best_offset_norm_m` | double | ‖(x, y)‖ of that offset · m |
| `aggregate_min_cost` | float | mean distance-transform cost at the argmin · **pixels**, capped at 5 |
| `cost_map_spread` | float | max − min over the grid · DT pixels |
| `cost_map_flat`, `match_cost_too_high` | bool | why a map update was skipped, if it was |
| `sampling_measurement_applied`, `valid` | bool | per-frame diagnostics |
| `frame`, `timestamp_ns` | scalar | passthrough from the input |

Alongside, when enabled: `LocalizationDebugSnapshot` (both DT rasters, raw and
aggregated cost volumes, argmin) via `set_debug_capture(true)`; per-frame CSV and
benchmark JSON from the eval apps; PNG layers from `viz_frame`.

File formats and download commands for every "Origin" above are in
[KITTI_DATA.md](KITTI_DATA.md#input-provenance).

> The grid's three axes are forward, left and heading, in the vehicle sense —
> see *Frames* above.

### Error metrics

Engine outputs are poses; the eval layer turns them into errors.
`kitti::PoseError` ([eval_metrics.h](../include/cam_loc/kitti/eval_metrics.h))
compares an estimate against a ground-truth pose and reports both a magnitude
and a decomposition:

| Field | Signed | Frame · units |
|-------|--------|---------------|
| `translation_m` | no | ‖t_est − t_gt‖ · m |
| `yaw_deg` | no | heading, [0, 180] · deg |
| `longitudinal_m` | **yes**, + = ahead of truth | GT **vehicle** axes · m |
| `lateral_m` | **yes**, + = left of truth | GT vehicle axes · m |
| `vertical_m` | **yes**, + = above truth | GT vehicle axes · m |

The error is rotated into the **ground-truth** vehicle frame, not the estimate's:
the axes an error is reported on must not move with the error being reported, or
a heading mistake rotates its own yardstick. `ToVehicleAxes` is the conversion,
which is `core::Frames::ToVehicle` — the same one the pose grid uses, so the
components mean what a grid offset means.

Being a rotation, the split loses nothing: `translation_m² = longitudinal_m² +
lateral_m² + vertical_m²`. The vertical term is carried only to close that
identity — no measurement constrains z, roll or pitch.

`ErrorSummary` rolls a sequence up with an RMSE, a signed mean (`bias_*`) and a
worst-frame excursion (`max_abs_*`) per axis. The first two are both needed. A steady 0.3 m along-track lag and 0.3 m of symmetric
along-track jitter have identical RMSE, and the failure this project is prone to
is the first one — lane geometry aliases along the road, so a hypothesis that has
slid forward costs almost nothing. `SequenceEvalTest.BiasSurvivesWhereRmseDoesNot`
holds that distinction.

Nothing gates on the per-axis numbers. `BenchmarkThresholds` is unchanged: a
case fails on translation RMSE, yaw RMSE, match rate, flat rate or mean frame
time, and on nothing else.

## Device placement

| Stage | CPU | CUDA (when `use_cuda`) |
|-------|-----|------------------------|
| KF predict / updates | ✓ | — |
| DT raster + EDT | ✓ fallback | ✓ Felzenszwalb GPU |
| Pose-grid cost sampling | ✓ | ✓ image + BEV kernels |
| Temporal aggregation | ✓ fallback | ✓ `AggregateCostsGpu` |
| Argmin + variance | ✓ | ✓ GPU reduce |

## Module boundaries

```
apps/ (CLI)
  run_sequence, eval_sequence, eval_perception_compare,
  benchmark, viz_frame, preprocess_kitti
        │
        ▼
cam_loc::LocalizationEngine
  ├── map::CreateMapLoader → corridor / JSON / OSM
  ├── perception::PerceptionAdapter (+ noise, resolve)
  ├── core::LocalizationKF
  ├── core::PoseSampler, CostAggregator, DistanceTransform
  └── cuda::* (optional, linked via cam_loc_cuda)
```

## Map sources (KITTI)

| Source | When | Implementation |
|--------|------|----------------|
| Trajectory corridor | Default (no `--map-path`) | `TrajectoryCorridorMap`: lane geometry on the road surface, plus poles and signs beside it, all offset from the GT path |
| JSON polylines | `--map-path *.json` | World-frame polylines |
| OSM XML | `--map-path *.osm` + georef | `OsmMapLoader`, `MapGeoref` |

See [KITTI_DATA.md](KITTI_DATA.md) for layout and georef JSON.

## Key source files

| Area | Path |
|------|------|
| Engine orchestration | `src/core/localization_engine.cc` |
| EKF | `src/core/localization_kf.cc` |
| Pose grid + aggregation | `src/core/pose_sampler.cc`, `cost_aggregator.cc` |
| CUDA | `src/cuda/distance_transform_kernels.cu`, `distance_transform_gpu.cc` |
| Params | `include/cam_loc/types/params.h` |
| Debug capture | `include/cam_loc/core/localization_debug.h` |
