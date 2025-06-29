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
│    d. Argmin + sub-cell step on the local quadratic             │
│    e. Drop only if no minimum or a bad fit; else EKF update     │
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

> **BEV is off by default — but no longer on measurement.** The figure that
> justified it, 0.0022 m becoming 0.0078 m, came from a run with no map error,
> and that run is a closed loop: the oracle projects the geometry the matcher
> scores, so the image branch is already perfect and a second branch that says
> nothing about along-track can only dilute it.
>
> Against a map surveyed to 0.2 m the picture inverts. Translation RMSE is a
> wash — 0.3083 m against 0.3174 m with the branch on — and the **match rate
> goes 95% to 100%**, because the extra lateral evidence carries frames the gate
> would otherwise drop. At 6 px of perception noise it is 62% to 90%, at a 1%
> scale error 68% to 92%, and ANEES moves toward 1 rather than away in each.
>
> The structural claim survives and is visible in the numbers: the along-track
> column barely moves in any of those pairs, because inverse perspective on lane
> geometry genuinely carries no information about position along the road. What
> does not survive is the conclusion drawn from it. Off remains the default
> because a single synthetic sequence is thin evidence for flipping one, not
> because the branch was measured to hurt.

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

- Ring buffer `window_size` deep (default 12, `--aggregation-window`) — a frame
  stops contributing once its plane has moved past the grid half-extent, 7.5 m
  at the default grid, which is around six frames at KITTI speeds
- Weights decay linearly with distance travelled **since** each history frame
  (`distance_decay`, default 0.01 → zero at 100 m)
- History whose plane has moved further than the grid is wide is dropped: its
  warped offset falls off the grid, where sampling can only return a clamped
  border value
- The weighted average is fused 50/50 with the current frame; if no history
  carries weight, the current frame is left alone
- Consecutive aggregates share all but one of their frames, so the same evidence
  reaches the filter repeatedly. `effective_frames()` reports `1 / Σpᵢ²` over
  the fusion weights, and the map measurement covariance is widened by it. The
  50/50 fuse caps it near 4: history never outweighs the current frame by more
  than one to one

### Sub-cell refinement

The argmin alone quantizes the answer to the cell pitch. `RefinedOffset` steps
to the minimum of the local quadratic, `-H⁻¹g`, from the same
`FitLocalQuadratic` the measurement covariance reads its curvature from — one
fit, so the pose the filter is given and the uncertainty it comes with cannot
disagree. Unlike one parabola per axis it sees the cross terms, and a basin at
an angle to the grid axes is a cross term and nothing else.

The eigenvalues are damped before inverting. The basin is extremely
anisotropic: on this data the along-track curvature runs under a thousandth of
the sharpest, so an undamped `-g/λ` along it is enormous, chases noise, and
carries into the axes that are determined. Damping by a fraction of the sharpest
curvature leaves those almost untouched and shrinks the flat direction's step
towards the cell centre. The step is then held inside half a cell.

Dropping the flat directions outright instead of damping them gives the same
answer to three figures, and the damping constant can move two decades without
changing it. That insensitivity *is* the measurement: the direction carries no
information to trade off.

**Still no measurement here shows this is an improvement, and now for a
different reason.** The earlier verdict rested on the axis split: every apparent
gain was along-track, and along-track was being handed to a ground-truth motion
model, so the refinement was substituting truth for a measurement rather than
measuring better.

Against a map surveyed to 0.2 m the gain does not reappear — it is buried. At
6 px of perception noise the total error is 0.272 m, of which 0.259 m is lateral
and 0.083 m along-track, and the whole quantity the refinement moves is a
sub-cell correction of a few centimetres inside an error the map has already
set. The refinement is now *below the noise floor of its own benchmark*.

That is a more honest place to be than the old number, and it is still not
evidence. What would settle it is a sequence where the along-track axis is
genuinely observable and genuinely wrong — upright landmarks the map has
surveyed badly, with odometry that drifts — so that a better sub-cell estimate
has something to be better than.

It is kept on the argument rather than the number. A per-axis parabola on a
near-flat axis takes a noise-driven half-cell step every frame, and that
direction lies at an angle to the grid where only the cross terms reach it.
Reading the pose and the covariance off the same local fit is worth something on
its own: two fits would let the estimate and the confidence in it describe
different surfaces.

### Adaptive extent

The grid is anchored on the filter estimate, so the offset that reaches truth is
distributed roughly as the filter's covariance. Once the filter is locked on,
most of a fixed grid is spent proving that a pose five metres away is still
wrong. With `adaptive_extent.enabled` (`--adaptive-extent`) the engine sizes the
searched region from that covariance, resolved onto the vehicle axes the grid is
indexed by, at `sigma_multiple` sigmas per axis.

The grid does not change size — history warping, the CUDA aggregate, mode
extraction and the covariance all read a volume of fixed dimensions. Only which
cells are *scored* changes; the rest hold `max_cost`, which cannot win and
carries negligible softmax weight.

Two things force it back open, and both are necessary:

- **A gated frame.** A skipped map update is the filter saying it cannot see
  where it is, and shrinking the search on a covariance nothing has corrected
  since is how a localizer loses lock and then cannot find its way back.
- **A winner on the window's edge.** This is the one signal available online
  that the covariance the window came from is too small. It has to exist: an
  over-confident filter sizes the window from its own mistake, and would
  otherwise never look far enough to notice. On mis-modelled odometry the smoke sequence
  reads 0.302 m with the check against a fixed grid's 0.308 m; without it the
  window narrows on a covariance nothing has corrected and the run loses lock.

Measured on 120 smoke frames and 60 of KITTI 00, oracle perception:

| | cells scored | frame latency | translation RMSE |
|---|---|---|---|
| smoke, fixed | 8463 | 6.4 ms | 0.3083 m |
| smoke, adaptive | 1064 | 5.6 ms | 0.3017 m |

Latency falls by less than the cell count does, because the distance transform
and the map query do not shrink with the search. Accuracy is unchanged, and on
the smoke sequence marginally better — a narrower window cannot be captured by a
distant spurious minimum.

It is off by default, though the cost that justified that has gone with the
closed loop. Against a surveyed map the adaptive window is a small *improvement*
even on mis-modelled odometry — 0.302 m against a fixed grid's 0.308 m, at
1064 cells instead of 8463 — because a narrower window cannot be captured by a
distant spurious minimum. Off remains the default for the same reason as the
bird's-eye branch: one synthetic sequence is thin evidence for flipping one.

The CPU search fans the flattened hypothesis index out over `cost_threads`
threads (`--cost-threads`, 0 asks the hardware). Each hypothesis owns one cell,
so the blocks share nothing and any thread count gives bitwise the same grid.
On macOS this is the only acceleration there is: CUDA has no toolchain there,
so `IsAvailable()` is false and every GPU entry point returns `kNotImplemented`.

### Kalman filter (`LocalizationKF`)

- **State:** SE(3) pose in KITTI world frame (error-state formulation)
- **Predict:** `Predict(T_curr_prev, Q)` — relative transform from egomotion,
  with `Q` built by `MotionProcessCov` from the step it is about to take

The process noise is sized **per metre travelled**, not per frame. Odometry
error is produced by motion: a vehicle standing at a light accumulates none of
it, and a fixed per-frame `Q` says otherwise — it inflates the covariance
through every stop, so the filter leaves the light less sure of a pose that
never moved. Standing still adds only a small floor, which exists so `P` cannot
reach singularity and leave every downstream gate undefined.

The per-axis sigmas are stated in **vehicle axes** and rotated into the world
frame through the current attitude, because they mean different things: along-
track scale error dominates wheel and visual odometry, sideways and vertical
slip are far smaller, and heading is separated from roll and pitch, which a road
vehicle barely accumulates. Stating them in vehicle axes is also what makes the
big along-track term follow the direction the vehicle is actually pointing,
rather than the direction it happened to face at initialization.
- **Updates (in order when applicable):**
  1. **Map observation** — best (x, y, yaw) from aggregated cost grid + covariance from cost surface spread
  2. **Global measurement** — full pose (GT prior with `--use-gt`, or an odometry anchor with `--use-global-ego` when map matching failed)

Beside the argmin the engine reports the volume's **separated local minima**.
The argmin says where the best hypothesis is; it does not say whether anything
else fits nearly as well. `mode_margin` is the cost gap to the next minimum at
least two cells away, and a small gap means the winner is a choice rather than a
conclusion — the lane-only trough carries several minima centimetres apart in
cost and metres apart in pose. On the smoke sequence every frame has a second
minimum, at a mean margin of 1.12 DT pixels with oracle perception and 0.63 with
4 px of noise.

A runner-up that cannot be ruled out makes the measurement a two-component
mixture rather than one Gaussian, and a mixture's covariance carries
`p₁·p₂·ddᵀ` over the separation `d`. That term inflates along the direction the
two candidates disagree on and nowhere else, vanishes when the runner-up is far
behind on cost, and reaches `(d/2)²` when the two are equally good.

Two gates skip the map update, for opposite reasons. A volume with **no
separated minimum** has no hypothesis to offer: every pose fits alike and the
argmin is whichever cell rounding favoured. A uniformly **high** one says none
of them fit — which is what the last stretch of a finite map looks like, and
what a frame of bad perception looks like. Either way the filter coasts on the
motion model instead.

Everything between those two is a measurement of some quality, and its quality
belongs in the covariance rather than in a threshold. A frame whose winner
barely beats a rival is not evidence-free; it is evidence about a smaller set of
directions, and that is what `ModeAmbiguityCovariance` expresses.

The measurement covariance comes from the curvature of the cost basin. Near the
minimum `c(δ) ≈ c_min + ½ δᵀHδ`, so the pose is uncertain by however far δ can
move before the cost rises by the noise in the cost itself: `cov ≈ 2σ_c H⁻¹`,
with the residual `c_min` standing in for σ_c. `step²/12` is added, since no
grid search resolves better than the cell it searched.

The earlier softmax-weighted spread remains the fallback for a border argmin or
a Hessian that is not positive definite. It returns `cost_softmax_scale · H⁻¹`
for a quadratic basin — the right shape, but scaled by a tuning constant rather
than by anything measured, which is why it is no longer the primary path.

Either way the covariance is clamped to what the search could resolve — no
direction may report a sigma wider than the grid, since nothing outside it was
scored — and enters the 6×6 whole, cross terms included, at the vehicle-frame
indices it measures: forward, left and yaw. Taking only its diagonal would
assume the uncertainty lines up with the grid axes, and the direction lane
geometry fails to constrain runs along the road, not along whichever axis the
grid was indexed by. The whole 6×6 is then rotated into the world frame the
error state uses.

That is what makes the update **partial** rather than all-or-nothing: a frame
that pins heading and lateral offset but cannot place the vehicle along the road
corrects the two it can, and the along-track direction, pinned at the extent
searched, moves the filter almost not at all. Under a 1% odometry scale error
the smoke sequence holds 0.264 m laterally while drifting 0.892 m along track —
the observability split the feature table above predicts, now visible in the
covariance rather than hidden behind a single skipped update.

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

Reading order, if you want the pipeline rather than a single answer:
`frames.h` first (every transform below is named in its convention), then
`localization_engine.cc` for the per-frame flow above, then `pose_sampler.cc`
for how a hypothesis becomes a cost, and `localization_kf.cc` for what happens
to the winner. `params.h` is the list of every knob and its default.


| Area | Path |
|------|------|
| Engine orchestration | `src/core/localization_engine.cc` |
| EKF | `src/core/localization_kf.cc` |
| Pose grid + aggregation | `src/core/pose_sampler.cc`, `cost_aggregator.cc` |
| Local quadratic | `src/core/cost_quadratic.cc` |
| CUDA | `src/cuda/distance_transform_kernels.cu`, `distance_transform_gpu.cc` |
| Params | `include/cam_loc/types/params.h` |
| Debug capture | `include/cam_loc/core/localization_debug.h` |
