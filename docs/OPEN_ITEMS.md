# Open Items (tracked)

Last updated: 2026-07-07

## CUDA (Phase 3 — complete)

- [x] GPU distance transform (`computeDistanceTransformGpu`, Felzenszwalb EDT)
- [x] GPU temporal aggregation (`aggregateCostsGpu` in `CostAggregator`)

## Perception (Phase 4 — partial)

- [ ] Multi-camera stereo in `preprocess_kitti`
- [ ] Per-camera perception JSON layout
- [x] Perception noise + oracle/noisy compare (`eval_perception_compare`, `run_perception_tuning.sh`)
- [x] Map-matching quality metrics (match rate, cost spread, flat rate)

## Map (Phase 5 — partial)

- [x] Native `.osm` XML parsing (`parseOsmXml`, `OsmMapLoader::loadFromOsmFile`)
- [x] Lat/lon → KITTI world georef (`MapGeoref`, `--map-georef`, `--map-origin-lat/lon`)
- [x] Corridor / JSON / OSM unified via `createMapLoader`
- [x] Spatial grid index for large maps (`PolylineMap::rebuildSpatialIndex`)
- [ ] GPS tie-point calibration tooling for real sequences

## Real KITTI evaluation

- [ ] Download full KITTI Odometry images (optional; poses+calib via `scripts/download_kitti_odometry.sh`)
- [x] Semantic KITTI labels download script (`download_semantic_kitti_labels.sh`)
- [ ] Velodyne scans for lidar preprocess (80 GB KITTI odometry velodyne zip)
- [x] `eval_sequence` app with ATE RMSE + CSV export

## Advanced refinements (optional)

- [ ] Cost-surface variance refinement for tighter sampling covariance
- [ ] Multi-camera image DT fusion
- [ ] Health / flat cost-map gating tuning on real data (use `--cost-flat-threshold` sweep)

## Known limitations / correctness (roadmap)

These are correctness issues in otherwise-implemented code, not missing features. Most are
masked by the current test setup — straight-line, rotation-free, oracle perception, GT-derived
map: a self-consistent loop that cannot fail — so they require validation on turning and real
KITTI data, not just the smoke tests.

### Coordinate frame (highest priority; several symptoms)

- [ ] The pinhole path uses the KITTI cam0 frame (Z forward, Y down) while the BEV raster,
      corridor map, and `Projection::offsetToTransform` assume a Z-up / X-forward vehicle frame.
      The pose-grid yaw axis rotates about the optical axis rather than vehicle heading, and
      `yawFromRotation` (eval yaw-error metric) measures the wrong component on real sequences.
- [ ] `Projection::imageToGroundRig` intersects a Z=const plane (not the road plane) and is
      always called with `ground_z_rig = 0`, so BEV perception collapses to the camera origin
      and the BEV cost branch is degenerate (masked by the image branch + oracle testing).
- [ ] Fix plan: choose one consistent frame (cam0); rework `rigToBevPixel` / `imageToGroundRig` /
      `computeBevCosts` (CPU + CUDA); add a curved-trajectory synthetic sequence with known
      ground truth as the validation harness.

### EKF

- [ ] `LocalizationKF::update` forms the rotation residual in the world frame (`R * omega_body`)
      but applies the correction on the right (body frame), so it only converges to the
      measurement when attitude ≈ identity. Needs a rotating-pose test.
- [ ] Covariance uses `P ← P + Q` (no state Jacobian) and `(I − K)P` (not Joseph form) —
      acceptable for smooth motion but not a calibrated filter.

### Temporal aggregation

- [ ] `CostAggregator::frameWeight` decays on absolute cumulative distance from sequence start,
      so all history weights reach zero after ~`1/distance_decay` (~100 m) and fusion silently
      turns off. Should decay relative to the current frame; add a multi-hundred-meter test.

### Pose-grid cost

- [ ] `isCostMapFlat` measures spread over the whole grid including the `max_cost * 10` sentinel
      used for hypotheses that project zero map points, so the ambiguity gate is largely inert
      whenever any hypothesis sees no map. Compute flatness over populated cells only.
- [ ] Cost is the mean DT over projected points, so a hypothesis projecting few but well-aligned
      points can beat one aligning many. Add count-aware scoring / minimum-support penalty.

### Validation gaps

- [ ] The engine integration test asserts only that the pipeline runs (no pose-accuracy check).
- [ ] No turning-trajectory test, no long-run (aggregation) test, and no recorded real KITTI-00
      accuracy number.
