# Implementation Phases

> ✅ marks a feature as **implemented**, not fully validated. Several phases have known
> correctness issues (coordinate frame, EKF rotation frame, aggregation decay, cost gating)
> tracked under "Known limitations / correctness" in [OPEN_ITEMS.md](OPEN_ITEMS.md). They are
> masked by the current straight-line, rotation-free, oracle smoke tests.

## Phase 0 — Scaffold ✅

## Phase 1 — CPU pose grid (BEV + image) ✅

## Phase 2 — Temporal aggregation ✅

## Phase 3 — CUDA ✅

- [x] GPU parallel pose-grid image cost kernel
- [x] GPU BEV pose-grid cost kernel
- [x] GPU argmin reduction (wired into `CostGrid` / engine)
- [x] GPU distance transform (Felzenszwalb column/row passes)
- [x] GPU temporal aggregation (`aggregateCostsGpu` in `CostAggregator`)

## Phase 4 — Perception pipeline (partial) ✅

- [x] `preprocess_kitti` — Semantic KITTI LiDAR labels → image polylines → JSON
- [x] `scripts/download_semantic_kitti_labels.sh` + `run_perception_eval.sh`
- [ ] Multi-camera stereo support

## Phase 5 — Real map (partial) ✅

- [x] `OsmMapLoader` — preprocessed OSM/HD polylines from JSON
- [x] `--map-path` in `run_sequence` (JSON or `.osm`)
- [x] KITTI odometry download script + `eval_sequence` benchmark
- [x] Native `.osm` XML parsing + `MapGeoref`
- [x] Unified `createMapLoader` (corridor / JSON / OSM)
- [ ] GPS tie-point calibration on real sequences
