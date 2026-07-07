# camera-map-localization

[![CI](https://github.com/yanghuafang/camera-map-localization/actions/workflows/ci.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Camera map-matching localization** on [KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) — a self-contained C++17 reference implementation of a production-style localization stack, without proprietary automotive SDKs.

Given lane and road-boundary perception (from Semantic KITTI or synthesized from a map), the system refines vehicle pose by matching perception to a polyline map using distance-transform costs over a 3-DOF pose grid, then fuses the result into an SE(3) error-state Kalman filter.

> **Validation scope:** This is a reference implementation currently validated end-to-end on synthetic data (trajectory-corridor map + oracle/synthesized perception). Accuracy on real KITTI sequences with turns is **not yet validated** — known coordinate-frame and fusion limitations are tracked in [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md).

## Why this project

| | |
|---|---|
| **Portable** | Plain CMake + Eigen; no proprietary automotive SDKs or closed middleware. Builds on Linux and macOS (Intel or Apple Silicon) laptops, with optional CUDA on NVIDIA GPUs. |
| **Complete pipeline** | Not just a pose solver — includes map loading (corridor / JSON / OSM), perception I/O, KF fusion, eval tools, benchmarks, and debug visualization. |
| **Reproducible on KITTI** | Smoke test needs no dataset download; real Odometry eval uses documented scripts and fixed regression thresholds. |
| **Inspectable** | Offline PNG layers (DT, cost grid, trajectories) and optional ROS 2 RViz playback expose every stage of map matching. |
| **Clean-room** | Original C++ implementation written for public release, free of proprietary SDK code. |

KITTI Odometry ships poses and calibration but no HD map. This repo builds a **trajectory corridor map** from ground truth for algorithm validation, and supports real Semantic KITTI perception and external OSM maps when you want to go further.

## How it works (per frame)

```
Egomotion predict (SE(3) KF)
        ↓
Perception → BEV/image distance transforms
        ↓
Pose grid search (x, y, yaw) + temporal cost aggregation
        ↓
Argmin → map observation → EKF update (skip if cost surface is flat)
        ↓
Optional global / plane measurements → LocalizationResult
```

**Map matching core:** rasterize lane and boundary polylines into bird's-eye and image canvases, compute Felzenszwalb distance transforms, score thousands of pose hypotheses by sampling map points against those transforms, then aggregate costs across recent frames using relative motion.

**Fusion:** relative odometry drives the predict step; the best grid pose (with covariance from the cost surface) is the primary map update. Flat or ambiguous cost maps are detected and skipped so the filter does not ingest bad geometry.

Details: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Quick start (no KITTI download)

```bash
# CUDA auto-enables on Linux (CPU stubs when nvcc is absent); on macOS it defaults off.
cmake -S . -B build -DCAMLOC_BUILD_TESTS=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"

./scripts/ci.sh                 # build + 34 tests + smoke benchmark (CPU)
./scripts/run_smoke.sh          # 50-frame localization on synthetic data
./scripts/run_viz_smoke.sh      # PNG debug panel for one frame
```

The `scripts/*.sh` helpers auto-detect the core count and run on both Linux and macOS. With CUDA and an NVIDIA GPU (Linux), `./scripts/run_benchmark.sh` adds micro-benchmarks (DT and pose grid, ~100× speedup on smoke data).

## Validate your build

| Layer | What it checks | How |
|-------|----------------|-----|
| **Unit tests** | Parsers, map loaders, DT, pose grid, KF, CUDA parity (skipped without GPU) | `ctest --test-dir build` — see [docs/TESTING.md](docs/TESTING.md) |
| **Regression benchmark** | Translation/yaw RMSE, match rate, frame latency vs thresholds | `./scripts/run_benchmark.sh` — see [docs/BENCHMARK.md](docs/BENCHMARK.md) |
| **Visualization** | DT heatmaps, cost slices, GT vs estimate trajectories | `./scripts/run_viz_smoke.sh` or `viz_frame` — see [docs/VISUALIZATION.md](docs/VISUALIZATION.md) |

CI (GitHub Actions) runs the same CPU path as `./scripts/ci.sh`.

## Applications

| Binary | Purpose |
|--------|---------|
| `run_sequence` | Run localization; print mean pose error |
| `eval_sequence` | RMSE, match rate, CSV export |
| `eval_perception_compare` | Oracle vs file vs noisy perception |
| `benchmark` | Regression suite + micro-benchmarks |
| `viz_frame` | Offline PNG debug panels |
| `preprocess_kitti` | Semantic KITTI labels → perception JSON |

## Documentation

**Start here** for setup and running on real KITTI:

- [docs/BUILD.md](docs/BUILD.md) — prerequisites and CMake options
- [docs/KITTI_DATA.md](docs/KITTI_DATA.md) — data layout and formats
- [docs/KITTI_RUN.md](docs/KITTI_RUN.md) — download, eval, perception pipelines

**Reference and contributing:**

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — algorithm and module map
- [docs/TESTING.md](docs/TESTING.md) · [docs/BENCHMARK.md](docs/BENCHMARK.md) · [docs/VISUALIZATION.md](docs/VISUALIZATION.md)
- [docs/OPEN_ITEMS.md](docs/OPEN_ITEMS.md) — roadmap
- [CONTRIBUTING.md](CONTRIBUTING.md)

Full index of all guides: [docs/README.md](docs/README.md) (table of contents for the `docs/` folder).

## Repository layout

```
camera-map-localization/
├── include/cam_loc/     # Public C++ API (namespace cam_loc)
├── src/                 # Core library + CUDA
├── apps/                # CLI tools
├── ros/cam_loc_ros/     # Optional ROS 2 RViz node
├── scripts/             # Smoke, benchmark, download helpers
├── tests/               # GoogleTest suite
├── data/                # Local datasets (mostly gitignored)
└── docs/                # Detailed guides
```

## License

MIT — see [LICENSE](LICENSE). Do not commit proprietary third-party source code.
