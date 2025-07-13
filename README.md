# camera-map-localization

[![Lint](https://github.com/yanghuafang/camera-map-localization/actions/workflows/lint.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/lint.yml)
[![Build](https://github.com/yanghuafang/camera-map-localization/actions/workflows/build.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/build.yml)
[![Sanitizers](https://github.com/yanghuafang/camera-map-localization/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/sanitizers.yml)
[![CUDA](https://github.com/yanghuafang/camera-map-localization/actions/workflows/cuda.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/cuda.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Where is the car, given what the camera sees and what the map says?**

A readable C++17 implementation of map-matching localization on
[KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php): project
an HD map into the camera, score thousands of pose hypotheses against detected
landmarks, and fuse the best one into an error-state Kalman filter. No
proprietary automotive SDKs, no framework — plain CMake and Eigen, building on a
laptop.

It is written to be **read**. Every non-obvious decision carries the reason it
was made, and where the implementation falls short of its own documentation, it
says so.

## Try it in three commands

```bash
./scripts/install_deps_macos.sh   # or install_deps_ubuntu.sh
./scripts/ci.sh --no-style        # build + 58 tests + smoke benchmark
./scripts/run_smoke.sh            # localize on a synthetic sequence
```

No dataset download. The smoke sequence is generated locally.

## What it does, per frame

```
Egomotion → EKF predict
        ↓
Perception polylines → distance transform
        ↓
Pose grid: score (forward, left, yaw) hypotheses against the map
        ↓
Aggregate over recent frames → argmin → sub-cell refine
        ↓
Gate on ambiguity and fit → EKF update
```

On the synthetic oracle sequence: **3 mm translation RMSE, 0.005° heading RMSE,
10 ms/frame** on one CPU core. Full numbers and caveats in
[BENCHMARK.md](docs/BENCHMARK.md) — the map there is derived from ground truth,
so that figure bounds the *backend*, not localization against a surveyed map.

## The idea worth taking away

A localizer searching three degrees of freedom needs landmarks that constrain
all three, and map features are not interchangeable:

| Feature | Lateral | Along-track | Heading |
|---------|---------|-------------|---------|
| Lane line, road boundary | strong | **~none** | strong |
| Pole, traffic sign | strong | **strong** | moderate |

Lane geometry runs *parallel* to travel, so sliding a hypothesis down the road
costs nothing — a system fed only lane detections has an unobservable degree of
freedom no amount of tuning will fix. Upright landmarks are what pin it.

Two consequences shape this codebase. Poles and traffic signs are extracted from
[SemanticKITTI](http://www.semantic-kitti.org/) with a *vertical* scan, because a
horizontal one meets a pole two pixels at a time and discards it. And the cost is
averaged **per class**, not per point — a lane contributes a hundred points and a
pole contributes three, so a per-point average lets the uninformative majority
outvote the landmark that actually knows where you are.

[ARCHITECTURE.md](docs/ARCHITECTURE.md) has the full table and the reasoning.

## What is real, and what is not

Honesty about scope is part of the point:

- **Perception is an input.** No detector is trained or run here. Landmarks come
  from SemanticKITTI labels, or from projecting the map at the ground-truth pose
  (the "oracle" — an upper bound, never an accuracy claim).
- **KITTI Odometry ships no HD map**, so the default map is synthesized from the
  ground-truth path. That makes the oracle path a closed loop: it measures
  whether the search, the frames and the filter agree, not localization accuracy.
- **Traffic lights and crosswalks are absent** because SemanticKITTI has no class
  for either — not because they were skipped.
- **The CUDA path is verified but not continuously.** All four GPU/CPU parity
  tests pass on an RTX A6000, but GitHub's runners have no GPU, so that result is
  only as fresh as the last manual run on a real machine.

[OPEN_ITEMS.md](docs/OPEN_ITEMS.md) is the full list.

## Why it might be worth your time

| | |
|---|---|
| **Complete, not a fragment** | Map loading, perception I/O, pose search, temporal fusion, EKF, evaluation, benchmarks, and PNG/RViz debug views. |
| **Portable** | Plain CMake + Eigen. macOS and Linux, Intel or Apple Silicon. One script installs the dependencies; the configure needs no network. |
| **Gated** | 58 tests, ASan/UBSan clean, `clang-format` + `clang-tidy` in CI, threshold-checked regression benchmark. |
| **Documented at the frame level** | Every public type states which frame and which units it is in — the thing this problem is easiest to get wrong. |

## Tools

| Binary | Purpose |
|--------|---------|
| `run_sequence` | Localize a sequence; print mean pose error |
| `eval_sequence` | RMSE (total and per vehicle axis), match rate, per-frame CSV |
| `eval_perception_compare` | Oracle vs file vs noisy perception |
| `benchmark` | Threshold-gated regression suite + micro-benchmarks |
| `viz_frame` | Offline PNG debug panels |
| `preprocess_kitti` | SemanticKITTI labels → perception JSON |

| Script | Purpose |
|--------|---------|
| `ci.sh` | Every gate: format, build, tests, benchmark, tidy. `--debug`, `--asan`, `--ubsan`, `--cuda`, `--cuda-host` |
| `coverage.sh` | Line and function coverage |
| `docs.sh` | Doxygen API reference |
| `remote_ubuntu.sh` | Run any of the above on a Linux host — the only way to test CUDA from a Mac |
| `run_all.sh` | Run every script in order, macOS or Ubuntu, including the RViz playback ssh cannot show |

## Documentation

- [BUILD.md](docs/BUILD.md) — prerequisites, CMake options, build directories
- [KITTI_DATA.md](docs/KITTI_DATA.md) — data layout, formats, where each input comes from
- [KITTI_RUN.md](docs/KITTI_RUN.md) — download, evaluation, perception pipelines
- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — the algorithm, its frames, and its input/output contract
- [TESTING.md](docs/TESTING.md) · [BENCHMARK.md](docs/BENCHMARK.md) · [VISUALIZATION.md](docs/VISUALIZATION.md)
- [OPEN_ITEMS.md](docs/OPEN_ITEMS.md) — what is unfinished, unverified, or deliberately out of scope
- [CONTRIBUTING.md](CONTRIBUTING.md) — style, commit convention, review gates

Full index: [docs/README.md](docs/README.md).

## Layout

```
include/cam_loc/   Public API (namespace cam_loc)
src/               Library + CUDA kernels
apps/              Command-line tools
tests/             GoogleTest suite
scripts/           Setup, build, test, docs, remote
docs/              Guides + Doxygen config
ros/cam_loc_ros/   Optional ROS 2 RViz playback
```

## License

MIT — see [LICENSE](LICENSE).
