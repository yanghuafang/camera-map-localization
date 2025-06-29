# camera-map-localization

[![Lint](https://github.com/yanghuafang/camera-map-localization/actions/workflows/lint.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/lint.yml)
[![Build](https://github.com/yanghuafang/camera-map-localization/actions/workflows/build.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/build.yml)
[![Sanitizers](https://github.com/yanghuafang/camera-map-localization/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/sanitizers.yml)
[![CUDA](https://github.com/yanghuafang/camera-map-localization/actions/workflows/cuda.yml/badge.svg)](https://github.com/yanghuafang/camera-map-localization/actions/workflows/cuda.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**Where is the car, given what the camera sees and what the map says?**

A readable C++17 implementation of map-matching localization on
[KITTI Odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php): project
an HD map into the camera, score pose hypotheses against detected landmarks, and
fuse the best one into an error-state Kalman filter. No proprietary automotive
SDKs, no framework — plain CMake and Eigen, building on a laptop.

It is written to be **read**. Every non-obvious decision carries the reason it
was made, and where the implementation falls short of its own documentation, it
says so.

## Try it in three commands

```bash
./scripts/install_deps_macos.sh   # or install_deps_ubuntu.sh
./scripts/ci.sh --no-style        # build + unit tests
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

## Documentation

- [BUILD.md](docs/BUILD.md) — prerequisites, CMake options, build directories
- [KITTI_DATA.md](docs/KITTI_DATA.md) — data layout, formats, where each input comes from
- [KITTI_RUN.md](docs/KITTI_RUN.md) — download, evaluation, perception pipelines
- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — the algorithm, its frames, and its input/output contract
- [TESTING.md](docs/TESTING.md) — unit tests, sanitizers, style gates
- [BENCHMARK.md](docs/BENCHMARK.md) — regression suite, thresholds, measured numbers
- [VISUALIZATION.md](docs/VISUALIZATION.md) — offline PNG debug views and ROS 2 RViz playback
- [CONTRIBUTING.md](CONTRIBUTING.md) — style, commit convention, review gates

## License

MIT — see [LICENSE](LICENSE).
