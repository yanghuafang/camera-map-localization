# Documentation

Detailed guides for **camera-map-localization**. For project overview, motivation, and quick start, see the [root README](../README.md).

## Getting started

1. [BUILD.md](BUILD.md) — system requirements, third-party dependencies, CMake options, build targets
2. [KITTI_DATA.md](KITTI_DATA.md) — where to put datasets, file formats, perception JSON contract
3. [KITTI_RUN.md](KITTI_RUN.md) — smoke test, real KITTI download, eval and perception pipelines
4. [TESTING.md](TESTING.md) — unit tests and regression checks
5. [BENCHMARK.md](BENCHMARK.md) — accuracy/performance suite and interpreting results
6. [VISUALIZATION.md](VISUALIZATION.md) — offline PNG debug and ROS 2 RViz playback

## Design reference

- [ARCHITECTURE.md](ARCHITECTURE.md) — algorithm pipeline, EKF predict/update, module map
- [IMPLEMENTATION_PHASES.md](IMPLEMENTATION_PHASES.md) — feature completion checklist
- [OPEN_ITEMS.md](OPEN_ITEMS.md) — remaining work

## Scripts

See [scripts/README.md](../scripts/README.md) for a table of helper shell scripts.

| Script | Purpose |
|--------|---------|
| `ci.sh` | Local CI mirror (CPU build + ctest + smoke benchmark) |
