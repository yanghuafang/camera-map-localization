# Documentation

Detailed guides for **camera-map-localization**. For project overview, motivation, and quick start, see the [root README](../README.md).

## Getting started

1. [BUILD.md](BUILD.md) — system requirements, third-party dependencies, CMake options, build targets
2. [KITTI_DATA.md](KITTI_DATA.md) — where to put datasets, file formats, perception JSON contract
3. [KITTI_RUN.md](KITTI_RUN.md) — smoke test, real KITTI download, eval and perception pipelines
4. [TESTING.md](TESTING.md) — unit tests and regression checks
5. [BENCHMARK.md](BENCHMARK.md) — accuracy/performance suite and interpreting results
6. [VISUALIZATION.md](VISUALIZATION.md) — offline PNG debug and ROS 2 RViz playback

The generated API reference is a build target rather than a checked-in page:

```bash
./scripts/docs.sh
```

It renders `include/cam_loc/` into `build/docs/html/`. These guides stay in the
repository rather than being rendered, so their links into `src/` keep working.
Configuration: [doxygen/Doxyfile](doxygen/Doxyfile) and
[doxygen/ApiMainPage.md](doxygen/ApiMainPage.md).

## Design reference

- [ARCHITECTURE.md](ARCHITECTURE.md) — algorithm pipeline, EKF predict/update, module map
- [OPEN_ITEMS.md](OPEN_ITEMS.md) — what is unfinished, what is unverified, and what was left out on purpose

## Scripts

[scripts/README.md](../scripts/README.md) has the full table of helper shell
scripts. The ones you are most likely to want:

| Script | Purpose |
|--------|---------|
| `ci.sh` | Local CI mirror — format, CPU build, ctest, smoke benchmark, tidy |
| `format.sh` | `clang-format` plus a trailing-whitespace strip; `--check` for CI |
| `tidy.sh` | `clang-tidy` against the curated [`.clang-tidy`](../.clang-tidy) list |
| `prepare_smoke_kitti.sh` | Generate the synthetic sequence the smoke test runs on |

`ci.sh` also takes `--debug`/`--release`, `--asan`, `--ubsan` and `--cuda-host`
to select the build the gates run against; see [BUILD.md](BUILD.md#cmake-options).
