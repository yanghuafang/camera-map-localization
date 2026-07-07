# Build guide

## Project naming

| Layer | Name |
|-------|------|
| GitHub repository | `camera-map-localization` |
| CMake project | `camera_map_localization` |
| C++ namespace / headers | `cam_loc` under `include/cam_loc/` |
| Static libraries | `cam_loc_core`, `cam_loc_cuda` |
| CMake options | `CAMLOC_BUILD_CUDA`, `CAMLOC_BUILD_TESTS` |

The public repo name reflects **camera map localization**; internal `cam_loc` identifiers are kept for brevity and API stability.

## System requirements

| Component | Version |
|-----------|---------|
| Operating system | Linux (Ubuntu 24.04 / 26.04) or macOS 12+ (Intel or Apple Silicon) |
| CMake | ≥ 3.18 |
| C++ compiler | C++17 (GCC 9+, Clang 10+, or Apple Clang 12+) |
| CUDA toolkit (optional, Linux only) | ≥ 11.0, `nvcc` on `PATH` |
| Git | Required (dependencies are fetched via CMake `FetchContent`) |

### Platform notes

- **Linux** is the primary target and the only platform with GPU support; `CAMLOC_BUILD_CUDA` defaults **on** (falling back to CPU stubs when `nvcc` is missing).
- **macOS** builds with the stock Apple Clang toolchain (`xcode-select --install`). CUDA is unavailable on macOS, so `CAMLOC_BUILD_CUDA` defaults **off** and the CPU code path is used throughout — everything except the optional GPU kernels behaves identically to Linux.
- The `scripts/*.sh` helpers detect the CPU count portably (`nproc` on Linux, `getconf`/`sysctl` on macOS), so they run unchanged on both platforms.

Optional:

- **ROS 2** (Humble or Jazzy) — only for `ros/cam_loc_ros` RViz playback; see [VISUALIZATION.md](VISUALIZATION.md)
- **curl / unzip** — for `scripts/download_*.sh`

## Third-party dependencies

Fetched automatically at configure time (no system packages required for these):

| Library | Use |
|---------|-----|
| [Eigen 3.4](https://eigen.tuxfamily.org/) | Linear algebra |
| [nlohmann/json](https://github.com/nlohmann/json) | Perception + map JSON |
| [stb](https://github.com/nothings/stb) | Image read/write (preprocess, viz) |
| [GoogleTest 1.14](https://github.com/google/googletest) | Unit tests (`CAMLOC_BUILD_TESTS=ON`) |

CUDA builds additionally compile `src/cuda/*.cu` into `libcam_loc_cuda.a`.

## Configure and build

`$(getconf _NPROCESSORS_ONLN)` reports the core count on both Linux and macOS (substitute `$(nproc)` on Linux if you prefer).

```bash
# CPU-only (macOS, or Linux without a GPU)
cmake -B build -DCAMLOC_BUILD_CUDA=OFF -DCAMLOC_BUILD_TESTS=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"

# With CUDA (Linux + NVIDIA GPU; default on Linux)
cmake -B build -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"
```

### CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `CAMLOC_BUILD_CUDA` | `ON` (Linux) / `OFF` (macOS) | Build GPU kernels; falls back to CPU stubs if CUDA unavailable |
| `CAMLOC_BUILD_TESTS` | `ON` | Build `cam_loc_tests` and register CTest targets |

### Libraries and apps produced

| Target | Type |
|--------|------|
| `cam_loc_core` | Static library — localization engine, map, perception, KITTI I/O |
| `cam_loc_cuda` | Static library — GPU kernels (or CPU stubs) |
| `run_sequence`, `eval_sequence`, `eval_perception_compare`, `benchmark`, `viz_frame`, `preprocess_kitti` | CLI executables under `build/apps/` |
| `cam_loc_tests` | GoogleTest binary under `build/tests/` |

Build a single app:

```bash
cmake --build build --target eval_sequence
```

## ROS 2 package (optional)

ROS is **not** part of the main CMake tree. After building `cam_loc_core`:

```bash
./scripts/build_ros.sh
source ros/ws/install/setup.bash
```

Requires ROS 2 Humble/Jazzy with `rclcpp`, `rviz2`, `visualization_msgs`, `nav_msgs`, `sensor_msgs`. The ROS visualization is **Linux-only** (`build_ros.sh` expects a distro under `/opt/ros`); the core library, apps, and tests build and run on macOS without it.

## Troubleshooting

- **CUDA not found:** set `-DCAMLOC_BUILD_CUDA=OFF` or install `nvcc` and ensure it is on `PATH`.
- **FetchContent network errors:** configure with network access; dependencies clone from GitHub/GitLab on first run.
- **Linker errors with CUDA static libs:** build apps through the provided CMake targets (they link `cam_loc_core` + `cam_loc_cuda` in the correct order).
- **CMake from another directory:** always pass the source tree explicitly, e.g. `cmake -S . -B build` (all `scripts/*.sh` do this).

## Continuous integration

GitHub Actions runs [`.github/workflows/ci.yml`](../.github/workflows/ci.yml) on push/PR across a Linux + macOS matrix (`ubuntu-24.04` and `ubuntu-26.04` on x86_64, `macos-latest` on Apple Silicon / ARM64): CPU build, 34 unit tests, and `smoke_oracle_cpu` benchmark. Local equivalent:

```bash
./scripts/ci.sh
```
