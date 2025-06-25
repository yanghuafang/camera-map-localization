# Scripts

All scripts assume repository root as working directory (they resolve paths relative to `scripts/..`).

## Environment setup

| Script | Purpose |
|--------|---------|
| `install_deps_macos.sh` | Homebrew: `cmake ninja eigen@3 nlohmann-json googletest llvm`, plus an Xcode Command Line Tools check. `--groups build,style` installs a subset; CI takes only what each job uses |
| `install_deps_ubuntu.sh` | apt: `build-essential cmake ninja-build git libeigen3-dev nlohmann-json3-dev libgtest-dev libstb-dev clang-format clang-tidy clang llvm curl unzip`. `--groups build,style,coverage,data` installs a subset; CI takes only what each job uses |

Both take `--dry-run`. Between them they install everything the build links
against, so the configure itself needs no network. Neither installs CUDA or
ROS 2, which are large opt-ins with their own instructions.

## Build and test

| Script | Purpose |
|--------|---------|
| `ci.sh` | Local mirror of GitHub Actions (format + CPU build + ctest + smoke benchmark + tidy). Flags select the build under test: `--cuda`, `--cuda-host`, `--debug`/`--release`, `--asan`, `--ubsan`, `--no-style` |
| `run_smoke.sh` | Prepare smoke data + run `run_sequence` (CPU and CUDA if GPU present) |
| `run_benchmark.sh` | Smoke regression + micro-benchmarks; optional kitti00 if poses downloaded |
| `build_ros.sh` | Build optional `cam_loc_ros` package; installs ROS 2 first if it is missing (`--install-ros` to skip the prompt) |
| `coverage.sh` | Instrumented build + `ctest` + line/function report; `--html` for a browsable one |
| `remote_ubuntu.sh` | Run any of these on the Ubuntu host, optionally syncing the tree first |

Builds land **beside** the repository, one directory per configuration:
`../<repo>-build`, `../<repo>-build-asan-ubsan`, and so on: the default
configuration gets the bare name and every departure from it adds a tag. Out of
tree so `git status` never has to look past build output, one per
configuration so an instrumented binary is never the one you benchmark; see
[BUILD.md](../docs/BUILD.md#build-directories). `CAMLOC_BUILD_DIR` overrides
the scheme.

`remote_ubuntu.sh` exists for one reason: CUDA has no macOS toolchain, so the
GPU kernels can be compiled and tested only on Linux.

```bash
./scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda
```

The remote command runs in the directory matching the local one, so the same
thing from inside `scripts/` uses the paths you would type here:

```bash
./remote_ubuntu.sh --sync
./remote_ubuntu.sh ./run_smoke.sh
```

Anchoring at the repository root instead would make one command line mean two
different things depending on which side you typed it.

## Style gates

| Script | Purpose |
|--------|---------|
| `format.sh` | `clang-format` over `src/`, `include/`, `apps/`, `tests/`, `ros/`, plus a trailing-whitespace strip that also covers the scripts, docs and CMakeLists; `--check` reports without writing |
| `tidy.sh` | `clang-tidy` against the curated list in [`.clang-tidy`](../.clang-tidy); `--fix` applies what it can and re-formats |

Both are run by `ci.sh` and by CI:

```bash
brew install llvm                          # macOS — Xcode ships neither tool
sudo apt install clang-format clang-tidy   # Ubuntu
```

`tidy.sh` needs a compile database, so configure once first (`cmake -S . -B build
-DCAMLOC_BUILD_TESTS=ON`). It takes its file list from that database rather than
from a directory walk, which is why it analyzes the CUDA host wrapper only in a
CUDA build and never tries to analyze `ros/` — see the header comment in the
script for what goes wrong otherwise.

`lib.sh` is not run directly: it holds the shared helpers (`camloc_nproc`, the
clang-tool resolver, the source-file list) that the scripts above source.

## Evaluation pipelines

| Script | Purpose |
|--------|---------|
| `run_real_kitti.sh` | `eval_sequence` on seq 00 if odometry data present |
| `run_perception_eval.sh` | Preprocess (if velodyne) + eval with file/auto perception |
| `run_perception_tuning.sh` | `eval_perception_compare` oracle vs noisy |

## Data preparation

| Script | Purpose |
|--------|---------|
| `prepare_smoke_kitti.sh [frames]` | Generate `<repo>-data/smoke_kitti/` (default 120 poses along +Z at 0.5 m/frame, plus calib) |
| `download_kitti_odometry.sh [dest]` | Fetch poses + calib zips into `<repo>-data/kitti_odometry/`; `--force` re-fetches |
| `download_semantic_kitti_labels.sh [kitti_root]` | Fetch Semantic KITTI label archives |

## Visualization

| Script | Purpose |
|--------|---------|
| `run_viz_smoke.sh` | Offline PNG panel for smoke frame 20 |
| `run_ros_viz.sh` | Launch ROS 2 RViz node on smoke data |

See [docs/VISUALIZATION.md](../docs/VISUALIZATION.md) for manual `viz_frame` / `ros2 launch` usage.
