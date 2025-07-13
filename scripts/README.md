# Scripts

All scripts assume repository root as working directory (they resolve paths relative to `scripts/..`).

## Environment setup

| Script | Purpose |
|--------|---------|
| `install_deps_macos.sh` | Homebrew: `cmake ninja eigen@3 nlohmann-json googletest llvm doxygen graphviz`, an Xcode Command Line Tools check, and a clone of `stb` (no formula exists). `--groups build,style,docs` installs a subset; CI takes only what each job uses |
| `install_deps_ubuntu.sh` | apt: `build-essential cmake ninja-build git libeigen3-dev nlohmann-json3-dev libgtest-dev libstb-dev clang-format clang-tidy clang llvm doxygen graphviz curl unzip`. `--groups build,style,coverage,docs,data` installs a subset; CI takes only what each job uses |

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
| `run_all.sh` | Run every other script here in dependency order, continuing past failures, and print a summary. macOS or Ubuntu |

Builds land **beside** the repository, one directory per configuration:
`../<repo>-build`, `../<repo>-build-asan-ubsan`, and so on: the default
configuration gets the bare name and every departure from it adds a tag. Out of
tree so `git status` never has to look past build output. One per configuration
so switching between Release and a sanitizer build is not a full rebuild, and so
an instrumented binary is never the one you benchmark. `CAMLOC_BUILD_DIR`
overrides the scheme.

`remote_ubuntu.sh` exists for one reason: CUDA has no macOS toolchain, so the
GPU kernels can be compiled and tested only on Linux.

```bash
./scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda
```

The remote command runs in the directory matching the local one, so the same
thing from inside `scripts/` uses the paths you would type here:

```bash
./remote_ubuntu.sh --sync
./remote_ubuntu.sh ./build_ros.sh
```

Anchoring at the repository root instead would make one command line mean two
different things depending on which side you typed it, and `./build_ros.sh`
would not be found at all.

`run_all.sh` is the other half of that: ssh has no display, so `run_ros_viz.sh`
and the `build_ros.sh` it depends on can only run from a desktop session on the
machine itself. Sync from the laptop, then run it in a terminal there — or just
run it here, since it works on macOS too. The plan adapts to the platform in
three places: which `install_deps` script it runs, whether `ci.sh` gets a real
`nvcc` or only `--cuda-host`, and where ROS 2 lives.

```bash
./scripts/run_all.sh --list   # the plan, and why anything would be skipped
./scripts/run_all.sh          # ~25 min; RViz is bounded at 60s
```

It keeps going after a failing step, so one broken script does not hide the
state of all the others; each step's output also lands in
`<repo>-data/run_all_logs/`, named in the closing summary. Every script in this
directory is either in its plan or in its skip list with a reason, and it
refuses to run if it finds one in neither — so adding a script forces a decision
about whether this covers it.

## Documentation

| Script | Purpose |
|--------|---------|
| `docs.sh` | Doxygen API reference for `include/` → `../<repo>-build/docs/html/`; `--open` opens it |

`docs.sh` reads [`docs/doxygen/Doxyfile`](../docs/doxygen/Doxyfile) and does not
go through CMake — doxygen and `dot` are the only tools it needs. It fails when
Doxygen writes to its warning log, since Doxygen exits 0 after complaining about
a broken reference. It is not run by `ci.sh`, so a Doxygen upgrade cannot
redden a pull request that changed nothing here; `.github/workflows/docs.yml`
runs it on a push to `main` and publishes the result to GitHub Pages.

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

`ci.sh` builds into `build/` normally, and into `build-<tags>/` for a debug or
sanitizer configuration, so those do not force a rebuild of the plain one every
time you switch — and so an instrumented binary cannot be mistaken for the one
you benchmark. `CAMLOC_BUILD_DIR` overrides the choice, and is passed through to
`tidy.sh`.

`lib.sh` is not run directly: it holds the shared helpers (`camloc_nproc`, the
clang-tool resolver, the source-file list) that the scripts above source.

## Data preparation

| Script | Purpose |
|--------|---------|
| `prepare_smoke_kitti.sh [frames]` | Generate `<repo>-data/smoke_kitti/` (default 120 poses along +Z at 0.5 m/frame, plus calib) |
| `download_kitti_odometry.sh <dest>` | Fetch poses + calib zips into `<repo>-data/kitti_odometry/` |
| `download_semantic_kitti_labels.sh <kitti_root>` | Fetch Semantic KITTI label archives |

## Evaluation pipelines

| Script | Purpose |
|--------|---------|
| `run_real_kitti.sh` | `eval_sequence` on seq 00 if odometry data present |
| `run_perception_eval.sh` | Preprocess (if velodyne) + eval with file/auto perception |
| `run_perception_tuning.sh` | `eval_perception_compare` oracle vs noisy |

## Visualization

| Script | Purpose |
|--------|---------|
| `run_viz_smoke.sh` | Offline PNG panel for smoke frame 20 |
| `run_ros_viz.sh` | Launch ROS 2 RViz node on smoke data |

See [docs/VISUALIZATION.md](../docs/VISUALIZATION.md) for manual `viz_frame` / `ros2 launch` usage.
