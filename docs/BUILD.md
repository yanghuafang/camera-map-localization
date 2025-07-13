# Build guide

## Project naming

| Layer | Name |
|-------|------|
| GitHub repository | `camera-map-localization` |
| CMake project | `camera_map_localization` |
| C++ namespace / headers | `cam_loc` under `include/cam_loc/` |
| Static libraries | `cam_loc_core`, `cam_loc_cuda` |
| CMake options | `CAMLOC_*` — see the CMake options table below |

The public repo name reflects **camera map localization**; internal `cam_loc` identifiers are kept for brevity and API stability.

## System requirements

| Component | Version |
|-----------|---------|
| Operating system | Linux or macOS (Intel or Apple Silicon) |
| CMake | ≥ 3.18 |
| C++ compiler | C++17 (GCC 9+, Clang 10+, or Apple Clang 12+) |
| CUDA toolkit (optional, Linux only) | ≥ 11.0, `nvcc` on `PATH` |
| Git | Required (to clone this repository; on macOS also for `stb`, which has no formula) |

### Platform notes

- **Linux** is the primary target and the only platform with GPU support; `CAMLOC_BUILD_CUDA` defaults **on** (falling back to CPU stubs when `nvcc` is missing).
- **macOS** builds with the stock Apple Clang toolchain (`xcode-select --install`). CUDA is unavailable on macOS, so `CAMLOC_BUILD_CUDA` defaults **off** and the CPU code path is used throughout — everything except the optional GPU kernels behaves identically to Linux.
- The `scripts/*.sh` helpers detect the CPU count portably (`nproc` on Linux, `getconf`/`sysctl` on macOS), so they run unchanged on both platforms.

Optional:

- **ROS 2** (Humble or Jazzy) — only for `ros/cam_loc_ros` RViz playback; see [VISUALIZATION.md](VISUALIZATION.md)
- **curl / unzip** — for `scripts/download_*.sh` (preinstalled on macOS)

### Installing them

From a clean machine:

```bash
./scripts/install_deps_macos.sh     # Homebrew
./scripts/install_deps_ubuntu.sh    # apt
```

Both take `--dry-run` to print the package list and exit. They install the
compiler toolchain, CMake, Ninja, the `clang-format` / `clang-tidy` gates, and
Doxygen + Graphviz for the API reference. They deliberately do **not** install
the C++ libraries below — CMake fetches those — nor CUDA or ROS 2, which are
large opt-ins with their own vendor instructions.

## Third-party dependencies

Four libraries, all resolved from the **system package manager**:

| Library | Homebrew | apt (Ubuntu 26.04) | Used for |
|---------|----------|--------------------|----------|
| Eigen | `eigen@3` 3.4.1 ✓ | `libeigen3-dev` 3.4.0 ✓ | Linear algebra |
| nlohmann/json | `nlohmann-json` 3.12 ✓ | `nlohmann-json3-dev` 3.11.3 ✓ | Perception + map JSON |
| stb | **no formula** — cached clone | `libstb-dev` ✓ | Image read/write |
| GoogleTest | `googletest` 1.18 ✓ | `libgtest-dev` 1.17 ✓ | Unit tests |

`./scripts/install_deps_{macos,ubuntu}.sh` install all of them. With them
present the configure needs **no network** — on Ubuntu it drops from minutes to
under a second — and a blocked or throttled route to github.com stops being a
build failure.

**On macOS it is `eigen@3`, not `eigen`.** The unversioned formula is 5.x, and
Eigen's own version file declares 5.x incompatible with a request for 3.4, so
CMake declines it rather than silently building against an untested major
version:

```
Could not find a configuration file for package "Eigen3" that is compatible
with requested version "3.4".
  ... version: 5.0.1
  The version found is not compatible with the version requested.
```

`eigen@3` is keg-only, so CMakeLists adds its prefix to `CMAKE_PREFIX_PATH` —
the same thing `scripts/lib.sh` does for the `llvm` keg.

### When a dependency is missing

There is no fetch-at-configure fallback: the configure fails and names the
script that fixes it.

```
CMake Error at CMakeLists.txt:...
  Missing dependencies:
    nlohmann/json  (brew: nlohmann-json, apt: nlohmann-json3-dev)

  Install them with:
    ./scripts/install_deps_macos.sh     (macOS)
    ./scripts/install_deps_ubuntu.sh    (Ubuntu)
```

Cloning them instead was the older behaviour, and it cost a couple of minutes
and a quarter-gigabyte for every build directory created — four configurations
meant four copies of the same sources — while turning a blocked route to
github.com into a wall of CMake internals. One route is simpler to read and
strictly faster.

The configure reports what it resolved, because the versions differ between
machines:

```
-- Eigen 3.4.1, nlohmann/json 3.12.0, stb /Users/.../camera-map-localization-deps/stb
-- GoogleTest 1.18.0
```

CUDA builds additionally compile `src/cuda/*.cu` into `libcam_loc_cuda.a`.

## Configure and build

`$(getconf _NPROCESSORS_ONLN)` reports the core count on both Linux and macOS (substitute `$(nproc)` on Linux if you prefer).

The scripts pick the build directory for you, which is the usual way in:

```bash
./scripts/ci.sh --no-style          # configure, build, test
./scripts/ci.sh --cuda --no-style   # with the GPU kernels (Linux)
```

By hand, naming the directory yourself:

```bash
B=../camera-map-localization-build
cmake -S . -B "$B" -DCAMLOC_BUILD_CUDA=OFF -DCAMLOC_BUILD_TESTS=ON
cmake --build "$B" -j"$(getconf _NPROCESSORS_ONLN)"
```

### Build directories

Builds land **beside** the repository, one directory per configuration:

```
camera-map-localization/                     the repository
camera-map-localization-build/               ./scripts/ci.sh
camera-map-localization-build-debug/         ./scripts/ci.sh --debug
camera-map-localization-build-asan-ubsan/    ./scripts/ci.sh --asan --ubsan
camera-map-localization-build-coverage/      ./scripts/coverage.sh
```

The default configuration gets the bare name and every departure from it adds a
tag, so two builds that differ only in type still land in different directories.

Out of tree so that a `git status` never has to look past build output, and so
that deleting a configuration is `rm -rf` on something that is not the working
tree. One per configuration so switching between Release and a sanitizer build
is not a full rebuild, and so an instrumented binary is never the one you
benchmark.

`CAMLOC_BUILD_DIR` overrides the scheme; CI uses it to keep its build inside the
workspace where the artifact upload can find it.

### Dataset directory

Datasets go beside the repository too, in `../camera-map-localization-data/`:

```
camera-map-localization-data/
  smoke_kitti/        ./scripts/prepare_smoke_kitti.sh   (generated)
  kitti_odometry/     ./scripts/download_kitti_odometry.sh
  perception/         preprocess_kitti output
  map/                optional world-frame map override
  *.json, *.csv       benchmark and eval output
```

The build directories are out of tree because build output is not source. The
datasets are out of tree for that reason and a blunter one: KITTI's velodyne
archive alone is about 80 GB, and a directory that size inside a working tree
makes every `git status`, every editor index and every `rsync` pay for it.
Nothing there is ours to version — it is downloaded or regenerated — so the
repository is better off with nowhere to put it.

`CAMLOC_DATA_DIR` overrides the scheme. CI sets it to a path inside the
workspace, because `actions/upload-artifact` cannot reach outside. `scripts/lib.sh`
(`camloc_data_dir`) and `CMakeLists.txt` compute the same default independently —
the scripts pass it to the apps as `--data-root`, and CMake passes it to the
tests, one of which skips when the smoke sequence is absent.

### Build type

`CMAKE_BUILD_TYPE` defaults to **`Release`**. This matters more than it usually
does: left unset, CMake passes no `-O` flag at all, and unoptimized Eigen inlines
nothing — the pose grid is a triple loop over Eigen expressions, and the smoke
sequence runs at ~2.6 s/frame that way instead of ~15 ms. A timing taken from a
build with no build type is not a measurement of the algorithm.

```bash
./scripts/ci.sh --debug   # or -DCMAKE_BUILD_TYPE=Debug by hand
```

Use `Debug` for a debugger or a sanitizer run, `RelWithDebInfo` when you want
both optimization and symbols, and `Release` for anything you intend to quote a
number from.

### CMake options

| Option | Default | Description |
|--------|---------|-------------|
| `CAMLOC_BUILD_CUDA` | `ON` (Linux) / `OFF` (macOS) | Build GPU kernels; falls back to CPU stubs if CUDA unavailable |
| `CAMLOC_CUDA_HOST_ONLY` | `OFF` | Compile the CUDA **host** paths against the CPU stub — type-checks everything under `#ifdef CAMLOC_CUDA_ENABLED` without nvcc or a GPU |
| `CAMLOC_BUILD_TESTS` | `ON` | Build `cam_loc_tests` and register CTest targets |
| `CAMLOC_SANITIZER` | *(empty)* | `-fsanitize=` list, e.g. `address`, `undefined`, `address,undefined` |
| `CAMLOC_COVERAGE` | `OFF` | Instrument for source-based coverage (Clang only; see `scripts/coverage.sh`) |
| `CAMLOC_WERROR` | `OFF` | Treat compiler warnings as errors |

The tree builds warning-free under `-Wall -Wextra`, which is always on for
cam_loc's own targets (the dependencies arrive as imported targets and keep
their own settings, and `stb` is included as a system directory so its warnings
are not reported as ours). That includes `distance_transform_kernels.cu`, which
`add_custom_command` hands to `nvcc` rather than to a CMake target, so
`src/cuda/CMakeLists.txt` forwards the two flags itself with
`-Xcompiler=-Wall,-Wextra`. It is the one file no CI job can warn about, since
GitHub's runners have no `nvcc`. `CAMLOC_WERROR` is off by default and not enabled in CI: `-Wall` means
something different to each of the three compilers in the matrix, so gating on
it would turn a compiler upgrade into a red build on unrelated pull requests.
Turn it on locally when you want the enforcement:

```bash
cmake -S . -B ../camera-map-localization-build -DCAMLOC_WERROR=ON
```

### Sanitizers

`CAMLOC_SANITIZER` sets matching compile and link flags on cam_loc's own
targets, and adds `-fno-sanitize-recover=undefined` so a UBSan finding is an exit
code rather than a line in an otherwise-passing log.

Pair it with **`RelWithDebInfo`, not `Debug`**. The instinct is that a sanitizer
wants `-O0` for readable frames, but this tree is Eigen expression templates:
unoptimized it runs some two hundred times slower, and that *multiplies* with the
sanitizer's own overhead rather than adding to it. One integration test went from
half a second to nine minutes that way. `-O2` with `-g` and
`-fno-omit-frame-pointer` — both set by `CAMLOC_SANITIZER` — still names frames
worth reading, and the whole suite finishes in about eight seconds.

The sanitizer build also gets `-UNDEBUG`, because an optimized build type would
otherwise switch off Eigen's own bounds and dimension assertions — exactly the
class of bug the sanitizers are there to catch.

```bash
B=../camera-map-localization-build-asan-ubsan
cmake -S . -B "$B" -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCAMLOC_SANITIZER=address,undefined
cmake --build "$B" -j"$(getconf _NPROCESSORS_ONLN)"
ctest --test-dir "$B" --output-on-failure
```

ThreadSanitizer is not offered: the library, apps and tests are single-threaded,
so it would instrument nothing, and it cannot be combined with AddressSanitizer.

`./scripts/ci.sh --asan --ubsan` does the same and picks its own build directory
so it does not force a rebuild of the plain one.

Eigen, nlohmann/json and stb are header-only, so they are instrumented along
with the code that includes them. GoogleTest comes prebuilt from the package
manager and is not — which is fine, since the runtime still checks every
instrumented translation unit and cam_loc's own code is what these are pointed
at.

### CUDA host paths

Everything inside `#ifdef CAMLOC_CUDA_ENABLED` is invisible to a CPU-only build,
so those call sites can drift out of sync with `cuda/distance_transform.h`
without any CPU build noticing. `CAMLOC_CUDA_HOST_ONLY=ON` links the CPU stub but
still defines the macro, so the compiler checks all of it on a machine with
neither nvcc nor a GPU; at run time `IsAvailable()` returns false and every path
falls back to the CPU, so tests and benchmarks behave exactly as CPU-only.

```bash
./scripts/ci.sh --cuda-host
```

### API documentation

```bash
./scripts/docs.sh          # → ../<repo>-build/docs/html/index.html
./scripts/docs.sh --open   # and open it
```

`.github/workflows/docs.yml` runs the same script on a push to `main` and
publishes the result to GitHub Pages. It is a separate workflow from `ci.yml`
and not a build gate: the warning check is strict and Doxygen's warning set
moves between releases, so a Doxygen upgrade would otherwise redden pull
requests that changed nothing here. A failure stops the site being republished
and nothing else.

Deliberately not a CMake target: Doxygen and `dot` are the only tools involved,
and requiring a configure to render
documentation would be a worse trade. Both come from the dependency installers
above.

The script fails when Doxygen writes anything to its warning log, because
Doxygen exits 0 after complaining about a broken reference; a green exit status
alone means nothing. It is not wired into CI: the reference is generated on
demand, nothing downstream consumes it, and a docs job would mostly fail on
Doxygen upgrades rather than on changes to this repository.

The configuration is [`docs/doxygen/Doxyfile`](doxygen/Doxyfile) — a short file
listing only the settings that differ from Doxygen's defaults, each with the
reason — and [`docs/doxygen/ApiMainPage.md`](doxygen/ApiMainPage.md), the
landing page. It covers `include/` only; the narrative guides stay in the
repository, where their links into `src/` resolve.

Headers are documented with `///` comments whose first sentence is the brief;
`@param` and `@return` are used where a parameter carries a unit, a frame, or a
constraint, and left off where the signature already says it.

### Libraries and apps produced

| Target | Type |
|--------|------|
| `cam_loc_core` | Static library — localization engine, map, perception, KITTI I/O |
| `cam_loc_cuda` | Static library — GPU kernels (or CPU stubs) |
| `cam_loc_app_common` | INTERFACE target — header-only helpers shared by the CLI front-ends |
| `run_sequence`, `eval_sequence`, `eval_perception_compare`, `benchmark`, `viz_frame`, `preprocess_kitti` | CLI executables under `<build dir>/apps/` |
| `cam_loc_tests` | GoogleTest binary under `<build dir>/tests/` |

Build a single app:

```bash
cmake --build build --target eval_sequence
```

## ROS 2 package (optional)

ROS is **not** part of the main CMake tree. After building `cam_loc_core`:

```bash
./scripts/build_ros.sh
source ../camera-map-localization-build-ros/ws/install/setup.bash
```

Requires ROS 2 Humble or Jazzy with `rclcpp`, `rviz2`, `visualization_msgs`, `nav_msgs`, `sensor_msgs`. `build_ros.sh` installs it when it is missing, asking first: from `packages.ros.org` into `/opt/ros` on Ubuntu, or from [RoboStack](https://robostack.github.io) into `../<repo>-ros` on macOS, since Homebrew carries no ROS formula and ROS 2's own macOS support is a Tier 3 source build. The core library, apps, and tests build and run without any of it.

## Troubleshooting

- **CUDA not found:** set `-DCAMLOC_BUILD_CUDA=OFF` or install `nvcc` and ensure it is on `PATH`.
- **`Missing dependencies:` at configure time:** run `./scripts/install_deps_macos.sh` or `./scripts/install_deps_ubuntu.sh`. The message lists each missing library with its Homebrew and apt package name.
- **Linker errors with CUDA static libs:** build apps through the provided CMake targets (they link `cam_loc_core` + `cam_loc_cuda` in the correct order).
- **CMake from another directory:** always pass the source tree explicitly, e.g. `cmake -S . -B <dir>` (all `scripts/*.sh` do this).

## Continuous integration

GitHub Actions splits the checks across workflows by cadence and blast radius, not by topic.
Each job reports its own status check; branch protection lists them individually, so a new
matrix leg has to be added there before it gates anything.

| Workflow | Check | What it runs |
|----------|-------|--------------|
| [`Lint`](../.github/workflows/lint.yml) | `clang-format` | `scripts/format.sh --check`, pinned to `clang-format-18` |
| | `clang-tidy` | configure only, then `scripts/tidy.sh`, pinned to `clang-tidy-18` |
| [`Build`](../.github/workflows/build.yml) | `Ubuntu` | `cpu` preset: build, `ctest`, `smoke_oracle_cpu` benchmark |
| | `macOS` | the same, under Apple Clang |
| [`Sanitizers`](../.github/workflows/sanitizers.yml) | `Ubuntu / ASan + UBSan` | `asan-ubsan` preset, then `ctest`. LeakSanitizer rides along here |
| | `macOS / ASan + UBSan` | the same without LSan, which macOS/arm64 does not support |
| [`CUDA`](../.github/workflows/cuda.yml) | `nvcc` | real `nvcc` compile of `src/cuda/`, then `ctest`. No hosted runner has a device, so the CUDA paths take the CPU fallback |
| [`Docs`](../.github/workflows/docs.yml) | `build`, `deploy` | Doxygen to GitHub Pages, every push to `main`. Not a pull-request gate |

`Lint` is its own file because its findings do not depend on the host and neither job needs a
compile, so a formatting slip reports in under a minute rather than behind a build. Both tools
are installed by version: the unversioned packages follow the runner image, and an unpinned
formatter eventually has CI and an editor disagree about a file nobody edited. The runner
images are pinned for the same reason.

The two sanitizer legs are not equivalent. LeakSanitizer rides along with ASan on Linux and is
unsupported on macOS/arm64, so `Ubuntu / ASan + UBSan` checks strictly more; `detect_leaks` is
left at each platform's default rather than forced, since setting it would abort every test on
the platform that cannot honour it. Both legs run weekly as well as per push, because a
sanitizer pass is more likely to break from a runner-image toolchain bump than from a commit.

`Docs` carries no `paths:` filter. This repository is maintained as a single amended commit, so
a force-push leaves no common ancestor between the old and new head; GitHub's compare then
fails and the changed-file set a filter matches against is empty, which means a filtered
workflow never runs at all. Doxygen on every push to `main` costs about a minute and cannot go
stale.

Each job installs only what it uses: `scripts/install_deps_ubuntu.sh --groups build` for the
builds, `--groups docs` for the Doxygen job, and a version-pinned `clang-format-18` /
`clang-tidy-18` for the two style jobs. The install scripts remain the only package list in
the project.

No workflow file contains a compiler or CMake flag. Every configuration is a preset in
[`CMakePresets.json`](../CMakePresets.json), which is what makes a red job reproducible:

```bash
cmake --preset asan-ubsan
cmake --build --preset asan-ubsan
ctest --preset asan-ubsan
```

Run `cmake --list-presets` for the full set.

Local equivalent of the whole sweep, in order:

```bash
./scripts/ci.sh
```
