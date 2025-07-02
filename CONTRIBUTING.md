# Contributing

Thank you for contributing to **camera-map-localization**. This project is a clean-room C++ implementation of camera map-matching localization for KITTI; it must remain free of proprietary automotive SDK code.

## Before you start

1. Read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for the per-frame pipeline (predict → map match → EKF updates).
2. Skim [docs/BUILD.md](docs/BUILD.md) and [docs/TESTING.md](docs/TESTING.md).
3. For data or eval changes, check [docs/KITTI_DATA.md](docs/KITTI_DATA.md) and [scripts/README.md](scripts/README.md).

## Development setup

```bash
git clone <your-fork-url>
cd camera-map-localization

./scripts/install_deps_macos.sh    # or install_deps_ubuntu.sh
./scripts/ci.sh
```

`ci.sh` is the whole gate set: format, build, tests, smoke benchmark, clang-tidy.
Builds land beside the repository, one directory per configuration — see
[docs/BUILD.md](docs/BUILD.md#build-directories).

The two style gates need `clang-format` and `clang-tidy`:

```bash
brew install llvm              # macOS — Xcode ships neither tool
sudo apt install clang-format clang-tidy   # Ubuntu
```

Without them, `./scripts/ci.sh --no-style` runs the build and tests alone.

Sanitizers, and the GPU host paths without a GPU:

```bash
./scripts/ci.sh --asan --ubsan
./scripts/ci.sh --cuda-host
```

With CUDA locally:

```bash
./scripts/ci.sh --cuda
./scripts/run_benchmark.sh
```

## Pull requests

1. **Branch** from `main` (or `master` if that is the default).
2. **Scope** — Keep changes focused. Separate unrelated fixes into different PRs when possible.
3. **Tests** — Add or update GoogleTest coverage for new behavior. Regression thresholds are defined per-case in `src/benchmark/benchmark_runner.cc`; update only when intentionally changing algorithm behavior.
4. **Docs** — Update the relevant guide under `docs/` and `README.md` if user-facing behavior, CLI flags, or data layout changes.
5. **Scripts** — If you add a helper script, document it in `scripts/README.md`.
6. **CI** — PRs must pass four workflows:
   [`Lint`](.github/workflows/lint.yml) (`clang-format`, `clang-tidy`),
   [`Build`](.github/workflows/build.yml) (`Ubuntu`, `macOS` — the CPU build, `ctest` and the
   `smoke_oracle_cpu` benchmark), [`Sanitizers`](.github/workflows/sanitizers.yml)
   (`Ubuntu / ASan + UBSan`, `macOS / ASan + UBSan`) and
   [`CUDA`](.github/workflows/cuda.yml) (`nvcc`). Every configuration they build
   is a preset in `CMakePresets.json`, so any red job reproduces locally with the same
   commands — e.g. `cmake --preset asan-ubsan`, `cmake --build --preset asan-ubsan`,
   `ctest --preset asan-ubsan`.

## Commit messages

Write them as **Conventional Commits** — `type(scope): description`, lowercase, imperative, no trailing period. The types in use are `feat`, `fix`, `refactor`, `perf`, `docs`, `test`, `build` and `ci`; the scope is the subsystem the change lands in (`core`, `map`, `perception`, `cuda`, `viz`, `kitti`, `scripts`, `ros`). For example:

```
perf(cuda): batch the pose-grid upload into one transfer
```

Use the body to say **why**, not what — the diff already says what. Wrap it at 72 columns and prefer a few bullets, one per decision, each carrying the reason it went that way and what it cost:

- What was wrong before, in terms a reader can check against the code.
- Why this shape and not the obvious alternative.
- What moved as a consequence — a threshold, a benchmark number, a doc claim that stopped being true.

Keep unrelated changes out of the same commit, and do not commit generated benchmark JSON or downloaded data alongside a code change.

## Code guidelines

This project follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Most of it is enforced rather than described — `./scripts/format.sh` and `./scripts/tidy.sh` are the authority, and CI runs both. What follows is the part that needs saying anyway.

- **Language:** C++17, no extensions.
- **Formatting:** `clang-format` using the repo's [`.clang-format`](.clang-format) — Google base style, 2-space indent, 80-column limit. Run `./scripts/format.sh` before committing; `--check` is what CI runs.
- **Files:** sources are `.cc`, headers `.h`, both `lower_case`. A test for `foo.cc` is `foo_test.cc`.
- **Header guards:** `#define` guards, not `#pragma once`, named for the include path — `include/cam_loc/core/cost_grid.h` guards with `CAM_LOC_CORE_COST_GRID_H_`.
- **Naming:** types and functions `UpperCamelCase`; variables, parameters and members `lower_case`, with a trailing underscore on private members; constants and enumerators `kUpperCamelCase`; namespaces `lower_case`. Two exemptions are configured in [`.clang-tidy`](.clang-tidy) and explained there: a trivial accessor may be named for the member it returns (`step_x()`, `set_debug_capture()`), and a matrix or transform may use geometry notation (`T_world_rig`, `K`, `R0_rect`) rather than being renamed into something that reads as a translation. `readability-identifier-naming` enforces the rest.
- **Includes:** project headers are quoted and spelled as a path from an include root — `#include "cam_loc/core/cost_grid.h"`. Order is the guide's: related header, C system, C++ standard library, other libraries, this project. `.clang-format` regroups automatically, so writing them in any order and running `format.sh` is enough.
- **Exceptions:** not used. Errors travel as `cam_loc::Status` in the library and as a `bool` out-parameter in CLI argument parsing; `apps/common/arg_parse.h` has non-throwing numeric parsing for that. Each `main()` keeps one top-level `catch` as a backstop against a library that throws anyway — that is the single deliberate exception to the rule, and the reason is that the alternative is `std::terminate` with no diagnostic.
- **Static analysis:** `./scripts/tidy.sh` runs `clang-tidy` against the curated list in [`.clang-tidy`](.clang-tidy). That list is deliberately not the full upstream set: this is grid-index arithmetic and Eigen expressions, and several upstream families read that shape as a defect. Each disabled family carries the reason it was disabled — if a check would help, re-argue it there rather than silencing findings case by case.
- **Warnings:** the build is `-Wall -Wextra` and clean. `-DCAMLOC_WERROR=ON` makes them errors; it is off by default and not gated in CI, since `-Wall` differs between compilers and releases. Run it locally on your change.
- **Headers:** Public API under `include/cam_loc/`; implementation in `src/`.
- **Project naming:** repository is **camera-map-localization**; CMake project `camera_map_localization`. Keep the `cam_loc` namespace and the library target names unless doing a deliberate API break.
- **CUDA:** GPU code in `src/cuda/`; must have CPU path or stub for CI (`CAMLOC_BUILD_CUDA=OFF`). A helper used only on the GPU path belongs inside `#ifdef CAMLOC_CUDA_ENABLED` — left outside it, a CPU-only build reports it as an unused function. The flip side is that no CPU-only build compiles what is *inside* those blocks, so a rename can leave them behind: run `./scripts/ci.sh --cuda-host` after any rename that touches `src/core/`, `src/benchmark/` or `include/cam_loc/cuda/`. CI catches the same drift in `CUDA / nvcc`, which compiles those blocks too — this is just the faster local check, needing no toolkit.
- **Comments:** Explain **intent and trade-offs** — non-obvious algorithm steps (EKF, cost aggregation, georef), and why a thing is done the way it is. Do not narrate obvious code line by line.
- **API docs:** Public headers use `///` comments whose first sentence is the brief (Doxygen's `JAVADOC_AUTOBRIEF`); no `@brief` tag is needed. Add `@param` / `@return` where a parameter carries a **unit, a frame, or a constraint**, and leave them off where the signature already says it — `@param uv Pixel coordinates` is the narration the previous point rules out. Use `@warning` when the implementation does not do what the name promises, and link the tracking item. Build with `./scripts/docs.sh`, which fails on any Doxygen warning.
- **Dependencies:** Prefer something Homebrew and apt both ship, and wire it into `CMakeLists.txt` and *both* `scripts/install_deps_*.sh`; a dependency only one platform can install is a dependency half the readers cannot build. Do not add heavy ones without discussion. Add a third-party include directory as `SYSTEM` so its warnings are not reported as ours.

## Algorithm changes

When touching localization core (`localization_engine`, `localization_kf`, `pose_sampler`, `cost_aggregator`, CUDA kernels):

1. Run full unit tests and `./scripts/run_benchmark.sh` (or at least `--filter smoke`).
2. Note behavior changes in the PR description (RMSE, match rate, latency).
3. Update [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) if the pipeline order or measurements change.

## What not to commit

- Downloaded KITTI archives, generated perception, benchmark JSON outputs (see `.gitignore`).
- IDE/agent configs (`.cursor/`, `AGENTS.md`).
- Proprietary automotive SDK source, headers, or copied test data.
- Large binary datasets; document download steps instead.

## Reporting issues

Include:

- OS, compiler, CMake/CUDA versions
- Exact configure/build commands
- Minimal repro (prefer `<repo>-data/smoke_kitti` + `./scripts/run_smoke.sh`)
- Relevant log output or `eval_sequence` CSV snippet

## License

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
