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

cmake -B build -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"
./scripts/prepare_smoke_kitti.sh 120
ctest --test-dir build --output-on-failure
```

Quick CI-equivalent check (CPU-only, matches GitHub Actions):

```bash
./scripts/ci.sh
```

With CUDA locally:

```bash
./scripts/ci.sh --cuda
./scripts/run_benchmark.sh
```

## Pull requests

1. **Branch** from `main` (or `master` if that is the default).
2. **Scope** — Keep changes focused. Separate unrelated fixes into different PRs when possible.
3. **Tests** — Add or update GoogleTest coverage for new behavior. Regression thresholds are defined per-case in `src/benchmark/benchmark_runner.cpp`; update only when intentionally changing algorithm behavior.
4. **Docs** — Update the relevant guide under `docs/` and `README.md` if user-facing behavior, CLI flags, or data layout changes.
5. **Scripts** — If you add a helper script, document it in `scripts/README.md`.
6. **CI** — PRs must pass the [CI workflow](.github/workflows/ci.yml) (build, ctest, `smoke_oracle_cpu` benchmark).

## Code guidelines

- **Language:** C++17, no extensions.
- **Style:** Match surrounding files (naming, includes, error handling via `cam_loc::Status`).
- **Headers:** Public API under `include/cam_loc/`; implementation in `src/`.
- **Naming:** Repository is **camera-map-localization**; CMake project `camera_map_localization`. Keep `cam_loc` namespace and library target names unless doing a deliberate API break.
- **CUDA:** GPU code in `src/cuda/`; must have CPU path or stub for CI (`CAMLOC_BUILD_CUDA=OFF`).
- **Comments:** Explain non-obvious algorithm steps (EKF, cost aggregation, georef). Avoid narrating obvious code.
- **Dependencies:** Prefer CMake `FetchContent` for small libs; do not add heavy new dependencies without discussion.

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
- Minimal repro (prefer `data/smoke_kitti` + `./scripts/run_smoke.sh`)
- Relevant log output or `eval_sequence` CSV snippet

## License

By contributing, you agree that your contributions are licensed under the [MIT License](LICENSE).
