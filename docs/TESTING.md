# Testing

## Unit tests (GoogleTest)

Build and run everything:

```bash
./scripts/ci.sh --no-style
```

Builds land beside the repository, one directory per configuration
(`../<repo>-build`, `../<repo>-build-asan-ubsan`, …); see
[BUILD.md](BUILD.md#build-directories).

`CAMLOC_BUILD_CUDA` defaults on under Linux and off on macOS; add
`-DCAMLOC_BUILD_CUDA=ON` explicitly if you want the GPU kernels on a machine
where the default is off. `CMAKE_BUILD_TYPE` defaults to `Release` — see
[BUILD.md](BUILD.md#build-type) for why that is not a detail.

Or, by hand:

```bash
B=../camera-map-localization-build
ctest --test-dir "$B" --output-on-failure
"$B"/tests/cam_loc_tests --gtest_filter='LocalizationEngineTest.*'
```

### Coverage areas

| Area | Examples |
|------|----------|
| Frames | `FramesTest` — the cam0 ↔ vehicle convention everything geometric rests on |
| Math / KITTI I/O | `MathTest`, `CalibParserTest`, `PoseReaderTest` |
| Map | `CorridorMapTest` — including that lane boundaries are *lateral* and that upright landmarks are emitted |
| Perception | `PerceptionJsonTest`, `SemanticKittiTest`, `SemanticLidarTest`, `ResolveTest` |
| Core matching | `DistanceTransformTest`, `PoseSamplerTest` — including along-track recovery and sub-cell refinement |
| Filter | `LocalizationKfTest` — convergence away from identity attitude, and covariance well-formedness |
| End to end | `LocalizationEngineTest` — straight *and* turning, with pose accuracy asserted |
| CUDA parity | `CudaTest` (GPU vs CPU when CUDA available) |
| Eval / benchmark | `PoseErrorTest` — the error split onto vehicle axes, including that it follows GT heading; `SequenceEvalTest`, `BenchmarkTest` |
| Visualization | `VizTest` |

Several of these exist because the defect they cover was invisible to the older
suite: a straight, rotation-free, oracle-perception run agrees with itself
whatever the frames are doing.

Smoke benchmark test (`BenchmarkTest.SmokeOracleCpuPasses`) requires prepared smoke data; it skips if `<repo>-data/smoke_kitti` is missing.

## Smoke integration (no KITTI download)

```bash
./scripts/prepare_smoke_kitti.sh 120
./scripts/run_smoke.sh
```

## Regression benchmark (CI-friendly)

```bash
./scripts/ci.sh              # same checks as GitHub Actions (CPU)
./scripts/run_benchmark.sh   # full smoke + micro-benchmarks (+ kitti00 if data present)
```

See [BENCHMARK.md](BENCHMARK.md) for case list and thresholds.

## Coverage

```bash
./scripts/coverage.sh          # per-file line and function report
./scripts/coverage.sh --html   # browsable line-by-line
```

Clang only — the instrumentation and the report tools have to come from one
toolchain. It measures `src/` and `include/`; `apps/` and `tests/` are excluded,
since neither is the thing under test.

## Sanitizers

ASan and UBSan run over the same unit tests. UBSan is configured
non-recovering, so a finding fails the run instead of printing a line:

```bash
./scripts/ci.sh --asan --ubsan
```

That builds into its own directory, so switching back and forth does not force a
full rebuild — and so you cannot accidentally benchmark an instrumented binary.
The equivalent by hand is in [BUILD.md](BUILD.md#sanitizers).

Sanitizer builds use `RelWithDebInfo`, not `Debug` — see
[BUILD.md](BUILD.md#sanitizers) for why that distinction is worth about nine
minutes per test. The suite takes roughly eight seconds under ASan + UBSan
against under a second in `Release`.

ThreadSanitizer is not offered: nothing in the library, apps or tests starts a
thread, so it would have nothing to instrument.

## CUDA host paths without a GPU

Code inside `#ifdef CAMLOC_CUDA_ENABLED` is not compiled by a CPU-only build, so
a rename can leave it behind while every gate above stays green — which is how
those call sites last drifted out of sync with the CUDA header. This compiles
them against the CPU stub, needing neither nvcc nor a GPU:

```bash
./scripts/ci.sh --cuda-host
```

CI has no job of its own for this: the `CUDA / nvcc` job compiles a strict superset of what
`--cuda-host` compiles, so it already catches the same drift. This stays the fast local check,
because it needs neither the toolkit nor a GPU. Neither is a substitute for running the
kernels on real hardware — it checks that the host side still compiles and
links, not that the GPU results are right; `CudaTest` covers that, and skips
without a GPU.

## Style gates

Not tests, but CI runs them and a red gate blocks a PR just the same, so they
belong in the same pass:

```bash
./scripts/format.sh          # clang-format + trailing-whitespace strip
./scripts/format.sh --check  # what CI runs: reports and exits 1, writes nothing
./scripts/tidy.sh            # clang-tidy against the curated .clang-tidy list
./scripts/tidy.sh --fix      # apply what clang-tidy can fix, then re-format
```

Both need `clang-format` / `clang-tidy` (`brew install llvm` on macOS, since
Xcode ships neither; `sudo apt install clang-format clang-tidy` on Ubuntu).
`tidy.sh` additionally needs a compile database, so configure once first.

`tidy.sh` is also what enforces the Google naming rules: `.clang-tidy` runs
`readability-identifier-naming` configured to the guide, so a `lowerCamelCase`
function or a `CamelCase` parameter fails the gate rather than surviving review.
The two exemptions it carries — variable-like accessors, and matrix notation
such as `T_world_rig` — are written out with their reasons in that file.

`tidy.sh` analyzes the entries in the build directory's `compile_commands.json`
that fall under `src/`, `apps/` or `tests/` — both halves of that matter.

Taking the list from the database, rather than walking the source tree, is what
keeps the two build configurations honest: in a CPU-only build the database
holds `distance_transform_stub.cc` and not `distance_transform_gpu.cc`, and
handing clang-tidy a file it has no command for makes it invent one — reporting
a missing `cuda_runtime.h` and a cascade of parse errors that say nothing about
the code. `ros/cam_loc_ros` is outside the database for the same reason (colcon
builds it separately) and is not analyzed.

Bounding it to `src/`, `apps/` and `tests/` is the other half: it keeps the list
to code this repository is responsible for, whatever else a build directory
happens to compile.

## Recommended pre-push checklist

```bash
./scripts/ci.sh
./scripts/ci.sh --cuda-host --no-style   # if you touched anything under #ifdef CAMLOC_CUDA_ENABLED
./scripts/ci.sh --asan --ubsan --no-style

# On a machine with a GPU — nothing on macOS can check this:
./scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda
```
