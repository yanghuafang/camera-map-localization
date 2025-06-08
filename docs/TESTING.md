# Testing

## Unit tests (GoogleTest)

Build and run everything:

```bash
./scripts/ci.sh
```

`CAMLOC_BUILD_CUDA` defaults on under Linux and off on macOS; add
`-DCAMLOC_BUILD_CUDA=ON` explicitly if you want the GPU kernels on a machine
where the default is off. `CMAKE_BUILD_TYPE` defaults to `Release` — see
[BUILD.md](BUILD.md#build-type) for why that is not a detail.

Builds land beside the repository, one directory per configuration
(`../<repo>-build`, `../<repo>-build-asan-ubsan`, …); see
[BUILD.md](BUILD.md#build-directories).

Or, by hand:

```bash
B=../camera-map-localization-build
ctest --test-dir "$B" --output-on-failure
"$B"/tests/cam_loc_tests --gtest_filter='MathTest.*'
```

### Coverage areas

| Area | Examples |
|------|----------|
| Frames | `FramesTest` — the cam0 ↔ vehicle convention everything geometric rests on |
| Math / KITTI I/O | `MathTest`, `CalibParserTest`, `PoseReaderTest` |
| Map | `CorridorMapTest` — including that lane boundaries are *lateral* and that upright landmarks are emitted; `OsmMapTest` — JSON, OSM XML and georef |
| Perception | `PerceptionJsonTest`, `SemanticKittiTest`, `SemanticLidarTest` |
| Eval / benchmark | `PoseErrorTest` — the error split onto vehicle axes, including that it follows GT heading; `SequenceEvalTest`, `BenchmarkTest` |
| CUDA parity | `CudaTest` (GPU vs CPU when CUDA available) |
| End to end | `LocalizationEngineTest` — straight *and* turning, with pose accuracy asserted |
| Filter | `LocalizationKfTest` — convergence away from identity attitude, and covariance well-formedness |
| Core matching | `DistanceTransformTest`, `PoseSamplerTest` — including along-track recovery and sub-cell refinement; `CostGridTest`, `CostAggregatorTest` |

Smoke benchmark test (`BenchmarkTest.SmokeOracleCpuPasses`) requires prepared smoke data; it skips if `<repo>-data/smoke_kitti` is missing.

## Smoke integration (no KITTI download)

```bash
./scripts/prepare_smoke_kitti.sh 120
./scripts/run_smoke.sh
```

## Regression benchmark (CI-friendly)

```bash
./scripts/ci.sh              # same checks as GitHub Actions (CPU)
./scripts/run_benchmark.sh   # full smoke suite (+ kitti00 if data present)
```

See [BENCHMARK.md](BENCHMARK.md) for case list and thresholds.

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

ThreadSanitizer is `./scripts/ci.sh --tsan`. The CPU pose-grid search fans out
over threads, and the claim that makes it safe -- hypotheses write disjoint
cells, so nothing needs synchronizing -- is worth proving rather than asserting.
It cannot be combined with `--asan`; the two replace the same allocator.

## CUDA host paths without a GPU

Code inside `#ifdef CAMLOC_CUDA_ENABLED` is not compiled by a CPU-only build, so
a rename can leave it behind while every gate above stays green. This compiles
it against the CPU stub, needing neither nvcc nor a GPU:

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

Not a test, but `ci.sh` runs it and a red gate blocks a change just the same, so
it belongs in the same pass:

```bash
./scripts/format.sh          # clang-format + trailing-whitespace strip
./scripts/format.sh --check  # what the gate runs: reports and exits 1, writes nothing
```

It needs `clang-format` (`brew install llvm` on macOS, since Xcode ships it not;
`sudo apt install clang-format` on Ubuntu).

## Recommended pre-push checklist

```bash
./scripts/ci.sh
./scripts/ci.sh --cuda-host --no-style   # if you touched anything under #ifdef CAMLOC_CUDA_ENABLED
./scripts/ci.sh --asan --ubsan --no-style

# On a machine with a GPU — nothing on macOS can check this:
./scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda
```
