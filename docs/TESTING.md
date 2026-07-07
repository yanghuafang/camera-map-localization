# Testing

## Unit tests (GoogleTest)

Build with tests enabled:

```bash
cmake -B build -DCAMLOC_BUILD_TESTS=ON -DCAMLOC_BUILD_CUDA=ON
cmake --build build -j"$(getconf _NPROCESSORS_ONLN)"
```

Run the full suite:

```bash
ctest --test-dir build --output-on-failure
```

Or run the binary directly:

```bash
./build/tests/cam_loc_tests
./build/tests/cam_loc_tests --gtest_filter='LocalizationEngineTest.*'
```

### Coverage areas

| Area | Examples |
|------|----------|
| Math / KITTI I/O | `MathTest`, `CalibParserTest`, `PoseReaderTest` |
| Map | `CorridorMapTest`, `OsmMapLoaderTest` |
| Perception | `PerceptionJsonTest`, `ResolveTest` |
| Core matching | `DistanceTransformTest`, `PoseSamplerTest`, `LocalizationEngineTest` |
| CUDA parity | `CudaTest` (GPU vs CPU when CUDA available) |
| Eval / benchmark | `SequenceEvalTest`, `BenchmarkTest` |
| Visualization | `VizTest` |

Smoke benchmark test (`BenchmarkTest.SmokeOracleCpuPasses`) requires prepared smoke data; it skips if `data/smoke_kitti` is missing.

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

## Recommended pre-push checklist

```bash
./scripts/ci.sh
# Optional with GPU:
./scripts/ci.sh --cuda && ./scripts/run_benchmark.sh
```
