# Testing

## Unit tests (GoogleTest)

Build and run everything:

```bash
./scripts/ci.sh
```

`CMAKE_BUILD_TYPE` defaults to `Release` — see [BUILD.md](BUILD.md#build-type)
for why that is not a detail.

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
| Perception | `PerceptionJsonTest` |
| Eval | `PoseErrorTest` — the error split onto vehicle axes, including that it follows GT heading; `SequenceEvalTest` |
| End to end | `LocalizationEngineTest` — straight *and* turning, with pose accuracy asserted |
| Filter | `LocalizationKfTest` — convergence away from identity attitude, and covariance well-formedness |
| Core matching | `DistanceTransformTest`, `PoseSamplerTest` — including along-track recovery and sub-cell refinement; `CostGridTest`, `CostAggregatorTest` |

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

## Style gates

Not a test, but `ci.sh` runs it and a red gate blocks a change just the same, so
it belongs in the same pass:

```bash
./scripts/format.sh          # clang-format + trailing-whitespace strip
./scripts/format.sh --check  # what the gate runs: reports and exits 1, writes nothing
```

It needs `clang-format` (`brew install llvm` on macOS, since Xcode ships it not;
`sudo apt install clang-format` on Ubuntu).
