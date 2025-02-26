# Testing

## Unit tests (GoogleTest)

Build and run everything:

```bash
./scripts/ci.sh
```

Builds land beside the repository, one directory per configuration
(`../<repo>-build`, `../<repo>-build-debug`, …); see
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
| Math | `MathTest` — relative transforms and sequence id formatting |

## Style gates

Not a test, but `ci.sh` runs it and a red gate blocks a change just the same, so
it belongs in the same pass:

```bash
./scripts/format.sh          # clang-format + trailing-whitespace strip
./scripts/format.sh --check  # what the gate runs: reports and exits 1, writes nothing
```

It needs `clang-format` (`brew install llvm` on macOS, since Xcode ships it not;
`sudo apt install clang-format` on Ubuntu).
