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
| Math | `MathTest` — relative transforms and sequence id formatting |
