# Benchmarks

The `benchmark` app runs regression cases: accuracy, match quality and frame latency, each gated against a per-case threshold.

## Quick run

```bash
./scripts/run_benchmark.sh
```

This will:

1. Regenerate `<repo>-data/smoke_kitti/` via `prepare_smoke_kitti.sh`
2. Build `benchmark` if needed
3. Run **smoke** cases → `<repo>-data/benchmark_results.json`
4. Run **kitti00** cases if `<repo>-data/kitti_odometry/poses/00.txt` exists

## Manual invocation

```bash
B=../camera-map-localization-build
D=../camera-map-localization-data

# List cases
$B/apps/benchmark/benchmark --data-root "$D" --list

# Smoke regression only
$B/apps/benchmark/benchmark \
  --data-root "$D" \
  --filter smoke \
  --output-json "$D"/benchmark_results.json

# Report only (do not fail CI)
$B/apps/benchmark/benchmark --data-root "$D" --filter smoke --no-fail
```

## Default regression cases

| Case | Data | Mode | Checks |
|------|------|------|--------|
| `smoke_oracle_cpu` | `<repo>-data/smoke_kitti` | oracle, GT plane, CPU | RMSE, yaw, match rate |
| `smoke_oracle_cuda` | smoke | oracle, CUDA | same |
| `smoke_noisy_cuda` | smoke | noisy + CUDA | relaxed RMSE, match ≥ 0.8 |
| `kitti00_synth_cuda` | `<repo>-data/kitti_odometry` | auto synth, CUDA | needs poses+calib |
| `kitti00_real_cuda` | + `<repo>-data/perception` | file perception | optional |
| `kitti00_noisy_cuda` | + perception | noisy | optional |

Cases with missing data return `IoError` and are counted as skipped failures in the suite summary.

The `*_cuda` cases silently fall back to the CPU wherever CUDA is unavailable,
so they pass on a GPU-less host without exercising a GPU — which is why any
GPU number has to come from the Ubuntu box (`scripts/remote_ubuntu.sh`).

## Threshold reference

Expected gates (per-case RMSE, yaw, match-rate, and latency thresholds) are defined in [`src/benchmark/benchmark_runner.cc`](../src/benchmark/benchmark_runner.cc).

> **Build type dominates every timing here.** With no `CMAKE_BUILD_TYPE` the
> build carries no `-O` flag at all, and unoptimized Eigen inlines nothing: the
> same smoke case runs ~2.6 s/frame instead of ~15 ms. The default is now
> `Release` ([BUILD.md](BUILD.md#build-type)); a number measured under `--debug`
> says nothing about the algorithm.

Smoke results, Release, arm64 / Apple Clang 21:

| Case | Translation RMSE | Yaw RMSE | Match rate | Frame latency |
|------|-----------------|----------|------------|---------------|
| `smoke_oracle_cpu` | 0.0030 m | 0.0045° | 100% | 10.0 ms mean, 10.7 ms p95 |
| `smoke_noisy_cuda` | 0.0635 m | 0.0391° | 100% | 9.9 ms mean |

These are a real three-DOF search: forward, left and heading are all estimated,
and the residual is grid discretization plus filter lag.

## JSON output fields

`benchmark_results.json` carries `cuda_device`, empty on a CPU-only host — a
timing without the hardware behind it is not a measurement anyone can repeat —
and per case: `rmse_translation_m`,
`rmse_lateral_m`, `rmse_longitudinal_m`, `rmse_vertical_m`, `bias_lateral_m`,
`bias_longitudinal_m`, `max_abs_lateral_m`, `max_abs_longitudinal_m`,
`rmse_yaw_deg`, `match_rate`, `flat_rate`, `mean_frame_ms`, `p95_frame_ms`,
`passed`, `failure_reason`.

The three `rmse_*` axis fields decompose `rmse_translation_m` — they sum in
quadrature to it exactly, which is the cheapest check that the split is sound.
The `bias_*` fields are signed means, and they are the ones to read first: a
lateral or along-track error that is nearly all bias is a systematic offset, not
noise, and only the sign separates the two. The `max_abs_*` fields give the
worst single-frame excursion per axis, which an RMS over a long sequence hides.

None of them gates anything. `BenchmarkThresholds` fails a case on
`max_rmse_translation_m`, `max_rmse_yaw_deg`, `min_match_rate`, `max_flat_rate`
or `max_mean_frame_ms`, and on nothing else. A per-axis limit wants numbers
measured across the real sequences rather than a split of the existing
translation budget.

## Unit test

`BenchmarkTest.SmokeOracleCpuPasses` runs the `smoke_oracle_cpu` case with 10 frames (faster than the full app default).
