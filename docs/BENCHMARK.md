# Benchmarks

The `benchmark` app runs regression cases (accuracy + match quality + frame latency) and optional micro-benchmarks (distance transform and pose-grid throughput).

## Quick run

```bash
./scripts/run_benchmark.sh
```

This will:

1. Regenerate `<repo>-data/smoke_kitti/` via `prepare_smoke_kitti.sh`
2. Build `benchmark` if needed
3. Run **smoke** cases → `<repo>-data/benchmark_results.json`
4. Run **micro-benchmarks** (DT + pose grid; the CUDA variants only when a GPU is present)
5. Run **kitti00** cases if `<repo>-data/kitti_odometry/poses/00.txt` exists

## Manual invocation

```bash
B=../camera-map-localization-build
D=../camera-map-localization-data

# List cases
$B/apps/benchmark/benchmark --list --repo-root .

# Smoke regression only
$B/apps/benchmark/benchmark \
  --repo-root . \
  --filter smoke \
  --output-json "$D"/benchmark_results.json

# Micro-benchmarks (30 iterations)
$B/apps/benchmark/benchmark --repo-root . --micro

# Report only (do not fail CI)
$B/apps/benchmark/benchmark --repo-root . --filter smoke --no-fail
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
and the residual is grid discretization plus filter lag. Earlier versions of this
table reported 0 m, which was not accuracy — the pose grid's second axis was
sweeping camera height and the corridor map's "lane boundaries" were above and
below the camera, so the centre cell won by construction.

Micro-benchmarks, same build (`--micro`, 30 iterations):

| Kernel | CPU mean | CPU p95 |
|--------|----------|---------|
| Distance transform | 2.4 ms | 2.6 ms |
| Pose grid (image) | 1.1 ms | 1.2 ms |

## CUDA

Measured on Ubuntu 26.04, g++ 15.2, nvcc 12.4, **NVIDIA RTX A6000**. CPU figures
here are that box's 20-core Xeon-class host, not the Apple Silicon numbers above,
so compare within the table and not across it.

| Kernel | CPU mean | CUDA mean | Speedup |
|--------|----------|-----------|---------|
| Distance transform | 11.0 ms | 6.7 ms | 1.6× |
| Pose grid (image) | 6.4 ms | 0.89 ms | **7.2×** |

End to end, same 50-frame smoke case:

| Case | Translation RMSE | Yaw RMSE | Frame latency |
|------|-----------------|----------|---------------|
| `smoke_oracle_cpu` | 0.0030 m | 0.0045° | 39.9 ms mean, 44.7 ms p95 |
| `smoke_oracle_cuda` | 0.0068 m | 0.0050° | 12.8 ms mean, **7.9 ms p95** |

Read the p95 on the CUDA row. A p95 *below* the mean says a few frames are far
slower than the rest, and here it is the first one: creating the CUDA context
and its allocations costs roughly a quarter of a second, which spread over fifty
frames adds about 5 ms to the mean and nothing to the other forty-nine. Running
the whole suite in one process shows it directly — the second CUDA case, on an
already-warm context, reports 6.6 ms mean against 7.6 ms p95, with the ordering
back the right way round.

So steady state is ~7 ms against the CPU's ~40 ms, roughly 5×, and 12.8 ms is
what a fifty-frame run costs including start-up. Neither is the 7.2× of the pose
grid alone: the distance transform gains only 1.6×, and the map queries,
perception I/O and the EKF update never leave the CPU.

The small RMSE difference between the two is expected: the kernels work in
`float` where the CPU path uses `double`, so the two argmins occasionally land
on different cells. Both are far inside the regression threshold.

> A previously published "~100× speedup" was measured against an **unoptimized**
> CPU build and was not a GPU result. The figures above replace it.
>
> Note also that the `*_cuda` cases silently fall back to the CPU wherever CUDA
> is unavailable, so they pass on a GPU-less host without exercising a GPU —
> which is why these numbers require the Ubuntu box (`scripts/remote_ubuntu.sh`).

## JSON output fields

`benchmark_results.json` includes per case: `rmse_translation_m`,
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

None of them gates anything. `BenchmarkThresholds` is unchanged, so a case still
fails on `max_rmse_translation_m`, `max_rmse_yaw_deg`, `min_match_rate`,
`max_flat_rate` or `max_mean_frame_ms`, and on nothing else. A per-axis limit
wants numbers measured across the real sequences rather than a split of the
existing translation budget.

## Unit test

`BenchmarkTest.SmokeOracleCpuPasses` runs the `smoke_oracle_cpu` case with 10 frames (faster than the full app default).
