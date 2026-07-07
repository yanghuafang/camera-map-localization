# Benchmarks

The `benchmark` app runs regression cases (accuracy + match quality + frame latency) and optional micro-benchmarks (distance transform and pose-grid throughput).

## Quick run

```bash
./scripts/run_benchmark.sh
```

This will:

1. Regenerate `data/smoke_kitti/` via `prepare_smoke_kitti.sh`
2. Build `benchmark` if needed
3. Run **smoke** cases → `data/benchmark_results.json`
4. Run **micro-benchmarks** (DT + pose grid, CPU vs CUDA)
5. Run **kitti00** cases if `data/kitti_odometry/poses/00.txt` exists

## Manual invocation

```bash
# List cases
./build/apps/benchmark/benchmark --list --repo-root .

# Smoke regression only
./build/apps/benchmark/benchmark \
  --repo-root . \
  --filter smoke \
  --output-json data/benchmark_results.json

# Micro-benchmarks (30 iterations)
./build/apps/benchmark/benchmark --repo-root . --micro

# Report only (do not fail CI)
./build/apps/benchmark/benchmark --repo-root . --filter smoke --no-fail
```

## Default regression cases

| Case | Data | Mode | Checks |
|------|------|------|--------|
| `smoke_oracle_cpu` | `data/smoke_kitti` | oracle, GT plane, CPU | RMSE, yaw, match rate |
| `smoke_oracle_cuda` | smoke | oracle, CUDA | same |
| `smoke_noisy_cuda` | smoke | noisy + CUDA | relaxed RMSE, match ≥ 0.8 |
| `kitti00_synth_cuda` | `data/kitti_odometry` | auto synth, CUDA | needs poses+calib |
| `kitti00_real_cuda` | + `data/perception` | file perception | optional |
| `kitti00_noisy_cuda` | + perception | noisy | optional |

Cases with missing data return `IoError` and are counted as skipped failures in the suite summary.

## Threshold reference

Expected gates (per-case RMSE, yaw, match-rate, and latency thresholds) are defined in [`src/benchmark/benchmark_runner.cpp`](../src/benchmark/benchmark_runner.cpp).

Typical smoke results (oracle, 50 frames, GT sampling plane):

| Metric | CPU (~4 s/frame) | CUDA (~30 ms/frame) |
|--------|------------------|---------------------|
| Translation RMSE | 0 m | 0 m |
| Match rate | 100% | 100% |
| Yaw RMSE | &lt; 0.5° | &lt; 0.5° |

Micro-benchmarks (smoke calib, indicative):

| Kernel | CPU mean | CUDA mean |
|--------|----------|-----------|
| Distance transform | ~65 ms | ~6 ms |
| Pose grid (image) | ~1300 ms | ~1 ms |

Exact numbers depend on GPU and grid size.

## JSON output fields

`benchmark_results.json` includes per case: `rmse_translation_m`, `rmse_yaw_deg`, `match_rate`, `flat_rate`, `mean_frame_ms`, `p95_frame_ms`, `passed`, `failure_reason`.

## Unit test

`BenchmarkTest.SmokeOracleCpuPasses` runs the `smoke_oracle_cpu` case with 10 frames (faster than the full app default).
