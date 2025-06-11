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
$B/apps/benchmark/benchmark --data-root "$D" --list

# Smoke regression only
$B/apps/benchmark/benchmark \
  --data-root "$D" \
  --filter smoke \
  --output-json "$D"/benchmark_results.json

# Micro-benchmarks (30 iterations)
$B/apps/benchmark/benchmark --data-root "$D" --micro

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

## Threshold reference

Expected gates (per-case RMSE, yaw, match-rate, and latency thresholds) are defined in [`src/benchmark/benchmark_runner.cc`](../src/benchmark/benchmark_runner.cc).

> **Build type dominates every timing here.** With no `CMAKE_BUILD_TYPE` the
> build carries no `-O` flag at all, and unoptimized Eigen inlines nothing: the
> same smoke case runs ~530 ms/frame instead of ~6.8 ms. The default is now
> `Release` ([BUILD.md](BUILD.md#build-type)); a number measured under `--debug`
> says nothing about the algorithm.

Smoke results, Release, arm64 / Apple Clang 21 — the full 400 m route:

| Case | Translation RMSE | Yaw RMSE | Match rate | Frame latency |
|------|-----------------|----------|------------|---------------|
| `smoke_oracle_cpu` | 0.289 m | 0.195° | 93.8% | 6.4 ms mean, 6.7 ms p95 |
| `smoke_noisy_cuda` | 0.272 m | 0.086° | 92.5% | 6.5 ms mean |

**That error is the map.** The corridor is surveyed to 0.2 m and the estimate
lands about there. No localizer beats the map it matches against, so a figure
near the survey sigma is the right answer, not a poor one — a regression would
look like the search losing the basin or the match rate collapsing.

**And the route length is part of the measurement.** The survey error
decorrelates over 80 m, so a route shorter than a few correlation lengths sees
one draw of it; the earlier 60 m sequence reported whatever that single
displacement happened to be, which moved by 7× between two standard libraries
drawing from the same seed. 400 m spans five correlation lengths.

Sweeping the survey sigma over the full route, split into the axis the map
constrains and the one it does not:

| what is broken | lateral | along-track | total | ANEES |
|----------------|---------|-------------|-------|-------|
| **nothing — `--map-error-m 0`, the closed loop** | 0.002 m | 0.020 m | 0.020 m | 0.002 |
| map surveyed to 0.05 m | 0.066 m | — | 0.066 m | 0.08 |
| map surveyed to 0.10 m | 0.131 m | — | 0.131 m | 0.18 |
| **map surveyed to 0.20 m (the default)** | 0.263 m | 0.161 m | 0.308 m | 0.30 |
| map surveyed to 0.40 m | 0.485 m | — | 0.546 m | 0.36 |
| + 6 px of perception noise | 0.259 m | 0.083 m | 0.272 m | 0.19 |
| + 1% odometry scale error | 0.264 m | 0.892 m | 0.930 m | **1.66** |

Translation RMSE tracks the survey sigma at roughly 1.4×, linearly — the local
map inside one query is displaced almost bodily, and a bodily displacement is
indistinguishable from being somewhere else.

The **along-track** column is the one to watch. It stays near the lateral figure
until the odometry is corrupted, then runs away: 400 m of 1% scale error is 4 m
of accumulated drift, and lane geometry gives the matcher almost nothing to pull
it back with. That is also where ANEES crosses 1 — the filter is over-confident
under drift, with 67% of frames inside the 95% bound instead of 95%. The 60 m
route was too short to show it.

Micro-benchmarks, same build (`--micro`, 30 timed iterations, best of 12):

| Kernel | CPU mean | CPU p95 |
|--------|----------|---------|
| Distance transform | 4.90 ms | 5.70 ms |
| Pose grid (image) | 0.35 ms | 0.40 ms |
| Pose grid (image), one thread | 1.45 ms | 1.48 ms |

The last two rows are the same kernel with the fan-out on and off, so its
speedup is read within one build rather than against a number from another
machine: **4.1× on ten cores**. Not ten, because the smoke map is small — each
hypothesis is roughly 170 ns of work — so thread startup and memory bandwidth
cap it.

Read that 4.1× as what threading buys *within this implementation*, not as what
the change bought. Splitting the search into blocks meant flattening three
nested loops into a single index, and recovering `(ix, iy, iw)` from it costs a
division and a modulo per hypothesis: single-threaded, the flattened kernel runs
1.45 ms against the 1.13 ms the nested loops took — roughly 30% slower. Measured against
the code it replaced the net gain is **3.4×**, and on the twenty-thread Linux
host 1.9× rather than 2.4×. The same 30% shows on both machines, which is what
says it is the index arithmetic and not the scheduler. On KITTI sequence 00, whose local map carries far more points,
end-to-end frame latency goes 17.3 ms to 9.3 ms.

`--micro` discards a few calls before timing and reuses each kernel's output
buffer, so what it reports is the kernel. Timing the allocation as well made the
distance transform bimodal — its 1.9 MB output either comes back from the
allocator's cache for nothing or costs a further 2.3 ms in first-touch page
faults — and that is what used to swing this row between 2.7 and 4.4 ms. The
engine does allocate one distance transform per frame, so that cost is real; it
is simply not the kernel's.

## CUDA

Measured on Ubuntu 26.04, g++ 15.2, nvcc 12.4, **NVIDIA RTX A6000**, hosted by
an i9-9820X: ten physical cores with SMT, so `nproc` reports 20. These are that
box's numbers, not the Apple Silicon ones above; compare within the table, not
across it.

| Kernel | CPU (best of 12) | CUDA | Speedup |
|--------|------------------|------|---------|
| Distance transform | 7.2 ms | 2.8 ms | 2.6× |
| Pose grid (image), 20 threads | 3.3 ms | 0.86 ms | 3.8× |
| Pose grid (image), one thread | 7.9 ms | — | — |

Both columns come from the same runs on an otherwise idle host, with both GPUs
reporting 0% utilisation. That is not a detail: the A6000 is shared, and a
timing taken while another job holds it measures that job — on the distance
transform, enough to put the GPU *above* the CPU.

The thread fan-out is **2.4×** here, against 4.4× on ten Apple Silicon cores.
Both hosts have ten physical cores, so that gap is the machine — memory
bandwidth and single-core throughput — and not contention: the single-threaded
pose grid alone takes 7.9 ms here against 1.46 ms there.

> The CPU column is held down further by the host's `powersave` governor, which
> idles these cores at 1.2 GHz against a 3.3 GHz base and ramps only under load.
> Setting `performance` needs root, which is why best-of-N is used instead: it
> picks the run whose clocks had come up.

End to end, same 50-frame smoke case:

| Case | Translation RMSE | Yaw RMSE | Frame latency |
|------|-----------------|----------|---------------|
| `smoke_oracle_cpu` | 0.289 m | 0.195° | 43.8 ms best of five |
| `smoke_oracle_cuda` | 0.289 m | 0.195° | 11.7 ms mean, **6.8 ms p95** |

> Both RMSE columns are deterministic and agree with the arm64 table above to
> the digit. Both latency columns are the best of five runs on the same idle
> host; the CPU runs spread 43.8 to 45.7 ms, the CUDA ones 11.7 to 15.1 ms.

Read the p95 on the CUDA row. A p95 *below* the mean says a few frames are far
slower than the rest, and here it is the first one: creating the CUDA context
and its allocations costs roughly a quarter of a second, which spread over fifty
frames adds about 5 ms to the mean and nothing to the other forty-nine. Running
the whole suite in one process shows it directly — the second CUDA case, on an
already-warm context, reports 6.3 ms mean against 6.5 ms p95, with the ordering
back the right way round.

Steady state is therefore ~6.5 ms, and 11.7 ms is what a fifty-frame run costs
including start-up. Neither is the 3.8× of the pose grid alone: the distance
transform gains only 2.7×, the map queries, perception I/O and the EKF update
never leave the CPU, and the CPU side of the comparison now uses every core.

The two RMSE figures agree to three digits: the kernels work in `float` where the CPU
path uses `double`, so the two argmins occasionally land on different cells.

The same effect sets a floor on how closely two *machines* can be expected to
agree. Seeded inputs are portable — `cam_loc::Sampler` sees to that, so macOS
and Linux build the identical map from the identical seed — but the arithmetic
is not: different vector units and FMA contraction differ in the last bit, and
the argmin is a *discrete* choice, so one ulp can tip a hypothesis into a
neighbouring cell and the trajectory diverges slightly from there on. Over 50
frames the two agree to six significant figures; over 400 they are about 1%
apart. A threshold has to leave room for that, which is why the gates sit well
above the measured value rather than snug against it.
Both are far inside the regression threshold.

> A speedup near 100× is what an **unoptimized** CPU baseline produces, not a
> GPU result; the figures above are all measured against `Release`.
>
> Note also that the `*_cuda` cases silently fall back to the CPU wherever CUDA
> is unavailable, so they pass on a GPU-less host without exercising a GPU —
> which is why these numbers require the Ubuntu box (`scripts/remote_ubuntu.sh`).

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

None of them gates anything. `BenchmarkThresholds` is unchanged, so a case still
fails on `max_rmse_translation_m`, `max_rmse_yaw_deg`, `min_match_rate`,
`max_flat_rate` or `max_mean_frame_ms`, and on nothing else. A per-axis limit
wants numbers measured across the real sequences rather than a split of the
existing translation budget.

## Unit test

`BenchmarkTest.SmokeOracleCpuPasses` runs the `smoke_oracle_cpu` case with 10 frames (faster than the full app default).
