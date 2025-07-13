# Open Items

Last updated: 2026-08-31.

What is genuinely unfinished, and what is finished but unverified. Items that
were closed are not listed — the git history has them, and a roadmap that
carries its own changelog stops being readable.

## Verification this project cannot do for itself

- [x] **The GPU path has now been run.** All four `CudaTest` parity tests pass on
      an RTX A6000 (Ubuntu 26.04, g++ 15.2, nvcc 12.4), and the benchmark numbers
      in [BENCHMARK.md](BENCHMARK.md) are measured rather than assumed. Doing so
      found one real defect: `AggregateKernel` still warped the *pose* into the
      history frame after the CPU had been changed to warp the *error*, so the
      two disagreed. Fixed, and `CudaTest.AggregateMatchesCpu` is what catches it.
- [ ] Nothing re-runs this automatically. GitHub's runners have no GPU, so the
      CUDA result is only as current as the last manual
      `scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda`.
- [ ] **No accuracy number on real KITTI.** Everything reported comes from the
      synthetic smoke sequence. Running on sequence 00 needs the velodyne archive
      (~80 GB) for `preprocess_kitti --mode lidar`, or a prepared label-PNG
      directory for `--mode png`.

## The map is derived from ground truth

KITTI Odometry ships no HD map, so `TrajectoryCorridorMap` builds one by
offsetting the ground-truth path: lane geometry on the road surface, poles and
signs beside it.

That makes every accuracy number here an upper bound on the *backend* — whether
the search, the frames and the filter agree — and not a measurement of
localization against a real map. The honest number needs a surveyed map, which
this dataset does not have.

- [ ] Import upright landmarks from OpenStreetMap. The parser reads ways only;
      `highway=street_lamp` and `traffic_signals` are nodes, and they are the one
      real-world source of pole-like geometry available without a survey.
- [ ] GPS tie-point calibration, so an OSM extract can be aligned to a sequence
      without hand-tuning the georef.

## Known limits of the current algorithm

- [ ] **The bird's-eye branch constrains only two of three DOF**, so it is off by
      default. A top-down view of lane geometry is invariant along the road, and
      only ground-plane classes can go through inverse perspective at all — an
      elevated pole would be placed at whatever range that assumption implies.
      Switching it on measurably costs accuracy (see
      [ARCHITECTURE.md](ARCHITECTURE.md)). It is kept because it is a real second
      view of lateral offset and heading; making it *useful* would mean weighting
      per DOF rather than averaging a scalar.
- [ ] **`PolylineMap::QueryLocalMap` closes gaps.** A polyline that leaves the
      query radius and re-enters comes back as one polyline with the gap bridged,
      so the matcher sees a segment of map that does not exist. Splitting it into
      runs would be more correct.
- [ ] **Dashed lane markings are stored as a continuous polyline.** The stripe
      *ends* carry along-track information that the current representation throws
      away.
- [ ] Multi-camera: `preprocess_kitti` reads cam0 only, and `Calibration::P1` is
      parsed and never used.

## Test coverage

`./scripts/coverage.sh` reports ~74% of lines in `src/`. The gaps that matter:

- [ ] `perception/adapter.cc` at 0% — the on-disk perception path is exercised
      only through `ResolvePerception`, never directly.
- [ ] `perception/resolve.cc` at ~29% — the file and noisy sources have no test;
      only the oracle branch runs.
- [ ] `map/polyline_map.cc` at ~57% — the spatial index only engages above 512
      points, which no test reaches.

## Deliberately not done

Recorded so they are not re-proposed as oversights.

- **No camera perception model.** Perception is an input to this project. The
  supported sources are SemanticKITTI labels, the ground-truth oracle, and noise
  on either.
- **No second localization algorithm.** One pose-grid-plus-filter pipeline, made
  to work well, rather than several to compare.
- **No dataset beyond KITTI Odometry and SemanticKITTI.** Traffic lights and
  crosswalks are therefore out of reach: SemanticKITTI has no class for either.
- **`-Werror` is not gated in CI.** `-Wall` differs between compilers and
  releases, so gating would turn a toolchain upgrade into a red build on
  unrelated changes. `CAMLOC_WERROR=ON` turns it on locally.
