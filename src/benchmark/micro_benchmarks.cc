/// Micro-benchmark kernels: distance transform and pose-grid image costs
/// (CPU/CUDA).
#include "cam_loc/benchmark/micro_benchmarks.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include "cam_loc/core/distance_transform_cpu.h"
#include "cam_loc/core/pose_sampler.h"
#include "cam_loc/cuda/distance_transform.h"
#include "cam_loc/perception/synthesize.h"
#include "cam_loc/types/params.h"

namespace cam_loc::benchmark {

namespace {

double Percentile(std::vector<double> samples, double p) {
  if (samples.empty()) return 0.0;
  std::sort(samples.begin(), samples.end());
  const double rank = p * static_cast<double>(samples.size() - 1);
  const auto lo = static_cast<size_t>(std::floor(rank));
  const auto hi = static_cast<size_t>(std::ceil(rank));
  const double t = rank - static_cast<double>(lo);
  return samples[lo] * (1.0 - t) + samples[hi] * t;
}

MicroBenchmarkResult Summarize(const std::string& name, bool use_cuda,
                               const std::vector<double>& samples) {
  MicroBenchmarkResult r;
  r.name = name;
  r.use_cuda = use_cuda;
  r.iterations = static_cast<int>(samples.size());
  if (samples.empty()) return r;
  double sum = 0.0;
  for (double s : samples) sum += s;
  r.mean_ms = sum / static_cast<double>(samples.size());
  r.p95_ms = Percentile(samples, 0.95);
  return r;
}

/// Times @p body @p iterations times, after a few untimed warm-up calls.
///
/// The first calls into a kernel are not measuring the kernel: they pay for the
/// output buffer's first allocation, the page faults that first touch it, and
/// the clock ramping up from idle. Here that ran 5 to 7 ms against a steady
/// 2.4, and over 30 samples it is about three of them -- enough to land on the
/// p95 and drag the mean with it.
///
/// @p body is expected to reuse its output buffer rather than allocate one per
/// call. Timing the allocation as well makes the figure bimodal: a 1.9 MB
/// output either comes back from the allocator's cache for nothing or costs a
/// further 2.3 ms in first-touch page faults, and which one happens is a
/// property of the machine, not of the kernel. The engine does allocate per
/// frame, so that cost is real -- it is just not what a kernel timing is for.
template <typename Body>
MicroBenchmarkResult TimeKernel(const std::string& name, bool use_cuda,
                                int iterations, Body body) {
  constexpr int kWarmupIters = 5;
  for (int i = 0; i < kWarmupIters; ++i) body();

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(iterations));
  for (int i = 0; i < iterations; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    body();
    const auto t1 = std::chrono::steady_clock::now();
    samples.push_back(
        std::chrono::duration<double, std::milli>(t1 - t0).count());
  }
  return Summarize(name, use_cuda, samples);
}

kitti::MapChunk StraightMap() {
  kitti::MapChunk map;
  kitti::MapPolyline3D left;
  left.type = kitti::PolylineType::kLaneSolid;
  kitti::MapPolyline3D right;
  right.type = kitti::PolylineType::kLaneSolid;
  for (int i = 0; i < 40; ++i) {
    const double x = static_cast<double>(i) * 2.0;
    left.points.emplace_back(x, -1.75, 0.0);
    right.points.emplace_back(x, 1.75, 0.0);
  }
  map.polylines.push_back(std::move(left));
  map.polylines.push_back(std::move(right));
  return map;
}

std::vector<uint8_t> SyntheticBinary(int w, int h) {
  std::vector<uint8_t> binary(static_cast<size_t>(w * h), 255);
  for (int y = h / 3; y < 2 * h / 3; y += 4) {
    for (int x = 0; x < w; ++x) {
      binary[static_cast<size_t>(y * w + x)] = 0;
    }
  }
  return binary;
}

}  // namespace

std::vector<MicroBenchmarkResult> RunMicroBenchmarks(
    const kitti::Calibration& calib, int iterations) {
  std::vector<MicroBenchmarkResult> out;
  constexpr int kW = 1241;
  constexpr int kH = 376;
  const auto binary = SyntheticBinary(kW, kH);

  // Reused across iterations, so what is timed is the kernel and not the 1.9 MB
  // allocation behind it -- see the comment on TimeKernel.
  std::vector<float> dt_out;
  out.push_back(TimeKernel("distance_transform_cpu", false, iterations, [&] {
    core::DistanceTransformCpu::Compute(binary, kW, kH, dt_out);
  }));

#ifdef CAMLOC_CUDA_ENABLED
  if (cuda::IsAvailable()) {
    out.push_back(TimeKernel("distance_transform_gpu", true, iterations, [&] {
      cuda::ComputeDistanceTransformGpu(binary, kW, kH, dt_out);
    }));
  }
#endif

  const auto map = StraightMap();
  core::Projection proj(calib);
  LocalizationParams params;
  core::PoseSampler sampler(params);
  sampler.set_projection(proj);
  const Mat44 T = Mat44::Identity();
  const auto perception = perception::SynthesizeFromMap(map, proj, T, 0);

  core::LabelledDistanceTransform dt;
  sampler.BuildImageDt(perception, dt);
  core::CostGrid costs(params.grid);

  {
    params.use_cuda = false;
    core::PoseSampler cpu_sampler(params);
    cpu_sampler.set_projection(proj);
    out.push_back(TimeKernel("pose_grid_image_cpu", false, iterations, [&] {
      cpu_sampler.ComputeImageCosts(map, T, dt, costs);
    }));
  }

  {
    // One thread, so the fan-out's speedup is visible in the same run rather
    // than compared against a number from another machine.
    params.use_cuda = false;
    params.cost_threads = 1;
    core::PoseSampler serial_sampler(params);
    serial_sampler.set_projection(proj);
    out.push_back(TimeKernel(
        "pose_grid_image_cpu_serial", false, iterations,
        [&] { serial_sampler.ComputeImageCosts(map, T, dt, costs); }));
    params.cost_threads = 0;
  }

#ifdef CAMLOC_CUDA_ENABLED
  if (cuda::IsAvailable()) {
    params.use_cuda = true;
    core::PoseSampler gpu_sampler(params);
    gpu_sampler.set_projection(proj);
    out.push_back(TimeKernel("pose_grid_image_gpu", true, iterations, [&] {
      gpu_sampler.ComputeImageCosts(map, T, dt, costs);
    }));
  }
#endif

  return out;
}

}  // namespace cam_loc::benchmark
