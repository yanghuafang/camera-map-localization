/// Micro-benchmark kernels: distance transform and pose-grid image costs (CPU/CUDA).
#include <cam_loc/benchmark/micro_benchmarks.hpp>

#include <cam_loc/core/distance_transform_cpu.hpp>
#include <cam_loc/cuda/distance_transform.hpp>
#include <cam_loc/core/pose_sampler.hpp>
#include <cam_loc/perception/synthesize.hpp>
#include <cam_loc/types/params.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace cam_loc::benchmark {

namespace {

double percentile(std::vector<double> samples, double p) {
  if (samples.empty()) return 0.0;
  std::sort(samples.begin(), samples.end());
  const double rank = p * static_cast<double>(samples.size() - 1);
  const size_t lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = static_cast<size_t>(std::ceil(rank));
  const double t = rank - static_cast<double>(lo);
  return samples[lo] * (1.0 - t) + samples[hi] * t;
}

MicroBenchmarkResult summarize(const std::string& name, bool use_cuda,
                               const std::vector<double>& samples) {
  MicroBenchmarkResult r;
  r.name = name;
  r.use_cuda = use_cuda;
  r.iterations = static_cast<int>(samples.size());
  if (samples.empty()) return r;
  double sum = 0.0;
  for (double s : samples) sum += s;
  r.mean_ms = sum / static_cast<double>(samples.size());
  r.p95_ms = percentile(samples, 0.95);
  return r;
}

kitti::MapChunk straightMap() {
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

std::vector<uint8_t> syntheticBinary(int w, int h) {
  std::vector<uint8_t> binary(static_cast<size_t>(w * h), 255);
  for (int y = h / 3; y < 2 * h / 3; y += 4) {
    for (int x = 0; x < w; ++x) {
      binary[static_cast<size_t>(y * w + x)] = 0;
    }
  }
  return binary;
}

}  // namespace

std::vector<MicroBenchmarkResult> runMicroBenchmarks(const kitti::Calibration& calib,
                                                     int iterations) {
  std::vector<MicroBenchmarkResult> out;
  constexpr int kW = 1241;
  constexpr int kH = 376;
  const auto binary = syntheticBinary(kW, kH);

  {
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(iterations));
    for (int i = 0; i < iterations; ++i) {
      std::vector<float> dt;
      const auto t0 = std::chrono::steady_clock::now();
      core::DistanceTransformCpu::compute(binary, kW, kH, dt);
      const auto t1 = std::chrono::steady_clock::now();
      samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    out.push_back(summarize("distance_transform_cpu", false, samples));
  }

#ifdef CAMLOC_CUDA_ENABLED
  if (cuda::isAvailable()) {
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(iterations));
    for (int i = 0; i < iterations; ++i) {
      std::vector<float> dt;
      const auto t0 = std::chrono::steady_clock::now();
      cuda::computeDistanceTransformGpu(binary, kW, kH, dt);
      const auto t1 = std::chrono::steady_clock::now();
      samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    out.push_back(summarize("distance_transform_gpu", true, samples));
  }
#endif

  const auto map = straightMap();
  core::Projection proj(calib);
  LocalizationParams params;
  core::PoseSampler sampler(params);
  sampler.setProjection(proj);
  const Mat44 T = Mat44::Identity();
  const auto perception = perception::synthesizeFromMap(map, proj, T, 0);

  core::LabelledDistanceTransform dt;
  sampler.buildImageDt(perception, dt);
  core::CostGrid costs(params.grid);

  {
    params.use_cuda = false;
    core::PoseSampler cpu_sampler(params);
    cpu_sampler.setProjection(proj);
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(iterations));
    for (int i = 0; i < iterations; ++i) {
      const auto t0 = std::chrono::steady_clock::now();
      cpu_sampler.computeImageCosts(map, T, dt, costs);
      const auto t1 = std::chrono::steady_clock::now();
      samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    out.push_back(summarize("pose_grid_image_cpu", false, samples));
  }

#ifdef CAMLOC_CUDA_ENABLED
  if (cuda::isAvailable()) {
    params.use_cuda = true;
    core::PoseSampler gpu_sampler(params);
    gpu_sampler.setProjection(proj);
    std::vector<double> samples;
    samples.reserve(static_cast<size_t>(iterations));
    for (int i = 0; i < iterations; ++i) {
      const auto t0 = std::chrono::steady_clock::now();
      gpu_sampler.computeImageCosts(map, T, dt, costs);
      const auto t1 = std::chrono::steady_clock::now();
      samples.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    out.push_back(summarize("pose_grid_image_gpu", true, samples));
  }
#endif

  return out;
}

}  // namespace cam_loc::benchmark
