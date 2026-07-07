#pragma once

/// Isolated micro-benchmarks for hot paths (distance transform, pose-grid image costs).
///
/// Uses synthetic imagery and a straight two-lane map; timings are mean/p95 over iterations.

#include <cam_loc/kitti/types.hpp>

#include <string>
#include <vector>

namespace cam_loc::benchmark {

/// Timing result for one micro-benchmark kernel (CPU or CUDA).
struct MicroBenchmarkResult {
  std::string name;
  bool use_cuda = false;
  double mean_ms = 0.0;
  double p95_ms = 0.0;
  int iterations = 0;
};

/// Run DT + pose-grid image cost benchmarks with smoke-kitty calibration.
std::vector<MicroBenchmarkResult> runMicroBenchmarks(const kitti::Calibration& calib,
                                                       int iterations = 30);

}  // namespace cam_loc::benchmark
