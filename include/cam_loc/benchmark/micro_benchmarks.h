#ifndef CAM_LOC_BENCHMARK_MICRO_BENCHMARKS_H_
#define CAM_LOC_BENCHMARK_MICRO_BENCHMARKS_H_

/// Isolated micro-benchmarks for hot paths (distance transform, pose-grid image
/// costs).
///
/// Uses synthetic imagery and a straight two-lane map; timings are mean/p95
/// over iterations.

#include <string>
#include <vector>

#include "cam_loc/kitti/types.h"

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
std::vector<MicroBenchmarkResult> RunMicroBenchmarks(
    const kitti::Calibration& calib, int iterations = 30);

}  // namespace cam_loc::benchmark

#endif  // CAM_LOC_BENCHMARK_MICRO_BENCHMARKS_H_
