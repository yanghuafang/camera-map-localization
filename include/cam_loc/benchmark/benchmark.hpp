#pragma once

/// Regression benchmark suite wrapping sequence evaluation.
///
/// Each BenchmarkCase runs runSequenceEval and checks pose / matching / latency thresholds.
/// Missing dataset paths are skipped (IoError) rather than failing the whole suite.

#include <cam_loc/kitti/sequence_eval.hpp>
#include <cam_loc/perception/resolve.hpp>

#include <string>
#include <vector>

namespace cam_loc::benchmark {

/// Pass/fail gates applied to a SequenceEvalSummary.
struct BenchmarkThresholds {
  double max_rmse_translation_m = 1.0;
  double max_rmse_yaw_deg = 5.0;
  double min_match_rate = 0.5;
  double max_flat_rate = 0.5;
  double max_mean_frame_ms = 60000.0;
};

/// One regression scenario (dataset paths, perception mode, CUDA/GT flags, thresholds).
struct BenchmarkCase {
  std::string name;
  std::string kitti_root;
  std::string perception_root;
  int sequence = 0;
  int start_frame = 0;
  int max_frames = 50;
  bool use_cuda = false;
  bool use_gt_plane = false;
  cam_loc::perception::PerceptionSource perception_source =
      cam_loc::perception::PerceptionSource::kAuto;
  cam_loc::perception::PerceptionNoiseParams noise;
  BenchmarkThresholds thresholds;
};

/// Outcome of a single benchmark case (summary + pass/fail reason).
struct BenchmarkResult {
  std::string name;
  bool passed = false;
  std::string failure_reason;
  cam_loc::kitti::SequenceEvalSummary summary;
  int num_frames = 0;
};

/// Aggregate pass/fail counts over a vector of cases.
struct BenchmarkSuiteResult {
  std::vector<BenchmarkResult> cases;
  int passed = 0;
  int failed = 0;
};

/// Built-in regression cases (smoke + optional real KITTI paths).
std::vector<BenchmarkCase> defaultBenchmarkSuite(const std::string& repo_root);

bool checkThresholds(const BenchmarkCase& spec, const cam_loc::kitti::SequenceEvalSummary& summary,
                     std::string& out_reason);

/// Load poses/calib/map, run sequence eval, and check thresholds for one case.
Status runBenchmarkCase(const BenchmarkCase& spec, BenchmarkResult& out);

/// Run all cases; IoError cases are marked failed/skipped without aborting.
Status runBenchmarkSuite(const std::vector<BenchmarkCase>& cases, BenchmarkSuiteResult& out);

}  // namespace cam_loc::benchmark
