/// Default regression cases (smoke + optional real KITTI paths under data/).
#include <cam_loc/benchmark/benchmark.hpp>

#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/kitti/sequence_eval_runner.hpp>
#include <cam_loc/map/map_loader_util.hpp>

#include <sstream>

namespace cam_loc::benchmark {

namespace {

std::string joinReasons(const std::vector<std::string>& parts) {
  std::ostringstream oss;
  for (size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) oss << "; ";
    oss << parts[i];
  }
  return oss.str();
}

}  // namespace

std::vector<BenchmarkCase> defaultBenchmarkSuite(const std::string& repo_root) {
  const std::string smoke = repo_root + "/data/smoke_kitti";
  const std::string kitti = repo_root + "/data/kitti_odometry";
  const std::string perception = repo_root + "/data/perception";

  std::vector<BenchmarkCase> cases;

  {
    BenchmarkCase c;
    c.name = "smoke_oracle_cpu";
    c.kitti_root = smoke;
    c.max_frames = 50;
    c.use_gt_plane = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kOracle;
    c.thresholds.max_rmse_translation_m = 0.02;
    c.thresholds.max_rmse_yaw_deg = 1.0;
    c.thresholds.min_match_rate = 0.95;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }
  {
    BenchmarkCase c;
    c.name = "smoke_oracle_cuda";
    c.kitti_root = smoke;
    c.max_frames = 50;
    c.use_gt_plane = true;
    c.use_cuda = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kOracle;
    c.thresholds.max_rmse_translation_m = 0.02;
    c.thresholds.max_rmse_yaw_deg = 1.0;
    c.thresholds.min_match_rate = 0.95;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }
  {
    BenchmarkCase c;
    c.name = "smoke_noisy_cuda";
    c.kitti_root = smoke;
    c.max_frames = 50;
    c.use_cuda = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kNoisy;
    c.noise.pixel_std = 4.0;
    c.noise.point_dropout = 0.05;
    c.thresholds.max_rmse_translation_m = 0.5;
    c.thresholds.max_rmse_yaw_deg = 2.0;
    c.thresholds.min_match_rate = 0.8;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }
  {
    BenchmarkCase c;
    c.name = "kitti00_synth_cuda";
    c.kitti_root = kitti;
    c.max_frames = 80;
    c.start_frame = 10;
    c.use_cuda = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kAuto;
    c.thresholds.max_rmse_translation_m = 0.05;
    c.thresholds.max_rmse_yaw_deg = 1.0;
    c.thresholds.min_match_rate = 0.9;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }
  {
    BenchmarkCase c;
    c.name = "kitti00_real_cuda";
    c.kitti_root = kitti;
    c.perception_root = perception;
    c.max_frames = 80;
    c.start_frame = 10;
    c.use_cuda = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kFile;
    c.thresholds.max_rmse_translation_m = 0.1;
    c.thresholds.max_rmse_yaw_deg = 2.0;
    c.thresholds.min_match_rate = 0.85;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }
  {
    BenchmarkCase c;
    c.name = "kitti00_noisy_cuda";
    c.kitti_root = kitti;
    c.perception_root = perception;
    c.max_frames = 80;
    c.start_frame = 10;
    c.use_cuda = true;
    c.perception_source = cam_loc::perception::PerceptionSource::kNoisy;
    c.noise.pixel_std = 4.0;
    c.thresholds.max_rmse_translation_m = 0.15;
    c.thresholds.max_rmse_yaw_deg = 2.0;
    c.thresholds.min_match_rate = 0.8;
    c.thresholds.max_mean_frame_ms = 30000.0;
    cases.push_back(c);
  }

  return cases;
}

bool checkThresholds(const BenchmarkCase& spec, const cam_loc::kitti::SequenceEvalSummary& summary,
                     std::string& out_reason) {
  std::vector<std::string> fails;
  const auto& t = spec.thresholds;
  if (summary.pose.rmse_translation_m > t.max_rmse_translation_m) {
    fails.push_back("rmse_translation_m=" + std::to_string(summary.pose.rmse_translation_m) +
                    " > " + std::to_string(t.max_rmse_translation_m));
  }
  if (summary.pose.rmse_yaw_deg > t.max_rmse_yaw_deg) {
    fails.push_back("rmse_yaw_deg=" + std::to_string(summary.pose.rmse_yaw_deg) + " > " +
                    std::to_string(t.max_rmse_yaw_deg));
  }
  if (summary.matching.match_rate < t.min_match_rate) {
    fails.push_back("match_rate=" + std::to_string(summary.matching.match_rate) + " < " +
                    std::to_string(t.min_match_rate));
  }
  if (summary.matching.flat_rate > t.max_flat_rate) {
    fails.push_back("flat_rate=" + std::to_string(summary.matching.flat_rate) + " > " +
                    std::to_string(t.max_flat_rate));
  }
  if (summary.mean_frame_ms > t.max_mean_frame_ms) {
    fails.push_back("mean_frame_ms=" + std::to_string(summary.mean_frame_ms) + " > " +
                    std::to_string(t.max_mean_frame_ms));
  }
  if (fails.empty()) {
    out_reason.clear();
    return true;
  }
  out_reason = joinReasons(fails);
  return false;
}

Status runBenchmarkCase(const BenchmarkCase& spec, BenchmarkResult& out) {
  out = BenchmarkResult{};
  out.name = spec.name;

  // Load dataset → map → sequence eval → threshold check.
  const std::string poses_path =
      kitti::resolvePosesPath(spec.kitti_root, spec.sequence);
  std::vector<kitti::Pose> poses;
  if (kitti::loadPosesFile(poses_path, poses) != Status::kOk) {
    out.failure_reason = "missing poses: " + poses_path;
    return Status::kIoError;
  }

  kitti::Calibration calib;
  const std::string calib_path =
      kitti::resolveCalibPath(spec.kitti_root, spec.sequence);
  if (kitti::parseCalibrationFile(calib_path, calib) != Status::kOk) {
    out.failure_reason = "missing calib: " + calib_path;
    return Status::kIoError;
  }

  map::MapLoadOptions map_opt;
  map_opt.poses = &poses;
  std::shared_ptr<map::IMapLoader> map_loader;
  if (map::createMapLoader(map_opt, map_loader) != Status::kOk) {
    out.failure_reason = "map load failed";
    return Status::kInvalidArgument;
  }

  kitti::SequenceEvalConfig cfg;
  cfg.perception_root = spec.perception_root;
  cfg.perception_source = spec.perception_source;
  cfg.sequence = spec.sequence;
  cfg.start_frame = spec.start_frame;
  cfg.max_frames = spec.max_frames;
  cfg.noise = spec.noise;
  cfg.localization.use_cuda = spec.use_cuda;
  cfg.localization.use_gt_sampling_plane = spec.use_gt_plane;

  std::vector<kitti::FrameEvalRecord> records;
  if (kitti::runSequenceEval(poses, calib, map_loader, cfg, records) != Status::kOk) {
    out.failure_reason = "eval failed";
    return Status::kInvalidArgument;
  }

  out.summary = kitti::summarizeEval(records);
  out.num_frames = static_cast<int>(records.size());
  out.passed = checkThresholds(spec, out.summary, out.failure_reason);
  return Status::kOk;
}

Status runBenchmarkSuite(const std::vector<BenchmarkCase>& cases, BenchmarkSuiteResult& out) {
  out = BenchmarkSuiteResult{};
  for (const auto& spec : cases) {
    BenchmarkResult result;
    const Status st = runBenchmarkCase(spec, result);
    if (st == Status::kIoError) {
      result.passed = false;
      if (result.failure_reason.empty()) {
        result.failure_reason = "skipped (data not available)";
      }
      out.cases.push_back(result);
      ++out.failed;
      continue;
    }
    if (st != Status::kOk) {
      result.passed = false;
      out.cases.push_back(result);
      ++out.failed;
      continue;
    }
    out.cases.push_back(result);
    if (result.passed) {
      ++out.passed;
    } else {
      ++out.failed;
    }
  }
  return Status::kOk;
}

}  // namespace cam_loc::benchmark
