// Regression test for smoke benchmark case thresholds.
#include "cam_loc/benchmark/benchmark.h"

#include <filesystem>

#include <gtest/gtest.h>

#include "cam_loc/kitti/calib_parser.h"

TEST(BenchmarkTest, SmokeOracleCpuPasses) {
  auto cases = cam_loc::benchmark::DefaultBenchmarkSuite(CAMLOC_DATA_DIR);
  cam_loc::benchmark::BenchmarkCase* smoke = nullptr;
  for (auto& c : cases) {
    if (c.name == "smoke_oracle_cpu") {
      smoke = &c;
      break;
    }
  }
  ASSERT_NE(smoke, nullptr);
  smoke->max_frames = 10;  // Full suite uses 50; keep unit test fast.
  const std::string poses_path =
      cam_loc::kitti::ResolvePosesPath(smoke->kitti_root, smoke->sequence);
  if (!std::filesystem::exists(poses_path)) {
    GTEST_SKIP() << "Smoke KITTI not prepared at " << smoke->kitti_root;
  }

  cam_loc::benchmark::BenchmarkResult result;
  const auto st = cam_loc::benchmark::RunBenchmarkCase(*smoke, result);
  if (st == cam_loc::Status::kIoError) {
    GTEST_SKIP() << "Smoke data unavailable";
  }
  ASSERT_EQ(st, cam_loc::Status::kOk);
  EXPECT_TRUE(result.passed) << result.failure_reason;
  EXPECT_GT(result.num_frames, 0);
  EXPECT_LT(result.summary.pose.rmse_translation_m, 0.02);
}

TEST(BenchmarkTest, ThresholdCheckFailsWhenExceeded) {
  cam_loc::benchmark::BenchmarkCase spec;
  spec.thresholds.max_rmse_translation_m = 0.001;
  cam_loc::kitti::SequenceEvalSummary summary;
  summary.pose.rmse_translation_m = 0.01;
  summary.matching.match_rate = 1.0;
  std::string reason;
  EXPECT_FALSE(cam_loc::benchmark::CheckThresholds(spec, summary, reason));
  EXPECT_FALSE(reason.empty());
}
