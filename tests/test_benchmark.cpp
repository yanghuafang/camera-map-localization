// Regression test for smoke benchmark case thresholds.
#include <cam_loc/benchmark/benchmark.hpp>
#include <cam_loc/kitti/calib_parser.hpp>

#include <gtest/gtest.h>

#include <filesystem>

TEST(BenchmarkTest, SmokeOracleCpuPasses) {
  const std::string root =
      std::filesystem::path(TEST_DATA_DIR).parent_path().parent_path().string();
  auto cases = cam_loc::benchmark::defaultBenchmarkSuite(root);
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
      cam_loc::kitti::resolvePosesPath(smoke->kitti_root, smoke->sequence);
  if (!std::filesystem::exists(poses_path)) {
    GTEST_SKIP() << "Smoke KITTI not prepared at " << smoke->kitti_root;
  }

  cam_loc::benchmark::BenchmarkResult result;
  const auto st = cam_loc::benchmark::runBenchmarkCase(*smoke, result);
  if (st == cam_loc::Status::kIoError) {
    GTEST_SKIP() << "Smoke data unavailable";
  }
  ASSERT_EQ(st, cam_loc::Status::kOk);
  EXPECT_TRUE(result.passed) << result.failure_reason;
  EXPECT_GT(result.num_frames, 0);
  EXPECT_LT(result.summary.pose.rmse_translation_m, 0.05);
}

TEST(BenchmarkTest, ThresholdCheckFailsWhenExceeded) {
  cam_loc::benchmark::BenchmarkCase spec;
  spec.thresholds.max_rmse_translation_m = 0.001;
  cam_loc::kitti::SequenceEvalSummary summary;
  summary.pose.rmse_translation_m = 0.01;
  summary.matching.match_rate = 1.0;
  std::string reason;
  EXPECT_FALSE(cam_loc::benchmark::checkThresholds(spec, summary, reason));
  EXPECT_FALSE(reason.empty());
}
