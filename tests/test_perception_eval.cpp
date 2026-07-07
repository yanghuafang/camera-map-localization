// Unit tests for perception resolve modes and eval metrics.
#include <cam_loc/perception/noise.hpp>
#include <cam_loc/perception/resolve.hpp>
#include <cam_loc/kitti/sequence_eval.hpp>
#include <cam_loc/map/trajectory_corridor_map.hpp>
#include <cam_loc/kitti/calib_parser.hpp>
#include <cam_loc/core/projection.hpp>

#include <gtest/gtest.h>

namespace {

cam_loc::kitti::FramePerception straightLane() {
  cam_loc::kitti::FramePerception p;
  p.frame = 0;
  cam_loc::kitti::Polyline2D pl;
  pl.type = cam_loc::kitti::PolylineType::kLaneSolid;
  for (int i = 0; i < 20; ++i) {
    pl.points.emplace_back(200.0 + i * 5.0, 180.0);
  }
  p.lane_lines.push_back(std::move(pl));
  return p;
}

}  // namespace

TEST(PerceptionNoiseTest, JittersVertices) {
  const auto base = straightLane();
  cam_loc::perception::PerceptionNoiseParams params;
  params.pixel_std = 3.0;
  const auto noisy = cam_loc::perception::addPerceptionNoise(base, params, 123);
  ASSERT_EQ(noisy.lane_lines.size(), 1u);
  ASSERT_EQ(noisy.lane_lines[0].points.size(), base.lane_lines[0].points.size());
  double sum_diff = 0.0;
  for (size_t i = 0; i < base.lane_lines[0].points.size(); ++i) {
    sum_diff += (noisy.lane_lines[0].points[i] - base.lane_lines[0].points[i]).norm();
  }
  EXPECT_GT(sum_diff, 0.0);
}

TEST(PerceptionNoiseTest, DropoutRemovesGeometry) {
  const auto base = straightLane();
  cam_loc::perception::PerceptionNoiseParams params;
  params.polyline_dropout = 1.0;
  const auto noisy = cam_loc::perception::addPerceptionNoise(base, params, 7);
  EXPECT_TRUE(noisy.lane_lines.empty());
}

TEST(SequenceEvalTest, SummarizeMatchingQuality) {
  std::vector<cam_loc::kitti::FrameEvalRecord> recs(4);
  for (auto& r : recs) {
    r.min_cost = 1.f;
    r.cost_spread = 0.5f;
    r.sampling_applied = true;
  }
  recs[3].sampling_applied = false;
  recs[3].cost_map_flat = true;
  const auto q = cam_loc::kitti::summarizeMatchingQuality(recs);
  EXPECT_NEAR(q.match_rate, 0.75, 1e-6);
  EXPECT_NEAR(q.flat_rate, 0.25, 1e-6);
}

TEST(ResolveTest, OracleSynthesizesFromCorridor) {
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 30; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i) * 0.5;
    poses.push_back(p);
  }
  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::parseCalibrationFile(TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  cam_loc::map::TrajectoryCorridorMap map;
  ASSERT_EQ(map.buildFromPoses(poses, 1.75, 1.0), cam_loc::Status::kOk);

  cam_loc::kitti::MapChunk chunk;
  ASSERT_EQ(map.queryLocalMap(poses[10].T_world_cam0, 50.0, chunk), cam_loc::Status::kOk);
  ASSERT_GE(chunk.polylines.size(), 1u);

  cam_loc::core::Projection proj(calib);
  cam_loc::kitti::FramePerception perception;
  cam_loc::perception::PerceptionResolveInfo info;
  const auto st = cam_loc::perception::resolvePerception(
      cam_loc::perception::PerceptionSource::kOracle, "", 0, 10, map, 50.0, proj,
      poses[10].T_world_cam0, {}, 1, perception, info);
  ASSERT_EQ(st, cam_loc::Status::kOk);
  EXPECT_TRUE(info.synthesized);
  EXPECT_GE(perception.lane_lines.size(), 1u);
}
