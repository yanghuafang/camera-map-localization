// Unit tests for perception resolve modes and eval metrics.
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cam_loc/core/frames.h"
#include "cam_loc/core/projection.h"
#include "cam_loc/kitti/calib_parser.h"
#include "cam_loc/kitti/eval_metrics.h"
#include "cam_loc/kitti/sequence_eval.h"
#include "cam_loc/map/trajectory_corridor_map.h"
#include "cam_loc/perception/noise.h"
#include "cam_loc/perception/resolve.h"

namespace {

cam_loc::kitti::FramePerception StraightLane() {
  cam_loc::kitti::FramePerception p;
  p.frame = 0;
  cam_loc::kitti::Polyline2D pl;
  pl.type = cam_loc::kitti::PolylineType::kLaneSolid;
  for (int i = 0; i < 20; ++i) {
    pl.points.emplace_back(200.0 + i * 5.0, 180.0);
  }
  p.features.push_back(std::move(pl));
  return p;
}

}  // namespace

TEST(PerceptionNoiseTest, JittersVertices) {
  const auto base = StraightLane();
  cam_loc::perception::PerceptionNoiseParams params;
  params.pixel_std = 3.0;
  const auto noisy = cam_loc::perception::AddPerceptionNoise(base, params, 123);
  ASSERT_EQ(noisy.features.size(), 1u);
  ASSERT_EQ(noisy.features[0].points.size(), base.features[0].points.size());
  double sum_diff = 0.0;
  for (size_t i = 0; i < base.features[0].points.size(); ++i) {
    sum_diff +=
        (noisy.features[0].points[i] - base.features[0].points[i]).norm();
  }
  EXPECT_GT(sum_diff, 0.0);
}

TEST(PerceptionNoiseTest, DropoutRemovesGeometry) {
  const auto base = StraightLane();
  cam_loc::perception::PerceptionNoiseParams params;
  params.polyline_dropout = 1.0;
  const auto noisy = cam_loc::perception::AddPerceptionNoise(base, params, 7);
  EXPECT_TRUE(noisy.empty());
}

TEST(PoseErrorTest, SeparatesAlongTrackFromCrossTrack) {
  // GT at the origin with identity rotation: cam0 +Z is vehicle forward and
  // cam0 -X is vehicle left.
  const cam_loc::Mat44 gt = cam_loc::Mat44::Identity();

  cam_loc::Mat44 ahead = cam_loc::Mat44::Identity();
  ahead(2, 3) = 3.0;
  const auto e_ahead = cam_loc::kitti::PoseError(ahead, gt);
  EXPECT_NEAR(e_ahead.longitudinal_m, 3.0, 1e-9);
  EXPECT_NEAR(e_ahead.lateral_m, 0.0, 1e-9);
  EXPECT_NEAR(e_ahead.vertical_m, 0.0, 1e-9);

  cam_loc::Mat44 left = cam_loc::Mat44::Identity();
  left(0, 3) = -2.0;
  const auto e_left = cam_loc::kitti::PoseError(left, gt);
  EXPECT_NEAR(e_left.lateral_m, 2.0, 1e-9);
  EXPECT_NEAR(e_left.longitudinal_m, 0.0, 1e-9);
}

TEST(PoseErrorTest, AxesFollowVehicleHeading) {
  // The same world-frame offset, read at two GT headings 90 degrees apart. What
  // was along-track at the first is cross-track at the second; if the rotation
  // into the GT frame were dropped, both would report the same split.
  cam_loc::Mat44 estimate = cam_loc::Mat44::Identity();
  estimate(2, 3) = 4.0;

  const auto straight =
      cam_loc::kitti::PoseError(estimate, cam_loc::Mat44::Identity());
  EXPECT_NEAR(straight.longitudinal_m, 4.0, 1e-9);
  EXPECT_NEAR(straight.lateral_m, 0.0, 1e-9);

  // Yaw is a rotation about cam0 -Y (vehicle up). At +90 degrees the vehicle
  // faces world -X, so a world +Z offset is now off its right-hand side.
  cam_loc::Mat44 gt_turned = cam_loc::Mat44::Identity();
  gt_turned.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(M_PI / 2.0, cam_loc::core::Frames::UpCam0())
          .toRotationMatrix();
  const auto turned = cam_loc::kitti::PoseError(estimate, gt_turned);
  EXPECT_NEAR(turned.longitudinal_m, 0.0, 1e-9);
  EXPECT_NEAR(turned.lateral_m, -4.0, 1e-9);
}

TEST(PoseErrorTest, ComponentsReconstructTranslationNorm) {
  // The split is a rotation, so it loses nothing: at an arbitrary pose and
  // offset the three components still square up to the reported magnitude.
  cam_loc::Mat44 gt = cam_loc::Mat44::Identity();
  gt.block<3, 3>(0, 0) = Eigen::AngleAxisd(0.7, cam_loc::core::Frames::UpCam0())
                             .toRotationMatrix();
  gt.block<3, 1>(0, 3) = cam_loc::Vec3(12.0, -1.5, 30.0);

  cam_loc::Mat44 estimate = gt;
  estimate.block<3, 1>(0, 3) += cam_loc::Vec3(0.4, 0.2, -0.9);

  const auto e = cam_loc::kitti::PoseError(estimate, gt);
  const double sum_sq = e.lateral_m * e.lateral_m +
                        e.longitudinal_m * e.longitudinal_m +
                        e.vertical_m * e.vertical_m;
  EXPECT_NEAR(std::sqrt(sum_sq), e.translation_m, 1e-9);
}

TEST(SequenceEvalTest, BiasSurvivesWhereRmseDoesNot) {
  // The point of reporting a signed mean: a steady along-track lag and
  // symmetric along-track jitter of the same size are one number under RMSE.
  std::vector<cam_loc::kitti::TrajectoryError> lagging(8);
  for (auto& e : lagging) {
    e.longitudinal_m = -0.3;
    e.translation_m = 0.3;
  }
  std::vector<cam_loc::kitti::TrajectoryError> jittering(8);
  for (size_t i = 0; i < jittering.size(); ++i) {
    jittering[i].longitudinal_m = (i % 2 == 0) ? 0.3 : -0.3;
    jittering[i].translation_m = 0.3;
  }

  const auto lag = cam_loc::kitti::SummarizeErrors(lagging);
  const auto jit = cam_loc::kitti::SummarizeErrors(jittering);
  EXPECT_NEAR(lag.rmse_longitudinal_m, jit.rmse_longitudinal_m, 1e-12);
  EXPECT_NEAR(lag.bias_longitudinal_m, -0.3, 1e-12);
  EXPECT_NEAR(jit.bias_longitudinal_m, 0.0, 1e-12);
  EXPECT_NEAR(lag.max_abs_longitudinal_m, 0.3, 1e-12);
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
  const auto q = cam_loc::kitti::SummarizeMatchingQuality(recs);
  EXPECT_NEAR(q.match_rate, 0.75, 1e-6);
  EXPECT_NEAR(q.flat_rate, 0.25, 1e-6);
}

TEST(ResolveTest, OracleSynthesizesFromCorridor) {
  // Long enough for the corridor builder to place upright landmarks, which it
  // does every pole_spacing_m along the route.
  std::vector<cam_loc::kitti::Pose> poses;
  for (int i = 0; i < 160; ++i) {
    cam_loc::kitti::Pose p;
    p.frame = i;
    p.T_world_cam0 = cam_loc::Mat44::Identity();
    p.T_world_cam0(2, 3) = static_cast<double>(i) * 0.5;
    poses.push_back(p);
  }
  cam_loc::kitti::Calibration calib;
  ASSERT_EQ(cam_loc::kitti::ParseCalibrationFile(
                TEST_DATA_DIR "/calib_minimal.txt", calib),
            cam_loc::Status::kOk);

  cam_loc::map::CorridorMapOptions corridor_opts;
  corridor_opts.sample_step_m = 1.0;
  cam_loc::map::TrajectoryCorridorMap map;
  ASSERT_EQ(map.BuildFromPoses(poses, corridor_opts), cam_loc::Status::kOk);

  cam_loc::kitti::MapChunk chunk;
  ASSERT_EQ(map.QueryLocalMap(poses[10].T_world_cam0, 50.0, chunk),
            cam_loc::Status::kOk);
  ASSERT_GE(chunk.polylines.size(), 1u);

  cam_loc::core::Projection proj(calib);
  cam_loc::kitti::FramePerception perception;
  cam_loc::perception::PerceptionResolveInfo info;
  const auto st = cam_loc::perception::ResolvePerception(
      cam_loc::perception::PerceptionSource::kOracle, "", 0, 10, map, 50.0,
      proj, poses[10].T_world_cam0, {}, 1, perception, info);
  ASSERT_EQ(st, cam_loc::Status::kOk);
  EXPECT_TRUE(info.synthesized);
  EXPECT_FALSE(perception.empty());
  // The corridor map carries upright landmarks, so oracle perception has to
  // surface them too -- they are what makes along-track position observable.
  EXPECT_GE(perception.CountOf(cam_loc::kitti::PolylineType::kPole), 1);
}
