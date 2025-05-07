// Odometry error injection: scale, heading drift, determinism and accumulation.

#include "cam_loc/kitti/egomotion_noise.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cam_loc/core/frames.h"

namespace {

using cam_loc::Mat44;
using cam_loc::Vec3;
using cam_loc::core::Frames;
using cam_loc::kitti::AddEgomotionNoise;
using cam_loc::kitti::Egomotion;
using cam_loc::kitti::EgomotionNoiseParams;

/// One metre straight ahead: cam0 Z is forward.
Egomotion StraightStep(int frame, double metres = 1.0) {
  Egomotion ego;
  ego.global.frame = frame;
  ego.T_curr_prev = Mat44::Identity();
  ego.T_curr_prev(2, 3) = metres;
  return ego;
}

double HeadingOf(const Mat44& T) {
  return cam_loc::YawFromRotation(T.block<3, 3>(0, 0));
}

TEST(EgomotionNoiseTest, DisabledParamsAreAPassThrough) {
  const Egomotion in = StraightStep(7);
  const Egomotion out = AddEgomotionNoise(in, EgomotionNoiseParams{}, 5);
  EXPECT_TRUE(out.T_curr_prev.isApprox(in.T_curr_prev));
}

TEST(EgomotionNoiseTest, ScaleErrorLengthensTheStep) {
  EgomotionNoiseParams p;
  p.scale_error = 0.02;
  const Egomotion out = AddEgomotionNoise(StraightStep(3, 4.0), p, 5);
  // Bound to a name first: the comma in block<3, 1> would otherwise be read as
  // a macro argument separator.
  const double travelled_m = out.T_curr_prev.block<3, 1>(0, 3).norm();
  EXPECT_NEAR(travelled_m, 4.08, 1e-9);
}

TEST(EgomotionNoiseTest, HeadingDriftScalesWithDistanceNotFrames) {
  // The whole reason the unit is degrees per metre: one 10 m step and ten 1 m
  // steps have to drift the same, and a per-frame sigma would not.
  EgomotionNoiseParams p;
  p.heading_bias_deg_per_m = 0.1;

  const Egomotion long_step = AddEgomotionNoise(StraightStep(1, 10.0), p, 5);
  double accumulated = 0.0;
  for (int f = 0; f < 10; ++f) {
    accumulated +=
        HeadingOf(AddEgomotionNoise(StraightStep(f, 1.0), p, 5).T_curr_prev);
  }
  EXPECT_NEAR(std::abs(HeadingOf(long_step.T_curr_prev)), 1.0 * M_PI / 180.0,
              1e-9);
  EXPECT_NEAR(std::abs(accumulated), 1.0 * M_PI / 180.0, 1e-9);
}

TEST(EgomotionNoiseTest, AStationaryVehicleAccumulatesNoError) {
  EgomotionNoiseParams p;
  p.scale_error = 0.05;
  p.heading_bias_deg_per_m = 0.5;
  p.heading_sigma_deg_per_m = 0.5;
  const Egomotion out = AddEgomotionNoise(StraightStep(4, 0.0), p, 5);
  EXPECT_TRUE(out.T_curr_prev.isApprox(Mat44::Identity()));
}

TEST(EgomotionNoiseTest, DriftIsAboutTheVehicleUpAxis) {
  // Heading, not roll about the optical axis -- the classic way a cam0-framed
  // perturbation goes wrong.
  EgomotionNoiseParams p;
  p.heading_bias_deg_per_m = 1.0;
  const Egomotion out = AddEgomotionNoise(StraightStep(2, 1.0), p, 5);

  const Vec3 axis =
      cam_loc::RotationToAngleAxis(out.T_curr_prev.block<3, 3>(0, 0));
  ASSERT_GT(axis.norm(), 1e-9);
  EXPECT_NEAR(std::abs(axis.normalized().dot(Frames::UpCam0())), 1.0, 1e-9);
}

TEST(EgomotionNoiseTest, SameSeedAndFrameGiveTheSamePerturbation) {
  EgomotionNoiseParams p;
  p.scale_sigma = 0.05;
  p.heading_sigma_deg_per_m = 0.3;
  const Egomotion a = AddEgomotionNoise(StraightStep(11), p, 42);
  const Egomotion b = AddEgomotionNoise(StraightStep(11), p, 42);
  EXPECT_TRUE(a.T_curr_prev.isApprox(b.T_curr_prev));
}

TEST(EgomotionNoiseTest, ConsecutiveFramesAreDecorrelated) {
  EgomotionNoiseParams p;
  p.scale_sigma = 0.05;
  const Egomotion a = AddEgomotionNoise(StraightStep(11), p, 42);
  const Egomotion b = AddEgomotionNoise(StraightStep(12), p, 42);
  EXPECT_FALSE(a.T_curr_prev.isApprox(b.T_curr_prev));
}

TEST(EgomotionNoiseTest, TheGlobalPoseStaysTruthful) {
  // Only the prediction input is corrupted. The filter still initializes from
  // a known start pose, which is a different assumption from perfect odometry.
  EgomotionNoiseParams p;
  p.scale_error = 0.1;
  Egomotion in = StraightStep(6);
  in.global.T_world_cam0(0, 3) = 123.0;
  const Egomotion out = AddEgomotionNoise(in, p, 5);
  EXPECT_TRUE(out.global.T_world_cam0.isApprox(in.global.T_world_cam0));
  EXPECT_TRUE(out.cov_global.isApprox(in.cov_global));
}

}  // namespace
