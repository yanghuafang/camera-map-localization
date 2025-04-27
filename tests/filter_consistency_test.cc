// Filter-consistency metrics: NEES per frame and its rollup.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cam_loc/kitti/eval_metrics.h"

namespace {

using cam_loc::Mat44;
using cam_loc::Mat66;
using cam_loc::Vec3;
using cam_loc::kitti::ConsistencySummary;
using cam_loc::kitti::FrameConsistency;
using cam_loc::kitti::kChiSquare6Dof95;
using cam_loc::kitti::kPoseErrorDof;
using cam_loc::kitti::PoseNees;
using cam_loc::kitti::SummarizeConsistency;

Mat44 PoseAt(const Vec3& t,
             const Eigen::Matrix3d& R = Eigen::Matrix3d::Identity()) {
  Mat44 T = Mat44::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;
  return T;
}

TEST(FilterConsistencyTest, ExactEstimateScoresZero) {
  const Mat44 T = PoseAt(Vec3(5.0, 0.0, 12.0));
  const auto c = PoseNees(T, T, Mat66::Identity());
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.nees, 0.0, 1e-12);
}

TEST(FilterConsistencyTest, TranslationErrorIsNormalizedByItsVariance) {
  // A 0.3 m error against a 0.3 m sigma is exactly one sigma: NEES 1.
  Mat66 P = Mat66::Identity();
  P(0, 0) = 0.09;
  const auto c = PoseNees(PoseAt(Vec3(0.3, 0.0, 0.0)), PoseAt(Vec3::Zero()), P);
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.nees, 1.0, 1e-9);
}

TEST(FilterConsistencyTest, HalvingSigmaQuadruplesNees) {
  // The over-confidence this metric exists to catch: the same error against a
  // covariance shrunk fourfold reports four times the NEES.
  const Mat44 est = PoseAt(Vec3(0.2, -0.1, 0.4));
  const Mat44 gt = PoseAt(Vec3::Zero());
  const auto honest = PoseNees(est, gt, Mat66::Identity());
  const auto over_confident = PoseNees(est, gt, Mat66::Identity() * 0.25);
  ASSERT_TRUE(honest.valid);
  ASSERT_TRUE(over_confident.valid);
  EXPECT_NEAR(over_confident.nees, 4.0 * honest.nees, 1e-9);
}

TEST(FilterConsistencyTest, RotationHalfUsesTheFilterResidualConvention) {
  // e_rot = Log(R_est · R_gtᵀ), so a 2° error about any axis contributes
  // (2° in rad)² under unit variance -- and the answer must not depend on the
  // attitude the two poses share.
  const double angle = 2.0 * M_PI / 180.0;
  const Vec3 axis = Vec3(0.0, -1.0, 0.0);
  const Eigen::Matrix3d R_shared =
      Eigen::AngleAxisd(0.7, Vec3(1.0, 2.0, 3.0).normalized())
          .toRotationMatrix();
  const Eigen::Matrix3d R_err =
      Eigen::AngleAxisd(angle, axis).toRotationMatrix();

  const auto c = PoseNees(PoseAt(Vec3::Zero(), R_err * R_shared),
                          PoseAt(Vec3::Zero(), R_shared), Mat66::Identity());
  ASSERT_TRUE(c.valid);
  EXPECT_NEAR(c.nees, angle * angle, 1e-9);
}

TEST(FilterConsistencyTest, SingularCovarianceCarriesNoVerdict) {
  Mat66 P = Mat66::Identity();
  P(2, 2) = 0.0;
  const auto c = PoseNees(PoseAt(Vec3(0.0, 0.0, 0.5)), PoseAt(Vec3::Zero()), P);
  EXPECT_FALSE(c.valid);
  EXPECT_EQ(c.nees, 0.0);
}

TEST(FilterConsistencyTest, SummaryIgnoresFramesWithoutAVerdict) {
  std::vector<FrameConsistency> samples;
  samples.push_back({kPoseErrorDof, true});
  samples.push_back({3 * kPoseErrorDof, true});
  samples.push_back({1e6, false});  // must not drag the mean anywhere

  const ConsistencySummary s = SummarizeConsistency(samples);
  EXPECT_EQ(s.num_frames, 2);
  EXPECT_NEAR(s.mean_nees, 2.0 * kPoseErrorDof, 1e-9);
  EXPECT_NEAR(s.anees, 2.0, 1e-9);
}

TEST(FilterConsistencyTest, ExceedanceRateSeparatesUniformFromOccasionalError) {
  // Two sequences with the same mean NEES: one uniformly over-confident, one
  // mostly fine with a single divergent frame. ANEES cannot tell them apart
  // and the exceedance rate can, which is why both are reported.
  std::vector<FrameConsistency> uniform(4, {12.0, true});
  std::vector<FrameConsistency> spiky(3, {0.0, true});
  spiky.push_back({48.0, true});

  const ConsistencySummary a = SummarizeConsistency(uniform);
  const ConsistencySummary b = SummarizeConsistency(spiky);
  EXPECT_NEAR(a.mean_nees, b.mean_nees, 1e-9);
  EXPECT_NEAR(a.within_95_rate, 1.0, 1e-9);
  EXPECT_NEAR(b.within_95_rate, 0.75, 1e-9);
  EXPECT_GT(48.0, kChiSquare6Dof95);
}

TEST(FilterConsistencyTest, EmptyInputIsNotAConsistentFilter) {
  const ConsistencySummary s = SummarizeConsistency({});
  EXPECT_EQ(s.num_frames, 0);
  EXPECT_EQ(s.anees, 0.0);
  EXPECT_EQ(s.within_95_rate, 0.0);
}

}  // namespace
