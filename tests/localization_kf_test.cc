// The filter's rotation handling, which is only exercised away from identity.
#include "cam_loc/core/localization_kf.h"

#include <gtest/gtest.h>

#include "cam_loc/core/frames.h"

namespace {

using cam_loc::Mat44;
using cam_loc::Mat66;
using cam_loc::Vec3;
using cam_loc::core::Frames;
using cam_loc::core::LocalizationKF;
using cam_loc::core::SE3State;

SE3State StateAt(double yaw_rad, const Vec3& t) {
  const Mat44 T = Frames::OffsetToCam0Transform(0.0, 0.0, yaw_rad);
  SE3State s;
  s.rotation = T.block<3, 3>(0, 0);
  s.translation = t;
  return s;
}

double HeadingOf(const SE3State& s) {
  return Frames::HeadingFromCam0Rotation(s.rotation);
}

}  // namespace

// The residual used to be formed in the world frame while the correction was
// applied on the right, so the update only converged where the two frames
// coincide -- near identity attitude. Starting the filter already turned makes
// that visible.
TEST(LocalizationKfTest, ConvergesToAMeasurementAwayFromIdentity) {
  LocalizationKF kf;
  kf.Initialize(StateAt(1.0, Vec3(5.0, 0.0, 3.0)), Mat66::Identity());

  const SE3State measurement = StateAt(1.2, Vec3(5.5, 0.0, 3.4));
  const Mat66 meas_cov = Mat66::Identity() * 1e-4;
  for (int i = 0; i < 30; ++i) {
    kf.Update(measurement, meas_cov);
  }

  EXPECT_NEAR(HeadingOf(kf.state()), HeadingOf(measurement), 1e-3);
  EXPECT_LT((kf.state().translation - measurement.translation).norm(), 1e-3);
}

// Predict must leave the estimate on the path the increments describe, whatever
// the attitude: composing a body-frame step onto a rotated pose is where a
// left/right mix-up shows up as drift.
TEST(LocalizationKfTest, PredictFollowsBodyFrameMotion) {
  LocalizationKF kf;
  kf.Initialize(StateAt(0.0, Vec3::Zero()), Mat66::Identity() * 1e-6);

  Mat44 truth = Mat44::Identity();
  const Mat44 step = Frames::OffsetToCam0Transform(1.0, 0.0, 0.05);
  for (int i = 0; i < 50; ++i) {
    truth = truth * step;
    kf.Predict(step, LocalizationKF::DefaultProcessCov());
  }

  const Vec3 truth_t = truth.block<3, 1>(0, 3);
  EXPECT_LT((kf.state().translation - truth_t).norm(), 1e-9);
  const Eigen::Matrix3d truth_r = truth.block<3, 3>(0, 0);
  EXPECT_NEAR(Frames::HeadingFromCam0Rotation(kf.state().rotation),
              Frames::HeadingFromCam0Rotation(truth_r), 1e-9);
}

// Covariance must stay symmetric and positive under repeated updates; the
// Joseph form is what guarantees it.
TEST(LocalizationKfTest, CovarianceStaysWellFormed) {
  LocalizationKF kf;
  kf.Initialize(StateAt(0.3, Vec3(1.0, 0.0, 2.0)), Mat66::Identity());

  const SE3State measurement = StateAt(0.3, Vec3(1.0, 0.0, 2.0));
  for (int i = 0; i < 100; ++i) {
    kf.Predict(Frames::OffsetToCam0Transform(0.5, 0.0, 0.0),
               LocalizationKF::DefaultProcessCov());
    kf.Update(measurement, Mat66::Identity() * 1e-3);
  }

  const Mat66 P = kf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-9)
      << "P not symmetric";
  EXPECT_GT(P.diagonal().minCoeff(), 0.0) << "P has a non-positive variance";
}
