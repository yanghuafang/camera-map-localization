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
    kf.Predict(step, kf.MotionProcessCov(step, cam_loc::OdometryNoiseParams{}));
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
    const Mat44 step = Frames::OffsetToCam0Transform(0.5, 0.0, 0.0);
    kf.Predict(step, kf.MotionProcessCov(step, cam_loc::OdometryNoiseParams{}));
    kf.Update(measurement, Mat66::Identity() * 1e-3);
  }

  const Mat66 P = kf.covariance();
  EXPECT_LT((P - P.transpose()).cwiseAbs().maxCoeff(), 1e-9)
      << "P not symmetric";
  EXPECT_GT(P.diagonal().minCoeff(), 0.0) << "P has a non-positive variance";
}

// Process noise has to be produced by motion. A fixed per-frame Q inflates the
// covariance through every red light, and the filter leaves the light less sure
// of a pose that never moved.
TEST(LocalizationKfTest, ProcessNoiseGrowsWithDistanceNotFrames) {
  LocalizationKF kf;
  kf.Initialize(StateAt(0.0, Vec3::Zero()), Mat66::Identity() * 1e-6);
  const cam_loc::OdometryNoiseParams params;

  const Mat44 stopped = Mat44::Identity();
  const Mat44 rolling = Frames::OffsetToCam0Transform(2.0, 0.0, 0.0);
  const double stopped_trace =
      kf.MotionProcessCov(stopped, params).block<3, 3>(0, 0).trace();
  const double rolling_trace =
      kf.MotionProcessCov(rolling, params).block<3, 3>(0, 0).trace();

  EXPECT_GT(rolling_trace, 100.0 * stopped_trace);
  EXPECT_GT(stopped_trace, 0.0) << "a zero floor makes P singular";
}

// Twice the step, four times the variance: sigma is linear in distance.
TEST(LocalizationKfTest, ProcessNoiseIsQuadraticInStepLength) {
  LocalizationKF kf;
  kf.Initialize(StateAt(0.0, Vec3::Zero()), Mat66::Identity() * 1e-6);
  cam_loc::OdometryNoiseParams params;
  params.stationary_translation_sigma_m = 0.0;
  params.stationary_rotation_sigma_deg = 0.0;

  const Mat44 one = Frames::OffsetToCam0Transform(1.0, 0.0, 0.0);
  const Mat44 two = Frames::OffsetToCam0Transform(2.0, 0.0, 0.0);
  const double q1 = kf.MotionProcessCov(one, params).block<3, 3>(0, 0).trace();
  const double q2 = kf.MotionProcessCov(two, params).block<3, 3>(0, 0).trace();
  EXPECT_NEAR(q2, 4.0 * q1, 1e-12);
}

// The along-track sigma is the big one, and it has to land on the axis the
// vehicle is actually pointing down -- not on world X because that is where it
// sat at initialization.
TEST(LocalizationKfTest, AlongTrackNoiseFollowsTheVehicleHeading) {
  cam_loc::OdometryNoiseParams params;
  params.stationary_translation_sigma_m = 0.0;
  const Mat44 step = Frames::OffsetToCam0Transform(3.0, 0.0, 0.0);

  LocalizationKF north;
  north.Initialize(StateAt(0.0, Vec3::Zero()), Mat66::Identity() * 1e-6);
  LocalizationKF turned;
  turned.Initialize(StateAt(M_PI / 2.0, Vec3::Zero()),
                    Mat66::Identity() * 1e-6);

  const Mat66 qn = north.MotionProcessCov(step, params);
  const Mat66 qt = turned.MotionProcessCov(step, params);

  // Same total uncertainty, pointing somewhere else. Both traces are bound to
  // names first: the comma in block<3, 3> would read as a macro separator.
  const double trace_north = qn.block<3, 3>(0, 0).trace();
  const double trace_turned = qt.block<3, 3>(0, 0).trace();
  EXPECT_NEAR(trace_north, trace_turned, 1e-12);

  const Vec3 forward_world = Frames::ToCam0(Vec3(1.0, 0.0, 0.0));
  const double along_north =
      forward_world.dot(qn.block<3, 3>(0, 0) * forward_world);
  const double along_turned =
      forward_world.dot(qt.block<3, 3>(0, 0) * forward_world);
  EXPECT_GT(along_north, 10.0 * along_turned);
}
