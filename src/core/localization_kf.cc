/// SE(3) error-state Kalman filter: predict from ego motion, update from map /
/// global measurements.
#include "cam_loc/core/localization_kf.h"

#include "cam_loc/types/status.h"

namespace cam_loc::core {

namespace {

/// SO(3) logarithm: rotation matrix → angle-axis vector (axis * angle).
Eigen::Vector3d RotationToAngleAxis(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

/// Skew-symmetric matrix of a vector, so that Skew(a)·b == a × b.
Eigen::Matrix3d Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d m;
  // clang-format off
  m <<     0.0, -v.z(),  v.y(),
        v.z(),     0.0, -v.x(),
       -v.y(),  v.x(),     0.0;
  // clang-format on
  return m;
}

/// SO(3) exponential: small angle-axis → incremental rotation matrix.
Eigen::Matrix3d AngleAxisToRotation(const Eigen::Vector3d& omega) {
  const double angle = omega.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::AngleAxisd aa(angle, omega / angle);
  return aa.toRotationMatrix();
}

}  // namespace

Mat66 LocalizationKF::DefaultProcessCov() {
  Mat66 Q = Mat66::Identity();
  // Process noise on translation (m²) and rotation (rad²) blocks.
  Q.block<3, 3>(0, 0) *= 0.01;
  Q.block<3, 3>(3, 3) *= 0.0001;
  return Q;
}

void LocalizationKF::Initialize(const SE3State& state, const Mat66& cov) {
  state_ = state;
  cov_ = cov;
  initialized_ = true;
}

void LocalizationKF::Predict(const Mat44& T_curr_prev,
                             const Mat66& process_cov) {
  if (!initialized_) return;

  // Relative motion from odometry / VO: T_curr_prev = T_world_prev⁻¹ ·
  // T_world_curr.
  const Eigen::Matrix3d R_delta = T_curr_prev.block<3, 3>(0, 0);
  const Vec3 t_delta = T_curr_prev.block<3, 1>(0, 3);

  // The step, rotated into the world frame. Taken before the nominal state
  // moves, because the error Jacobian below is linearized about the old pose.
  const Vec3 t_world_delta = state_.rotation * t_delta;

  // Nominal-state propagation: compose incremental motion in the current body
  // frame.
  state_.translation = t_world_delta + state_.translation;
  state_.rotation = state_.rotation * R_delta;

  // Error-state Jacobian for a world-frame error and a body-frame increment.
  //
  // Rotation: R_true = Exp(δ)·R, and the increment multiplies on the right, so
  // Exp(δ)·R·ΔR leaves δ untouched — the rotation block is identity, not
  // R_delta.
  //
  // Translation: t_true = t + δt and the step is carried by the *true*
  // attitude, so the attitude error tilts it. To first order
  // δt⁺ = δt − [R·t_delta]ₓ·δ, which is the off-diagonal block below. Dropping
  // it (the old P ← P + Q) let heading uncertainty accumulate without ever
  // feeding the position uncertainty it actually causes.
  Mat66 jacobian = Mat66::Identity();
  jacobian.block<3, 3>(0, 3) = -Skew(t_world_delta);
  cov_ = jacobian * cov_ * jacobian.transpose() + process_cov;
}

void LocalizationKF::Update(const SE3State& measurement,
                            const Mat66& meas_cov) {
  if (!initialized_) return;

  // Innovation (observation residual) in the 6-DOF error state.
  Eigen::VectorXd residual(6);

  // Position: measured minus predicted, both in world frame.
  residual.head<3>() = measurement.translation - state_.translation;

  // Orientation, in the world frame to match the translation half and the
  // covariance: R_meas = Exp(dω) · R, so dω = Log(R_meas · Rᵀ).
  const Eigen::Matrix3d R_err =
      measurement.rotation * state_.rotation.transpose();
  residual.tail<3>() = RotationToAngleAxis(R_err);

  // Standard Kalman update with identity observation model H = I.
  const Mat66 S = cov_ + meas_cov;
  const Mat66 K = cov_ * S.inverse();

  Eigen::VectorXd delta = K * residual;

  // Apply correction to nominal state. The rotation correction goes on the
  // left, because that is the side the residual above was formed on.
  state_.translation += delta.head<3>();
  const Eigen::Matrix3d dR = AngleAxisToRotation(delta.tail<3>());
  state_.rotation = dR * state_.rotation;

  // Joseph form: (I−K)P(I−K)ᵀ + KRKᵀ stays symmetric positive-semidefinite
  // under finite precision, where the shorter (I−K)P does not. The cost is one
  // extra 6x6 product per update, which is nothing next to the pose grid.
  const Mat66 I = Mat66::Identity();
  const Mat66 IKH = I - K;
  cov_ = IKH * cov_ * IKH.transpose() + K * meas_cov * K.transpose();
}

}  // namespace cam_loc::core
