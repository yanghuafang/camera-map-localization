/// SE(3) error-state Kalman filter: predict from ego motion, update from map / global measurements.
#include <cam_loc/core/localization_kf.hpp>

#include <cam_loc/types/status.hpp>

namespace cam_loc::core {

namespace {

/// SO(3) logarithm: rotation matrix → angle-axis vector (axis * angle).
Eigen::Vector3d rotationToAngleAxis(const Eigen::Matrix3d& R) {
  Eigen::AngleAxisd aa(R);
  return aa.angle() * aa.axis();
}

/// SO(3) exponential: small angle-axis → incremental rotation matrix.
Eigen::Matrix3d angleAxisToRotation(const Eigen::Vector3d& omega) {
  const double angle = omega.norm();
  if (angle < 1e-12) {
    return Eigen::Matrix3d::Identity();
  }
  Eigen::AngleAxisd aa(angle, omega / angle);
  return aa.toRotationMatrix();
}

}  // namespace

Mat66 LocalizationKF::defaultProcessCov() {
  Mat66 Q = Mat66::Identity();
  // Process noise on translation (m²) and rotation (rad²) blocks.
  Q.block<3, 3>(0, 0) *= 0.01;
  Q.block<3, 3>(3, 3) *= 0.0001;
  return Q;
}

void LocalizationKF::initialize(const SE3State& state, const Mat66& cov) {
  state_ = state;
  cov_ = cov;
  initialized_ = true;
}

void LocalizationKF::predict(const Mat44& T_curr_prev, const Mat66& process_cov) {
  if (!initialized_) return;

  // Relative motion from odometry / VO: T_curr_prev = T_world_prev⁻¹ · T_world_curr.
  const Eigen::Matrix3d R_delta = T_curr_prev.block<3, 3>(0, 0);
  const Vec3 t_delta = T_curr_prev.block<3, 1>(0, 3);

  // Nominal-state propagation: compose incremental motion in the current body frame.
  state_.translation = state_.rotation * t_delta + state_.translation;
  state_.rotation = state_.rotation * R_delta;

  // Covariance propagation (v1 scaffold: P ← P + Q, no explicit state Jacobian).
  cov_ = cov_ + process_cov;
}

void LocalizationKF::update(const SE3State& measurement, const Mat66& meas_cov) {
  if (!initialized_) return;

  // Innovation (observation residual) in the 6-DOF error state.
  Eigen::VectorXd residual(6);

  // Position: measured minus predicted, both in world frame.
  residual.head<3>() = measurement.translation - state_.translation;

  // Orientation: R_err = R⁻¹ · R_meas, mapped to world-frame angle-axis residual.
  const Eigen::Matrix3d R_err = state_.rotation.transpose() * measurement.rotation;
  const Eigen::Vector3d omega_body = rotationToAngleAxis(R_err);
  residual.tail<3>() = state_.rotation * omega_body;

  // Standard Kalman update with identity observation model H = I.
  const Mat66 S = cov_ + meas_cov;
  const Mat66 K = cov_ * S.inverse();

  Eigen::VectorXd delta = K * residual;

  // Apply correction to nominal state.
  state_.translation += delta.head<3>();
  const Eigen::Matrix3d dR = angleAxisToRotation(delta.tail<3>());
  state_.rotation = state_.rotation * dR;

  // Joseph form omitted in v1; use (I − K) P for simplicity.
  const Mat66 I = Mat66::Identity();
  cov_ = (I - K) * cov_;
}

}  // namespace cam_loc::core
