#pragma once

#include <cam_loc/types/params.hpp>
#include <cam_loc/types/status.hpp>

#include <vector>

namespace cam_loc::core {

struct SE3State {
  Vec3 translation = Vec3::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

/// Error-state Kalman filter on SE(3) pose in the KITTI world frame.
///
/// State (nominal): translation t, rotation R (world from rig).
/// Error state (6-DOF): [dx, dy, dz, dωx, dωy, dωz] — translation in world,
/// rotation as a small angle-axis perturbation applied on the right (R ← R·Exp(dω)).
///
/// Per frame the engine calls:
///   1. predict(T_curr_prev, Q)  — motion model from relative odometry
///   2. update(z_map, R_map)     — map-matching observation (optional)
///   3. update(z_global, R_global) — optional GT / VO global prior
class LocalizationKF {
 public:
  void initialize(const SE3State& state, const Mat66& cov);
  bool isInitialized() const { return initialized_; }

  /// Prediction: propagate nominal pose with relative ego T_curr_prev; grow covariance by Q.
  void predict(const Mat44& T_curr_prev, const Mat66& process_cov);

  /// Update: fuse a full-pose measurement (map sample, GT, or VO) via linearized Kalman gain.
  void update(const SE3State& measurement, const Mat66& meas_cov);

  SE3State state() const { return state_; }
  Mat66 covariance() const { return cov_; }

  static Mat66 defaultProcessCov();

 private:
  SE3State state_;
  Mat66 cov_ = Mat66::Identity();
  bool initialized_ = false;
};

}  // namespace cam_loc::core
