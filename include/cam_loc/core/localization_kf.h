#ifndef CAM_LOC_CORE_LOCALIZATION_KF_H_
#define CAM_LOC_CORE_LOCALIZATION_KF_H_

#include <vector>

#include "cam_loc/types/params.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

struct SE3State {
  Vec3 translation = Vec3::Zero();
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
};

/// Error-state Kalman filter on SE(3) pose in the KITTI world frame.
///
/// State (nominal): translation t, rotation R (world from rig).
///
/// Error state (6-DOF): `[dx, dy, dz, dωx, dωy, dωz]`, **both halves in the
/// world frame**. The rotation error is a small angle-axis perturbation applied
/// on the left, `R ← Exp(dω)·R`, which is what makes it a world-frame quantity
/// like the translation half. Getting these two out of step is the classic way
/// an error-state filter converges only near identity attitude, so the residual
/// in Update is formed the same way.
///
/// Per frame the engine calls:
///   1. Predict(T_curr_prev, Q)  — motion model from relative odometry
///   2. Update(z_map, R_map)     — map-matching observation (optional)
///   3. Update(z_global, R_global) — optional GT / VO global prior
class LocalizationKF {
 public:
  /// @param cov Initial 6×6 error-state covariance, ordered
  ///        `[x, y, z, ωx, ωy, ωz]` — the order every Mat66 below uses.
  void Initialize(const SE3State& state, const Mat66& cov);
  bool initialized() const { return initialized_; }

  /// Propagate the nominal pose by one relative motion.
  ///
  /// @param T_curr_prev Relative motion in the body frame,
  ///        `T_world_prev⁻¹ · T_world_curr`.
  /// @param process_cov Added as `P ← J·P·Jᵀ + Q`, where J is the error-state
  ///        Jacobian of this motion model.
  /// @note No-op before Initialize.
  void Predict(const Mat44& T_curr_prev, const Mat66& process_cov);

  /// Fuse a full-pose measurement (map sample, GT, or VO), observation model
  /// `H = I`.
  ///
  /// @param meas_cov Measurement covariance in the same world-frame error
  ///        basis as the state; a measurement expressed in vehicle axes has to
  ///        be rotated into it first.
  void Update(const SE3State& measurement, const Mat66& meas_cov);

  SE3State state() const { return state_; }
  Mat66 covariance() const { return cov_; }

  /// Process noise used by the engine each frame: 0.01 m² on translation,
  /// 1e-4 rad² on rotation.
  static Mat66 DefaultProcessCov();

 private:
  SE3State state_;
  Mat66 cov_ = Mat66::Identity();
  bool initialized_ = false;
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_LOCALIZATION_KF_H_
