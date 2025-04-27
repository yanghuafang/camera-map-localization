#ifndef CAM_LOC_KITTI_EVAL_METRICS_H_
#define CAM_LOC_KITTI_EVAL_METRICS_H_

/// Pose error metrics, filter-consistency metrics, and rollups for sequence
/// evaluation and benchmarks.

#include <algorithm>
#include <cmath>
#include <vector>

#include "cam_loc/types/status.h"

namespace cam_loc::kitti {

/// Pose error for one frame: translation as a magnitude and resolved onto the
/// vehicle axes, plus heading.
///
/// The components are what make the magnitude diagnostic. The pose grid
/// searches forward, left and yaw, and those axes are not equally observable:
/// lane geometry runs parallel to travel, so it pins lateral offset well and
/// along-track position barely at all. A 3-D norm reports a 0.3 m along-track
/// lag and 0.3 m of lateral wander as the same number, which is exactly the
/// distinction worth seeing.
///
/// The components are signed, unlike the two magnitudes. The failure mode they
/// exist to expose is a *bias*, and taking absolute values first is what hides
/// one.
struct TrajectoryError {
  /// ‖t_est − t_gt‖, metres. Unsigned.
  double translation_m = 0.0;
  /// Heading error, degrees in [0, 180]. Unsigned.
  double yaw_deg = 0.0;
  /// Along-track error, metres. Positive when the estimate is ahead of truth.
  double longitudinal_m = 0.0;
  /// Cross-track error, metres. Positive when the estimate is left of truth.
  double lateral_m = 0.0;
  /// Height error, metres. Positive when the estimate is above truth. No
  /// measurement constrains this axis; it is carried so that the three
  /// components reconstruct translation_m exactly.
  double vertical_m = 0.0;
};

/// Translation + yaw error between estimate and ground-truth rigid transforms.
///
/// The translation error is resolved onto the vehicle axes of @p ground_truth,
/// not of @p estimate: the frame an error is reported in must not move with the
/// error being reported, or a heading mistake rotates its own yardstick.
inline TrajectoryError PoseError(const Mat44& estimate,
                                 const Mat44& ground_truth) {
  TrajectoryError err;
  const Vec3 d_world =
      estimate.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3);
  err.translation_m = d_world.norm();

  const Eigen::Matrix3d R_gt = ground_truth.block<3, 3>(0, 0);
  const Vec3 d_vehicle = ToVehicleAxes(R_gt.transpose() * d_world);
  err.longitudinal_m = d_vehicle.x();
  err.lateral_m = d_vehicle.y();
  err.vertical_m = d_vehicle.z();

  const double ye = YawFromRotation(estimate.block<3, 3>(0, 0));
  const double yg = YawFromRotation(ground_truth.block<3, 3>(0, 0));
  double d = std::abs(ye - yg) * 180.0 / M_PI;
  if (d > 180.0) d = 360.0 - d;
  err.yaw_deg = d;
  return err;
}

/// Mean / RMSE / max statistics over a trajectory error series.
///
/// Each vehicle axis carries both an RMSE and a signed mean, because they
/// answer different questions: the RMSE is how far off, the signed mean is
/// whether it is off in one direction. A steady 0.3 m along-track lag and 0.3 m
/// of symmetric along-track jitter have the same RMSE, and only the bias tells
/// them apart.
struct ErrorSummary {
  double mean_translation_m = 0.0;
  double rmse_translation_m = 0.0;
  double max_translation_m = 0.0;
  double mean_yaw_deg = 0.0;
  double rmse_yaw_deg = 0.0;
  /// Per-axis RMSE, metres. These reconstruct the translation RMSE exactly:
  /// `rmse_translation_m² = rmse_longitudinal_m² + rmse_lateral_m² +
  /// rmse_vertical_m²`.
  double rmse_longitudinal_m = 0.0;
  double rmse_lateral_m = 0.0;
  double rmse_vertical_m = 0.0;
  /// Signed means, metres; sign convention as in TrajectoryError.
  double bias_longitudinal_m = 0.0;
  double bias_lateral_m = 0.0;
  /// Largest excursion on each axis in either direction, metres.
  double max_abs_longitudinal_m = 0.0;
  double max_abs_lateral_m = 0.0;
  int num_frames = 0;
};

inline ErrorSummary SummarizeErrors(
    const std::vector<TrajectoryError>& errors) {
  ErrorSummary s;
  s.num_frames = static_cast<int>(errors.size());
  if (errors.empty()) return s;

  double sum_t = 0.0;
  double sum_t2 = 0.0;
  double sum_y = 0.0;
  double sum_y2 = 0.0;
  double sum_lon = 0.0;
  double sum_lon2 = 0.0;
  double sum_lat = 0.0;
  double sum_lat2 = 0.0;
  double sum_vert2 = 0.0;
  for (const auto& e : errors) {
    sum_t += e.translation_m;
    sum_t2 += e.translation_m * e.translation_m;
    sum_y += e.yaw_deg;
    sum_y2 += e.yaw_deg * e.yaw_deg;
    sum_lon += e.longitudinal_m;
    sum_lon2 += e.longitudinal_m * e.longitudinal_m;
    sum_lat += e.lateral_m;
    sum_lat2 += e.lateral_m * e.lateral_m;
    sum_vert2 += e.vertical_m * e.vertical_m;
    s.max_translation_m = std::max(s.max_translation_m, e.translation_m);
    s.max_abs_longitudinal_m =
        std::max(s.max_abs_longitudinal_m, std::abs(e.longitudinal_m));
    s.max_abs_lateral_m = std::max(s.max_abs_lateral_m, std::abs(e.lateral_m));
  }
  const auto n = static_cast<double>(errors.size());
  s.mean_translation_m = sum_t / n;
  s.rmse_translation_m = std::sqrt(sum_t2 / n);
  s.mean_yaw_deg = sum_y / n;
  s.rmse_yaw_deg = std::sqrt(sum_y2 / n);
  s.rmse_longitudinal_m = std::sqrt(sum_lon2 / n);
  s.rmse_lateral_m = std::sqrt(sum_lat2 / n);
  s.rmse_vertical_m = std::sqrt(sum_vert2 / n);
  s.bias_longitudinal_m = sum_lon / n;
  s.bias_lateral_m = sum_lat / n;
  return s;
}

/// 95th-percentile point of the chi-square distribution with six degrees of
/// freedom. A consistent filter's NEES exceeds this on 5% of frames.
inline constexpr double kChiSquare6Dof95 = 12.5916;

/// Degrees of freedom in the error state, and so in the NEES below.
inline constexpr int kPoseErrorDof = 6;

/// Normalized estimation error squared for one frame: `eᵀ P⁻¹ e`.
///
/// This is the question RMSE cannot answer. RMSE says how far the estimate is
/// from truth; NEES says whether the filter *knew* it would be that far. A
/// filter that is 10 cm off and reports a 10 cm sigma is doing its job; one
/// that is 10 cm off while reporting 1 cm is lying to everything downstream,
/// and the trajectory error alone never shows it.
struct FrameConsistency {
  /// `eᵀ P⁻¹ e`. Its expected value is kPoseErrorDof when the filter is
  /// consistent, larger when the covariance is too small for the error it
  /// actually makes.
  double nees = 0.0;
  /// False when the covariance could not be inverted, or when the filter had
  /// not initialized. Such a frame carries no verdict and is left out of the
  /// rollup rather than counted as consistent.
  bool valid = false;
};

/// NEES of one pose estimate against truth, under the filter's own covariance.
///
/// The error vector is formed exactly as LocalizationKF::Update forms its
/// residual — translation in the world frame, rotation as
/// `Log(R_est · R_gtᵀ)`, both halves world-frame — because a NEES computed in
/// a different basis than the covariance it divides by is a number with no
/// meaning.
///
/// @param covariance Error-state covariance ordered `[x, y, z, ωx, ωy, ωz]`.
inline FrameConsistency PoseNees(const Mat44& estimate,
                                 const Mat44& ground_truth,
                                 const Mat66& covariance) {
  FrameConsistency out;

  Eigen::Matrix<double, 6, 1> e;
  e.head<3>() = estimate.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3);
  const Eigen::Matrix3d R_err =
      estimate.block<3, 3>(0, 0) * ground_truth.block<3, 3>(0, 0).transpose();
  e.tail<3>() = RotationToAngleAxis(R_err);

  // LDLᵀ rather than inverse(): the covariance is symmetric positive definite
  // by construction, and a solve on a near-singular P is the case worth
  // detecting rather than propagating as a huge finite number.
  const Eigen::LDLT<Mat66> ldlt(covariance);
  if (ldlt.info() != Eigen::Success) return out;

  // isPositive() accepts a semi-definite matrix, so it is not the test wanted
  // here: a P with a zero eigenvalue claims one axis is known exactly, and any
  // error along that axis divides by zero. Inspecting D directly is what
  // distinguishes definite from semi-definite. The tolerance is relative,
  // because these variances span m² and rad².
  const auto d = ldlt.vectorD();
  const double d_max = d.maxCoeff();
  if (!(d_max > 0.0)) return out;
  if (d.minCoeff() <= 1e-12 * d_max) return out;

  const Eigen::Matrix<double, 6, 1> solved = ldlt.solve(e);
  if (!solved.allFinite()) return out;

  out.nees = e.dot(solved);
  out.valid = true;
  return out;
}

/// Filter-consistency rollup over a sequence.
///
/// Read `anees` first: 1.0 is a filter whose covariance matches its error,
/// above 1.0 is over-confident, below is conservative. The two numbers beside
/// it say whether the average is representative — a mean dragged up by a
/// handful of divergent frames and a filter that is uniformly too sure of
/// itself both give anees > 1, and only the exceedance rate tells them apart.
struct ConsistencySummary {
  /// Mean NEES over valid frames.
  double mean_nees = 0.0;
  /// Mean NEES divided by kPoseErrorDof: 1.0 when the covariance is honest.
  double anees = 0.0;
  /// Fraction of valid frames whose NEES stays under kChiSquare6Dof95. A
  /// consistent filter sits near 0.95.
  double within_95_rate = 0.0;
  /// Frames that carried a verdict. Fewer than the frames evaluated when the
  /// covariance was not invertible on some of them.
  int num_frames = 0;
};

inline ConsistencySummary SummarizeConsistency(
    const std::vector<FrameConsistency>& samples) {
  ConsistencySummary s;
  double sum = 0.0;
  int within = 0;
  for (const auto& c : samples) {
    if (!c.valid) continue;
    ++s.num_frames;
    sum += c.nees;
    if (c.nees <= kChiSquare6Dof95) ++within;
  }
  if (s.num_frames == 0) return s;

  const auto n = static_cast<double>(s.num_frames);
  s.mean_nees = sum / n;
  s.anees = s.mean_nees / static_cast<double>(kPoseErrorDof);
  s.within_95_rate = static_cast<double>(within) / n;
  return s;
}

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_EVAL_METRICS_H_
