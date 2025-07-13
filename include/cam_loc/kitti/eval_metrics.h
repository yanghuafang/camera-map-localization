#ifndef CAM_LOC_KITTI_EVAL_METRICS_H_
#define CAM_LOC_KITTI_EVAL_METRICS_H_

/// Pose error metrics and rollups for sequence evaluation and benchmarks.

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

}  // namespace cam_loc::kitti

#endif  // CAM_LOC_KITTI_EVAL_METRICS_H_
