#pragma once

/// Pose error metrics and rollups for sequence evaluation and benchmarks.

#include <cam_loc/types/status.hpp>

#include <cmath>
#include <vector>

namespace cam_loc::kitti {

struct TrajectoryError {
  double translation_m = 0.0;
  double yaw_deg = 0.0;
};

/// Translation + yaw error between estimate and ground-truth rigid transforms.
inline TrajectoryError poseError(const Mat44& estimate, const Mat44& ground_truth) {
  TrajectoryError err;
  err.translation_m = (estimate.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3)).norm();
  const double ye = yawFromRotation(estimate.block<3, 3>(0, 0));
  const double yg = yawFromRotation(ground_truth.block<3, 3>(0, 0));
  double d = std::abs(ye - yg) * 180.0 / M_PI;
  if (d > 180.0) d = 360.0 - d;
  err.yaw_deg = d;
  return err;
}

/// Mean / RMSE / max statistics over a trajectory error series.
struct ErrorSummary {
  double mean_translation_m = 0.0;
  double rmse_translation_m = 0.0;
  double max_translation_m = 0.0;
  double mean_yaw_deg = 0.0;
  double rmse_yaw_deg = 0.0;
  int num_frames = 0;
};

inline ErrorSummary summarizeErrors(const std::vector<TrajectoryError>& errors) {
  ErrorSummary s;
  s.num_frames = static_cast<int>(errors.size());
  if (errors.empty()) return s;

  double sum_t = 0.0;
  double sum_t2 = 0.0;
  double sum_y = 0.0;
  double sum_y2 = 0.0;
  for (const auto& e : errors) {
    sum_t += e.translation_m;
    sum_t2 += e.translation_m * e.translation_m;
    sum_y += e.yaw_deg;
    sum_y2 += e.yaw_deg * e.yaw_deg;
    s.max_translation_m = std::max(s.max_translation_m, e.translation_m);
  }
  const double n = static_cast<double>(errors.size());
  s.mean_translation_m = sum_t / n;
  s.rmse_translation_m = std::sqrt(sum_t2 / n);
  s.mean_yaw_deg = sum_y / n;
  s.rmse_yaw_deg = std::sqrt(sum_y2 / n);
  return s;
}

}  // namespace cam_loc::kitti
