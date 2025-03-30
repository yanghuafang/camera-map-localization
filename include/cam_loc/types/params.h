#ifndef CAM_LOC_TYPES_PARAMS_H_
#define CAM_LOC_TYPES_PARAMS_H_

/// Localization tuning parameters.
///
/// Grouped by pipeline stage: pose hypothesis grid → temporal aggregation.

#include "cam_loc/types/status.h"

namespace cam_loc {

/// 3-D pose hypothesis grid (x, y, yaw) searched each frame for map matching.
///
/// Cell counts should be odd so the zero offset is a real cell; an even count
/// truncates to a slightly asymmetric range. The defaults span ±5 m × ±7.5 m ×
/// ±3° about the anchor pose.
struct SamplingGridParams {
  int num_x = 21;             ///< Cells along the plane X axis.
  int num_y = 31;             ///< Cells along the plane Y axis.
  int num_yaw = 13;           ///< Cells in yaw.
  double step_x_m = 0.5;      ///< Cell pitch along X, m.
  double step_y_m = 0.5;      ///< Cell pitch along Y, m.
  double step_yaw_deg = 0.5;  ///< Cell pitch in yaw, degrees (radians inside).

  int TotalHypotheses() const { return num_x * num_y * num_yaw; }
};

/// Sliding-window cost aggregation over recent frames.
struct AggregationParams {
  /// Frames retained.
  ///
  /// A frame stops contributing once its plane has moved further than the grid
  /// half-extent -- 7.5 m at the default grid -- because beyond that its warped
  /// offset falls off the grid and SampleContinuous can only return a clamped
  /// border value. At KITTI speeds that is around six frames, so a much longer
  /// window costs memory and buys nothing: sweeping 1 to 70 frames moves
  /// translation RMSE by 5 mm and ANEES from 1.09 to 0.95, and nothing at all
  /// past 40. The window trades a little accuracy for a calmer covariance.
  int window_size = 12;
  /// Weight falls linearly as `1 − distance_decay · distance_m`, so history
  /// stops contributing after `1 / distance_decay` metres. See CostAggregator
  /// for which distance this is measured from.
  float distance_decay = 0.01f;
};

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_PARAMS_H_
