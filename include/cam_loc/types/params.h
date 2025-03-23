#ifndef CAM_LOC_TYPES_PARAMS_H_
#define CAM_LOC_TYPES_PARAMS_H_

/// Localization tuning parameters.

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

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_PARAMS_H_
