#ifndef CAM_LOC_CORE_COST_QUADRATIC_H_
#define CAM_LOC_CORE_COST_QUADRATIC_H_

/// Local quadratic model of a cost volume about one cell.

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Gradient and Hessian of the cost at a cell, in offset units.
///
/// Two consumers need the same numbers: the sub-cell refinement, which steps to
/// the model's minimum, and the measurement covariance, which reads its
/// curvature. Fitting it once is what keeps the pose the filter is given and
/// the uncertainty it is given with it consistent.
struct LocalQuadratic {
  /// ∂c/∂(x, y, yaw), per metre and per radian.
  Vec3 gradient = Vec3::Zero();
  /// ∂²c/∂(x, y, yaw)², including the cross terms a per-axis fit cannot see.
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
  /// False on a border cell, whose neighbourhood is incomplete.
  bool valid = false;
};

/// Central differences of @p grid about @p cell.
LocalQuadratic FitLocalQuadratic(const CostGrid& grid,
                                 const CostGrid::ArgMinResult& cell);

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_COST_QUADRATIC_H_
