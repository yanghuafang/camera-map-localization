#ifndef CAM_LOC_CORE_SAMPLING_COVARIANCE_H_
#define CAM_LOC_CORE_SAMPLING_COVARIANCE_H_

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Map-matching measurement covariance for (forward, left, yaw), m² and rad².
struct SamplingConfidence {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  bool valid = false;
};

class SamplingCovariance {
 public:
  /// Covariance of the argmin, from the curvature of the cost basin around it.
  ///
  /// Near the minimum the cost is `c(δ) ≈ c_min + ½ δᵀHδ`. The pose is
  /// uncertain by however far δ can move before the cost rises by the noise in
  /// the cost itself, giving `cov ≈ 2σ_c H⁻¹`. The residual `c_min` is used as
  /// σ_c: a hypothesis that already misfits by two DT pixels cannot resolve
  /// pose to better than two pixels' worth of geometry.
  ///
  /// Adding `step²/12`, the variance of a value known only to lie in one cell,
  /// keeps the result honest about the grid it was measured on.
  ///
  /// @param cost_scale Temperature of the softmax fallback, used when the
  ///        argmin sits on a border or the Hessian is not positive definite.
  ///        The fallback returns a spread proportional to this constant, so it
  ///        is a tuning knob rather than a measurement -- which is why it is
  ///        only the fallback.
  static SamplingConfidence Compute(const CostGrid& grid,
                                    const CostGrid::ArgMinResult& argmin,
                                    float cost_scale = 1.f);
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_SAMPLING_COVARIANCE_H_
