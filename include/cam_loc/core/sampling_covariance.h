#ifndef CAM_LOC_CORE_SAMPLING_COVARIANCE_H_
#define CAM_LOC_CORE_SAMPLING_COVARIANCE_H_

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Softmax-weighted spread of the cost surface around the argmin → R_map for
/// (x, y, yaw).
struct SamplingConfidence {
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
  bool valid = false;
};

class SamplingCovariance {
 public:
  static SamplingConfidence Compute(const CostGrid& grid,
                                    const CostGrid::ArgMinResult& argmin,
                                    float cost_scale = 1.f);
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_SAMPLING_COVARIANCE_H_
