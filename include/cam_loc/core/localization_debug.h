#ifndef CAM_LOC_CORE_LOCALIZATION_DEBUG_H_
#define CAM_LOC_CORE_LOCALIZATION_DEBUG_H_

#include "cam_loc/core/cost_grid.h"
#include "cam_loc/core/pose_sampler.h"
#include "cam_loc/kitti/types.h"
#include "cam_loc/types/status.h"

namespace cam_loc::core {

/// Intermediate map-matching state captured for visualization (one frame).
/// Holds raw vs aggregated cost grids, DT images, and the argmin hypothesis.
struct LocalizationDebugSnapshot {
  bool valid = false;
  Mat44 T_world_plane = Mat44::Identity();
  kitti::MapChunk local_map;
  kitti::FramePerception perception;
  LabelledDistanceTransform image_dt;
  LabelledDistanceTransform bev_dt;
  bool has_bev_dt = false;
  CostGrid raw_costs{SamplingGridParams{}};
  CostGrid aggregated_costs{SamplingGridParams{}};
  CostGrid::ArgMinResult argmin;
};

}  // namespace cam_loc::core

#endif  // CAM_LOC_CORE_LOCALIZATION_DEBUG_H_
