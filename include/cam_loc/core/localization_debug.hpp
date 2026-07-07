#pragma once

#include <cam_loc/core/cost_grid.hpp>
#include <cam_loc/core/pose_sampler.hpp>
#include <cam_loc/kitti/types.hpp>
#include <cam_loc/types/status.hpp>

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
