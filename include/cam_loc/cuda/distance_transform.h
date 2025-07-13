#ifndef CAM_LOC_CUDA_DISTANCE_TRANSFORM_H_
#define CAM_LOC_CUDA_DISTANCE_TRANSFORM_H_

#include <cstdint>
#include <string>
#include <vector>

#include "cam_loc/types/status_codes.h"

namespace cam_loc::cuda {

/// GPU acceleration for EDT, pose-cost evaluation, argmin, and temporal
/// aggregation. All 4×4 transforms are row-major; CostGrid layout matches
/// core::CostGrid indexing.

/// True when a usable CUDA device is visible.
bool IsAvailable();

/// Name of the device the GPU paths actually run on, e.g. "NVIDIA RTX A6000".
/// Empty when none is available.
///
/// Worth reporting rather than assuming: the entry points below run on CUDA
/// device 0, which is different hardware on every host, and on a multi-GPU box
/// is not the device number nvidia-smi shows -- that orders by PCI bus id while
/// the runtime does not. A parity or timing result is only meaningful next to
/// the card that produced it, so this names the card rather than the index.
std::string DeviceName();

struct PoseCostGpuParams {
  int num_x = 21;
  int num_y = 31;
  int num_yaw = 13;
  double step_x_m = 0.5;
  double step_y_m = 0.5;
  double step_yaw_deg = 0.5;
  double fx = 718.0;
  double fy = 718.0;
  double cx = 607.0;
  double cy = 185.0;
  float dt_max_cost = 5.f;
  int dt_width = 0;
  int dt_height = 0;
  float bev_x_min = -20.f;
  float bev_x_max = 20.f;
  float bev_y_min = -10.f;
  float bev_y_max = 10.f;
  float bev_mpp_x = 0.22857f;
  float bev_mpp_y = 0.11429f;
};

Status ComputeImagePoseCostsGpu(const float* T_world_plane,
                                const float* map_xyz, int num_points,
                                const uint8_t* map_labels,
                                const float* dt_distance,
                                const uint8_t* dt_labels,
                                const PoseCostGpuParams& params,
                                std::vector<float>& out_costs);

Status ComputeBevPoseCostsGpu(const float* T_world_plane, const float* map_xyz,
                              int num_points, const uint8_t* map_labels,
                              const float* dt_distance,
                              const uint8_t* dt_labels,
                              const PoseCostGpuParams& params,
                              std::vector<float>& out_costs);

Status ArgminGpu(const std::vector<float>& costs, int& out_index,
                 float& out_min);

/// Felzenszwalb EDT on GPU. Input binary row-major: 0 = feature, 255 =
/// background.
Status ComputeDistanceTransformGpu(const std::vector<uint8_t>& binary,
                                   int width, int height,
                                   std::vector<float>& out_distance);

struct CostAggregateGpuParams {
  int dim_x = 0;
  int dim_y = 0;
  int dim_w = 0;
  int nx = 0;
  int ny = 0;
  int nw = 0;
  float step_x = 0.f;
  float step_y = 0.f;
  float step_yaw = 0.f;
  float fuse_alpha = 0.5f;
};

/// Temporal cost aggregation. `T_world_plane` and `hist_inv_T` are row-major
/// 4x4. `hist_costs` is concatenated history volumes (num_hist * num_cells).
Status AggregateCostsGpu(std::vector<float>& inout_current,
                         const float T_world_plane[16],
                         const std::vector<float>& hist_inv_T,
                         const std::vector<float>& hist_weights,
                         const std::vector<float>& hist_costs,
                         const CostAggregateGpuParams& params);

}  // namespace cam_loc::cuda

#endif  // CAM_LOC_CUDA_DISTANCE_TRANSFORM_H_
