// CPU-only build stub: argmin falls back to std::min_element; other GPU entry points noop.

#include <cam_loc/cuda/distance_transform.hpp>

#include <algorithm>
#include <limits>

namespace cam_loc::cuda {

bool isAvailable() { return false; }

Status computeImagePoseCostsGpu(const float* /*T_world_plane*/, const float* /*map_xyz*/,
                                int /*num_points*/, const uint8_t* /*map_labels*/,
                                const float* /*dt_distance*/, const uint8_t* /*dt_labels*/,
                                const PoseCostGpuParams& /*params*/, std::vector<float>& /*out_costs*/) {
  return Status::kNotImplemented;
}

Status computeBevPoseCostsGpu(const float* /*T_world_plane*/, const float* /*map_xyz*/,
                              int /*num_points*/, const uint8_t* /*map_labels*/,
                              const float* /*dt_distance*/, const uint8_t* /*dt_labels*/,
                              const PoseCostGpuParams& /*params*/, std::vector<float>& /*out_costs*/) {
  return Status::kNotImplemented;
}

Status argminGpu(const std::vector<float>& costs, int& out_index, float& out_min) {
  if (costs.empty()) return Status::kInvalidArgument;
  const auto it = std::min_element(costs.begin(), costs.end());
  out_index = static_cast<int>(it - costs.begin());
  out_min = *it;
  return Status::kOk;
}

Status computeDistanceTransformGpu(const std::vector<uint8_t>& /*binary*/, int /*width*/,
                                   int /*height*/, std::vector<float>& /*out_distance*/) {
  return Status::kNotImplemented;
}

Status aggregateCostsGpu(std::vector<float>& /*inout_current*/, const float /*T_world_plane*/[16],
                         const std::vector<float>& /*hist_inv_T*/,
                         const std::vector<float>& /*hist_weights*/,
                         const std::vector<float>& /*hist_costs*/,
                         const CostAggregateGpuParams& /*params*/) {
  return Status::kNotImplemented;
}

}  // namespace cam_loc::cuda
