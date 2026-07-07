// CUDA host wrappers: H2D upload, kernel launch, D2H readback for cam_loc GPU paths.

#include <cam_loc/cuda/distance_transform.hpp>

#include "distance_transform_kernels.cuh"

#include <cuda_runtime.h>

#include <cmath>
#include <vector>

namespace cam_loc::cuda {

namespace {

#define CUDA_CHECK(expr)                                                                             \
  do {                                                                                             \
    cudaError_t err = (expr);                                                                      \
    if (err != cudaSuccess) {                                                                      \
      return Status::kCudaError;                                                                   \
    }                                                                                              \
  } while (0)

}  // namespace

bool isAvailable() {
  int count = 0;
  return cudaGetDeviceCount(&count) == cudaSuccess && count > 0;
}

Status computeImagePoseCostsGpu(const float* T_world_plane, const float* map_xyz, int num_points,
                                const uint8_t* map_labels, const float* dt_distance,
                                const uint8_t* dt_labels, const PoseCostGpuParams& params,
                                std::vector<float>& out_costs) {
  // Upload map + DT, launch poseCostKernel (one thread per hypothesis cell)
  if (!T_world_plane || !map_xyz || num_points <= 0 || !dt_distance || !dt_labels) {
    return Status::kInvalidArgument;
  }
  const int nx = (params.num_x - 1) / 2;
  const int ny = (params.num_y - 1) / 2;
  const int nw = (params.num_yaw - 1) / 2;
  const int total = params.num_x * params.num_y * params.num_yaw;
  const float step_yaw = static_cast<float>(params.step_yaw_deg * M_PI / 180.0);
  const size_t dt_n =
      static_cast<size_t>(params.dt_width) * static_cast<size_t>(params.dt_height);

  float* d_T = nullptr;
  float* d_map = nullptr;
  uint8_t* d_mlab = nullptr;
  float* d_dt = nullptr;
  uint8_t* d_dlab = nullptr;
  float* d_costs = nullptr;

  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_map, static_cast<size_t>(3 * num_points) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_mlab, static_cast<size_t>(num_points)));
  CUDA_CHECK(cudaMalloc(&d_dt, dt_n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_dlab, dt_n));
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(total) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_map, map_xyz, 3 * num_points * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_mlab, map_labels, num_points, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dt, dt_distance, dt_n * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dlab, dt_labels, dt_n, cudaMemcpyHostToDevice));

  cam_loc_launch_pose_cost_kernel(
      d_T, d_map, d_mlab, num_points, d_dt, d_dlab, params.dt_width, params.dt_height,
      static_cast<float>(params.fx), static_cast<float>(params.fy), static_cast<float>(params.cx),
      static_cast<float>(params.cy), params.dt_max_cost, nx, ny, nw,
      static_cast<float>(params.step_x_m), static_cast<float>(params.step_y_m), step_yaw, d_costs,
      total);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_costs.resize(static_cast<size_t>(total));
  CUDA_CHECK(cudaMemcpy(out_costs.data(), d_costs, total * sizeof(float), cudaMemcpyDeviceToHost));

  cudaFree(d_T);
  cudaFree(d_map);
  cudaFree(d_mlab);
  cudaFree(d_dt);
  cudaFree(d_dlab);
  cudaFree(d_costs);
  return Status::kOk;
}

Status computeBevPoseCostsGpu(const float* T_world_plane, const float* map_xyz, int num_points,
                              const uint8_t* map_labels, const float* dt_distance,
                              const uint8_t* dt_labels, const PoseCostGpuParams& params,
                              std::vector<float>& out_costs) {
  // Upload map + BEV DT, launch bevCostKernel (rig XY → pixel, no pinhole)
  if (!T_world_plane || !map_xyz || num_points <= 0 || !dt_distance || !dt_labels) {
    return Status::kInvalidArgument;
  }
  const int nx = (params.num_x - 1) / 2;
  const int ny = (params.num_y - 1) / 2;
  const int nw = (params.num_yaw - 1) / 2;
  const int total = params.num_x * params.num_y * params.num_yaw;
  const float step_yaw = static_cast<float>(params.step_yaw_deg * M_PI / 180.0);
  const size_t dt_n =
      static_cast<size_t>(params.dt_width) * static_cast<size_t>(params.dt_height);

  float* d_T = nullptr;
  float* d_map = nullptr;
  uint8_t* d_mlab = nullptr;
  float* d_dt = nullptr;
  uint8_t* d_dlab = nullptr;
  float* d_costs = nullptr;

  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_map, static_cast<size_t>(3 * num_points) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_mlab, static_cast<size_t>(num_points)));
  CUDA_CHECK(cudaMalloc(&d_dt, dt_n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_dlab, dt_n));
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(total) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_map, map_xyz, 3 * num_points * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_mlab, map_labels, num_points, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dt, dt_distance, dt_n * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dlab, dt_labels, dt_n, cudaMemcpyHostToDevice));

  cam_loc_launch_bev_cost_kernel(
      d_T, d_map, d_mlab, num_points, d_dt, d_dlab, params.dt_width, params.dt_height,
      params.bev_x_min, params.bev_x_max, params.bev_y_min, params.bev_y_max, params.bev_mpp_x,
      params.bev_mpp_y, params.dt_max_cost, nx, ny, nw, static_cast<float>(params.step_x_m),
      static_cast<float>(params.step_y_m), step_yaw, d_costs, total);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_costs.resize(static_cast<size_t>(total));
  CUDA_CHECK(cudaMemcpy(out_costs.data(), d_costs, total * sizeof(float), cudaMemcpyDeviceToHost));

  cudaFree(d_T);
  cudaFree(d_map);
  cudaFree(d_mlab);
  cudaFree(d_dt);
  cudaFree(d_dlab);
  cudaFree(d_costs);
  return Status::kOk;
}

Status argminGpu(const std::vector<float>& costs, int& out_index, float& out_min) {
  if (costs.empty()) return Status::kInvalidArgument;
  const int n = static_cast<int>(costs.size());
  float* d_costs = nullptr;
  float* d_mins = nullptr;
  int* d_idxs = nullptr;
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(n) * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(d_costs, costs.data(), n * sizeof(float), cudaMemcpyHostToDevice));
  const int tpb = 256;
  const int blocks = (n + tpb - 1) / tpb;
  CUDA_CHECK(cudaMalloc(&d_mins, static_cast<size_t>(blocks) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_idxs, static_cast<size_t>(blocks) * sizeof(int)));
  cam_loc_launch_argmin_kernel(d_costs, n, d_idxs, d_mins, blocks, tpb);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  std::vector<float> mins(static_cast<size_t>(blocks));
  std::vector<int> idxs(static_cast<size_t>(blocks));
  CUDA_CHECK(cudaMemcpy(mins.data(), d_mins, blocks * sizeof(float), cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(idxs.data(), d_idxs, blocks * sizeof(int), cudaMemcpyDeviceToHost));
  out_min = mins[0];
  out_index = idxs[0];
  for (int i = 1; i < blocks; ++i) {
    if (mins[static_cast<size_t>(i)] < out_min) {
      out_min = mins[static_cast<size_t>(i)];
      out_index = idxs[static_cast<size_t>(i)];
    }
  }
  cudaFree(d_costs);
  cudaFree(d_mins);
  cudaFree(d_idxs);
  return Status::kOk;
}

Status computeDistanceTransformGpu(const std::vector<uint8_t>& binary, int width, int height,
                                   std::vector<float>& out_distance) {
  // Two-pass Felzenszwalb EDT: edtColumnKernel then edtRowKernel (in-place buffer)
  if (width <= 0 || height <= 0 || width > 1280 || height > 1280 ||
      static_cast<size_t>(width * height) != binary.size()) {
    return Status::kInvalidArgument;
  }
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);

  uint8_t* d_binary = nullptr;
  float* d_buf = nullptr;
  CUDA_CHECK(cudaMalloc(&d_binary, n));
  CUDA_CHECK(cudaMalloc(&d_buf, n * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(d_binary, binary.data(), n, cudaMemcpyHostToDevice));

  cam_loc_launch_edt_kernel(d_binary, d_buf, width, height);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_distance.resize(n);
  CUDA_CHECK(cudaMemcpy(out_distance.data(), d_buf, n * sizeof(float), cudaMemcpyDeviceToHost));

  cudaFree(d_binary);
  cudaFree(d_buf);
  return Status::kOk;
}

Status aggregateCostsGpu(std::vector<float>& inout_current, const float T_world_plane[16],
                         const std::vector<float>& hist_inv_T, const std::vector<float>& hist_weights,
                         const std::vector<float>& hist_costs, const CostAggregateGpuParams& params) {
  // Warp each history volume into current plane frame, distance-weight, fuse in-place
  const int num_hist = static_cast<int>(hist_weights.size());
  const int num_cells = params.dim_x * params.dim_y * params.dim_w;
  if (num_hist <= 0 || num_cells <= 0 ||
      static_cast<int>(inout_current.size()) != num_cells ||
      static_cast<int>(hist_inv_T.size()) != num_hist * 16 ||
      static_cast<int>(hist_costs.size()) != num_hist * num_cells) {
    return Status::kInvalidArgument;
  }

  float* d_current = nullptr;
  float* d_T = nullptr;
  float* d_inv_T = nullptr;
  float* d_weights = nullptr;
  float* d_hist_costs = nullptr;

  CUDA_CHECK(cudaMalloc(&d_current, static_cast<size_t>(num_cells) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_inv_T, static_cast<size_t>(num_hist) * 16 * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_weights, static_cast<size_t>(num_hist) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_hist_costs, static_cast<size_t>(hist_costs.size()) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_current, inout_current.data(), static_cast<size_t>(num_cells) * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float), cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_inv_T, hist_inv_T.data(), static_cast<size_t>(num_hist) * 16 * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_weights, hist_weights.data(), static_cast<size_t>(num_hist) * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_hist_costs, hist_costs.data(), hist_costs.size() * sizeof(float),
                        cudaMemcpyHostToDevice));

  cam_loc_launch_aggregate_kernel(
      d_current, d_T, d_inv_T, d_weights, d_hist_costs, params.dim_x, params.dim_y, params.dim_w,
      params.nx, params.ny, params.nw, params.step_x, params.step_y, params.step_yaw,
      params.fuse_alpha, num_hist, num_cells);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(inout_current.data(), d_current, static_cast<size_t>(num_cells) * sizeof(float),
                        cudaMemcpyDeviceToHost));

  cudaFree(d_current);
  cudaFree(d_T);
  cudaFree(d_inv_T);
  cudaFree(d_weights);
  cudaFree(d_hist_costs);
  return Status::kOk;
}

}  // namespace cam_loc::cuda

#undef CUDA_CHECK
