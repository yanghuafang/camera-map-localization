// CUDA host wrappers: H2D upload, kernel launch, D2H readback for cam_loc GPU
// paths.

#include <cmath>
#include <string>
#include <vector>

#include <cuda_runtime.h>

#include "cam_loc/cuda/distance_transform.h"
#include "cuda/distance_transform_kernels.h"

namespace cam_loc::cuda {

namespace {

#define CUDA_CHECK(expr)         \
  do {                           \
    cudaError_t err = (expr);    \
    if (err != cudaSuccess) {    \
      return Status::kCudaError; \
    }                            \
  } while (0)

/// The device every entry point below binds to, resolved once per process.
struct DeviceChoice {
  int index = -1;
  std::string name;
};

/// Bind to CUDA device 0, and record what it turned out to be.
///
/// Device 0 rather than a "best card" heuristic on purpose: this runs on hosts
/// nobody here has seen, and a rule that silently prefers one card over another
/// is a rule that will one day pick the wrong one on somebody else's machine.
/// Device 0 is the convention every CUDA tool already assumes, so a reader who
/// wants a different card reaches for the knob that already exists --
/// CUDA_VISIBLE_DEVICES, which renumbers what the runtime enumerates, so
/// CUDA_VISIBLE_DEVICES=1 makes that card device 0 here.
///
/// The name is captured because device 0 is not the same hardware everywhere,
/// and a parity result or a timing means little without the card that produced
/// it. Note that a device number cannot be read off nvidia-smi: it orders by
/// PCI bus id, the runtime defaults to CUDA_DEVICE_ORDER=FASTEST_FIRST, and the
/// two need not agree. Hence DeviceName() rather than an index in the reports.
const DeviceChoice& Device() {
  static const DeviceChoice choice = [] {
    DeviceChoice c;
    int count = 0;
    if (cudaGetDeviceCount(&count) != cudaSuccess || count <= 0) return c;
    cudaDeviceProp prop{};
    if (cudaGetDeviceProperties(&prop, 0) != cudaSuccess) return c;
    c.index = 0;
    c.name = prop.name;
    return c;
  }();
  return choice;
}

/// Bind the calling thread to that device.
///
/// cudaSetDevice is per-thread, so this cannot be folded into the one-time
/// resolution above; it runs at the top of each entry point instead, where it
/// is a cheap no-op once the thread is already bound.
cudaError_t BindDevice() {
  const int index = Device().index;
  if (index < 0) return cudaErrorNoDevice;
  return cudaSetDevice(index);
}

}  // namespace

bool IsAvailable() { return Device().index >= 0; }

std::string DeviceName() { return Device().name; }

Status ComputeImagePoseCostsGpu(const float* T_world_plane,
                                const float* map_xyz, int num_points,
                                const uint8_t* map_labels,
                                const float* dt_distance,
                                const uint8_t* dt_labels,
                                const PoseCostGpuParams& params,
                                std::vector<float>& out_costs) {
  // Upload map + DT, launch PoseCostKernel (one thread per hypothesis cell)
  if (!T_world_plane || !map_xyz || num_points <= 0 || !dt_distance ||
      !dt_labels) {
    return Status::kInvalidArgument;
  }
  CUDA_CHECK(BindDevice());
  const int nx = (params.num_x - 1) / 2;
  const int ny = (params.num_y - 1) / 2;
  const int nw = (params.num_yaw - 1) / 2;
  const int total = params.num_x * params.num_y * params.num_yaw;
  const auto step_yaw = static_cast<float>(params.step_yaw_deg * M_PI / 180.0);
  const size_t dt_n = static_cast<size_t>(params.dt_width) *
                      static_cast<size_t>(params.dt_height);

  float* d_T = nullptr;
  float* d_map = nullptr;
  uint8_t* d_mlab = nullptr;
  float* d_dt = nullptr;
  uint8_t* d_dlab = nullptr;
  float* d_costs = nullptr;

  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(
      cudaMalloc(&d_map, static_cast<size_t>(3 * num_points) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_mlab, static_cast<size_t>(num_points)));
  CUDA_CHECK(cudaMalloc(&d_dt, dt_n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_dlab, dt_n));
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(total) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_map, map_xyz, 3 * num_points * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(
      cudaMemcpy(d_mlab, map_labels, num_points, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dt, dt_distance, dt_n * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dlab, dt_labels, dt_n, cudaMemcpyHostToDevice));

  CamLocLaunchPoseCostKernel(
      d_T, d_map, d_mlab, num_points, d_dt, d_dlab, params.dt_width,
      params.dt_height, static_cast<float>(params.fx),
      static_cast<float>(params.fy), static_cast<float>(params.cx),
      static_cast<float>(params.cy), params.dt_max_cost, nx, ny, nw,
      static_cast<float>(params.step_x_m), static_cast<float>(params.step_y_m),
      step_yaw, d_costs, total);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_costs.resize(static_cast<size_t>(total));
  CUDA_CHECK(cudaMemcpy(out_costs.data(), d_costs, total * sizeof(float),
                        cudaMemcpyDeviceToHost));

  cudaFree(d_T);
  cudaFree(d_map);
  cudaFree(d_mlab);
  cudaFree(d_dt);
  cudaFree(d_dlab);
  cudaFree(d_costs);
  return Status::kOk;
}

Status ComputeBevPoseCostsGpu(const float* T_world_plane, const float* map_xyz,
                              int num_points, const uint8_t* map_labels,
                              const float* dt_distance,
                              const uint8_t* dt_labels,
                              const PoseCostGpuParams& params,
                              std::vector<float>& out_costs) {
  // Upload map + BEV DT, launch BevCostKernel (rig XY → pixel, no pinhole)
  if (!T_world_plane || !map_xyz || num_points <= 0 || !dt_distance ||
      !dt_labels) {
    return Status::kInvalidArgument;
  }
  CUDA_CHECK(BindDevice());
  const int nx = (params.num_x - 1) / 2;
  const int ny = (params.num_y - 1) / 2;
  const int nw = (params.num_yaw - 1) / 2;
  const int total = params.num_x * params.num_y * params.num_yaw;
  const auto step_yaw = static_cast<float>(params.step_yaw_deg * M_PI / 180.0);
  const size_t dt_n = static_cast<size_t>(params.dt_width) *
                      static_cast<size_t>(params.dt_height);

  float* d_T = nullptr;
  float* d_map = nullptr;
  uint8_t* d_mlab = nullptr;
  float* d_dt = nullptr;
  uint8_t* d_dlab = nullptr;
  float* d_costs = nullptr;

  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(
      cudaMalloc(&d_map, static_cast<size_t>(3 * num_points) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_mlab, static_cast<size_t>(num_points)));
  CUDA_CHECK(cudaMalloc(&d_dt, dt_n * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_dlab, dt_n));
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(total) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_map, map_xyz, 3 * num_points * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(
      cudaMemcpy(d_mlab, map_labels, num_points, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dt, dt_distance, dt_n * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_dlab, dt_labels, dt_n, cudaMemcpyHostToDevice));

  CamLocLaunchBevCostKernel(
      d_T, d_map, d_mlab, num_points, d_dt, d_dlab, params.dt_width,
      params.dt_height, params.bev_x_min, params.bev_x_max, params.bev_y_min,
      params.bev_y_max, params.bev_mpp_x, params.bev_mpp_y, params.dt_max_cost,
      nx, ny, nw, static_cast<float>(params.step_x_m),
      static_cast<float>(params.step_y_m), step_yaw, d_costs, total);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_costs.resize(static_cast<size_t>(total));
  CUDA_CHECK(cudaMemcpy(out_costs.data(), d_costs, total * sizeof(float),
                        cudaMemcpyDeviceToHost));

  cudaFree(d_T);
  cudaFree(d_map);
  cudaFree(d_mlab);
  cudaFree(d_dt);
  cudaFree(d_dlab);
  cudaFree(d_costs);
  return Status::kOk;
}

Status ArgminGpu(const std::vector<float>& costs, int& out_index,
                 float& out_min) {
  if (costs.empty()) return Status::kInvalidArgument;
  CUDA_CHECK(BindDevice());
  const int n = static_cast<int>(costs.size());
  float* d_costs = nullptr;
  float* d_mins = nullptr;
  int* d_idxs = nullptr;
  CUDA_CHECK(cudaMalloc(&d_costs, static_cast<size_t>(n) * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(d_costs, costs.data(), n * sizeof(float),
                        cudaMemcpyHostToDevice));
  const int tpb = 256;
  const int blocks = (n + tpb - 1) / tpb;
  CUDA_CHECK(cudaMalloc(&d_mins, static_cast<size_t>(blocks) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_idxs, static_cast<size_t>(blocks) * sizeof(int)));
  CamLocLaunchArgminKernel(d_costs, n, d_idxs, d_mins, blocks, tpb);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());
  std::vector<float> mins(static_cast<size_t>(blocks));
  std::vector<int> idxs(static_cast<size_t>(blocks));
  CUDA_CHECK(cudaMemcpy(mins.data(), d_mins, blocks * sizeof(float),
                        cudaMemcpyDeviceToHost));
  CUDA_CHECK(cudaMemcpy(idxs.data(), d_idxs, blocks * sizeof(int),
                        cudaMemcpyDeviceToHost));
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

Status ComputeDistanceTransformGpu(const std::vector<uint8_t>& binary,
                                   int width, int height,
                                   std::vector<float>& out_distance) {
  // Two-pass Felzenszwalb EDT: EdtColumnKernel then EdtRowKernel (in-place
  // buffer)
  if (width <= 0 || height <= 0 || width > 1280 || height > 1280 ||
      static_cast<size_t>(width * height) != binary.size()) {
    return Status::kInvalidArgument;
  }
  CUDA_CHECK(BindDevice());
  const size_t n = static_cast<size_t>(width) * static_cast<size_t>(height);

  uint8_t* d_binary = nullptr;
  float* d_buf = nullptr;
  CUDA_CHECK(cudaMalloc(&d_binary, n));
  CUDA_CHECK(cudaMalloc(&d_buf, n * sizeof(float)));
  CUDA_CHECK(cudaMemcpy(d_binary, binary.data(), n, cudaMemcpyHostToDevice));

  CamLocLaunchEdtKernel(d_binary, d_buf, width, height);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  out_distance.resize(n);
  CUDA_CHECK(cudaMemcpy(out_distance.data(), d_buf, n * sizeof(float),
                        cudaMemcpyDeviceToHost));

  cudaFree(d_binary);
  cudaFree(d_buf);
  return Status::kOk;
}

Status AggregateCostsGpu(std::vector<float>& inout_current,
                         const float T_world_plane[16],
                         const std::vector<float>& hist_inv_T,
                         const std::vector<float>& hist_weights,
                         const std::vector<float>& hist_costs,
                         const CostAggregateGpuParams& params) {
  // Warp each history volume into current plane frame, distance-weight, fuse
  // in-place
  const int num_hist = static_cast<int>(hist_weights.size());
  const int num_cells = params.dim_x * params.dim_y * params.dim_w;
  if (num_hist <= 0 || num_cells <= 0 ||
      static_cast<int>(inout_current.size()) != num_cells ||
      static_cast<int>(hist_inv_T.size()) != num_hist * 16 ||
      static_cast<int>(hist_costs.size()) != num_hist * num_cells) {
    return Status::kInvalidArgument;
  }
  CUDA_CHECK(BindDevice());

  float* d_current = nullptr;
  float* d_T = nullptr;
  float* d_inv_T = nullptr;
  float* d_weights = nullptr;
  float* d_hist_costs = nullptr;

  CUDA_CHECK(
      cudaMalloc(&d_current, static_cast<size_t>(num_cells) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&d_T, 16 * sizeof(float)));
  CUDA_CHECK(
      cudaMalloc(&d_inv_T, static_cast<size_t>(num_hist) * 16 * sizeof(float)));
  CUDA_CHECK(
      cudaMalloc(&d_weights, static_cast<size_t>(num_hist) * sizeof(float)));
  CUDA_CHECK(cudaMalloc(
      &d_hist_costs, static_cast<size_t>(hist_costs.size()) * sizeof(float)));

  CUDA_CHECK(cudaMemcpy(d_current, inout_current.data(),
                        static_cast<size_t>(num_cells) * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_T, T_world_plane, 16 * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_inv_T, hist_inv_T.data(),
                        static_cast<size_t>(num_hist) * 16 * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_weights, hist_weights.data(),
                        static_cast<size_t>(num_hist) * sizeof(float),
                        cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_hist_costs, hist_costs.data(),
                        hist_costs.size() * sizeof(float),
                        cudaMemcpyHostToDevice));

  CamLocLaunchAggregateKernel(d_current, d_T, d_inv_T, d_weights, d_hist_costs,
                              params.dim_x, params.dim_y, params.dim_w,
                              params.nx, params.ny, params.nw, params.step_x,
                              params.step_y, params.step_yaw, params.fuse_alpha,
                              num_hist, num_cells);
  CUDA_CHECK(cudaGetLastError());
  CUDA_CHECK(cudaDeviceSynchronize());

  CUDA_CHECK(cudaMemcpy(inout_current.data(), d_current,
                        static_cast<size_t>(num_cells) * sizeof(float),
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
