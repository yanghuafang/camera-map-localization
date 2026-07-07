#include <cuda_runtime.h>

#include "distance_transform_kernels.cuh"

#include <cstdint>
#include <cmath>

// CUDA kernels: pose-cost evaluation, Felzenszwalb EDT, temporal aggregation, argmin.

namespace {

/// Bilinear DT sample with per-pixel type label gate (mismatched label → max_cost).
__device__ float sampleDt(const float* dt, const uint8_t* labels, int w, int h, float u, float v,
                          uint8_t expected, float max_cost) {
  if (u < 0.f || v < 0.f || u >= w - 1 || v >= h - 1) return max_cost;
  const int x0 = static_cast<int>(floorf(u));
  const int y0 = static_cast<int>(floorf(v));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const int lx = static_cast<int>(lroundf(u));
  const int ly = static_cast<int>(lroundf(v));
  if (lx >= 0 && ly >= 0 && lx < w && ly < h) {
    const uint8_t lab = labels[ly * w + lx];
    if (lab != 0 && lab != expected) return max_cost;
  }
  const float tx = u - x0;
  const float ty = v - y0;
  auto at = [&](int x, int y) { return dt[y * w + x]; };
  const float v00 = at(x0, y0);
  const float v10 = at(x1, y0);
  const float v01 = at(x0, y1);
  const float v11 = at(x1, y1);
  const float va = v00 * (1.f - tx) + v10 * tx;
  const float vb = v01 * (1.f - tx) + v11 * tx;
  return fminf(va * (1.f - ty) + vb * ty, max_cost);
}

/// World point → rig frame using row-major 4×4 (R^T · (p - t)).
__device__ void worldToRig(const float* T, float wx, float wy, float wz, float& rx, float& ry,
                           float& rz) {
  const float tx = T[3];
  const float ty = T[7];
  const float tz = T[11];
  const float dx = wx - tx;
  const float dy = wy - ty;
  const float dz = wz - tz;
  rx = T[0] * dx + T[4] * dy + T[8] * dz;
  ry = T[1] * dx + T[5] * dy + T[9] * dz;
  rz = T[2] * dx + T[6] * dy + T[10] * dz;
}

// Pass: image-space pose cost — one thread per CostGrid cell; pinhole project + DT sample.
__global__ void poseCostKernel(const float* T_plane, const float* map_xyz, const uint8_t* map_labels,
                               int num_points, const float* dt, const uint8_t* dt_labels, int dt_w,
                               int dt_h, float fx, float fy, float cx, float cy, float max_cost,
                               int nx, int ny, int nw, float step_x, float step_y, float step_yaw,
                               float* out_costs) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int dim_x = 2 * nx + 1;
  const int dim_y = 2 * ny + 1;
  const int total = dim_x * dim_y * (2 * nw + 1);
  if (idx >= total) return;

  const int dim_xy = dim_x * dim_y;
  const int iw = idx / dim_xy;
  const int rem = idx % dim_xy;
  const int iy = rem / dim_x;
  const int ix = rem % dim_x;

  const float ox = (ix - nx) * step_x;
  const float oy = (iy - ny) * step_y;
  const float yaw = (iw - nw) * step_yaw;
  const float c = cosf(yaw);
  const float s = sinf(yaw);

  float T_off[16] = {c, -s, 0, ox, s, c, 0, oy, 0, 0, 1, 0, 0, 0, 0, 1};
  float T_hyp[16];
  for (int r = 0; r < 4; ++r) {
    for (int ccol = 0; ccol < 4; ++ccol) {
      float sum = 0.f;
      for (int k = 0; k < 4; ++k) {
        sum += T_plane[r * 4 + k] * T_off[k * 4 + ccol];
      }
      T_hyp[r * 4 + ccol] = sum;
    }
  }

  float total_cost = 0.f;
  int count = 0;
  for (int p = 0; p < num_points; ++p) {
    const float wx = map_xyz[3 * p];
    const float wy = map_xyz[3 * p + 1];
    const float wz = map_xyz[3 * p + 2];
    float rx, ry, rz;
    worldToRig(T_hyp, wx, wy, wz, rx, ry, rz);
    if (rz <= 0.5f) continue;
    const float u = fx * rx / rz + cx;
    const float v = fy * ry / rz + cy;
    total_cost += sampleDt(dt, dt_labels, dt_w, dt_h, u, v, map_labels[p], max_cost);
    ++count;
  }
  out_costs[idx] = count > 0 ? total_cost / static_cast<float>(count) : max_cost * 10.f;
}

// Pass: BEV pose cost — rig XY → BEV pixel center, then DT sample (no pinhole).
__global__ void bevCostKernel(const float* T_plane, const float* map_xyz, const uint8_t* map_labels,
                              int num_points, const float* dt, const uint8_t* dt_labels, int dt_w,
                              int dt_h, float x_min, float x_max, float y_min, float y_max,
                              float mpp_x, float mpp_y, float max_cost, int nx, int ny, int nw,
                              float step_x, float step_y, float step_yaw, float* out_costs) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  const int dim_x = 2 * nx + 1;
  const int dim_y = 2 * ny + 1;
  const int total = dim_x * dim_y * (2 * nw + 1);
  if (idx >= total) return;

  const int dim_xy = dim_x * dim_y;
  const int iw = idx / dim_xy;
  const int rem = idx % dim_xy;
  const int iy = rem / dim_x;
  const int ix = rem % dim_x;

  const float ox = (ix - nx) * step_x;
  const float oy = (iy - ny) * step_y;
  const float yaw = (iw - nw) * step_yaw;
  const float c = cosf(yaw);
  const float s = sinf(yaw);

  float T_off[16] = {c, -s, 0, ox, s, c, 0, oy, 0, 0, 1, 0, 0, 0, 0, 1};
  float T_hyp[16];
  for (int r = 0; r < 4; ++r) {
    for (int ccol = 0; ccol < 4; ++ccol) {
      float sum = 0.f;
      for (int k = 0; k < 4; ++k) {
        sum += T_plane[r * 4 + k] * T_off[k * 4 + ccol];
      }
      T_hyp[r * 4 + ccol] = sum;
    }
  }

  float total_cost = 0.f;
  int count = 0;
  for (int p = 0; p < num_points; ++p) {
    const float wx = map_xyz[3 * p];
    const float wy = map_xyz[3 * p + 1];
    const float wz = map_xyz[3 * p + 2];
    float rx, ry, rz;
    worldToRig(T_hyp, wx, wy, wz, rx, ry, rz);
    if (rx < x_min || rx > x_max || ry < y_min || ry > y_max) continue;
    const float u = floorf((rx - x_min) / mpp_x) + 0.5f;
    const float v = floorf((ry - y_min) / mpp_y) + 0.5f;
    total_cost += sampleDt(dt, dt_labels, dt_w, dt_h, u, v, map_labels[p], max_cost);
    ++count;
  }
  out_costs[idx] = count > 0 ? total_cost / static_cast<float>(count) : max_cost * 10.f;
}

/// 1D squared-distance transform (Felzenszwalb parabola envelope; see edt1d in
/// distance_transform_cpu.cpp for the algorithm). v/z are caller-supplied scratch: v[k] is the
/// k-th envelope parabola location, z[k] its left boundary — kept off the stack by the caller.
__device__ void edt1dDevice(const float* f, float* d, int n, int* v, float* z) {
  int k = 0;
  v[0] = 0;
  z[0] = -INFINITY;
  z[1] = INFINITY;

  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.f * q - 2.f * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.f * q - 2.f * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = INFINITY;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
}

// Pass 1 (EDT): column-wise squared-distance transform; one thread per column.
__global__ void edtColumnKernel(const uint8_t* binary, float* f, int width, int height) {
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  if (x >= width || height > 1280) return;

  // Per-thread scratch is stack-allocated, so each transformed dimension is capped at 1280 px
  // (KITTI 1241x376 fits). Larger inputs are rejected by computeDistanceTransformGpu upstream.
  float col[1280];
  float tmp[1280];
  int v[1280];
  float z[1281];

  for (int y = 0; y < height; ++y) {
    col[y] = binary[y * width + x] == 0 ? 0.f : 1e8f;
  }
  edt1dDevice(col, tmp, height, v, z);
  for (int y = 0; y < height; ++y) {
    f[y * width + x] = tmp[y];
  }
}

// Pass 2 (EDT): row-wise transform + sqrt → pixel Euclidean distance; one thread per row.
__global__ void edtRowKernel(const float* f, float* out, int width, int height) {
  const int y = blockIdx.x * blockDim.x + threadIdx.x;
  if (y >= height || width > 1280) return;

  // Per-thread scratch is stack-allocated, so each transformed dimension is capped at 1280 px
  // (KITTI 1241x376 fits). Larger inputs are rejected by computeDistanceTransformGpu upstream.
  float row[1280];
  float tmp[1280];
  int v[1280];
  float z[1281];

  for (int x = 0; x < width; ++x) {
    row[x] = f[y * width + x];
  }
  edt1dDevice(row, tmp, width, v, z);
  for (int x = 0; x < width; ++x) {
    out[y * width + x] = sqrtf(tmp[x]);
  }
}

__device__ void mat4MulRow(const float* A, const float* B, float* C) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      float sum = 0.f;
      for (int k = 0; k < 4; ++k) {
        sum += A[r * 4 + k] * B[k * 4 + c];
      }
      C[r * 4 + c] = sum;
    }
  }
}

__device__ void offsetToTransformDevice(float x_m, float y_m, float yaw_rad, float* T) {
  const float c = cosf(yaw_rad);
  const float s = sinf(yaw_rad);
  T[0] = c;
  T[1] = -s;
  T[2] = 0.f;
  T[3] = x_m;
  T[4] = s;
  T[5] = c;
  T[6] = 0.f;
  T[7] = y_m;
  T[8] = 0.f;
  T[9] = 0.f;
  T[10] = 1.f;
  T[11] = 0.f;
  T[12] = 0.f;
  T[13] = 0.f;
  T[14] = 0.f;
  T[15] = 1.f;
}

__device__ void transformToOffsetDevice(const float* T, float& x_m, float& y_m, float& yaw_rad) {
  x_m = T[3];
  y_m = T[7];
  yaw_rad = atan2f(T[4], T[0]);
}

__device__ int clampIndexDevice(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

__device__ float sampleCostGridDevice(const float* costs, int dim_x, int dim_y, int dim_w, int nx,
                                      int ny, int nw, float step_x, float step_y, float step_yaw,
                                      float x_m, float y_m, float yaw_rad) {
  const float fx = x_m / step_x + static_cast<float>(nx);
  const float fy = y_m / step_y + static_cast<float>(ny);
  const float fw = yaw_rad / step_yaw + static_cast<float>(nw);

  const int x0 = clampIndexDevice(static_cast<int>(floorf(fx)), 0, dim_x - 1);
  const int y0 = clampIndexDevice(static_cast<int>(floorf(fy)), 0, dim_y - 1);
  const int w0 = clampIndexDevice(static_cast<int>(floorf(fw)), 0, dim_w - 1);
  const int x1 = clampIndexDevice(x0 + 1, 0, dim_x - 1);
  const int y1 = clampIndexDevice(y0 + 1, 0, dim_y - 1);
  const int w1 = clampIndexDevice(w0 + 1, 0, dim_w - 1);

  const float tx = fx - x0;
  const float ty = fy - y0;
  const float tw = fw - w0;

  auto at = [&](int ix, int iy, int iw) {
    return costs[iw * dim_x * dim_y + iy * dim_x + ix];
  };

  const float c000 = at(x0, y0, w0);
  const float c100 = at(x1, y0, w0);
  const float c010 = at(x0, y1, w0);
  const float c110 = at(x1, y1, w0);
  const float c001 = at(x0, y0, w1);
  const float c101 = at(x1, y0, w1);
  const float c011 = at(x0, y1, w1);
  const float c111 = at(x1, y1, w1);

  const float c00 = c000 * (1.f - tx) + c100 * tx;
  const float c10 = c010 * (1.f - tx) + c110 * tx;
  const float c01 = c001 * (1.f - tx) + c101 * tx;
  const float c11 = c011 * (1.f - tx) + c111 * tx;
  const float c0 = c00 * (1.f - ty) + c10 * ty;
  const float c1 = c01 * (1.f - ty) + c11 * ty;
  return c0 * (1.f - tw) + c1 * tw;
}

// Pass: temporal aggregation — warp history cost volumes into current plane, fuse 50/50.
__global__ void aggregateKernel(float* current, const float* T_curr, const float* hist_inv_T,
                                const float* hist_weights, const float* hist_costs, int dim_x,
                                int dim_y, int dim_w, int nx, int ny, int nw, float step_x,
                                float step_y, float step_yaw, float fuse_alpha, int num_hist,
                                int num_cells) {
  const int idx = blockIdx.x * blockDim.x + threadIdx.x;
  if (idx >= num_cells) return;

  const int dim_xy = dim_x * dim_y;
  const int iw = idx / dim_xy;
  const int rem = idx % dim_xy;
  const int iy = rem / dim_x;
  const int ix = rem % dim_x;

  const float x_m = (ix - nx) * step_x;
  const float y_m = (iy - ny) * step_y;
  const float yaw_rad = (iw - nw) * step_yaw;

  float T_offset[16];
  offsetToTransformDevice(x_m, y_m, yaw_rad, T_offset);

  float T_world_hyp[16];
  mat4MulRow(T_curr, T_offset, T_world_hyp);

  float agg = 0.f;
  float sum_w = 0.f;
  for (int h = 0; h < num_hist; ++h) {
    const float w = hist_weights[h];
    if (w <= 0.f) continue;

    const float* inv_T = hist_inv_T + h * 16;
    float T_offset_prev[16];
    mat4MulRow(inv_T, T_world_hyp, T_offset_prev);

    float ox = 0.f, oy = 0.f, oyaw = 0.f;
    transformToOffsetDevice(T_offset_prev, ox, oy, oyaw);

    const float past = sampleCostGridDevice(hist_costs + h * num_cells, dim_x, dim_y, dim_w, nx, ny,
                                            nw, step_x, step_y, step_yaw, ox, oy, oyaw);
    agg += w * past;
    sum_w += w;
  }

  if (sum_w > 1e-6f) {
    agg /= sum_w;
    current[idx] = fuse_alpha * current[idx] + (1.f - fuse_alpha) * agg;
  }
}

// Pass: block-parallel argmin with shared-memory reduction (host merges block winners).
__global__ void argminKernel(const float* costs, int n, int* block_idx, float* block_min) {
  // One dynamic shared buffer holds both arrays back to back: blockDim.x floats (the running
  // minima) followed by blockDim.x ints (their source indices). The launcher sizes it to match.
  extern __shared__ float s_min[];
  int* s_idx = reinterpret_cast<int*>(s_min + blockDim.x);
  const int tid = threadIdx.x;
  const int i = blockIdx.x * blockDim.x + tid;
  float val = 1e30f;
  int idx = 0;
  if (i < n) {
    val = costs[i];
    idx = i;
  }
  s_min[tid] = val;
  s_idx[tid] = idx;
  __syncthreads();
  for (int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (tid < s) {
      if (s_min[tid + s] < s_min[tid]) {
        s_min[tid] = s_min[tid + s];
        s_idx[tid] = s_idx[tid + s];
      }
    }
    __syncthreads();
  }
  if (tid == 0) {
    block_min[blockIdx.x] = s_min[0];
    block_idx[blockIdx.x] = s_idx[0];
  }
}

}  // namespace

extern "C" void cam_loc_launch_pose_cost_kernel(
    const float* d_T, const float* d_map, const uint8_t* d_mlab, int num_points,
    const float* d_dt, const uint8_t* d_dlab, int dt_w, int dt_h, float fx, float fy, float cx,
    float cy, float max_cost, int nx, int ny, int nw, float step_x, float step_y, float step_yaw,
    float* d_costs, int total) {
  const int tpb = 256;
  const int blocks = (total + tpb - 1) / tpb;
  poseCostKernel<<<blocks, tpb>>>(d_T, d_map, d_mlab, num_points, d_dt, d_dlab, dt_w, dt_h, fx,
                                  fy, cx, cy, max_cost, nx, ny, nw, step_x, step_y, step_yaw,
                                  d_costs);
}

extern "C" void cam_loc_launch_bev_cost_kernel(
    const float* d_T, const float* d_map, const uint8_t* d_mlab, int num_points,
    const float* d_dt, const uint8_t* d_dlab, int dt_w, int dt_h, float x_min, float x_max,
    float y_min, float y_max, float mpp_x, float mpp_y, float max_cost, int nx, int ny, int nw,
    float step_x, float step_y, float step_yaw, float* d_costs, int total) {
  const int tpb = 256;
  const int blocks = (total + tpb - 1) / tpb;
  bevCostKernel<<<blocks, tpb>>>(d_T, d_map, d_mlab, num_points, d_dt, d_dlab, dt_w, dt_h, x_min,
                                 x_max, y_min, y_max, mpp_x, mpp_y, max_cost, nx, ny, nw, step_x,
                                 step_y, step_yaw, d_costs);
}

extern "C" void cam_loc_launch_argmin_kernel(const float* d_costs, int n, int* d_idxs,
                                             float* d_mins, int blocks, int tpb) {
  argminKernel<<<blocks, tpb, static_cast<size_t>(tpb) * (sizeof(float) + sizeof(int))>>>(
      d_costs, n, d_idxs, d_mins);
}

extern "C" void cam_loc_launch_edt_kernel(const uint8_t* d_binary, float* d_buf, int width,
                                          int height) {
  const int tpb = 256;
  const int col_blocks = (width + tpb - 1) / tpb;
  edtColumnKernel<<<col_blocks, tpb>>>(d_binary, d_buf, width, height);
  const int row_blocks = (height + tpb - 1) / tpb;
  edtRowKernel<<<row_blocks, tpb>>>(d_buf, d_buf, width, height);
}

extern "C" void cam_loc_launch_aggregate_kernel(
    float* d_current, const float* d_T_curr, const float* d_hist_inv_T, const float* d_hist_weights,
    const float* d_hist_costs, int dim_x, int dim_y, int dim_w, int nx, int ny, int nw,
    float step_x, float step_y, float step_yaw, float fuse_alpha, int num_hist, int num_cells) {
  const int tpb = 256;
  const int blocks = (num_cells + tpb - 1) / tpb;
  aggregateKernel<<<blocks, tpb>>>(d_current, d_T_curr, d_hist_inv_T, d_hist_weights, d_hist_costs,
                                   dim_x, dim_y, dim_w, nx, ny, nw, step_x, step_y, step_yaw,
                                   fuse_alpha, num_hist, num_cells);
}
