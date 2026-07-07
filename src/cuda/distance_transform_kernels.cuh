#pragma once

#include <cstdint>

// C-linkage launchers for CUDA kernels (implemented in distance_transform_kernels.cu).

extern "C" void cam_loc_launch_pose_cost_kernel(
    const float* d_T, const float* d_map, const uint8_t* d_mlab, int num_points,
    const float* d_dt, const uint8_t* d_dlab, int dt_w, int dt_h, float fx, float fy, float cx,
    float cy, float max_cost, int nx, int ny, int nw, float step_x, float step_y, float step_yaw,
    float* d_costs, int total);

extern "C" void cam_loc_launch_bev_cost_kernel(
    const float* d_T, const float* d_map, const uint8_t* d_mlab, int num_points,
    const float* d_dt, const uint8_t* d_dlab, int dt_w, int dt_h, float x_min, float x_max,
    float y_min, float y_max, float mpp_x, float mpp_y, float max_cost, int nx, int ny, int nw,
    float step_x, float step_y, float step_yaw, float* d_costs, int total);

extern "C" void cam_loc_launch_argmin_kernel(const float* d_costs, int n, int* d_idxs,
                                             float* d_mins, int blocks, int tpb);

extern "C" void cam_loc_launch_edt_kernel(const uint8_t* d_binary, float* d_buf, int width,
                                          int height);

extern "C" void cam_loc_launch_aggregate_kernel(
    float* d_current, const float* d_T_curr, const float* d_hist_inv_T, const float* d_hist_weights,
    const float* d_hist_costs, int dim_x, int dim_y, int dim_w, int nx, int ny, int nw,
    float step_x, float step_y, float step_yaw, float fuse_alpha, int num_hist, int num_cells);
