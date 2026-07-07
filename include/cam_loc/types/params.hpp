#pragma once

/// Localization tuning parameters and per-frame output.
///
/// Grouped by pipeline stage: pose hypothesis grid → temporal aggregation →
/// map-matching gates → debug / fallback modes.

#include <cam_loc/types/status.hpp>

namespace cam_loc {

/// 3-D pose hypothesis grid (x, y, yaw) searched each frame for map matching.
struct SamplingGridParams {
  int num_x = 21;
  int num_y = 31;
  int num_yaw = 13;
  double step_x_m = 0.5;
  double step_y_m = 0.5;
  double step_yaw_deg = 0.5;

  int totalHypotheses() const { return num_x * num_y * num_yaw; }
};

/// Sliding-window cost aggregation over recent frames (softmax-weighted pose fusion).
struct AggregationParams {
  int window_size = 70;
  float distance_decay = 0.01f;
};

/// Engine configuration: grid search, cost fusion, modality toggles, and debug flags.
struct LocalizationParams {
  // --- Pose grid & temporal fusion ---
  SamplingGridParams grid;
  AggregationParams aggregation;

  // --- Cost modalities ---
  bool enable_bev = true;
  bool enable_image = true;
  int num_cameras = 1;

  /// Fuse KITTI odometry pose as a loose global measurement when map matching fails.
  bool use_global_ego_measurement = false;

  /// Debug: inject near-perfect GT as an extra EKF update every frame.
  bool use_gt_global_prior = false;

  /// Debug: build pose grid at GT pose instead of KF estimate (oracle map matching).
  bool use_gt_sampling_plane = false;

  // --- Map matching gates ---
  double map_query_radius_m = 50.0;
  float cost_softmax_scale = 0.5f;
  float cost_flat_threshold = 0.05f;

  /// Use CUDA for pose-grid image cost evaluation when available.
  bool use_cuda = false;
};

/// Published pose and map-matching diagnostics for one processed frame.
struct LocalizationResult {
  Mat44 T_world_rig = Mat44::Identity();
  Mat66 covariance = Mat66::Identity();
  bool valid = false;
  int frame = 0;
  int64_t timestamp_ns = 0;
  Vec3 best_sample_xyyaw = Vec3::Zero();
  float aggregate_min_cost = 0.f;

  // --- Map-matching diagnostics (last frame) ---
  bool perception_synthesized = false;
  bool cost_map_flat = false;
  bool sampling_measurement_applied = false;
  float cost_map_spread = 0.f;
  double best_offset_norm_m = 0.0;
};

}  // namespace cam_loc
