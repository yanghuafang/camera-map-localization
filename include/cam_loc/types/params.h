#ifndef CAM_LOC_TYPES_PARAMS_H_
#define CAM_LOC_TYPES_PARAMS_H_

/// Localization tuning parameters and per-frame output.
///
/// Grouped by pipeline stage: pose hypothesis grid → temporal aggregation →
/// map-matching gates → debug / fallback modes.

#include "cam_loc/types/status.h"

namespace cam_loc {

/// 3-D pose hypothesis grid (x, y, yaw) searched each frame for map matching.
///
/// Cell counts should be odd so the zero offset is a real cell; an even count
/// truncates to a slightly asymmetric range. The defaults span ±5 m × ±7.5 m ×
/// ±3° about the anchor pose.
struct SamplingGridParams {
  int num_x = 21;             ///< Cells along the plane X axis.
  int num_y = 31;             ///< Cells along the plane Y axis.
  int num_yaw = 13;           ///< Cells in yaw.
  double step_x_m = 0.5;      ///< Cell pitch along X, m.
  double step_y_m = 0.5;      ///< Cell pitch along Y, m.
  double step_yaw_deg = 0.5;  ///< Cell pitch in yaw, degrees (radians inside).

  int TotalHypotheses() const { return num_x * num_y * num_yaw; }
};

/// Sliding-window cost aggregation over recent frames.
struct AggregationParams {
  /// Frames retained. Worth comparing against the grid extent: a window longer
  /// than the grid is wide holds frames whose warped offset has left it, and
  /// those contribute only clamped border values.
  int window_size = 70;
  /// Weight falls linearly as `1 − distance_decay · distance_m`, so history
  /// stops contributing after `1 / distance_decay` metres. See CostAggregator
  /// for which distance this is measured from.
  float distance_decay = 0.01f;
};

/// Engine configuration: grid search, cost fusion, modality toggles, and debug
/// flags.
struct LocalizationParams {
  // --- Pose grid & temporal fusion ---
  SamplingGridParams grid;
  AggregationParams aggregation;

  // --- Image raster ---

  /// Size of the canvas perception is rasterized onto, in pixels. It has to
  /// match the images perception was produced from: a polyline traced on a
  /// 1242x375 label raster and rasterized onto a 1241x376 canvas is a
  /// systematic offset that nothing downstream can see. KITTI sequence 00 is
  /// 1241x376; other sequences differ.
  int image_width = 1241;
  int image_height = 376;

  // --- Cost modalities ---
  /// Score the bird's-eye branch as well. **Off by default**, on measurement:
  /// on the smoke sequence it takes translation RMSE from 0.006 m to 0.022 m
  /// and yaw RMSE from 0.005 deg to 0.059 deg.
  ///
  /// The reason is structural, not a tuning accident. Only ground-plane classes
  /// can go through inverse perspective, and a top-down view of lane geometry
  /// is invariant along the road: sliding a hypothesis forward costs nothing,
  /// so the branch has no opinion about along-track position and averaging it
  /// in dilutes the branch that does. It still constrains lateral offset and
  /// heading, which is why it is kept and can be switched on.
  bool enable_bev = false;
  /// Score the image branch. Turning both branches off is an error rather than
  /// a silent no-op.
  bool enable_image = true;

  /// Fuse KITTI odometry pose as a loose global measurement when map matching
  /// fails.
  bool use_global_ego_measurement = false;

  /// Debug: inject near-perfect GT as an extra EKF update every frame.
  bool use_gt_global_prior = false;

  /// Debug: build pose grid at GT pose instead of KF estimate (oracle map
  /// matching).
  bool use_gt_sampling_plane = false;

  // --- Map matching gates ---

  /// Radius about the sampling-plane translation for the local map query, m.
  double map_query_radius_m = 50.0;
  /// Scale in the `exp(-(c - c_min) / scale)` weighting that turns the cost
  /// surface into a measurement covariance, in DT pixels. Smaller trusts the
  /// argmin more.
  float cost_softmax_scale = 0.5f;
  /// Skip the map update when the cost surface spans less than this, in DT
  /// pixels: every hypothesis fits equally well, so the argmin is noise.
  float cost_flat_threshold = 0.05f;
  /// Skip the map update when even the best hypothesis costs more than this, in
  /// DT pixels. A flat surface means "cannot tell which pose"; this means
  /// "none of them fit" -- which is what the last stretch of a finite map looks
  /// like, and what a frame of bad perception looks like. Without it the filter
  /// ingests the least-bad answer and is pulled off the trajectory.
  float max_match_cost = 3.0f;
  /// Points **of one class** a hypothesis must project before that class's
  /// mean cost is taken at face value. Below this the shortfall is scored as
  /// maximally wrong, so a pose that sees almost none of the map cannot win by
  /// aligning the few points it does see. Per class, not per hypothesis, since
  /// a pole contributes far fewer points than a lane. See PoseSampler.
  int min_support_points = 4;

  /// Use CUDA for pose-grid image cost evaluation when available.
  bool use_cuda = false;
};

/// Published pose and map-matching diagnostics for one processed frame.
struct LocalizationResult {
  /// The estimate: world ← rig (cam0), metres.
  Mat44 T_world_rig = Mat44::Identity();
  /// Filter covariance, error-state order `[x, y, z, ωx, ωy, ωz]`.
  Mat66 covariance = Mat66::Identity();
  /// False until the filter has been initialized.
  bool valid = false;
  int frame = 0;
  int64_t timestamp_ns = 0;
  /// Winning grid cell as a packed offset `(x_m, y_m, yaw_rad)` in the sampling
  /// plane — not a point.
  Vec3 best_sample_xyyaw = Vec3::Zero();
  /// Mean distance-transform cost at the argmin, in DT pixels (capped at 5).
  float aggregate_min_cost = 0.f;

  // --- Map-matching diagnostics (last frame) ---

  /// Cost surface was too flat to tell hypotheses apart; map update skipped.
  bool cost_map_flat = false;
  /// Even the best hypothesis fit badly; map update skipped.
  bool match_cost_too_high = false;
  /// A map-matching measurement reached the filter this frame.
  bool sampling_measurement_applied = false;
  /// max − min over the aggregated grid, in DT pixels. Every cell holds a real
  /// cost in [0, max_cost], so this measures how sharply the surface picks out
  /// a pose.
  float cost_map_spread = 0.f;
  /// ‖(x, y)‖ of best_sample_xyyaw, metres.
  double best_offset_norm_m = 0.0;
};

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_PARAMS_H_
