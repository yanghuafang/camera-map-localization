#ifndef CAM_LOC_TYPES_PARAMS_H_
#define CAM_LOC_TYPES_PARAMS_H_

/// Localization tuning parameters.
///
/// Grouped by pipeline stage: pose hypothesis grid → temporal aggregation.

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
  /// Frames retained.
  ///
  /// A frame stops contributing once its plane has moved further than the grid
  /// half-extent -- 7.5 m at the default grid -- because beyond that its warped
  /// offset falls off the grid and SampleContinuous can only return a clamped
  /// border value. At KITTI speeds that is around six frames, so a much longer
  /// window costs memory and buys nothing: sweeping 1 to 70 frames moves
  /// translation RMSE by 5 mm and ANEES from 1.09 to 0.95, and nothing at all
  /// past 40. The window trades a little accuracy for a calmer covariance.
  int window_size = 12;
  /// Weight falls linearly as `1 − distance_decay · distance_m`, so history
  /// stops contributing after `1 / distance_decay` metres. See CostAggregator
  /// for which distance this is measured from.
  float distance_decay = 0.01f;
};

/// Sizing the searched region of the grid from the filter's own confidence.
///
/// The grid is anchored on the filter estimate, so the offset that reaches
/// truth is distributed roughly as the filter's covariance. Once the filter is
/// locked on, most of a fixed grid is spent proving that a pose five metres
/// away is still wrong.
///
/// The grid itself does not change size -- history warping, the CUDA
/// aggregate, mode extraction and the covariance all read a volume of fixed
/// dimensions. Only which cells are *scored* changes; the rest are filled with
/// `max_cost`, which cannot win and carries negligible softmax weight.
struct AdaptiveExtentParams {
  bool enabled = false;
  /// Sigmas of the filter covariance the window has to span, per axis.
  double sigma_multiple = 3.0;
  /// Half-extent the window may never fall below, in cells. A window narrower
  /// than the sub-cell refinement it feeds would quantize the answer.
  int min_half_cells = 2;
  /// Frames to hold the full extent after a frame the map update was gated
  /// out of. A skipped frame is the filter saying it cannot see where it is,
  /// and shrinking the search on the strength of a covariance that has not
  /// been corrected in a while is how a localizer loses lock and cannot
  /// recover.
  int reexpand_frames = 10;
};

/// Map-matching configuration: pose grid, image raster, cost modalities and
/// the support gate.
struct LocalizationParams {
  // --- Pose grid ---
  SamplingGridParams grid;

  // --- Image raster ---

  /// Size of the canvas perception is rasterized onto, in pixels. It has to
  /// match the images perception was produced from: a polyline traced on a
  /// 1242x375 label raster and rasterized onto a 1241x376 canvas is a
  /// systematic offset that nothing downstream can see. KITTI sequence 00 is
  /// 1241x376; other sequences differ.
  int image_width = 1241;
  int image_height = 376;

  // --- Cost modalities ---

  /// Score the bird's-eye branch as well. Off by default: only ground-plane
  /// classes can go through inverse perspective, and a top-down view of lane
  /// geometry is invariant along the road, so the branch has no opinion about
  /// along-track position and averaging it in dilutes the branch that does. It
  /// still constrains lateral offset and heading, which is why it is kept.
  bool enable_bev = false;
  /// Score the image branch. Turning both branches off is an error rather than
  /// a silent no-op.
  bool enable_image = true;

  // --- Map matching gates ---

  /// Points **of one class** a hypothesis must project before that class's
  /// mean cost is taken at face value. Below this the shortfall is scored as
  /// maximally wrong, so a pose that sees almost none of the map cannot win by
  /// aligning the few points it does see. Per class, not per hypothesis, since
  /// a pole contributes far fewer points than a lane. See PoseSampler.
  int min_support_points = 4;

  // --- Search strategy ---

  /// Shrink the searched region of the grid as the filter grows confident.
  AdaptiveExtentParams adaptive_extent;

  /// Threads used for CPU pose-grid scoring; 0 asks the hardware.
  ///
  /// Hypotheses are independent and each owns one cell, so the result does not
  /// depend on this: any thread count gives bitwise the same grid.
  int cost_threads = 0;
};

}  // namespace cam_loc

#endif  // CAM_LOC_TYPES_PARAMS_H_
